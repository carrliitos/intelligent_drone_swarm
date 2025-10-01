import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import numpy as np
import pygame
import os
import sys
import time
import threading
import cv2
from pathlib import Path

from utils import logger
from utils import context
from drone_connection import DroneConnection
from command import Command
from command import SwarmCommand
from drone_log import DroneLogs
from vision import DetectorRT

import cflib

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

calibration_path = f"{directory}/src/utils/calibration_imgs/cam_param.npz"

UDP = "udp://192.168.43.51:2390"
RADIO_CHANNELS = {
  "7": "radio://0/80/2M/E7E7E7E7E7",
  "8": "radio://0/80/2M/E7E7E7E7E8",
  "9": "radio://0/80/2M/E7E7E7E7E9"
}

_latest_frame_lock = threading.Lock()
_latest_frame_np = None
_latest_ids = []

def run(connection_type, use_vision=False, use_control=False, swarm_uris=None):
  cflib.crtp.init_drivers(enable_debug_driver=False)
  time.sleep(1.0)

  drone = DroneConnection(connection_type)
  time.sleep(1.0)

  drone_logger = DroneLogs(drone)
  time.sleep(1.0)

  swarm_cmd = SwarmCommand(swarm_uris) if swarm_uris else None
  command = Command(drone=drone, 
                    drone_logger=drone_logger, 
                    swarm=swarm_cmd,
                    takeoff_alt=0.25)
  detector = None
  vision_thread = None
  stop_vision = threading.Event()

  try:
    logger.info("==========Connecting to drone==========")
    drone.connect()
    time.sleep(5.0)

    if use_vision:
      logger.info("==========Connecting to OpenCV==========")
      detector = DetectorRT(
        dictionary="4x4_1000",
        camera=2,
        calib_path=calibration_path,
        marker_length_m=0.025,
        width=1920, 
        height=1080, 
        fps=30,
        draw_axes=True,
        allowed_ids=[249],
        # Draw grid
        draw_grid=True,
        grid_step_px=60,
        draw_rule_of_thirds=False,
        draw_crosshair=False
      )
      detector.open()

      def _vision_loop():
        max_w, max_h = 960, 540
        last_ok = time.time()
        while not stop_vision.is_set():
          frame, results = detector.read()
          if frame is None:
            if time.time() - last_ok > 1.0:
              logger.warning("No camera frames for >1s; check USB bandwidth, device index, or conflicts.")
            time.sleep(0.01)
            continue
          last_ok = time.time()

          # preview resize to keep it light
          h, w = frame.shape[:2]
          scale = min(max_w / w, max_h / h, 1.0)
          if scale < 1.0:
            frame = cv2.resize(frame, (int(w*scale), int(h*scale)))

          # Store latest frame (BGR -> RGB) for pygame
          global _latest_frame_np
          global _latest_ids
          rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
          with _latest_frame_lock:
            _latest_frame_np = np.ascontiguousarray(rgb)
            ids = results.get("ids")
            _latest_ids = [] if ids is None else [int(i) for i in ids.flatten()]

      vision_thread = threading.Thread(target=_vision_loop, daemon=True)
      vision_thread.start()

      if use_control:
        ctrl_stop = threading.Event()
        def _ctrl_loop():
          try:
            command.follow_target_servo(
              detector=detector,
              stop_event=ctrl_stop,
              start_event=command.ibvs_enable_event,
              loop_hz=50,
              gains=dict(Kpx=0.6, Kdx=0.2,
                         Kpy=0.6, Kdy=0.2,
                         Kpz=1.0, Kiz=0.2,
                         Kpyaw=2.0, Kdyaw=0.3),
              vision_yaw_alpha=0.05,
              forward_nudge_alpha=0.03
            )
          except Exception as e:
            logger.error(f"Servo error: {e}")
        ctrl_thread = threading.Thread(target=_ctrl_loop, daemon=True)
        ctrl_thread.start()

    time.sleep(5.0)

    logger.info("==========Connecting to PyGame==========")
    command.pygame()
    time.sleep(5.0)
  except KeyboardInterrupt:
    logger.debug("Operation interrupted by user.")
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)
  finally:
    if detector:
      stop_vision.set()
      if vision_thread:
        vision_thread.join(timeout=2.0)
      try:
        if 'ctrl_stop' in locals():
          ctrl_stop.set()
        if 'ctrl_thread' in locals():
          ctrl_thread.join(timeout=2.0)
        detector.release()
      except Exception:
        pass

    if drone:
      drone._cf.close_link()

    if swarm_cmd:
      swarm_cmd.land()
      swarm_cmd.close()

def print_usage():
  print("Usage:")
  print("  fly udp [vision] [control]")
  print("  fly radio <7|8|9> [vision] [control]")
  print("  fly swarm <channels ...>  # e.g., swarm 7 8 9  (not shown here)")
  print("Notes:")
  print("  - 'vision' starts the ArUco webcam preview thread")
  print("  - 'control' starts the IBVS control loop (requires 'vision')")
  sys.exit(1)

def cli():
  if len(sys.argv) < 2:
    logger.error("Missing connection type argument.")
    print_usage()

  args = [a.lower() for a in sys.argv[1:]]
  connection_type = None
  use_vision = False
  use_control = False

  if args[0] == "udp":
    connection_type = UDP
    extras = set(args[1:])  # flags like 'vision', 'control'
  elif args[0] == "radio":
    if len(args) < 2:
      logger.error("Missing radio channel argument.")
      print_usage()
    channel = args[1]
    if channel in RADIO_CHANNELS:
      connection_type = RADIO_CHANNELS[channel]
    else:
      logger.error(f"Invalid radio channel: {channel}")
      print_usage()
    extras = set(args[2:])
  else:
    logger.error(f"Invalid connection type: {args[0]}")
    print_usage()

  use_vision = "vision" in extras
  use_control = "control" in extras

  if use_control and not use_vision:
    logger.warning("`control` requested without `vision`; disabling control.")
    use_control = False

  logger.info(f"Using connection: {connection_type}  | vision={use_vision} control={use_control}")
  run(connection_type, use_vision=use_vision, use_control=use_control)

if __name__ == '__main__':
  cli()
