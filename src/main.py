import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import contextlib
import numpy as np
import pygame
import os
import sys
import time
import threading
import cv2
from pathlib import Path
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env") 

from utils import logger
from utils import context
from utils import helpers
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
logger = logger.setup_logger(
  logger_name=logger_name, 
  log_file=logger_file, 
  log_level=os.getenv("LOG_LEVEL")
)

WAYPOINT_ENV_DEFAULT = os.getenv("WAYPOINT_LOGGING", "0")
UDP = os.getenv("UDP")
RADIO_CHANNELS = {
  "7": os.getenv("RADIO_CHANNEL_7"),
  "8": os.getenv("RADIO_CHANNEL_8"),
  "9": os.getenv("RADIO_CHANNEL_9")
}

_latest_frame_lock = threading.Lock()
_latest_frame_np = None
_latest_ids = []
_latest_src_wh = None # (width, height) of detector/ frame
_latest_frame_idx = 0

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
      logger.info("==========Connecting to vision==========")
      detector = DetectorRT(
        dictionary=os.getenv("ARUCO_DICTIONARY"),
        camera=helpers._i(os.getenv("CAMERA_CONNECTION")),
        width=helpers._i(os.getenv("CAMERA_WIDTH")),
        height=helpers._i(os.getenv("CAMERA_HEIGHT")),
        fps=helpers._i(os.getenv("CAMERA_FPS")),
        calib_path=os.getenv("CALIB_PATH"),
        marker_length_m=helpers._f(os.getenv("MARKER_LENGTH_M")),
        window_title=os.getenv("WINDOW_TITLE"),
        draw_axes=helpers._b(os.getenv("DRAW_AXES")), 
        allowed_ids=helpers._ids(os.getenv("ALLOWED_IDS")),
        draw_grid=helpers._b(os.getenv("DRAW_GRID")), 
        min_brightness=helpers._f(os.getenv("BRIGHTNESS_VALUE")), 
        grid_step_px=helpers._i(os.getenv("GRID_STEP_PX")),
        grid_color=os.getenv("GRID_COLOR"),
        grid_thickness=helpers._i(os.getenv("GRID_THICKNESS")),
        draw_rule_of_thirds=helpers._b(os.getenv("DRAW_RULE_OF_THIRDS")),
        draw_crosshair=helpers._b(os.getenv("DRAW_CROSSHAIR")),
        capture_cells=helpers._b(os.getenv("CAPTURE_CELLS")),
        fps_counter=helpers._i(os.getenv("FPS_COUNTER")),
        fps_display=helpers._f(os.getenv("FPS_DISPLAY"))
      )
      detector.open()
      setattr(sys.modules[__name__], "detector", detector)

      command.set_vision_hooks(
        on_click=detector.set_click_point,
        on_toggle=detector.toggle_delta,
        on_clear=detector.clear_click
      )

      def _vision_loop():
        max_w, max_h = 960, 540
        last_ok = time.time()

        global _latest_frame_idx

        while not stop_vision.is_set():
          frame, results = detector.read()
          if frame is None:
            if time.time() - last_ok > 1.0:
              logger.warning("No camera frames for >1s; check USB bandwidth, device index, or conflicts.")
            time.sleep(0.01)
            continue
          last_ok = time.time()

          # save original source size (detector frame)
          h, w = frame.shape[:2]
          global _latest_src_wh
          with _latest_frame_lock:
            _latest_frame_idx = (_latest_frame_idx + 1) % 2_000_000_000 # large wraparound counter; avoids unbounded growth
            _latest_src_wh = (w, h)

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
        logger.info("==========Control-loop open==========")
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
    # stop control/vision loops first
    if 'ctrl_stop' in locals(): ctrl_stop.set()
    if 'ctrl_thread' in locals(): ctrl_thread.join(timeout=2.0)

    if detector:
      stop_vision.set()
      if vision_thread: vision_thread.join(timeout=2.0)
      with contextlib.suppress(Exception): detector.release()

    # land swarm before closing any links
    if swarm_cmd:
      with contextlib.suppress(Exception): swarm_cmd.land()
      with contextlib.suppress(Exception): swarm_cmd.close()

    if drone:
      with contextlib.suppress(Exception): drone._cf.close_link()

def print_usage():
  print("Usage:")
  print("  fly udp [vision] [control]")
  print("  fly radio <7|8|9> [vision] [control]")
  print("  fly swarm <channels ...>  # e.g., swarm 7 8 9  (not shown here)")
  print("Notes:")
  print("  - 'vision' starts the ArUco webcam preview thread")
  print("  - 'control' starts the IBVS control loop (requires 'vision')")
  print("  - 'waypoint' enables click -> waypoint logging HUD/JSONL")
  sys.exit(1)

def cli():
  if len(sys.argv) < 2:
    logger.error("Missing connection type argument.")
    print_usage()

  args = [a.lower() for a in sys.argv[1:]]
  connection_type = None
  use_vision = False
  use_control = False
  waypoint_flag = False

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
  elif args[0] == "swarm":
    # tokens after "swarm"
    tail = sys.argv[2:]
    # split into channels (digits) and extras (strings)
    channels, extras = [], set()
    for tok in tail:
      if tok.isdigit():
        channels.append(tok)
      else:
        extras.add(tok.lower())

    if not channels:
      logger.error("Provide at least one radio channel for swarm.")
      print_usage()

    bad = [c for c in channels if c not in RADIO_CHANNELS]
    if bad:
      logger.error(f"Invalid radio channel(s): {', '.join(bad)}")
      print_usage()

    swarm_uris = [RADIO_CHANNELS[c] for c in channels]
    first = channels[0]
    connection_type = RADIO_CHANNELS[first]
    use_vision = "vision" in extras
    use_control = "control" in extras and use_vision
    logger.info(f"Swarm URIs: {swarm_uris} | vision={use_vision} control={use_control}")
    run(connection_type, use_vision=use_vision, use_control=use_control, swarm_uris=swarm_uris)
    sys.exit(0)
  else:
    logger.error(f"Invalid connection type: {args[0]}")
    print_usage()

  use_vision = "vision" in extras
  use_control = "control" in extras
  waypoint_flag = "waypoint" in extras

  if use_control and not use_vision:
    logger.warning("`control` requested without `vision`; disabling control.")
    use_control = False

  # Allow CLI to force-enable waypoint logging
  if waypoint_flag:
    os.environ["WAYPOINT_LOGGING"] = "1"
  else:
    os.environ["WAYPOINT_LOGGING"] = WAYPOINT_ENV_DEFAULT

  logger.info(f"Using connection: {connection_type}  | vision={use_vision} control={use_control}")
  run(connection_type, use_vision=use_vision, use_control=use_control)

if __name__ == '__main__':
  cli()
