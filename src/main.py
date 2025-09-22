import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

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

def run(connection_type, use_vision=False, swarm_uris=None):
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
        draw_axes=False,
        # Draw grid
        draw_grid=False,
        grid_step_px=40,
        draw_rule_of_thirds=False,
        draw_crosshair=False
      )
      detector.open()

      def _vision_loop():
        # Loop to display annotated frames
        while not stop_vision.is_set():
          frame, results = detector.read()
          if frame is None:
            continue
          # Log detections
          ids = results.get("ids")
          if ids is not None:
            try:
              flat_ids = [int(x[0]) for x in ids]
            except Exception:
              pass

          try:
            cv2.imshow("Intelligent Drone Swarm (ArUco)", frame)
            if (cv2.waitKey(1) & 0xFF) == 27:  # ESC to close vision
              stop_vision.set()
              break
          except Exception:
            pass

      vision_thread = threading.Thread(target=_vision_loop, daemon=True)
      vision_thread.start()

      ctrl_stop = threading.Event()
      def _ctrl_loop():
        try:
          command.follow_target_ibvs(
            detector=detector,
            stop_event=ctrl_stop,
            desired_area_px=10000,
            loop_hz=20,
            use_vertical=True
          )
        except Exception as e:
          logger.error(f"IBVS error: {e}")
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
  print("  fly udp")
  print("  fly radio [7|8|9]")
  print("  (append 'vision' to enable ArUco webcam)")
  print("  fly swarm <channels ...> # e.g. swarm 7, 8, 9")
  sys.exit(1)

def cli():
  if len(sys.argv) < 2:
    logger.error("Missing connection type argument.")
    print_usage()

  arg = sys.argv[1].lower()
  connection_type = None
  use_vision = False

  if arg == "udp":
    connection_type = UDP
  elif arg == "radio":
    if len(sys.argv) < 3:
      logger.error("Missing radio channel argument.")
      print_usage()
    channel = sys.argv[2]
    if channel in RADIO_CHANNELS:
      connection_type = RADIO_CHANNELS[channel]
    else:
      logger.error(f"Invalid radio channel: {channel}")
      print_usage()
  else:
    logger.error(f"Invalid connection type: {arg}")
    print_usage()

  if len(sys.argv) >= 3 and sys.argv[-1].lower() == "vision":
    use_vision = True
  if len(sys.argv) >= 4 and sys.argv[3].lower() == "vision":
    use_vision = True

  logger.info(f"Using connection: {connection_type}")
  run(connection_type, use_vision=use_vision)

if __name__ == '__main__':
  cli()
