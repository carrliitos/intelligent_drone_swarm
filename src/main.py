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

UDP = "udp://192.168.43.51:2390"
RADIO_CHANNELS = {
  "7": "radio://0/80/2M/E7E7E7E7E7",
  "8": "radio://0/80/2M/E7E7E7E7E8",
  "9": "radio://0/80/2M/E7E7E7E7E9"
}

def run(connection_type, use_vision=False):
  cflib.crtp.init_drivers(enable_debug_driver=False)
  time.sleep(1.0)

  drone = DroneConnection(connection_type)
  time.sleep(1.0)

  drone_logger = DroneLogs(drone)
  time.sleep(1.0)

  command = Command(drone=drone, drone_logger=drone_logger)
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
        calib_path=None,
        marker_length_m=None,
        draw_axes=True
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
        detector.release()
      except Exception:
        pass

    if drone:
      drone._cf.close_link()

def print_usage():
  print("Usage:")
  print("  python main.py udp")
  print("  python main.py radio [7|8|9]")
  print("  (append 'vision' to enable ArUco webcam)")
  sys.exit(1)

if __name__ == '__main__':
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
