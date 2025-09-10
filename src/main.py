import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import os
import sys
import time
from pathlib import Path

from utils import logger
from utils import context
from drone_connection import DroneConnection
from command import Command
from drone_log import DroneLogs

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

def run(connection_type):
  cflib.crtp.init_drivers(enable_debug_driver=False)
  time.sleep(1.0)

  drone = DroneConnection(connection_type)
  time.sleep(1.0)

  drone_logger = DroneLogs(drone)
  time.sleep(1.0)

  command = Command(drone=drone, drone_logger=drone_logger)

  try:
    drone.connect()
    time.sleep(5) # 5 second wait
    
    command.pygame()
  except KeyboardInterrupt:
    logger.debug("Operation interrupted by user.")
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)
  finally:
    if drone:
      drone._cf.close_link()

def print_usage():
  print("Usage:")
  print("  python main.py udp")
  print("  python main.py radio [7|8|9]")
  sys.exit(1)

if __name__ == '__main__':
  if len(sys.argv) < 2:
    logger.error("Missing connection type argument.")
    print_usage()

  arg = sys.argv[1].lower()
  connection_type = None

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

  logger.info(f"Using connection: {connection_type}")
  run(connection_type)
