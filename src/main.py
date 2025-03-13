import os
import sys
import time
from pathlib import Path

from utils import logger
from utils import context
from esp_drone_udp import UDPConnection

import cflib

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

def main():
  cflib.crtp.init_drivers(enable_debug_driver=False)
  drone_udp = "udp://192.168.43.42:2390"
  drone = UDPConnection(drone_udp)

  try:
    while True:
      drone.connect()
  except KeyboardInterrupt:
    logger.debug("Operation interrupted by user.")
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)
  finally:
    if drone:
      logger.info("Closing connection to drone.")
      drone._cf.close_link()

if __name__ == '__main__':
  main()
