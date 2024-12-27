import os
from pathlib import Path

from utils import logger
from utils import context
from esp_drone import ESPDrone

import cflib

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

def main():
  cflib.crtp.init_drivers(enable_debug_driver=False)
  drone_udp = "udp://192.168.43.42:2390"
  le = ESPDrone(drone_udp)
  try:
    le.test_connection()
  except Exception as e:
    logger.error("Error: {e}")

if __name__ == '__main__':
  main()
