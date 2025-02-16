import os
import sys
from pathlib import Path

from utils import logger
from utils import context
from drone_connections_swarm import ESPDrone

import cflib

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

def main():
  cflib.crtp.init_drivers(enable_debug_driver=False)
  udp_list = {"udp://192.168.43.42:2390"}

  le = ESPDrone()
  try:
    while True:
      le.start()
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)

if __name__ == '__main__':
  main()
