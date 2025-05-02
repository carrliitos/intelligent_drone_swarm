import os
import sys
import time
from pathlib import Path

from utils import logger
from utils import context
from esp_drone_udp import UDPConnection
from command import Command
from drone_log import DroneLogs
from watchdog.observers import Observer
from pid_watchdog import CSVFileHandler

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
  drone_logger = DroneLogs(drone)
  command = Command(drone=drone,
                    drone_logger=drone_logger, 
                    thrust_start=10000, 
                    thrust_limit=40000, 
                    thrust_step=100, 
                    thrust_delay=0.01)

  try:
    drone.connect()
    time.sleep(5) # 5 second wait
    drone_logger.start_logging()

    event_handler = CSVFileHandler(command)
    observer = Observer()
    observer.schedule(event_handler, path="./src", recursive=False)
    observer.start()
    
    # command.gradual_thrust_increase()
    # command.hover()
    command.pygame()
  except KeyboardInterrupt:
    logger.debug("Operation interrupted by user.")
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)
  finally:
    if drone:
      logger.info("Stopping logging and closing connection to drone.")
      drone_logger.stop_logging()
      drone._stop_timer()
      drone._cf.close_link()

if __name__ == '__main__':
  main()
