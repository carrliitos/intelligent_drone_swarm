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

def run(connection_type):
  cflib.crtp.init_drivers(enable_debug_driver=False)
  drone = DroneConnection(connection_type)
  drone_logger = DroneLogs(drone)
  command = Command(drone=drone,
                    conn_str=drone,
                    drone_logger=drone_logger, 
                    thrust_start=10000, 
                    thrust_limit=40000, 
                    thrust_step=100, 
                    thrust_delay=0.01)

  try:
    drone.connect()
    time.sleep(5) # 5 second wait
    drone_logger.start_logging()
    
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
  arg = sys.argv[1]
  connection_type = None
  UDP = "udp://192.168.43.51:2390"
  RADIO7 = "radio://0/80/2M/E7E7E7E7E7"
  RADIO8 = "radio://0/80/2M/E7E7E7E7E8"
  RADIO9 = "radio://0/80/2M/E7E7E7E7E9"

  logger.info(f"Connection type: {arg}")
  
  if arg == "udp":
    connection_type = UDP
  elif arg == "radio":
    if sys.argv[2] == "7":
      connection_type = RADIO7
    elif sys.argv[2] == "8":
      connection_type = RADIO8
  else:
    logger.error("Invalid argument.")

  run(connection_type)
