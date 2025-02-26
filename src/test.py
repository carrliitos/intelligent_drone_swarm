import os
import sys
import time
from pathlib import Path

from utils import logger
from utils import context
from udp_connection import UDPConnection
from command_helper import (
  CommandHelper, 
  CrazyFlieXProvideable, 
  CrazyFlieYProvideable, 
  CrazyFlieDataProvider
)

import cflib

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

def main():
  cflib.crtp.init_drivers(enable_debug_driver=False)
  drone_udp = "udp://192.168.43.42:2390"

  le = UDPConnection(drone_udp)
  
  x_provider = CrazyFlieXProvideable(x=0.5)
  y_provider = CrazyFlieYProvideable(y=-0.3)
 
  pitch_provider = CrazyFlieDataProvider(x_provider)
  roll_provider = CrazyFlieDataProvider(x_provider)
  yaw_provider = CrazyFlieDataProvider(y_provider)
  thrust_provider = CrazyFlieDataProvider(y_provider) 

  settings = {"pitchRate": 0.8, "yawRate": 0.6, "maxThrust": 20000}
  try:
    le.connect()
    command_helper = CommandHelper(pitch_provider, 
                                   roll_provider, 
                                   yaw_provider, 
                                   thrust_provider, 
                                   settings)
    while True:
      command_helper.prepare_data()
      if int(time.time()) % 5 == 0:
        logger.info(f"Pitch: {command_helper.pitch}, Roll: {command_helper.roll}, Yaw: {command_helper.yaw}, Thrust: {command_helper.thrust}")

      le.thrust__gradual(thrust_limit=30000,
                         roll=command_helper.roll,
                         pitch=command_helper.pitch,
                         yawrate=command_helper.yaw)
      # le.thrust(thrust=25000, 
      #           roll=command_helper.roll, 
      #           pitch=command_helper.pitch, 
      #           yawrate=command_helper.yaw)
      time.sleep(0.01)
  except KeyboardInterrupt:
    logger.info("Process interrupted. Terminating...")
    sys.exit(0)
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)

if __name__ == '__main__':
  main()
