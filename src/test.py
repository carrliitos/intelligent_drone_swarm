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
  CrazyFlieDataProvider,
  FlightSettings
)
from command import Command

import cflib

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

def main():
  drone_udp = "udp://192.168.43.42:2390"

  le = UDPConnection(drone_udp)  # Establish UDP connection
  command = Command(le._cf)  # Create a command instance using the Crazyflie object

  x_provider = CrazyFlieXProvideable(x=0.5)
  y_provider = CrazyFlieYProvideable(y=0.3)

  pitch_provider = CrazyFlieDataProvider(x_provider)
  roll_provider = CrazyFlieDataProvider(x_provider)
  yaw_provider = CrazyFlieDataProvider(y_provider)
  thrust_provider = CrazyFlieDataProvider(y_provider)

  settings = FlightSettings(pitch_rate=0.8, yaw_rate=0.8, max_thrust=30000)
  command_helper = CommandHelper(pitch_provider, roll_provider, yaw_provider, thrust_provider, settings.__dict__)

  try:
    le.connect()

    while True:
      command_helper.prepare_data()
      command.thrust_gradual(
        thrust_limit=command_helper.max_thrust,
        roll=command_helper.roll,
        pitch=command_helper.pitch,
        yawrate=command_helper.yaw,
        step=500,  # Increase by 500 each step
        delay=0.05 # 50ms delay per step
      )
      time.sleep(0.01)
  except KeyboardInterrupt:
    logger.info("Process interrupted. Terminating...")
    sys.exit(0)
  except Exception as e:
    logger.error(f"Error: {e}")
    sys.exit(1)

if __name__ == '__main__':
  cflib.crtp.init_drivers(enable_debug_driver=False)
  main()
