import time
import os
from pathlib import Path

from utils import logger, context
from esp_drone_udp import UDPConnection

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class Command:
  def __init__(self, 
               drone: UDPConnection, 
               thrust_start: int, 
               thrust_limit: int, 
               thrust_step: int, 
               thrust_delay: float):
    """
    Handles gradual thrust commands for the drone.

    :param drone: Instance of UDPConnection.
    :param thrust_start: Initial thrust value.
    :param thrust_limit: Maximum thrust value.
    :param thrust_step: Step increment for thrust increase.
    :param thrust_delay: Delay between thrust updates.
    """
    self.drone = drone
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay

  def gradual_thrust_increase(self):
    """Gradually increases and decreases thrust for testing stability."""
    thrust = self.thrust_start

    try:
      logger.info(f"Gradually increasing thrust to {self.thrust_limit}...")

      # Gradually increase thrust
      while thrust <= self.thrust_limit:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        logger.info(f"Thrust: {thrust}")
        thrust += self.thrust_step
        time.sleep(self.thrust_delay)

      # Maintain max thrust for a short time
      logger.info("Holding max thrust...")
      hold_time = 5 # seconds
      start_time = time.time()
      while (time.time() - start_time) < hold_time:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, self.thrust_limit)
        time.sleep(self.thrust_delay)

      # Reduce thrust back to 0 gradually
      logger.info("Reducing thrust to 0...")
      while thrust >= 0:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        logger.info(f"Thrust: {thrust}")
        thrust -= int(self.thrust_step / 2)
        time.sleep(self.thrust_delay)

    except KeyboardInterrupt:
      logger.debug("Thrust control interrupted by user.")
    finally:
      self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)  # Ensure drone stops safely
      logger.info("Thrust set to 0 for safety.")
