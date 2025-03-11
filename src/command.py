import time
import os
import math
from pathlib import Path

from utils import logger
from utils import context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class Command:
  def __init__(self, drone_connection, thrust_start, thrust_limit, thrust_step, thrust_delay):
    self.drone_connection = drone_connection
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay

  def gradual_thrust_increase(self):
    """Gradually increases thrust to the specified limit while oscillating roll, pitch, and yaw."""
    thrust = self.thrust_start
    t = 0  # Time counter for oscillation
    dt = self.thrust_delay  # Use thrust delay as time increment

    try:
      logger.info(f"Gradually increasing thrust to {self.thrust_limit} with oscillations...")

      # Gradually increase thrust
      while thrust <= self.thrust_limit:
        roll = 50 * math.sin((5 * t))
        pitch = 50 * math.sin((5 * t) + math.pi / 3)  # Phase shift for variation
        yaw = 50 * math.sin((5 * t) + 2 * math.pi / 3)

        self.drone_connection.send_command(roll, pitch, yaw, thrust)
        logger.info(f"Thrust: {thrust}, Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        thrust += self.thrust_step
        time.sleep(dt)
        t += 0.2

      # Maintain max thrust for a short time
      logger.info("Holding max thrust...")
      for _ in range(10):
        roll = 50 * math.sin(5 * t)
        pitch = 50 * math.sin((5 * t) + math.pi / 3)
        yaw = 50 * math.sin((5 * t) + 2 * math.pi / 3)

        self.drone_connection.send_command(roll, pitch, yaw, self.thrust_limit)
        time.sleep(dt)
        t += 0.2

      # Reduce thrust back to 0 gradually
      logger.info("Reducing thrust to 0 with oscillations...")
      while thrust >= 0:
        roll = 50 * math.sin((5 * t))
        pitch = 50 * math.sin((5 * t) + math.pi / 3)
        yaw = 50 * math.sin((5 * t) + 2 * math.pi / 3)

        self.drone_connection.send_command(roll, pitch, yaw, thrust)
        logger.info(f"Thrust: {thrust}, Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        thrust -= int((self.thrust_step / 2))
        time.sleep(dt)
        t += 0.2

    except KeyboardInterrupt:
      logger.debug("Thrust control interrupted by user.")
