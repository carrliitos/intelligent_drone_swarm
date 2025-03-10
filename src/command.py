import time
import os
from pathlib import Path

from utils import logger
from utils import context

from command_packet import CommandPacket

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
    self.cmd_pckt = CommandPacket()

  def _send_command(self, roll: float, pitch: float, yaw: float, thrust: int):
    """Builds a command packet and sends it via UDP."""
    packet = self.cmd_pckt.build(roll, pitch, yaw, thrust)
    self.drone_connection.send_packet(packet)

  def gradual_thrust_increase(self):
    """Gradually increases thrust to the specified limit."""
    thrust = self.thrust_start
    try:
      logger.info(f"Gradually increasing thrust to {self.thrust_limit}...")

      # Gradually increase thrust
      while thrust <= self.thrust_limit:
        self._send_command(0.0, 0.0, 0.0, thrust)  # Sending only thrust commands
        logger.info(f"Thrust: {thrust}")
        thrust += self.thrust_step
        time.sleep(self.thrust_delay)

      # Maintain max thrust for a short time
      logger.info("Holding max thrust...")
      for _ in range(10):
        self._send_command(0.0, 0.0, 0.0, self.thrust_limit)
        time.sleep(self.thrust_delay)

      # Reduce thrust back to 0 gradually
      logger.info("Reducing thrust to 0...")
      while thrust >= 0:
        self._send_command(0.0, 0.0, 0.0, thrust)
        logger.info(f"Thrust: {thrust}")
        thrust -= int((self.thrust_step / 2))
        time.sleep(self.thrust_delay)

    except KeyboardInterrupt:
      logger.debug("Thrust control interrupted by user.")
