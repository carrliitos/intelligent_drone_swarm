import struct
import time
import os
from pathlib import Path

from utils import logger
from utils import context

# Logger setup
directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")


class DroneLogger:
  """Handles logging of telemetry data from an ESP32 drone via UDP."""

  CMD_START_LOGGING = 0x02
  CMD_STOP_LOGGING = 0x03

  def __init__(self, udp_connection, toc_handler):
    self.udp_connection = udp_connection
    self.toc_handler = toc_handler
    self.log_config = {}

  def add_variable(self, var_name, period_ms):
    """Add a telemetry variable to the logging configuration."""
    logger.inf(f"self.toc_handler.toc: {self.toc_handler.toc}")
    if var_name in self.toc_handler.toc:
      ident = self.toc_handler.toc[var_name]
      self.log_config[var_name] = {'id': ident, 'period': period_ms}
      logger.info(f"Added {var_name} to log configuration.")
    else:
      logger.warning(f"Variable {var_name} not found in TOC.")

  def start_logging(self):
    """Send a UDP command to start logging selected variables."""
    logger.info("Starting logging...")
    for var_name, config in self.log_config.items():
      packet = struct.pack('<BIB', self.CMD_START_LOGGING, config['id'], config['period'])
      self.udp_connection.send_packet(packet)
      logger.debug(f"Sent start log command for {var_name} (ID {config['id']}, Period {config['period']}ms)")

  def stop_logging(self):
    """Send a UDP command to stop all logging."""
    logger.info("Stopping logging...")
    packet = struct.pack('<B', self.CMD_STOP_LOGGING)
    self.udp_connection.send_packet(packet)
    logger.debug("Sent stop log command.")

  def process_log_packet(self, data):
    """Process an incoming telemetry packet."""
    if len(data) < 4:
      logger.warning("Received invalid log packet (too short).")
      return

    timestamp = struct.unpack_from('<I', data, 0)[0]  # Extract timestamp
    offset = 4
    log_results = {}

    for var_name, config in self.log_config.items():
      try:
        value = struct.unpack_from('<f', data, offset)[0]
        offset += 4
        log_results[var_name] = value
      except struct.error:
        logger.error(f"Failed to parse log variable {var_name} from packet.")

    logger.info(f"{timestamp} - {log_results}")

