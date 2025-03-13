import time
import os
import threading
import sys
from pathlib import Path
from cflib.crazyflie.log import LogConfig
from utils import logger, context
from esp_drone_udp import UDPConnection

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_file_name, logger_file)

class DroneLogs:
  def __init__(self, drone: UDPConnection):
    """
    Handles logging real-time drone data.

    Args:
      drone (UDPConnection): The Crazyflie UDP connection instance.
    """
    self.drone = drone
    self._cf = drone._cf
    self.log_config = None
    # self.log_variables = ["crtp.rxRate", "crtp.txRate"]
    self.log_variables = ["pm.vbatMV"]

  def start_logging(self):
    """
    Starts logging after ensuring the TOC is loaded.
    """
    if self._cf.state == 0:
      logger.error("Drone is not connected. Cannot start logging. Exiting...")
      sys.exit(0)

    logger.info("Waiting for TOC to be downloaded...")
    while self._cf.log.toc is None or not self._cf.log.toc.toc:
      time.sleep(0.1)

    logger.info("TOC downloaded. Starting logging...")
    time.sleep(2)

    # Start logging in a separate thread
    log_thread = threading.Thread(target=self._log_drone_data, daemon=True)
    log_thread.start()

  def _log_drone_data(self):
    """
    Log selected variables from the Crazyflie.
    """
    log_config = LogConfig(name='drone_logs', period_in_ms=500)
    try:
      for var in self.log_variables:
        log_config.add_variable(var, 'float')

      self._cf.log.add_config(log_config)
      log_config.data_received_cb.add_callback(self._log_callback)
      log_config.error_cb.add_callback(self._log_error_callback)
      log_config.start()
      logger.info("Started logging drone data.")

      # Keep logging until interrupted
      while self._cf.state != 0:
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if log_config.valid:
        log_config.stop()
        logger.info("Stopped logging drone data.")

  def _log_callback(self, timestamp, data, logconf):
    """
    Callback for receiving log data.
    """
    logger.info(f"Timestamp: {timestamp}, Data: {data}")

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"Logging error: {msg}")
