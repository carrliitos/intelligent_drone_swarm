import time
import sys
import os
from pathlib import Path
from cflib.crazyflie.log import LogConfig
from utils import logger
from utils import context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

class DroneLogs:
  def __init__(self, cf):
    """
    Handles logging real-time drone data.

    Args:
      cf (Crazyflie): The Crazyflie instance for logging.
    """
    self._cf = cf
    self.id_counter = 0  # Unique identifier for data rows
    self.log_config = None

    self.log_variables = ["pm.batteryLevel", "motor.m1", "motor.m2", "motor.m3", "motor.m4"]

  def start_logging(self):
    """
    Ensures the TOC is fully loaded before logging starts.
    """
    logger.info("Waiting for Crazyflie TOC to be ready...")

    # Wait until TOC is ready! (max 10 seconds) -- idk if 10 seconds is too long
    timeout = 10  # seconds
    start_time = time.time()
    while not self._cf.param.toc.toc or len(self._cf.param.toc.toc) == 0:
      if time.time() - start_time > timeout:
        logger.error("TOC failed to load in time. Logging aborted.")
        return
      time.sleep(0.5)

    logger.info("TOC is ready. Starting logging...")

    self.log_config = LogConfig(name="motor_battery", period_in_ms=50)
    for var in self.log_variables:
      try:
        self.log_config.add_variable(var, 'float')
      except KeyError:
        logger.warning(f"Variable {var} not found in TOC.")

    try:
      self._cf.log.add_config(self.log_config)
      self.log_config.data_received_cb.add_callback(self._log_callback)
      self.log_config.error_cb.add_callback(self._log_error_callback)
      self.log_config.start()
      logger.info(f"Started logging: {self.log_variables}")
    except Exception as e:
      logger.error(f"Error setting up logging: {e}")
      sys.exit(1)

  def _log_callback(self, timestamp, data, logconf):
    """
    Callback for log data.
    Logs battery level and motor values to the console.
    """
    log_entry = {
      "timestamp": timestamp,
      "pm.batteryLevel": data.get("pm.batteryLevel", "N/A"),
      "motor.m1": data.get("motor.m1", "N/A"),
      "motor.m2": data.get("motor.m2", "N/A"),
      "motor.m3": data.get("motor.m3", "N/A"),
      "motor.m4": data.get("motor.m4", "N/A"),
    }
    logger.info(f"Drone Log: {log_entry}")
    self.id_counter += 1

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"Logging error: {msg}")
