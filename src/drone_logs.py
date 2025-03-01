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
    self.log_config = None
    self.log_variables = ["pm.vbatMV", "pwm.m1_pwm", "pwm.m2_pwm", "pwm.m3_pwm", "pwm.m4_pwm"]

  def start_logging(self):
    """
    Starts logging after sending initial setpoints.
    """
    logger.info("Sending initial idle setpoints to stabilize logging...")
    self._cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)  # Give drone time to stabilize

    log_config = self._setup_logging(self.log_variables)
    if log_config:
      logger.info("Logging started successfully.")
    else:
      logger.error("Failed to start logging.")

  def _setup_logging(self, variables):
    """
    Set up logging for the specified variables.
    """
    log_config = LogConfig(name='motor_battery', period_in_ms=50)
    for variable in variables:
      try:
        log_config.add_variable(variable, 'float')
      except KeyError:
        logger.warning(f"Variable {variable} not found in TOC.")

    try:
      self._cf.log.add_config(log_config)
      log_config.data_received_cb.add_callback(self._log_callback)
      log_config.error_cb.add_callback(self._log_error_callback)
      log_config.start()
      logger.info("Started logging variables.")
      return log_config
    except Exception as e:
      logger.error(f"Error setting up logging: {e}")
      return None

  def _log_callback(self, timestamp, data, logconf):
    """
    Callback for receiving log data.
    """
    logger.info(f"Timestamp: {timestamp}, Data: {data}")

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"LogConfig {logconf} error: {msg}")
