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
    Ensures the TOC is fully loaded before starting logging.
    """
    logger.info("Waiting for TOC to be downloaded...")
    while self._cf.log.toc is None or not self._cf.log.toc.toc:
      time.sleep(0.1)  # Ensure the TOC has loaded before proceeding

    # Log TOC contents for debugging
    logger.info("TOC Contents:")
    toc = self._cf.log.toc
    for element_id, element in toc.toc.items():
      logger.debug(f"{element_id}: {element}")

    logger.info("TOC downloaded. Starting logging...")
    
    # Start logging in a separate thread to avoid blocking
    log_thread = threading.Thread(target=self._log_battery_motor_data)
    log_thread.start()

  def _log_battery_motor_data(self):
    """
    Log power and motor data from the ESP-Drone.
    """
    log_config = LogConfig(name='battery_motor', period_in_ms=100)
    try:
      for var in self.log_variables:
        log_config.add_variable(var, 'float')

      self._cf.log.add_config(log_config)
      log_config.data_received_cb.add_callback(self._log_callback)
      log_config.error_cb.add_callback(self._log_error_callback)
      log_config.start()
      logger.info("Started logging battery & motor data.")

      time.sleep(10)  # Adjust duration as needed
    except KeyError as e:
      logger.error(f"LogConfig error: {e}")
    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if log_config.valid:
        try:
          log_config.stop()
          logger.info("Stopped logging battery & motor data.")
        except Exception as e:
          logger.error(f"Error stopping log config: {e}")

  def _log_callback(self, timestamp, data, logconf):
    """
    Callback for receiving log data.
    """
    log_entry = {
      "timestamp": timestamp,
      "pm.vbatMV": data.get("pm.vbatMV", "N/A"),
      "pwm.m1_pwm": data.get("pwm.m1_pwm", "N/A"),
      "pwm.m2_pwm": data.get("pwm.m2_pwm", "N/A"),
      "pwm.m3_pwm": data.get("pwm.m3_pwm", "N/A"),
      "pwm.m4_pwm": data.get("pwm.m4_pwm", "N/A"),
    }
    logger.info(f"Drone Log: {log_entry}")

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"Logging error: {msg}")
