import time
import sys
import os
import threading
from threading import Thread
from pathlib import Path
from cflib.crazyflie.log import LogConfig
from utils import logger
from utils import context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_name, logger_file)

log_counter = 0

class DroneLogs:
  def __init__(self, cf):
    """
    Handles logging real-time drone data.

    Args:
      cf (Crazyflie): The Crazyflie instance for logging.
    """
    self._cf = cf
    self.log_config = None
    # self.log_variables = ["pm.vbatMV", "pwm.m1_pwm", "pwm.m2_pwm", "pwm.m3_pwm", "pwm.m4_pwm"]
    # self.log_variables = ["controller.cmd_thrust", "controller.cmd_roll", "controller.cmd_pitch", "controller.cmd_yaw"]
    # self.log_variables = ["controller.roll", "controller.pitch", "controller.yaw"]
    self.log_variables = ["pm.vbatMV", "motor.m1", "motor.m2", "motor.m3", "motor.m4"]
    # self.log_variables = ["gyro.x", "gyro.y", "gyro.z"]

  def start_logging(self):
    """
    Starts logging after ensuring the TOC is loaded.
    """
    logger.info("Waiting for TOC to be downloaded...")
    while self._cf.log.toc is None or not self._cf.log.toc.toc:
      time.sleep(0.1)

    # logger.info("TOC Contents:")
    # toc = self._cf.log.toc
    # for element_id, element in toc.toc.items():
    #   # logger.info(f"TOC Element: {element.group}.{element.name} (type: {element.ctype})")
    #   logger.debug(f"{element_id}: {element}")

    logger.info("TOC downloaded. Starting logging...")
    time.sleep(1)
    
    # Start logging in a separate thread
    log_thread = threading.Thread(target=self._log_battery_motor_data, daemon=True)
    log_thread.start()

  def _log_battery_motor_data(self):
    """
    Log power and motor data from the ESP-Drone.
    """
    log_config = LogConfig(name='battery_motor', period_in_ms=200)
    try:
      for var in self.log_variables:
        # log_config.add_variable(var, 'float')
        log_config.add_variable(var, 'uint32_t') # only for the `motor` group

      self._cf.log.add_config(log_config)
      log_config.data_received_cb.add_callback(self._log_callback)
      log_config.error_cb.add_callback(self._log_error_callback)
      log_config.start()
      logger.info("Started logging battery & motor data.")

      time.sleep(1000)
    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if log_config.valid:
        log_config.stop()
        logger.info("Stopped logging battery & motor data.")

  def _log_callback(self, timestamp, data, logconf):
    """
    Callback for receiving log data.
    """
    logger.info(data)

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"Logging error: {msg}")
