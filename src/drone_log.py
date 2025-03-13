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
    self._stop_event = threading.Event()  # Threading event to signal logging stop
    self.battery_log_config = None
    self.kalman_log_config = None

  def start_logging(self):
    """
    Starts logging after ensuring the TOC is loaded.
    """
    if not self._cf or self._cf.state == 0:
      logger.error("Drone is not connected. Cannot start logging. Exiting...")
      sys.exit(0)

    logger.info("Waiting for TOC to be downloaded...")
    while self._cf.log.toc is None or not self._cf.log.toc.toc:
      time.sleep(0.1)

    logger.info("TOC downloaded. Starting logging...")
    time.sleep(2)

    # Start logging in separate threads
    self._stop_event.clear()
    battery_logs = threading.Thread(target=self._battery_logs, daemon=True)
    kalman_states_logs = threading.Thread(target=self._kalman_states, daemon=True)

    battery_logs.start()
    kalman_states_logs.start()

  def stop_logging(self):
    """
    Stop logging.
    """
    self._stop_event.set()
    logger.info("Stopping drone logging...")

    if self.battery_log_config and self.battery_log_config.valid:
      self.battery_log_config.stop()
    if self.kalman_log_config and self.kalman_log_config.valid:
      self.kalman_log_config.stop()

  def _kalman_states(self):
    """
    Log Kalman states information from the Crazyflie.
    """
    self.kalman_log_config = LogConfig(name="kalman_states", period_in_ms=500)

    try:
      self.kalman_log_config.add_variable("kalman.stateX", "float")
      self.kalman_log_config.add_variable("kalman.stateY", "float")
      self.kalman_log_config.add_variable("kalman.stateZ", "float")

      self._cf.log.add_config(self.kalman_log_config)
      self.kalman_log_config.data_received_cb.add_callback(self._log_callback)
      self.kalman_log_config.error_cb.add_callback(self._log_error_callback)
      self.kalman_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.kalman_log_config and self.kalman_log_config.valid:
        self.kalman_log_config.stop()
        logger.info("Stopped Kalman logging.")

  def _battery_logs(self):
    """
    Log battery information from the Crazyflie.
    """
    self.battery_log_config = LogConfig(name="drone_battery", period_in_ms=500)

    try:
      self.battery_log_config.add_variable("pm.vbatMV", "uint16_t")
      self.battery_log_config.add_variable("pm.chargeCurrent", "float")
      self.battery_log_config.add_variable("pm.batteryLevel", "uint8_t")

      self._cf.log.add_config(self.battery_log_config)
      self.battery_log_config.data_received_cb.add_callback(self._log_callback)
      self.battery_log_config.error_cb.add_callback(self._log_error_callback)
      self.battery_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.battery_log_config and self.battery_log_config.valid:
        self.battery_log_config.stop()
        logger.info("Stopped battery logging.")

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
