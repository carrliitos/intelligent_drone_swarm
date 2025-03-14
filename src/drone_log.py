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
    self.gyro_log_config = None
    self.control_log_config = None

    self.lock = threading.Lock()

    self.cmd_thrust = 0.0
    self.cmd_roll = 0.0
    self.cmd_pitch = 0.0
    self.cmd_yaw = 0.0
    self.gyro_x = 0.0
    self.gyro_y = 0.0
    self.gyro_z = 0.0

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
    gyro_states_logs = threading.Thread(target=self._gyro_states, daemon=True)
    controller_logs = threading.Thread(target=self._control_states, daemon=True)

    battery_logs.start()
    gyro_states_logs.start()
    controller_logs.start()

  def stop_logging(self):
    """
    Stop logging.
    """
    self._stop_event.set()
    logger.info("Stopping drone logging...")

    if self.battery_log_config and self.battery_log_config.valid:
      self.battery_log_config.stop()
    if self.gyro_log_config and self.gyro_log_config.valid:
      self.gyro_log_config.stop()
    if self.control_log_config and self.control_log_config.valid:
      self.control_log_config.stop()

  def _control_states(self):
    """
    Log Control states information from the Crazyflie.
    """
    self.control_log_config = LogConfig(name="control_states", period_in_ms=500)

    try:
      self.control_log_config.add_variable("controller.cmd_roll", "float")
      self.control_log_config.add_variable("controller.cmd_pitch", "float")
      self.control_log_config.add_variable("controller.cmd_yaw", "float")
      self.control_log_config.add_variable("controller.cmd_thrust", "float")

      self._cf.log.add_config(self.control_log_config)
      self.control_log_config.data_received_cb.add_callback(self._log_callback)
      self.control_log_config.error_cb.add_callback(self._log_error_callback)
      self.control_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.control_log_config and self.control_log_config.valid:
        self.control_log_config.stop()
        logger.info("Stopped Controller logging.")

  def _gyro_states(self):
    """
    Log Gyro states information from the Crazyflie.
    """
    self.gyro_log_config = LogConfig(name="gyro_states", period_in_ms=500)

    try:
      self.gyro_log_config.add_variable("gyro.x", "float")
      self.gyro_log_config.add_variable("gyro.y", "float")
      self.gyro_log_config.add_variable("gyro.z", "float")

      self._cf.log.add_config(self.gyro_log_config)
      self.gyro_log_config.data_received_cb.add_callback(self._log_callback)
      self.gyro_log_config.error_cb.add_callback(self._log_error_callback)
      self.gyro_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.gyro_log_config and self.gyro_log_config.valid:
        self.gyro_log_config.stop()
        logger.info("Stopped Gyro logging.")

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
    with self.lock:
      if "controller.cmd_thrust" in data:
        self.cmd_thrust = data["controller.cmd_thrust"]
      if "controller.cmd_roll" in data:
        self.cmd_thrust = data["controller.cmd_roll"]
      if "controller.cmd_pitch" in data:
        self.cmd_thrust = data["controller.cmd_pitch"]
      if "controller.cmd_yaw" in data:
        self.cmd_thrust = data["controller.cmd_yaw"]
      if "gyro.x" in data:
        self.gyro_x = data["gyro.x"]
      if "gyro.y" in data:
        self.gyro_y = data["gyro.y"]
      if "gyro.z" in data:
        self.gyro_z = data["gyro.z"]

    logger.info(f"Timestamp: {timestamp}, Data: {data}")

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"Logging error: {msg}")

  def get_thrust(self):
    with self.lock:
      return self.cmd_thrust

  def get_roll(self):
    with self.lock:
      return self.cmd_roll

  def get_pitch(self):
    with self.lock:
      return self.cmd_pitch

  def get_yaw(self):
    with self.lock:
      return self.cmd_yaw

  def get_gyro_x(self):
    with self.lock:
      return self.gyro_x

  def get_gyro_y(self):
    with self.lock:
      return self.gyro_y

  def get_gyro_z(self):
    with self.lock:
      return self.gyro_z
