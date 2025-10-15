import time
import os
import threading
import sys
import pandas as pd
import datetime
import contextlib

from datetime import datetime
from pathlib import Path
from cflib.crazyflie.log import LogConfig
from utils import logger, context
from drone_connection import DroneConnection

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = logger.setup_logger(logger_file_name, logger_file)

class DroneLogs:
  def __init__(self, drone: DroneConnection):
    """
    Handles logging real-time drone data.

    Args:
      drone (DroneConnection): The Crazyflie Drone connection instance.
    """
    self.drone = drone
    self._cf = drone._cf
    self._stop_event = threading.Event()  # Threading event to signal logging stop
    self.battery_log_config = None
    self.gyro_log_config = None
    self.control_log_config = None
    self.stateEstimate_log_config = None
    self.stateEstimate_v_log_config = None
    self.stateEstimate_pid_log_config = None
    self.stateEstimate_kalman_log_config = None
    self.pid_rate_roll_log_config = None
    self.pid_rate_pitch_log_config = None
    self.pid_rate_yaw_log_config = None
    self.motor_log_config = None

    self.lock = threading.Lock()

    self.thrust = 0.0
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    self.gyro_x = 0.0
    self.gyro_y = 0.0
    self.gyro_z = 0.0
    self.pm_vbatMV = 0
    self.pm_batteryLevel = 0
    self.motor_m1 = 0
    self.motor_m2 = 0
    self.motor_m3 = 0
    self.motor_m4 = 0
    self.pid_rate__roll_outP = 0
    self.pid_rate__roll_outI = 0
    self.pid_rate__roll_outD = 0
    self.pid_rate__pitch_outP = 0
    self.pid_rate__pitch_outI = 0
    self.pid_rate__pitch_outD = 0
    self.pid_rate__yaw_outP = 0
    self.pid_rate__yaw_outI = 0
    self.pid_rate__yaw_outD = 0

    self.stateEstimate_x = 0.0
    self.stateEstimate_y = 0.0
    self.stateEstimate_z = 0.0
    self.stateEstimate_roll = 0.0
    self.stateEstimate_pitch = 0.0
    self.stateEstimate_yaw = 0.0
    self.stateEstimate_vx = 0.0
    self.stateEstimate_vy = 0.0
    self.stateEstimate_vz = 0.0
    self.range_zrange = 0.0
    self.kalman_varPX = 0.0
    self.kalman_varPY = 0.0
    self.kalman_varPZ = 0.0

    self.sys_canfly = 0
    self.sys_isFlying = 0
    self.sys_isTumbled = 0

    self._write_logs()

  def _write_logs(self):
    """
    Continuously write drone data to a CSV file in the background.
    """
    # curr_time = datetime.datetime.now().strftime("%H-%M-%S")
    log_path = f"{directory}/data/telemetry_log.csv"

    columns = [
        "timestamp",
        "sys_canfly", "sys_isFlying", "sys_isTumbled",
        "thrust", "roll", "pitch", "yaw",
        "stateEstimate_x", "stateEstimate_y", "stateEstimate_z",
        "stateEstimate_vx", "stateEstimate_vy", "stateEstimate_vz",
        "range_zrange", # fast/clean altitude reference
        "stateEstimate_roll", "stateEstimate_pitch", "stateEstimate_yaw",
        "kalman_varPX", "kalman_varPY", "kalman_varPZ",
        "gyro_x", "gyro_y", "gyro_z",
        "pm_vbatMV", "pm_batteryLevel",
        "motor_m1", "motor_m2", "motor_m3", "motor_m4",
        "roll_outP", "roll_outI", "roll_outD",
        "pitch_outP", "pitch_outI", "pitch_outD",
        "yaw_outP", "yaw_outI", "yaw_outD"
    ]

    with open(log_path, mode='w', encoding='utf-8') as f:
      f.write(",".join(columns) + "\n")

    def _log_writer():
      while not self._stop_event.is_set():
        with self.lock:
          row = [
              datetime.fromtimestamp(time.time()),
              self.sys_canfly, self.sys_isFlying, self.sys_isTumbled,
              self.thrust, self.roll, self.pitch, self.yaw,
              self.stateEstimate_x, self.stateEstimate_y, self.stateEstimate_z,
              self.stateEstimate_vx, self.stateEstimate_vy, self.stateEstimate_vz,
              self.range_zrange,
              self.stateEstimate_roll, self.stateEstimate_pitch, self.stateEstimate_yaw,
              self.kalman_varPX, self.kalman_varPY, self.kalman_varPZ,
              self.gyro_x, self.gyro_y, self.gyro_z,
              self.pm_vbatMV, self.pm_batteryLevel,
              self.motor_m1, self.motor_m2, self.motor_m3, self.motor_m4,
              self.pid_rate__roll_outP, self.pid_rate__roll_outI, self.pid_rate__roll_outD,
              self.pid_rate__pitch_outP, self.pid_rate__pitch_outI, self.pid_rate__pitch_outD,
              self.pid_rate__yaw_outP, self.pid_rate__yaw_outI, self.pid_rate__yaw_outD
          ]

        with open(log_path, mode='a', encoding='utf-8') as f:
          f.write(",".join(map(str, row)) + "\n")

        time.sleep(0.5)

    threading.Thread(target=_log_writer, daemon=True).start()

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
    sys_logs = threading.Thread(target=self._sys_logs, daemon=True)
    battery_logs = threading.Thread(target=self._battery_logs, daemon=True)
    gyro_states_logs = threading.Thread(target=self._gyro_states, daemon=True)
    controller_logs = threading.Thread(target=self._control_states, daemon=True)
    motor_logs = threading.Thread(target=self._motor_states, daemon=True)
    pid_rate_roll_logs = threading.Thread(target=self._pid_rate_roll, daemon=True)
    pid_rate_pitch_logs = threading.Thread(target=self._pid_rate_pitch, daemon=True)
    pid_rate_yaw_logs = threading.Thread(target=self._pid_rate_yaw, daemon=True)
    state_estimates_logs = threading.Thread(target=self._stateEstimate, daemon=True)
    state_estimates_v_logs = threading.Thread(target=self._stateEstimate_v, daemon=True)
    state_estimates_pid_logs = threading.Thread(target=self._stateEstimate_pid, daemon=True)
    state_estimates_kalman_logs = threading.Thread(target=self._stateEstimate_kalman, daemon=True)

    # Actually start logging
    sys_logs.start()
    battery_logs.start()
    gyro_states_logs.start()
    controller_logs.start()
    motor_logs.start()
    pid_rate_roll_logs.start()
    pid_rate_pitch_logs.start()
    pid_rate_yaw_logs.start()
    state_estimates_logs.start()
    state_estimates_v_logs.start()
    state_estimates_pid_logs.start()
    state_estimates_kalman_logs.start()

  def stop_logging(self):
    """
    Stop logging.
    """
    self._stop_event.set()
    logger.info("Stopping drone logging...")

    if self.sys_log_config and self.sys_log_config.valid:
      self.sys_log_config.stop()
    if self.battery_log_config and self.battery_log_config.valid:
      self.battery_log_config.stop()
    if self.gyro_log_config and self.gyro_log_config.valid:
      self.gyro_log_config.stop()
    if self.control_log_config and self.control_log_config.valid:
      self.control_log_config.stop()
    if self.stateEstimate_log_config and self.stateEstimate_log_config.valid:
      self.stateEstimate_log_config.stop()
    if self.stateEstimate_v_log_config and self.stateEstimate_v_log_config.valid:
      self.stateEstimate_v_log_config.stop()
    if self.stateEstimate_pid_log_config and self.stateEstimate_pid_log_config.valid:
      self.stateEstimate_pid_log_config.stop()
    if self.stateEstimate_kalman_log_config and self.stateEstimate_kalman_log_config.valid:
      self.stateEstimate_kalman_log_config.stop()
    if self.pid_rate_roll_log_config and self.pid_rate_roll_log_config.valid:
      self.pid_rate_roll_log_config.stop()
    if self.pid_rate_pitch_log_config and self.pid_rate_pitch_log_config.valid:
      self.pid_rate_pitch_log_config.stop()
    if self.pid_rate_yaw_log_config and self.pid_rate_yaw_log_config.valid:
      self.pid_rate_yaw_log_config.stop()
    if self.motor_log_config and self.motor_log_config.valid:
      self.motor_log_config.stop()

  def _stateEstimate(self):
    """
    Log state estimates.
    """
    self.stateEstimate_log_config = LogConfig(name="stateEstimate", period_in_ms=500)

    try:
      self.stateEstimate_log_config.add_variable("stateEstimate.x", "float")
      self.stateEstimate_log_config.add_variable("stateEstimate.y", "float")
      self.stateEstimate_log_config.add_variable("stateEstimate.z", "float")

      self.stateEstimate_log_config.add_variable("range.zrange", "float")

      self._cf.log.add_config(self.stateEstimate_log_config)
      self.stateEstimate_log_config.data_received_cb.add_callback(self._log_callback__stateEstimate)
      self.stateEstimate_log_config.error_cb.add_callback(self._log_error_callback)
      self.stateEstimate_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.stateEstimate_log_config and self.stateEstimate_log_config.valid:
        self.stateEstimate_log_config.stop()
        logger.info("Stopped stateEstimate logging.")

  def _stateEstimate_v(self):
    """
    Log state estimates.
    """
    self.stateEstimate_v_log_config = LogConfig(name="stateEstimate_v", period_in_ms=500)

    try:
      self.stateEstimate_v_log_config.add_variable("stateEstimate.vx", "float")
      self.stateEstimate_v_log_config.add_variable("stateEstimate.vy", "float")
      self.stateEstimate_v_log_config.add_variable("stateEstimate.vz", "float")

      self._cf.log.add_config(self.stateEstimate_v_log_config)
      self.stateEstimate_v_log_config.data_received_cb.add_callback(self._log_callback__stateEstimate_v)
      self.stateEstimate_v_log_config.error_cb.add_callback(self._log_error_callback)
      self.stateEstimate_v_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.stateEstimate_v_log_config and self.stateEstimate_v_log_config.valid:
        self.stateEstimate_v_log_config.stop()
        logger.info("Stopped stateEstimate_v logging.")

  def _stateEstimate_pid(self):
    """
    Log state estimates.
    """
    self.stateEstimate_pid_log_config = LogConfig(name="stateEstimate_pid", period_in_ms=500)

    try:
      self.stateEstimate_pid_log_config.add_variable("stateEstimate.roll", "float")
      self.stateEstimate_pid_log_config.add_variable("stateEstimate.pitch", "float")
      self.stateEstimate_pid_log_config.add_variable("stateEstimate.yaw", "float")

      self._cf.log.add_config(self.stateEstimate_pid_log_config)
      self.stateEstimate_pid_log_config.data_received_cb.add_callback(self._log_callback__stateEstimate_pid)
      self.stateEstimate_pid_log_config.error_cb.add_callback(self._log_error_callback)
      self.stateEstimate_pid_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.stateEstimate_pid_log_config and self.stateEstimate_pid_log_config.valid:
        self.stateEstimate_pid_log_config.stop()
        logger.info("Stopped stateEstimate_pid logging.")

  def _stateEstimate_kalman(self):
    self.stateEstimate_kalman_log_config = LogConfig(name="stateEstimate_kalman", period_in_ms=500)

    try:
      self.stateEstimate_kalman_log_config.add_variable("kalman.varPX", "float")
      self.stateEstimate_kalman_log_config.add_variable("kalman.varPY", "float")
      self.stateEstimate_kalman_log_config.add_variable("kalman.varPZ", "float")

      self._cf.log.add_config(self.stateEstimate_kalman_log_config)
      self.stateEstimate_kalman_log_config.data_received_cb.add_callback(self._log_callback__stateEstimate_kalman)
      self.stateEstimate_kalman_log_config.error_cb.add_callback(self._log_error_callback)
      self.stateEstimate_kalman_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.stateEstimate_kalman_log_config and self.stateEstimate_kalman_log_config.valid:
        self.stateEstimate_kalman_log_config.stop()
        logger.info("Stopped stateEstimate_kalman logging.")    

  def _pid_rate_roll(self):
    """
    Log PID Roll Rates.
    """
    self.pid_rate_roll_log_config = LogConfig(name="pid_rate_roll", period_in_ms=500)

    try:
      self.pid_rate_roll_log_config.add_variable("pid_rate.roll_outP", "float")
      self.pid_rate_roll_log_config.add_variable("pid_rate.roll_outI", "float")
      self.pid_rate_roll_log_config.add_variable("pid_rate.roll_outD", "float")

      self._cf.log.add_config(self.pid_rate_roll_log_config)
      self.pid_rate_roll_log_config.data_received_cb.add_callback(self._log_callback__pid_rate_roll)
      self.pid_rate_roll_log_config.error_cb.add_callback(self._log_error_callback)
      self.pid_rate_roll_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.pid_rate_roll_log_config and self.pid_rate_roll_log_config.valid:
        self.pid_rate_roll_log_config.stop()
        logger.info("Stopped pid_rate_roll logging.")

  def _pid_rate_pitch(self):
    """
    Log PID pitch Rates.
    """
    self.pid_rate_pitch_log_config = LogConfig(name="pid_rate_pitch", period_in_ms=500)

    try:
      self.pid_rate_pitch_log_config.add_variable("pid_rate.pitch_outP", "float")
      self.pid_rate_pitch_log_config.add_variable("pid_rate.pitch_outI", "float")
      self.pid_rate_pitch_log_config.add_variable("pid_rate.pitch_outD", "float")

      self._cf.log.add_config(self.pid_rate_pitch_log_config)
      self.pid_rate_pitch_log_config.data_received_cb.add_callback(self._log_callback__pid_rate_pitch)
      self.pid_rate_pitch_log_config.error_cb.add_callback(self._log_error_callback)
      self.pid_rate_pitch_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.pid_rate_pitch_log_config and self.pid_rate_pitch_log_config.valid:
        self.pid_rate_pitch_log_config.stop()
        logger.info("Stopped pid_rate_pitch logging.")

  def _pid_rate_yaw(self):
    """
    Log PID yaw Rates.
    """
    self.pid_rate_yaw_log_config = LogConfig(name="pid_rate_yaw", period_in_ms=500)

    try:
      self.pid_rate_yaw_log_config.add_variable("pid_rate.yaw_outP", "float")
      self.pid_rate_yaw_log_config.add_variable("pid_rate.yaw_outI", "float")
      self.pid_rate_yaw_log_config.add_variable("pid_rate.yaw_outD", "float")

      self._cf.log.add_config(self.pid_rate_yaw_log_config)
      self.pid_rate_yaw_log_config.data_received_cb.add_callback(self._log_callback__pid_rate_yaw)
      self.pid_rate_yaw_log_config.error_cb.add_callback(self._log_error_callback)
      self.pid_rate_yaw_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.pid_rate_yaw_log_config and self.pid_rate_yaw_log_config.valid:
        self.pid_rate_yaw_log_config.stop()
        logger.info("Stopped pid_rate_yaw logging.")

  def _motor_states(self):
    """
    Log Motor states information from the Crazyflie.
    """
    self.motor_log_config = LogConfig(name="motor_states", period_in_ms=500)

    try:
      self.motor_log_config.add_variable("motor.m1", "uint32_t")
      self.motor_log_config.add_variable("motor.m2", "uint32_t")
      self.motor_log_config.add_variable("motor.m3", "uint32_t")
      self.motor_log_config.add_variable("motor.m4", "uint32_t")

      self._cf.log.add_config(self.motor_log_config)
      self.motor_log_config.data_received_cb.add_callback(self._log_callback__motor)
      self.motor_log_config.error_cb.add_callback(self._log_error_callback)
      self.motor_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.motor_log_config and self.motor_log_config.valid:
        self.motor_log_config.stop()
        logger.info("Stopped Motor logging.")

  def _control_states(self):
    """
    Log Control states information from the Crazyflie.
    """
    self.control_log_config = LogConfig(name="control_states", period_in_ms=500)

    try:
      self.control_log_config.add_variable("stateEstimateZ.rateRoll", "float")
      self.control_log_config.add_variable("stateEstimateZ.ratePitch", "float")
      self.control_log_config.add_variable("stateEstimateZ.rateYaw", "float")
      self.control_log_config.add_variable("controller.cmd_thrust", "float")

      self._cf.log.add_config(self.control_log_config)
      self.control_log_config.data_received_cb.add_callback(self._log_callback__controller)
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
      self.gyro_log_config.data_received_cb.add_callback(self._log_callback__gyro)
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

  def _sys_logs(self):
    """
    Log system information from the Crazyflie.
    """
    self.sys_log_config = LogConfig(name="system", period_in_ms=500)

    try:
      self.sys_log_config.add_variable("sys.canfly", "uint8_t")
      self.sys_log_config.add_variable("sys.isFlying", "uint8_t")
      self.sys_log_config.add_variable("sys.isTumbled", "uint8_t")

      self._cf.log.add_config(self.sys_log_config)
      self.sys_log_config.data_received_cb.add_callback(self._log_callback__system)
      self.sys_log_config.error_cb.add_callback(self._log_error_callback)
      self.sys_log_config.start()

      # Keep logging until the stop event is triggered
      while self._cf and self._cf.state != 0 and not self._stop_event.is_set():
        time.sleep(1)

    except Exception as e:
      logger.error(f"Unexpected error: {e}")
    finally:
      if self.sys_log_config and self.sys_log_config.valid:
        self.sys_log_config.stop()
        logger.info("Stopped system logging.")

  def _battery_logs(self):
    """
    Log battery information from the Crazyflie.
    """
    self.battery_log_config = LogConfig(name="drone_battery", period_in_ms=500)

    try:
      self.battery_log_config.add_variable("pm.vbatMV", "uint16_t")
      self.battery_log_config.add_variable("pm.batteryLevel", "uint8_t")

      self._cf.log.add_config(self.battery_log_config)
      self.battery_log_config.data_received_cb.add_callback(self._log_callback__power)
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

  def _log_callback__stateEstimate(self, timestamp, data, logconf):
    """ Callback for state estimates data. """
    with self.lock:
      self.stateEstimate_x = data["stateEstimate.x"]
      self.stateEstimate_y = data["stateEstimate.y"]
      self.stateEstimate_z = data["stateEstimate.z"]

      self.range_zrange = data["range.zrange"]

  def _log_callback__stateEstimate_v(self, timestamp, data, logconf):
    with self.lock:
      self.stateEstimate_vx = data["stateEstimate.vx"]
      self.stateEstimate_vy = data["stateEstimate.vy"]
      self.stateEstimate_vz = data["stateEstimate.vz"]

  def _log_callback__stateEstimate_pid(self, timestamp, data, logconf):
    with self.lock:
      self.stateEstimate_roll = data["stateEstimate.roll"]
      self.stateEstimate_pitch = data["stateEstimate.pitch"]
      self.stateEstimate_yaw = data["stateEstimate.yaw"]

  def _log_callback__stateEstimate_kalman(self, timestamp, data, logconf):
    with self.lock:
      self.kalman_varPX = data["kalman.varPX"]
      self.kalman_varPY = data["kalman.varPY"]
      self.kalman_varPZ = data["kalman.varPZ"]

  def _log_callback__pid_rate_roll(self, timestamp, data, logconf):
    """ Callback for PID rates data. """
    with self.lock:
      self.pid_rate__roll_outP = data["pid_rate.roll_outP"]
      self.pid_rate__roll_outI = data["pid_rate.roll_outI"]
      self.pid_rate__roll_outD = data["pid_rate.roll_outD"]

  def _log_callback__pid_rate_pitch(self, timestamp, data, logconf):
    """ Callback for PID rates data. """
    with self.lock:
      self.pid_rate__pitch_outP = data["pid_rate.pitch_outP"]
      self.pid_rate__pitch_outI = data["pid_rate.pitch_outI"]
      self.pid_rate__pitch_outD = data["pid_rate.pitch_outD"]

  def _log_callback__pid_rate_yaw(self, timestamp, data, logconf):
    """ Callback for PID rates data. """
    with self.lock:
      self.pid_rate__yaw_outP = data["pid_rate.yaw_outP"]
      self.pid_rate__yaw_outI = data["pid_rate.yaw_outI"]
      self.pid_rate__yaw_outD = data["pid_rate.yaw_outD"]

  def _log_callback__motor(self, timestamp, data, logconf):
    """ Callback for motor data. """
    with self.lock:
      self.motor_m1 = data["motor.m1"]
      self.motor_m2 = data["motor.m2"]
      self.motor_m3 = data["motor.m3"]
      self.motor_m4 = data["motor.m4"]

  def _log_callback__gyro(self, timestamp, data, logconf):
    """ Callback for gyro data. """
    with self.lock:
      self.gyro_x = data["gyro.x"]
      self.gyro_y = data["gyro.y"]
      self.gyro_z = data["gyro.z"]

  def _log_callback__controller(self, timestamp, data, logconf):
    """ Callback for controller data. """
    with self.lock:
      self.thrust = data["controller.cmd_thrust"]
      self.roll = data["stateEstimateZ.rateRoll"]
      self.pitch = data["stateEstimateZ.ratePitch"]
      self.yaw = data["stateEstimateZ.rateYaw"]

  def _log_callback__system(self, timestamp, data, logconf):
    """ Callback for system data. """
    with self.lock:
      self.sys_canfly = data["sys.canfly"]
      self.sys_isFlying = data["sys.isFlying"]
      self.sys_isTumbled = data["sys.isTumbled"]

  def _log_callback__power(self, timestamp, data, logconf):
    """ Callback for power management data. """
    with self.lock:
      self.pm_vbatMV = data["pm.vbatMV"]
      self.pm_batteryLevel = data["pm.batteryLevel"]

  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"Logging error: {msg}")

  def get_thrust(self):
    with self.lock:
      return self.thrust

  def get_roll(self):
    with self.lock:
      return self.roll

  def get_pitch(self):
    with self.lock:
      return self.pitch

  def get_yaw(self):
    with self.lock:
      return self.yaw

  def get_gyro_x(self):
    with self.lock:
      return self.gyro_x

  def get_gyro_y(self):
    with self.lock:
      return self.gyro_y

  def get_gyro_z(self):
    with self.lock:
      return self.gyro_z

  def get_stateEstimate_x(self):
    with self.lock:
      return self.stateEstimate_x

  def get_stateEstimate_y(self):
    with self.lock:
      return self.stateEstimate_y

  def get_stateEstimate_z(self):
    with self.lock:
      return self.stateEstimate_z

  def get_stateEstimate_vx(self):
    with self.lock:
      return self.stateEstimate_vx

  def get_stateEstimate_vy(self):
    with self.lock:
      return self.stateEstimate_vy

  def get_stateEstimate_vz(self):
    with self.lock:
      return self.stateEstimate_vz

  def get_stateEstimate_roll(self):
    with self.lock:
      return self.stateEstimate_roll

  def get_stateEstimate_pitch(self):
    with self.lock:
      return self.stateEstimate_pitch

  def get_stateEstimate_yaw(self):
    with self.lock:
      return self.stateEstimate_yaw

  def get_range_zrange(self):
    with self.lock:
      return self.range_zrange

  def get_kalman_varPX(self):
    with self.lock:
      return self.kalman_varPX

  def get_kalman_varPY(self):
    with self.lock:
      return self.kalman_varPY

  def get_kalman_varPZ(self):
    with self.lock:
      return self.kalman_varPZ
