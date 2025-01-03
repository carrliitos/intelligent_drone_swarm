import time
import os
import sys
from threading import Thread
from pathlib import Path

from utils import logger
from utils import context

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class ESPDrone():
  def __init__(self, link_uri):
    self._cf = Crazyflie(rw_cache='./cache')
    self.link_uri = link_uri

    self._cf.connected.add_callback(self._connected)
    self._cf.disconnected.add_callback(self._disconnected)
    self._cf.connection_failed.add_callback(self._connection_failed)
    self._cf.connection_lost.add_callback(self._connection_lost)

    self._cf.open_link(link_uri)

    logger.info(f"Connecting to {link_uri}")
    logger.info(f"Crazyflie Status: {self._cf.state} ({{'0': 'DISCONNECTED', '1': INITIALIZED, '2': CONNECTED, '3': SETUP_FINISHED}})")

  def _connected(self, link_uri):
    """
    This callback is called form the Crazyflie API when a Crazyflie has been 
    connected and the TOCs have been downloaded.
    """

    # Start a separate thread to do the motor test.
    # Do not hijack the calling thread!
    Thread(target=self._ramp_motors).start()

  def _connection_failed(self, link_uri, msg):
    """
    Callback when connection initial connection fails (i.e no Crazyflie at the 
    specified address).
    """
    logger.error(f"Connection to {link_uri} failed: {msg}")

  def _connection_lost(self, link_uri, msg):
    """
    Callback when disconnected after a connection has been made (i.e Crazyflie 
    moves out of range).
    """
    logger.error(f"Connection to {link_uri} lost: {msg}")

  def _disconnected(self, link_uri):
    """Callback when the Crazyflie is disconnected (called in all cases)"""
    logger.info(f"Disconnected from {link_uri}")

  def _ramp_motors(self):
    """
    Ramp motors to test connections with the ESP-drone while logging real-time variables.
    """
    logger.info("Starting motor ramp test...")

    thrust_levels = [5000, 10000, 15000, 20000, 15000, 10000, 5000]  # Define thrust levels
    duration_per_level = 1  # Duration for each thrust level in seconds
    pitch = 0
    roll = 0
    yawrate = 0

    # Unlock startup thrust protection and send idle commands
    logger.info("Sending initial idle setpoints to stabilize logging...")
    self._cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)  # Allow the system to stabilize

    # Set up logging for desired variables
    log_config = self._setup_logging(['pm.vbatMV', 'pwm.m1_pwm', 'pwm.m2_pwm', 'pwm.m3_pwm', 'pwm.m4_pwm'])

    for thrust in thrust_levels:
      start_time = time.time()
      logger.info(f"Ramping motors at thrust={thrust}")
      while time.time() - start_time < duration_per_level:
        logger.debug(f"Sending setpoint: roll={roll}, pitch={pitch}, yawrate={yawrate}, thrust={thrust}")
        self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(0.1)

    # Stop logging after motor ramp test
    if log_config:
      log_config.stop()
      logger.info("Stopped logging.")

    logger.info("Motor ramp test completed. Killing motors.")
    self._cf.commander.send_setpoint(0, 0, 0, 0)  # Kill motors
    # Make sure that the last packet leaves before the link is closed
    time.sleep(0.1)

  def _setup_logging(self, variables):
    """
    Set up logging for the specified variables.
    """

    log_config = LogConfig(name='ramp_test', period_in_ms=50)
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
      sys.exit(1)

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

  def test_connection(self):
    logger.info("Testing connection...")
    try:
      self._connected(self.link_uri)
      self._cf.close_link()
      logger.info(f"Crazyflie Status: {self._cf.state} ({{'0': 'DISCONNECTED', '1': INITIALIZED, '2': CONNECTED, '3': SETUP_FINISHED}})")
      logger.info("Connection test completed successfully.")
    except Exception as e:
      logger.error(f"Error during connection test: {e}")
      sys.exit(1)
