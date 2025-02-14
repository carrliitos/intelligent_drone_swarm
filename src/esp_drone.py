import time
import os
import sys
from threading import Thread
from pathlib import Path
import csv

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
    logger.info(f"1. Crazyflie Status: {self._cf.state} ({{'0': 'DISCONNECTED', '1': INITIALIZED, '2': CONNECTED, '3': SETUP_FINISHED}})")

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
    # sys.exit(1)

  def _ramp_motors(self):
    """
    Ramp motors to test connections with the ESP-drone while logging real-time variables.
    """
    logger.info("Starting motor ramp test...")

    thrust_mult = 1
    thrust_step = 50
    thrust = 20000
    pitch = 0
    roll = 0
    yawrate = 0

    # Unlock startup thrust protection and send idle commands
    logger.info("Sending initial idle setpoints to stabilize logging...")
    self._cf.commander.send_setpoint(0, 0, 0, 0)

    while thrust >= 20000:
      self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
      time.sleep(0.1)
      if thrust >= 25000:
        thrust_mult = -1
      thrust += thrust_step * thrust_mult

    logger.info("Motor ramp test completed. Killing motors.")
    self._cf.commander.send_setpoint(0, 0, 0, 0)
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

  def _setup_logging(self, variables, writer_key):
    """
    Set up logging for a group of variables.
    """
    log_config = LogConfig(name=writer_key, period_in_ms=50)
    for variable in variables:
      try:
        log_config.add_variable(variable, 'float')
      except KeyError:
        logger.warning(f"Variable {variable} not found in TOC.")

    try:
      self._cf.log.add_config(log_config)
      log_config.data_received_cb.add_callback(lambda ts, data, lc: self._log_callback(ts, data, lc, writer_key))
      log_config.error_cb.add_callback(self._log_error_callback)
      log_config.start()
      logger.info(f"Started logging variables: {variables}")
      return log_config
    except Exception as e:
      logger.error(f"Error setting up logging for {writer_key}: {e}")
      sys.exit(1)

  def _log_callback(self, timestamp, data, logconf, writer_key):
    """
    Callback for log data.
    """
    row = {"id": self.id_counter, "timestamp": timestamp, **data}
    self.writers[writer_key].writerow(row)
    logger.info(f"Logged data: {row}")

    if writer_key == "data3":  # Increment ID once for each complete set of variables
      self.id_counter += 1
    
  def _log_error_callback(self, logconf, msg):
    """
    Callback for logging errors.
    """
    logger.error(f"LogConfig {logconf} error: {msg}")

  def test_connection(self):
    logger.info("Testing connection...")
    try:
      while self._cf.state == 1:
        logger.info(f"0. Crazyflie Status: {self._cf.state} ({{'0': 'DISCONNECTED', '1': INITIALIZED, '2': CONNECTED, '3': SETUP_FINISHED}})")

        self._connected(self.link_uri)
        self._cf.close_link()
        logger.info("Connection test completed successfully.")
    except Exception as e:
      logger.error(f"Error during connection test: {e}")
      sys.exit(1)
