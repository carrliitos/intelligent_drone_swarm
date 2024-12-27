import time
import os
from threading import Thread
from pathlib import Path

from utils import logger
from utils import context

import cflib
from cflib.crazyflie import Crazyflie

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
    Ramp motors to test connections with the ESP-drone.
    """
    logger.info("Starting motor ramp test...")

    thrust_mult = 1
    thrust_step = 500
    thrust = 20000
    pitch = 0
    roll = 0
    yawrate = 0

    # Unlock startup thrust protection
    self._cf.commander.send_setpoint(0, 0, 0, 0)

    while thrust >= 20000:
      logger.debug(f"Sending setpoint: roll={roll}, pitch={pitch}, yawrate={yawrate}, thrust={thrust}")
      self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
      time.sleep(0.1)
      if thrust >= 25000:
        thrust_mult = -1
      thrust += thrust_step * thrust_mult

    logger.info("Motor ramp test completed. Stopping motors.")
    self._cf.commander.send_setpoint(0, 0, 0, 0)
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    self._cf.close_link()

  def test_connection(self):
    logger.info("Testing connection...")
    try:
      self._connected(self.link_uri)
      logger.info("Connection test completed successfully.")
    except Exception as e:
      logger.error(f"Error during connection test: {e}")
