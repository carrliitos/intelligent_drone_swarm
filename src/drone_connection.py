import os
import sys
import threading
import time
from threading import Thread
from pathlib import Path

from utils import logger, context

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

directory = context.get_context(os.path.abspath(__file__))
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_name}.log")

class DroneConnection:
  def __init__(self, link_uri):
    cflib.crtp.init_drivers()

    self.link_uri = link_uri
    self._scf = SyncCrazyflie(self.link_uri, cf=Crazyflie(rw_cache='./cache'))
    self._cf = self._scf.cf
    self.timer = None

    # Register connection callbacks
    self._cf.connected.add_callback(self._connected)
    self._cf.disconnected.add_callback(self._disconnected)
    self._cf.connection_failed.add_callback(self._connection_failed)
    self._cf.connection_lost.add_callback(self._connection_lost)

    # Open link to Crazyflie
    self._cf.open_link(link_uri)

    logger.info(f"Connecting to {self.link_uri}")

  def _connected(self, link_uri):
    """
    Callback triggered when the Crazyflie successfully connects.
    """

    logger.info(f"Connected to {link_uri}.")

  def _disconnected(self, link_uri):
    """
    Callback triggered when the Crazyflie disconnects.
    """
    logger.info(f"Disconnected from {link_uri}")

  def _connection_failed(self, link_uri, msg):
    """
    Callback triggered when the initial connection attempt fails.
    """
    logger.error(f"Connection to {link_uri} failed: {msg}")

  def _connection_lost(self, link_uri, msg):
    """
    Callback triggered when the connection is lost after being established.
    """
    logger.error(f"Connection to {link_uri} lost: {msg}")

  def connect(self):
    """
    Establishes a connection to the Crazyflie, ensuring the connection is active.
    """
    try:
      self._light_check()
      self._arm()
    except Exception as e:
      logger.error(f"Error during connection attempt: {e}")
      sys.exit(1)

  def _arm(self):
    self._cf.platform.send_arming_request(True)
    logger.info("Arming request complete.")
    time.sleep(1.0)

  def _light_check(self, delay=0.1):
    logger.info("Light check!")
    time.sleep(1.0)

    GREEN = 138

    for _ in range(20):
      self._cf.param.set_value('led.bitmask', GREEN)
      time.sleep(delay)
      self._cf.param.set_value('led.bitmask', 0)
      time.sleep(delay)
      self._cf.param.set_value('sound.effect', 100)

    logger.info("Light check complete.")
    time.sleep(1.0)
