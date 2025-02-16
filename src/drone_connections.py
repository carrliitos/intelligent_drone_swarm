import time
import os
import sys
import threading
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

class ESPDrone:
  def __init__(self, link_uri):
    self._cf = Crazyflie(rw_cache='./cache')
    self.link_uri = link_uri
    self.timer = None

    self._cf.connected.add_callback(self._connected)
    self._cf.disconnected.add_callback(self._disconnected)
    self._cf.connection_failed.add_callback(self._connection_failed)
    self._cf.connection_lost.add_callback(self._connection_lost)
    self._cf.open_link(link_uri)

    logger.info(f"Connecting to {self.link_uri}")
    logger.info(f"Crazyflie Status: {self._cf.state} ({{'0': 'DISCONNECTED', '1': INITIALIZED, '2': CONNECTED, '3': SETUP_FINISHED}})")

  def _connected(self, link_uri):
    """
    This callback is called form the Crazyflie API when a Crazyflie has been 
    connected and the TOCs have been downloaded.
    """

    # Start a separate thread to do the motor test.
    # Do not hijack the calling thread!
    Thread(target=self._idle).start()

  def _disconnected(self, link_uri):
    """Callback when the Crazyflie is disconnected (called in all cases)"""
    logger.info(f"Disconnected from {link_uri}")

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

  def _start_timer(self):
    self._stop_timer()
    self.timer = threading.Timer(0.05, self._idle)
    self.timer.start()

  def _stop_timer(self):
    if self.timer:
      self.timer.cancel()
      self.timer = None

  def _idle(self):
    self._cf.commander.send_setpoint(0, 0, 0, 0)
    self._start_timer()  # Restart for continuous updates

  def test_connection(self):
    logger.info("Testing connection...")
    try:
      while self._cf.state == 1:
        self._connected(self.link_uri)
    except Exception as e:
      logger.error(f"Error during connection test: {e}")
      sys.exit(1)

  def connect(self):
    try:
      while self._cf.state == 1:
        self._connected(self.link_uri)
    except Exception as e:
      logger.error(f"Error during connection test: {e}")
      sys.exit(1)
