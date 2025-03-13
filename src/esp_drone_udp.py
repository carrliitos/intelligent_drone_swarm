import os
import sys
import threading
import time
from threading import Thread
from pathlib import Path

from utils import logger, context

import cflib
from cflib.crazyflie import Crazyflie

directory = context.get_context(os.path.abspath(__file__))
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_name}.log")

class UDPConnection:
  def __init__(self, link_uri):
    self.link_uri = link_uri
    self._cf = Crazyflie(rw_cache='./cache')
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

    # Start the idle loop.
    Thread(target=self._idle, daemon=True).start()

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

  def _start_timer(self):
    """Starts a recurring timer to send idle commands to the drone."""
    self._stop_timer()  # Ensure no duplicate timers
    self.timer = threading.Timer(0.05, self._idle)
    self.timer.start()

  def _stop_timer(self):
    """Stops the active timer if it exists."""
    if self.timer:
      self.timer.cancel()
      self.timer = None

  def _idle(self):
    """Sends a zero-setpoint command to keep the Crazyflie active."""
    self._cf.commander.send_setpoint(0, 0, 0, 0)
    self._start_timer()  # Restart for continuous updates

  def connect(self):
    """
    Establishes a connection to the Crazyflie, ensuring the connection is active.
    """
    try:
      while self._cf.state == 1:
        self._connected(self.link_uri)
        self._thrust_test()

        logger.info(f"We are connected ({self._cf.state}). CTRL+C to disconnect.")
        return self
    except Exception as e:
      logger.error(f"Error during connection attempt: {e}")
      sys.exit(1)

  def _thrust_test(self):
    logger.info("Thrust test...")

    test_delay = 0.01
    for _ in range(100):
      self._cf.commander.send_setpoint(0, 0, 0, 15000)
      time.sleep(test_delay)
    logger.info("Thrust test complete.")
