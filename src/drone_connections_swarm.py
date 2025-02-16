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
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

udp_list = {"udp://192.168.43.42:2390"}

class ESPDrone:
  def __init__(self):
    self.factory = CachedCfFactory(rw_cache='./cache')
    self.link_uris = udp_list
    self.timer = None

  def _wait_for_param_download(self):
    while not self.cf.param.is_updated:
      time.sleep(1.0)
    logger.info('Parameters downloaded for', self.cf.link_uri)

  def _connect(self):
    Thread(target=self._idle).start()

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

  def start(self):
    cflib.crtp.init_drivers()
    try:
      with Swarm(self.link_uris, factory=self.factory) as swarm:
        swarm.parallel(self._wait_for_param_download)
        swarm.parallel(self._connect)
    except Exception as e:
      logger.error(f"Error during connection test: {e}")
      sys.exit(1)
