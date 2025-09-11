import time
from typing import Iterable, Dict

import cflib.crtp
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

WINDOW = 10
PERIOD_MS = 500
THRESHOLD = 0.001
MAX_WAIT_SEC = 10.0  # hard cap per CF

class SwarmCommand:
  def __init__(self, uris: Iterable[str], takeoff_alt: float = 0.5):
    """
    Wraps a cflib Swarm for "manual" MotionCommander-style broadcast control
    """
    self.uris = tuple(uris)
    self.factory = CachedCfFactory(rw_cache='./cache')
    self.swarm: Swarm | None = None
    self.mcs: Dict[str, MotionCommander] = {}
    self.takeoff_alt = takeoff_alt
    self.speed_xy = 0.50
    self.speed_z  = 0.50
    self.yaw_rate = 90.0

  def _wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
      time.sleep(1.0)

  def _wait_for_position_estimator(scf):
    cf = scf.cf
    log_config = LogConfig(name='Kalman Variance', period_in_ms=PERIOD_MS)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_history = [1000.0] * WINDOW
    var_y_history = [1000.0] * WINDOW
    var_z_history = [1000.0] * WINDOW

    start = time.time()
    with SyncLogger(scf, log_config) as logger:
      for _, data, _ in logger:
        var_x_history.append(float(data['kalman.varPX'])); var_x_history.pop(0)
        var_y_history.append(float(data['kalman.varPY'])); var_y_history.pop(0)
        var_z_history.append(float(data['kalman.varPZ'])); var_z_history.pop(0)

        min_x, max_x = min(var_x_history), max(var_x_history)
        min_y, max_y = min(var_y_history), max(var_y_history)
        min_z, max_z = min(var_z_history), max(var_z_history)

        if ((max_x - min_x) < THRESHOLD and
            (max_y - min_y) < THRESHOLD and
            (max_z - min_z) < THRESHOLD):
          return True

        if time.time() - start > MAX_WAIT_SEC:
          return False

  def _reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.estimator', '2')   # EKF
    cf.param.set_value('stabilizer.controller', '1')  # PID
    cf.param.set_value('commander.enHighLevel', '1')  # HL commander
    try:
      _ = cf.param.get_value('deck.bcFlow2')
    except Exception:
      pass
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.4)
    SwarmCommand._wait_for_position_estimator(scf)

  def _arm(scf):
    scf.cf.platform.send_arming_request(True)
    time.sleep(0.5)
