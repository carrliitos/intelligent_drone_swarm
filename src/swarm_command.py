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
  def __init__(self, uris: Iterable[str], takeoff_alt: float = 0.25):
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

  @staticmethod
  def _wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
      time.sleep(1.0)

  @staticmethod
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

  @staticmethod
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

  @staticmethod
  def _arm(scf):
    scf.cf.platform.send_arming_request(True)
    time.sleep(0.5)

  def open(self):
    if self.swarm is not None:
      return

    cflib.crtp.init_drivers(enable_debug_driver=False)
    self.swarm = Swarm(self.uris, factory=self.factory)
    self.swarm.open_links()

    # Reset + param download + arm (in parallel)
    self.swarm.parallel_safe(self._reset_estimator)
    self.swarm.parallel_safe(self._wait_for_param_download)
    self.swarm.parallel_safe(self._arm)

    # Create MotionCommander per drone
    for uri, scf in self.swarm._cfs.items():
      self.mcs[uri] = MotionCommander(scf)

  def close(self):
    if self.swarm is None:
      return

    try:
      # Stop/land before close
      for mc in self.mcs.values():
        try: mc.stop()
        except: pass
      self.swarm.close_links()
    finally:
      self.swarm = None
      self.mcs.clear()

  def enter_manual(self):
    # Takeoff all
    for mc in self.mcs.values():
      try:
        mc.take_off(self.takeoff_alt, velocity=0.4)
      except Exception:
        # Already flying is fine
        pass

  def land(self):
    for mc in self.mcs.values():
      try: mc.land()
      except: pass

  def _apply_move(self, fn_name: str, *args):
    for mc in self.mcs.values():
      getattr(mc, fn_name)(*args)

  def _stop_all(self):
    for mc in self.mcs.values():
      try: mc.stop()
      except: pass

  def broadcast_go(self, keys):
    moved = False
    # planar
    if keys.up:
      self._apply_move('start_forward', self.speed_xy); moved = True
    elif keys.down:
      self._apply_move('start_back', self.speed_xy); moved = True
    elif keys.left:
      self._apply_move('start_left', self.speed_xy); moved = True
    elif keys.right:
      self._apply_move('start_right', self.speed_xy); moved = True
    # yaw
    elif keys.a:
      self._apply_move('start_turn_left', self.yaw_rate); moved = True
    elif keys.d:
      self._apply_move('start_turn_right', self.yaw_rate); moved = True
    # vertical
    elif keys.r:
      self._apply_move('start_up', self.speed_z); moved = True
    elif keys.f:
      self._apply_move('start_down', self.speed_z); moved = True

    if not moved:
      self._stop_all()
