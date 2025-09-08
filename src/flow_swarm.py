import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander

WINDOW = 10
PERIOD_MS = 500
THRESHOLD = 0.001
MAX_WAIT_SEC = 10.0  # hard cap per CF

uris = {
  'radio://0/80/2M/E7E7E7E7E7',
  # 'radio://0/80/2M/E7E7E7E7E8',
  # 'radio://0/80/2M/E7E7E7E7E9',
}

def wait_for_param_download(scf):
  while not scf.cf.param.is_updated:
    time.sleep(1.0)
  print('Parameters downloaded for', scf.cf.link_uri)

def wait_for_position_estimator(scf):
  cf = scf.cf
  print(f"[{cf.link_uri}] Waiting for estimator to find position...")

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
        print(f"[{cf.link_uri}] Estimator stable in {time.time()-start:.2f}s")
        return True

      if time.time() - start > MAX_WAIT_SEC:
        print(f"[WARN][{cf.link_uri}] Estimator not stable by {MAX_WAIT_SEC}s "
              f"(Δx={max_x-min_x:.4f}, Δy={max_y-min_y:.4f}, Δz={max_z-min_z:.4f})")
        return False

def reset_estimator(scf):
  cf = scf.cf
  t0 = time.time()
  cf.param.set_value('stabilizer.estimator', '2')
  try:
    flow = cf.param.get_value('deck.bcFlow2')
    print(f"{scf.cf.link_uri}: Flow deck detected: {flow}")
  except Exception:
    pass

  cf.param.set_value('kalman.resetEstimation', '1')
  time.sleep(0.1)
  cf.param.set_value('kalman.resetEstimation', '0')
  time.sleep(0.4)  # small stillness pause helps

  ok = wait_for_position_estimator(scf)

def arm(scf):
  scf.cf.platform.send_arming_request(True)
  time.sleep(1.0)

def _thrust_test(scf, start=20000, peak=30000, step=2000, hold_s=0.5, rate_hz=100):
  """
  Safely ramps motor thrust and holds at each step.
  Use with props OFF or the drone firmly restrained.
  """
  cf = scf.cf
  dt = 1.0 / rate_hz

  # Send a couple of zero setpoints to (a) wake the commander and (b) clear any latch
  for _ in range(int(0.2 / dt)):
    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(dt)

  print(f"[{cf.link_uri}] Thrust test: {start}→{peak} (step {step})")

  # Ramp up in steps
  thrust = start
  while thrust <= peak:
    t_end = time.time() + hold_s
    print(f"  Holding thrust={thrust}")
    while time.time() < t_end:
      cf.commander.send_setpoint(0, 0, 0, thrust)
      time.sleep(dt)
    thrust += step

  # Back to zero and send a stop setpoint to cleanly disarm motors
  for _ in range(int(0.3 / dt)):
    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(dt)
  cf.commander.send_stop_setpoint()

  print(f"[{cf.link_uri}] Thrust test complete.")

def light_check(scf, delay=0.1):
  print("Light check!")
  time.sleep(1)

  GREEN = 138

  for _ in range(10):
    scf.cf.param.set_value('led.bitmask', GREEN)
    time.sleep(delay)
    scf.cf.param.set_value('led.bitmask', 0)
    time.sleep(delay)
    scf.cf.param.set_value('sound.effect', 100)

def go(scf):
  mc = MotionCommander(scf)

  try:
    mc.take_off(0.5, velocity=0.4)
    time.sleep(2.0)

    mc.up(0.5, 1.0)
    time.sleep(2.0)
    
    mc.forward(0.5, 1.0)
    time.sleep(2.0)
    mc.back(0.5, 1.0)
    time.sleep(2.0)
    
    mc.down(0.5, 1.0)
    time.sleep(2.0)

    mc.turn_left(90, 100)
    mc.turn_right(90, 100)
    time.sleep(2.0)
  finally:
    mc.land()
    mc.stop()

if __name__ == '__main__':
  cflib.crtp.init_drivers()

  factory = CachedCfFactory(rw_cache='./cache')
  with Swarm(uris, factory=factory) as swarm:
    try:
      swarm.parallel_safe(reset_estimator)

      print('Waiting for parameters to be downloaded...')
      swarm.parallel_safe(wait_for_param_download)
      time.sleep(1)

      swarm.parallel_safe(arm)
      time.sleep(1)

      swarm.parallel_safe(light_check)
      time.sleep(1)

      # swarm.parallel_safe(go)
    finally:
      swarm.close_links()
