"""
Simple example that connects to the crazyflie at `URI` and runs a figure 8
sequence. This script requires some kind of location system, it has been
tested with the flow deck and the lighthouse positioning system.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

if __name__ == '__main__':
  # Initialize the low-level drivers
  cflib.crtp.init_drivers()

  with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    reset_estimator(scf)
    time.sleep(1)

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    for y in range(10):
      cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
      time.sleep(0.1)

    for _ in range(20):
      cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
      time.sleep(0.1)

    for _ in range(50):
      cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 0.4)
      time.sleep(0.1)

    for _ in range(50):
      cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 0.4)
      time.sleep(0.1)

    for _ in range(20):
      cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
      time.sleep(0.1)

    for y in range(10):
      cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
      time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()
