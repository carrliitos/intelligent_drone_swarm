import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')
logging.basicConfig(level=logging.ERROR)

if __name__ == '__main__':
  cflib.crtp.init_drivers()

  scf = SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache'))
  scf.open_link()

  try:
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)

    mc = MotionCommander(scf)
    try:
      mc.take_off(0.5, velocity=0.4)
      time.sleep(20.0)

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
      mc.turn_left(90, 100)
      mc.turn_left(90, 100)
      mc.turn_right(90, 100)
      mc.turn_right(90, 100)
      time.sleep(5.0)
    finally:
      mc.land()
      mc.stop()

  finally:
    scf.close_link()
