import time
import os
import pandas as pd
import pygame
from pathlib import Path

from utils import logger, context
from drone_connection import DroneConnection
from drone_log import DroneLogs
from cflib.positioning.motion_commander import MotionCommander

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class Command:
  def __init__(self, 
               drone: DroneConnection, 
               conn_str: str,
               drone_logger: DroneLogs,
               thrust_start: int, 
               thrust_limit: int, 
               thrust_step: int, 
               thrust_delay: float):
    """
    Handles gradual thrust commands for the drone.

    :param drone: Instance of DroneConnection.
    :param conn_str: Drone connection string.
    :param drone_logger: Instance of DroneLogs.
    :param thrust_start: Initial thrust value.
    :param thrust_limit: Maximum thrust value.
    :param thrust_step: Step increment for thrust increase.
    :param thrust_delay: Delay between thrust updates.
    """
    self.drone = drone
    self.conn_str = conn_str
    self.drone_logger = drone_logger
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0

  def _flow_ready(self):
    """
    Ensure the Kalman estimator + Mellinger controller are set, Flowdeck is 
    present, and reset the estimator so z/vel start sane before any motion.
    """
    cf = self.drone._cf
    cf.param.set_value("stabilizer.estimator", "2") # 2 = Kalman
    cf.param.set_value("stabilizer.controller", "2") # 2 = Mellinger

    try:
      if int(cf.param.get_value('deck.bcFlow2')) != 1:
        logger.warning("Flowdeck not reported as attached (deck.bcFlow2 != 1). Hover may drift.")
    except Exception:
      logger.info("deck.bcFlow2 not readable (older FW?) — continuing without check.")

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.5)

  def _hover_mc(self, height: float=0.6, hold_s: float=3.0):
    """
    Take off to `height` (meters), hover for `hold_s` seconds, then land.
    Flow works best ~0.3–1.0 m above a matte, textured floor.
    """
    self._flow_ready()
    cf = self.drone._cf
    logger.info(f"Taking off to {height:.2f} m and hovering for {hold_s:.1f}s...")
    try:
      with MotionCommander(cf, default_height=height) as mc:
        t0 = time.time()
        while (time.time() - t0) < hold_s:
          # Nothing to do — MC keeps altitude; just keep the loop alive
          time.sleep(0.05)
    finally:
      logger.info("Hover complete (landed).")

  def pygame(self,
             default_height: float=0.5,
             lin_speed: float=0.20,   # m/s (x/y)
             z_speed: float=0.20,     # m/s (up/down)
             yaw_step_deg: float=10.0 # discrete yaw step per keypress
            ):
    """
    Arrow keys: translational motion (Flow frame: +x fwd, +y left)
      UP forward, DOWN back, LEFT left, RIGHT right
    W/S: up/down (z)
    A/D: yaw left/right in small discrete steps
    H:   snap to hover (stop linear motion)
    Space: hover + exit
    Backspace/Close window: land/exit
    """
    self._flow_ready()
    logger.info("Starting PyGame controls.")

    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Crazyflie MotionCommander")
    font = pygame.font.SysFont("monospace", 16)

    cf = self.drone._cf
    done = False
    dt = 0.05 # ~20 Hz command loop

    try:
      with MotionCommander(cf, default_height=default_height) as mc:
        while not done:
          for event in pygame.event.get():
            if event.type == pygame.QUIT:
              done = True
            if event.type == pygame.KEYDOWN:
              if event.key == pygame.K_BACKSPACE:
                done = True
              if event.key == pygame.K_SPACE: # Emergency hover + exit
                mc.stop() # zero vx, vy, vz, and yawrate
                done = True
              if event.key == pygame.K_h: # Snap to hover
                mc.stop()
              if event.key == pygame.K_a: # discrete yaw to avoid continuous spin
                mc.turn_left(yaw_step_deg)
              if event.key == pygame.K_d:
                mc.turn_right(yaw_step_deg)
          
          keys = pygame.key.get_pressed()
          
          # linear velocities
          vx = 0.0  # + forward
          vy = 0.0  # + left
          vz = 0.0  # + up

          if keys[pygame.K_UP]:
            vx += lin_speed
          if keys[pygame.K_DOWN]:
            vx -= lin_speed
          if keys[pygame.K_LEFT]:
            vy += lin_speed   # left is +y in CF frame
          if keys[pygame.K_RIGHT]:
            vy -= lin_speed   # right is -y
          if keys[pygame.K_w]:
            vz += z_speed
          if keys[pygame.K_s]:
            vz -= z_speed

          mc.start_linear_motion(vx, vy, vz)

          screen.fill((0, 0, 0))
          info = [
            "MotionCommander Controls",
            "========================",
            "↑/↓     : Forward / Back",
            "←/→     : Left / Right",
            "W / S   : Up / Down",
            "A / D   : Yaw left/right (discrete)",
            "H       : Hover (stop)",
            "Space   : Hover + Exit",
            "Backspc : Exit",
            "",
            f"default_height: {default_height:.2f} m",
            f"vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  vz={vz:+.2f} m/s"
          ]
          for i, line in enumerate(info):
            text = font.render(line, True, (255, 255, 255))
            screen.blit(text, (20, 20 + i * 24))
          pygame.display.flip()

          time.sleep(dt)
    finally:
      # Exiting MotionCommander context auto-lands
      pygame.quit()
      logger.info("PyGame MC control ended (landed).")
