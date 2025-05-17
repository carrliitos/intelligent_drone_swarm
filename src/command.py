import time
import os
import pandas as pd
import pygame
from pathlib import Path

from utils import logger, context
from esp_drone_udp import UDPConnection
from drone_log import DroneLogs
from pid_controller import PIDController

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class Command:
  def __init__(self, 
               drone: UDPConnection, 
               drone_udp: str,
               drone_logger: DroneLogs,
               thrust_start: int, 
               thrust_limit: int, 
               thrust_step: int, 
               thrust_delay: float):
    """
    Handles gradual thrust commands for the drone.

    :param drone: Instance of UDPConnection.
    :param drone_udp: Drone UDP string.
    :param drone_logger: Instance of DroneLogs.
    :param thrust_start: Initial thrust value.
    :param thrust_limit: Maximum thrust value.
    :param thrust_step: Step increment for thrust increase.
    :param thrust_delay: Delay between thrust updates.
    """
    self.drone = drone
    self.drone_udp = drone_udp
    self.drone_logger = drone_logger
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0

  def _let_it_fly(self, current_thrust, limit):
    """Gradually increases and decreases thrust for testing stability."""
    thrust = current_thrust
    max_thrust = limit

    try:
      logger.info(f"Gradually increasing thrust to {max_thrust}...")

      # Gradually increase thrust
      while thrust <= max_thrust:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        thrust += self.thrust_step
        time.sleep(self.thrust_delay)

      # Maintain max thrust for a short time
      hold_time = 5 # seconds
      logger.info(f"Holding max thrust for {hold_time}s...")
      start_time = time.time()
      while (time.time() - start_time) < hold_time:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, max_thrust)
        time.sleep(self.thrust_delay)

      # Reduce thrust back to 0 gradually
      logger.info("Reducing thrust to 0...")
      while thrust >= 0:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        thrust -= int(self.thrust_step / 2)
        time.sleep(self.thrust_delay)

    except KeyboardInterrupt:
      logger.debug("Thrust control interrupted by user.")
    finally:
      self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)  # Ensure drone stops safely
      logger.info("Thrust set to 0 for safety.")

  def pygame(self):
    """
    Interactive PyGame controller for drone thrust and orientation.
    """
    done = False
    thrust = self.thrust_start
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    logger.info("In pygame function")
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Drone Flight Controls")
    font = pygame.font.SysFont("monospace", 16)

    try:
      while not done:
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            pygame.quit()
            exit()
          if event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
            done = True

        keys = pygame.key.get_pressed()
        mods = pygame.key.get_mods()

        if keys[pygame.K_w]:
          thrust = min(thrust + self.thrust_step, self.thrust_limit)
        if keys[pygame.K_s]:
          thrust = max(self.thrust_start, thrust - self.thrust_step)

        if keys[pygame.K_LEFT]:
          roll += -0.001
        if keys[pygame.K_RIGHT]:
          roll += 0.001

        if keys[pygame.K_UP]:
          pitch += 0.05
        if keys[pygame.K_DOWN]:
          pitch += -0.05

        if keys[pygame.K_a]:
          yaw += -0.001
        if keys[pygame.K_d]:
          yaw += 0.001

        if keys[pygame.K_SPACE]:
          self._let_it_fly(thrust, self.thrust_limit)

        self.drone._cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        screen.fill((0, 0, 0))

        instructions = [
          "Drone Control",
          "=======================================",
          "W/S         | Increase/Decrease Thrust",
          "←/→         | Roll Left/Right",
          "↑/↓         | Pitch Up/Down",
          "A/D         | Yaw Left/Right",
          "Backspace   | Exit",
          "Spacebar    | Just let it fly, man.",
          "",
          f"Connected to: {self.drone_udp}",
          f"Current Thrust Limit: {self.thrust_limit}",
          f"Current Thrust Step: {self.thrust_step}",
          "",
          f"Roll   : {roll:.3f}",
          f"Pitch  : {pitch:.3f}",
          f"Yaw    : {yaw:.3f}",
          f"Thrust : {thrust}"
        ]

        for i, line in enumerate(instructions):
          text = font.render(line, True, (255, 255, 255))
          screen.blit(text, (20, 20 + i * 25))

        pygame.display.flip()
        time.sleep(self.thrust_delay)

    finally:
      logger.info("Reducing thrust to 0...")
      time.sleep(1)
      while thrust >= 0:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        thrust -= int(self.thrust_step / 2)

        time.sleep(self.thrust_delay)
      pygame.quit()

