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
               drone_logger: DroneLogs,
               thrust_start: int, 
               thrust_limit: int, 
               thrust_step: int, 
               thrust_delay: float):
    """
    Handles gradual thrust commands for the drone.

    :param drone: Instance of UDPConnection.
    :param drone_logger: Instance of DroneLogs.
    :param thrust_start: Initial thrust value.
    :param thrust_limit: Maximum thrust value.
    :param thrust_step: Step increment for thrust increase.
    :param thrust_delay: Delay between thrust updates.
    """
    self.drone = drone
    self.drone_logger = drone_logger
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0

    # Rate-based PID controls
    pid_vals = self._load_initial_pid()
    self.roll_rate_pid = PIDController(*pid_vals["roll"], output_limits=(-30, 30))
    self.pitch_rate_pid = PIDController(*pid_vals["pitch"], output_limits=(-30, 30))
    self.yaw_rate_pid = PIDController(*pid_vals["yaw"], output_limits=(-100, 100))

  def _load_initial_pid(self):
    df = pd.read_csv(f"{directory}/src/pid.csv")
    return {
      "roll": (df.loc[df["k"] == "roll", "p"].iloc[0],
               df.loc[df["k"] == "roll", "i"].iloc[0],
               df.loc[df["k"] == "roll", "d"].iloc[0]),
      "pitch": (df.loc[df["k"] == "pitch", "p"].iloc[0],
                df.loc[df["k"] == "pitch", "i"].iloc[0],
                df.loc[df["k"] == "pitch", "d"].iloc[0]),
      "yaw": (df.loc[df["k"] == "yaw", "p"].iloc[0],
              df.loc[df["k"] == "yaw", "i"].iloc[0],
              df.loc[df["k"] == "yaw", "d"].iloc[0])
    }

  def gradual_thrust_increase(self):
    """Gradually increases and decreases thrust for testing stability."""
    thrust = self.thrust_start

    try:
      logger.info(f"Gradually increasing thrust to {self.thrust_limit}...")

      # Gradually increase thrust
      while thrust <= self.thrust_limit:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        thrust += self.thrust_step
        time.sleep(self.thrust_delay)

      # Maintain max thrust for a short time
      logger.info("Holding max thrust...")
      hold_time = 30 # seconds
      start_time = time.time()
      while (time.time() - start_time) < hold_time:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, self.thrust_limit)
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

  def hover(self):
    target_roll_rate = 0.0     # Keep roll stable (deg/sec)
    target_pitch_rate = 0.0    # Keep pitch stable (deg/sec)
    target_yaw_rate = 0.0      # Maintain yaw direction (deg/sec)
    thrust = self.thrust_limit # Fixed thrust

    hold_time = 5 # seconds
    start_time = time.time()
    while (time.time() - start_time) < hold_time:
    # while True:
      current_roll_rate = self.drone_logger.get_roll()
      current_pitch_rate = self.drone_logger.get_pitch()
      current_yaw_rate = self.drone_logger.get_yaw()

      roll_correction = self.roll_rate_pid.update(current_roll_rate)
      pitch_correction = self.pitch_rate_pid.update(current_pitch_rate)
      yaw_rate_correction = self.yaw_rate_pid.update(current_yaw_rate)

      self.drone._cf.commander.send_setpoint(
        roll_correction,     # Roll correction (degrees)
        pitch_correction,    # Pitch correction (degrees)
        yaw_rate_correction, # Yaw rate correction (degrees/sec)
        int(thrust)          # Thrust value (PWM)
      )

      time.sleep(self.thrust_delay)

  def pygame(self):
    done = False
    thrust = self.thrust_start
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    logger.info("In pygame function")
    screen = pygame.display.set_mode((277, 638))
    pygame.joystick.init()
    logger.debug(f"Joystick count: {pygame.joystick.get_count()}")

    try:
      while not done:
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            pygame.quit()
            exit()

          if event.type == pygame.KEYDOWN:
            # Thrust
            if event.key == pygame.K_EQUALS:
              thrust = min(thrust + self.thrust_step, self.thrust_limit)

            if event.key == pygame.K_MINUS:
              thrust = max(self.thrust_start, thrust - self.thrust_step)

            # Roll
            # if event.key == pygame.K_LEFT:
            #   roll_step = -0.0001
            #   roll = roll + roll_step
            #   self.roll_rate_pid.update(roll)

            # if event.key == pygame.K_RIGHT:
            #   roll_step = 0.0001
            #   roll = roll + roll_step
            #   self.roll_rate_pid.update(roll)

            # Pitch
            # Yaw 

            # Finish
            if event.key == pygame.K_BACKSPACE:
              done = True

            logger.info(f"roll={roll}, pitch={pitch}, yaw={yaw}, thrust={thrust}")

        self.drone._cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        time.sleep(self.thrust_delay)

      pygame.quit()

    finally:
      # Reduce thrust back to 0 gradually
      logger.info("Reducing thrust to 0...")
      while thrust >= 0:
        self.drone._cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        thrust -= int(self.thrust_step / 2)
        time.sleep(self.thrust_delay)
