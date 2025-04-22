import time
import os
import pandas as pd
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
               thrust_delay: float,
               thrust_control: dict[str, bool]):
    """
    Handles gradual thrust commands for the drone.

    :param drone: Instance of UDPConnection.
    :param drone_logger: Instance of DroneLogs.
    :param thrust_start: Initial thrust value.
    :param thrust_limit: Maximum thrust value.
    :param thrust_step: Step increment for thrust increase.
    :param thrust_delay: Delay between thrust updates.
    :param thrust_control: A shared dictionary with boolean flags used to 
                           control thrust dynamically. When 'increase' is True, 
                           thrust increases; when 'decrease' is True, thrust 
                           decreases. These flags are set via keyboard input 
                           (e.g., spacebar and CTRL+spacebar).
    """
    self.drone = drone
    self.drone_logger = drone_logger
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay
    self.thrust_control = thrust_control

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

  def manual_hover(self):
    """
    Hover loop with manual thrust adjustment.
    Spacebar to increase thrust, Ctrl+Space to decrease.
    """
    thrust = self.thrust_start
    max_thrust = self.thrust_limit
    step = self.thrust_step
    delay = self.thrust_delay

    logger.info("Manual hover control started. Use SPACE to increase thrust, CTRL+SPACE to decrease.")

    while True:
      current_roll = self.drone_logger.get_roll()
      current_pitch = self.drone_logger.get_pitch()
      current_yaw = self.drone_logger.get_yaw()

      roll_corr = self.roll_rate_pid.update(current_roll)
      pitch_corr = self.pitch_rate_pid.update(current_pitch)
      yaw_corr = self.yaw_rate_pid.update(current_yaw)

      if self.thrust_control["space"] and not self.thrust_control["ctrl"]:
        thrust = min(thrust + step, max_thrust)
      elif self.thrust_control["space"] and self.thrust_control["ctrl"]:
        thrust = max(thrust - step, 0)

      self.drone._cf.commander.send_setpoint(
        roll_corr,
        pitch_corr,
        yaw_corr,
        int(thrust)
      )

      time.sleep(delay)

