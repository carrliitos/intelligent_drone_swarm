import time
import os
from pathlib import Path

from pid_controller import PIDController

from utils import logger
from utils import context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class Command:
  def __init__(self, drone_connection, thrust_start, thrust_limit, thrust_step, thrust_delay):
    self.drone_connection = drone_connection
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay
    self.current_altitude = 0.0
    self.current_thrust = 0

  def gradual_thrust_increase(self):
    """Gradually increases thrust to the specified limit."""
    thrust = self.thrust_start
    try:
      logger.info(f"Gradually increasing thrust to {self.thrust_limit}...")

      # Gradually increase thrust
      while thrust <= self.thrust_limit:
        self.drone_connection.send_command(0.0, 0.0, 0.0, thrust)
        logger.info(f"Thrust: {thrust}")
        thrust += self.thrust_step
        time.sleep(self.thrust_delay)

      # Maintain max thrust for a short time
      logger.info("Holding max thrust...")
      for _ in range(10):
        self.drone_connection.send_command(0.0, 0.0, 0.0, self.thrust_limit)
        time.sleep(self.thrust_delay)

      # Reduce thrust back to 0 gradually
      logger.info("Reducing thrust to 0...")
      while thrust >= 0:
        self.drone_connection.send_command(0.0, 0.0, 0.0, thrust)
        logger.info(f"Thrust: {thrust}")
        thrust -= int((self.thrust_step / 2))
        time.sleep(self.thrust_delay)

    except KeyboardInterrupt:
      logger.debug("Thrust control interrupted by user.")

  def hover(self, desired_altitude):
    # Use PID with output limits
    pid = PIDController(
      kp=5000.0, 
      ki=500.0, 
      kd=1000.0, 
      setpoint=desired_altitude, 
      output_limits=(0, self.thrust_limit)
    )
    dt = 0.1  # Time step (100ms)

    logger.info(f"Maintaining hover at {desired_altitude}m...")

    thrust = self.thrust_start
    try:
      logger.info("Arming motors...")
      while thrust <= 10000:
        self.drone_connection.send_command(0.0, 0.0, 0.0, thrust)
        logger.info(f"Thrust: {thrust}")
        thrust += 500
        time.sleep(dt)

      logger.info("Hover attempt.")

      for _ in range(100):
        self.current_altitude = self.get_current_altitude()
        thrust_adjustment = pid.update(self.current_altitude, dt)

        # Limit thrust based on PID output and maximum allowed thrust
        self.current_thrust = max(min(thrust_adjustment, self.thrust_limit), 0)
        self.drone_connection.send_command(0.0, 0.0, 0.0, self.current_thrust)
        logger.info(f"Altitude: {self.current_altitude:.2f}m, Thrust: {self.current_thrust}")
        time.sleep(dt)

    except KeyboardInterrupt:
      logger.debug("Hover control interrupted by user.")

  def get_current_altitude(self):
    # Simulate altitude for now (to be replaced with actual sensor data)
    self.current_altitude += (self.current_thrust / 60000.0) * 0.1  # Simulate altitude gain
    return self.current_altitude
