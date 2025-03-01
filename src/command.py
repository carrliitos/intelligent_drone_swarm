import time

class Command:
  def __init__(self, cf, initial_thrust=15000):
    """
    Handles drone command operations, particularly thrust control.
    
    Args:
      cf (Crazyflie): The Crazyflie instance for communication.
      initial_thrust (int): The starting thrust value.
    """
    self._cf = cf
    self.thrust = initial_thrust  # Initialize thrust level

  def thrust_gradual(self, thrust_limit, roll, pitch, yawrate, step=100, delay=0.05):
    """
    Gradually increases thrust to the specified thrust limit.

    Args:
      thrust_limit (int): Maximum thrust limit.
      roll (float): Roll value.
      pitch (float): Pitch value.
      yawrate (float): Yaw rate.
      step (int): Incremental step for increasing thrust.
      delay (float): Time delay between thrust updates.
    """
    while self.thrust < thrust_limit:
      self.thrust = min(self.thrust + step, thrust_limit)  # Increase step-by-step
      self._cf.commander.send_setpoint(roll, pitch, yawrate, int(self.thrust))
      time.sleep(delay)  # Wait before increasing again

    # Once at max thrust, maintain it
    self._cf.commander.send_setpoint(roll, pitch, yawrate, int(self.thrust))

  def immediate_thrust(self, thrust, roll=0, pitch=0, yawrate=0):
    """
    Immediately sets thrust to a specific value.

    Args:
      thrust (int): Desired thrust value.
      roll (float): Roll value.
      pitch (float): Pitch value.
      yawrate (float): Yaw rate.
    """
    self._cf.commander.send_setpoint(roll, pitch, yawrate, int(thrust))
