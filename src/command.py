import time

class Command:
  def __init__(self, drone_connection, thrust_start, thrust_limit, thrust_step, thrust_delay):
    self.drone_connection = drone_connection
    self.thrust_start = thrust_start
    self.thrust_limit = thrust_limit
    self.thrust_step = thrust_step
    self.thrust_delay = thrust_delay

  def gradual_thrust_increase(self):
    """Gradually increases thrust to the specified limit."""
    thrust = self.thrust_start
    try:
      print(f"Gradually increasing thrust to {self.thrust_limit}...")

      # Gradually increase thrust
      while thrust <= self.thrust_limit:
        self.drone_connection.send_command(0.0, 0.0, 0.0, thrust)  # Sending only thrust commands
        print(f"Thrust: {thrust}")
        thrust += self.thrust_step
        time.sleep(self.thrust_delay)

      # Maintain max thrust for a short time
      print("Holding max thrust...")
      for _ in range(10):
        self.drone_connection.send_command(0.0, 0.0, 0.0, self.thrust_limit)
        time.sleep(self.thrust_delay)

      # Reduce thrust back to 0 gradually
      print("Reducing thrust to 0...")
      while thrust >= 0:
        self.drone_connection.send_command(0.0, 0.0, 0.0, thrust)
        print(f"Thrust: {thrust}")
        thrust -= int((self.thrust_step / 2))
        time.sleep(self.thrust_delay)

    except KeyboardInterrupt:
      print("Thrust control interrupted by user.")
