import socket
import time
import threading
from command_packet import CommandPacket

# Configuration for thrust control
THRUST_START = 0       # Starting thrust value
THRUST_LIMIT = 30000   # Maximum thrust limit
THRUST_STEP = 1000     # Increment per step
THRUST_DELAY = 0.1     # Delay between each step in seconds

class Command:
  def __init__(self, drone):
    self.drone = drone
    self.command_packet = CommandPacket()

    self.lock = threading.Lock()

  def _send_command(self, roll, pitch, yaw, thrust):
    packet = self.command_packet.build_command_packet(roll, pitch, yaw, thrust)
    self.drone._send_data(packet)

  def flight_test(self):
    """Gradually increases thrust to the specified limit."""
    thrust = THRUST_START
    try:
      print(f"Gradually increasing thrust to {THRUST_LIMIT}...")

      # Gradually increase thrust
      while thrust <= THRUST_LIMIT:
        self._send_command(0.0, 0.0, 0.0, thrust)  # Sending only thrust commands
        print(f"Thrust: {thrust}")
        thrust += THRUST_STEP
        time.sleep(THRUST_DELAY)

      # Maintain max thrust for a short time
      print("Holding max thrust...")
      for _ in range(100):
        self._send_command(0.0, 0.0, 0.0, THRUST_LIMIT)
        time.sleep(THRUST_DELAY)

      # Reduce thrust back to 0 gradually
      print("Reducing thrust to 0...")
      while thrust >= 0:
        self._send_command(0.0, 0.0, 0.0, thrust)
        print(f"Thrust: {thrust}")
        thrust -= THRUST_STEP
        time.sleep(THRUST_DELAY)

    except KeyboardInterrupt:
      print("Thrust control interrupted by user.")