import time
from esp_drone_udp import send_command

# Configuration for thrust control
THRUST_START = 0       # Starting thrust value
THRUST_LIMIT = 60000   # Maximum thrust limit
THRUST_STEP = 2000     # Increment per step
THRUST_DELAY = 0.1     # Delay between each step in seconds

def gradual_thrust_increase():
  """Gradually increases thrust to the specified limit."""
  thrust = THRUST_START
  try:
    print(f"Gradually increasing thrust to {THRUST_LIMIT}...")

    # Gradually increase thrust
    while thrust <= THRUST_LIMIT:
      send_command(0.0, 0.0, 0.0, thrust)  # Sending only thrust commands
      print(f"Thrust: {thrust}")
      thrust += THRUST_STEP
      time.sleep(THRUST_DELAY)

    # Maintain max thrust for a short time
    print("Holding max thrust...")
    for _ in range(10):
      send_command(0.0, 0.0, 0.0, THRUST_LIMIT)
      time.sleep(THRUST_DELAY)

    # Reduce thrust back to 0 gradually
    print("Reducing thrust to 0...")
    while thrust >= 0:
      send_command(0.0, 0.0, 0.0, thrust)
      print(f"Thrust: {thrust}")
      thrust -= THRUST_STEP
      time.sleep(THRUST_DELAY)

  except KeyboardInterrupt:
    print("Thrust control interrupted by user.")
