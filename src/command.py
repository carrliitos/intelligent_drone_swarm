import time
from esp_drone_udp import send_command, send_log_request, receive_packet
# from esp_now import send_command
# from esp_multidrone_udp import send_command

# Configuration for thrust control
THRUST_START = 0       # Starting thrust value
THRUST_LIMIT = 30000   # Maximum thrust limit
THRUST_STEP = 1000     # Increment per step
THRUST_DELAY = 0.05     # Delay between each step in seconds

def gradual_thrust_increase():
  """Gradually increases thrust to the specified limit."""
  thrust = THRUST_START
  try:
    print(f"Gradually increasing thrust to {THRUST_LIMIT}...")

    # Gradually increase thrust
    while thrust <= THRUST_LIMIT:
      send_command(0.0, 0.0, 0.0, thrust)  # Sending only thrust commands
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
      thrust -= THRUST_STEP
      time.sleep(THRUST_DELAY)

  except KeyboardInterrupt:
    print("Thrust control interrupted by user.")

def fetch_logs():
  """Fetch and display logs for rxRate and txRate."""
  # Request rxRate
  send_log_request('rxRate')
  time.sleep(0.1)
  rx_response = receive_packet()
  if rx_response:
    # Extract rxRate (assuming 2-byte uint16_t at index 1 and 2)
    rx_rate = int.from_bytes(rx_response[1:3], 'little')
    print(f"rxRate: {rx_rate}")

  # Request txRate
  send_log_request('txRate')
  time.sleep(0.1)
  tx_response = receive_packet()
  if tx_response:
    # Extract txRate (assuming 2-byte uint16_t at index 1 and 2)
    tx_rate = int.from_bytes(tx_response[1:3], 'little')
    print(f"txRate: {tx_rate}")
