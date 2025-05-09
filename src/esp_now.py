import network
import espnow
from command_packet import build_command_packet, build_crtp_log_request

# MAC address of the target drone
DRONE_MAC = b'\x60\x55\xf9\xda\x18\x06'  # Update with actual drone MAC

# Set up WLAN in STA mode
w0 = network.WLAN(network.STA_IF)
w0.active(True)

# Initialize ESP-NOW
e = espnow.ESPNow()
e.active(True)
e.add_peer(DRONE_MAC)

print(f"Connected to MAC: {DRONE_MAC}")

def send_command(roll: float, pitch: float, yaw: float, thrust: int):
  """Build and send a flight control command via ESP-NOW."""
  packet = build_command_packet(roll, pitch, yaw, thrust)
  send_packet(packet)

def send_packet(data: bytes):
  """Send raw bytes to the drone via ESP-NOW."""
  try:
    e.send(DRONE_MAC, data)
  except Exception as err:
    print(f"ESP-NOW send error: {err}")

def send_log_request(log_variable: str):
  """Send a log request (e.g., rxRate/txRate) to the drone."""
  packet = build_crtp_log_request(log_variable)
  send_packet(packet)

def close_connection():
  """Dummy close to match UDP interface (ESP-NOW has no socket to close)."""
  print("ESP-NOW session closed (noop).")
