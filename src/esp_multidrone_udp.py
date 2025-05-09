import socket
from command_packet import build_command_packet, build_crtp_log_request

# Drone IPs
DRONES = {
  "spud1": "192.168.43.41",
  "spud5": "192.168.43.51",
}
DRONE_PORT = 2390
APP_PORT = 2399  # Port on the laptop to receive

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', APP_PORT))
print(f"UDP socket created and bound to port {APP_PORT}")

def send_command(drone_id: str, roll: float, pitch: float, yaw: float, thrust: int):
  """Send a flight command to a specific drone."""
  if drone_id not in DRONES:
    print(f"Unknown drone ID: {drone_id}")
    return
  packet = build_command_packet(roll, pitch, yaw, thrust)
  send_packet(DRONES[drone_id], packet)

def broadcast_command(roll: float, pitch: float, yaw: float, thrust: int):
  """Send the same command to all drones."""
  packet = build_command_packet(roll, pitch, yaw, thrust)
  for ip in DRONES.values():
    send_packet(ip, packet)

def send_packet(drone_ip: str, data: bytes):
  try:
    sock.sendto(data, (drone_ip, DRONE_PORT))
  except Exception as e:
    print(f"Failed to send packet to {drone_ip}: {e}")

def send_log_request(drone_id: str, log_variable: str):
  """Send a log request to a specific drone."""
  if drone_id not in DRONES:
    print(f"Unknown drone ID: {drone_id}")
    return
  packet = build_crtp_log_request(log_variable)
  send_packet(DRONES[drone_id], packet)

def receive_packet(buffer_size=1024):
  try:
    data, addr = sock.recvfrom(buffer_size)
    print(f"Received packet: {data.hex()} from {addr}")
    return data
  except Exception as e:
    print(f"Failed to receive packet: {e}")
    return None

def close_connection():
  sock.close()
  print("UDP socket closed.")
