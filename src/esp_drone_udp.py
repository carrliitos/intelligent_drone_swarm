import socket
from command_packet import build_command_packet
from command_packet import build_crtp_log_request

# Define connection parameters
APP_IP = "192.168.43.42" # App IP address
APP_PORT = 2399          # App port for sending/receiving
DRONE_PORT = 2390        # ESP-Drone's listening port

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', APP_PORT))

print(f"UDP socket created and bound to {APP_IP}:{APP_PORT}")

def send_command(roll: float, pitch: float, yaw: float, thrust: int):
  """Builds a command packet and sends it via UDP."""
  packet = build_command_packet(roll, pitch, yaw, thrust)
  send_packet(packet)

def send_packet(data: bytes):
  """Send a UDP packet to the ESP-Drone."""
  try:
    sock.sendto(data, (APP_IP, DRONE_PORT))
    print(f"Sent packet: {data.hex()}")
  except Exception as e:
    print(f"Failed to send packet: {e}")

def receive_packet(buffer_size=1024):
  """Receive a UDP packet from the ESP-Drone."""
  try:
    data, addr = sock.recvfrom(buffer_size)
    print(f"Received packet: {data.hex()} from {addr}")
    return data
  except Exception as e:
    print(f"Failed to receive packet: {e}")
    return None

def close_connection():
  """Close the UDP socket."""
  sock.close()
  print("UDP socket closed.")

def send_log_request(log_variable: str):
  """Sends a log request packet to the ESP-Drone."""
  packet = build_crtp_log_request(log_variable)
  send_packet(packet)