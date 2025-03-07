import socket
from command_packet import CommandPacket

class UDPConnection:
  def __init__(self, app_ip, app_port, drone_port):
    self.app_ip = app_ip
    self.app_port = app_port
    self.drone_port = drone_port
    self.cmd_pckt = CommandPacket()

  def open_connection(self):
    # Create and bind the UDP socket
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind(('', self.app_port))
    print(f"UDP socket created and bound to {self.app_ip}:{self.app_port}")

    return self

  def close_connection(self):
    """Close the UDP socket."""
    self.sock.close()
    print("UDP socket closed.")

  def send_command(self, roll: float, pitch: float, yaw: float, thrust: int):
    """Builds a command packet and sends it via UDP."""
    packet = self.cmd_pckt.build(roll, pitch, yaw, thrust)
    self.send_packet(packet)

  def send_packet(self, data: bytes):
    """Send a UDP packet to the ESP-Drone."""
    try:
      self.sock.sendto(data, (self.app_ip, self.drone_port))
    except Exception as e:
      print(f"Failed to send packet: {e}")

  def receive_packet(self, buffer_size=1024):
    """Receive a UDP packet from the ESP-Drone."""
    try:
      data, addr = self.sock.recvfrom(buffer_size)
      print(f"Received packet: {data.hex()} from {addr}")
      return data
    except Exception as e:
      print(f"Failed to receive packet: {e}")
      return None
