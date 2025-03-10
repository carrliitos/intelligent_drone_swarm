import socket
import os
from pathlib import Path

from command_packet import CommandPacket
from utils import logger
from utils import context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(
  logger_name, 
  f"{directory}/logs/{logger_file_name}.log"
)

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
    logger.info(f"UDP socket created and bound to {self.app_ip}:{self.app_port}")

    return self

  def close_connection(self):
    """Close the UDP socket."""
    self.sock.close()
    logger.info("UDP socket closed.")

  def send_command(self, roll: float, pitch: float, yaw: float, thrust: int):
    """Builds a command packet and sends it via UDP."""
    packet = self.cmd_pckt.build(roll, pitch, yaw, thrust)
    self.send_packet(packet)

  def send_packet(self, data: bytes):
    """Send a UDP packet to the ESP-Drone."""
    try:
      self.sock.sendto(data, (self.app_ip, self.drone_port))
    except Exception as e:
      logger.error(f"Failed to send packet: {e}")

  def receive_packet(self, buffer_size=1024):
    """Receive a UDP packet from the ESP-Drone."""
    try:
      data, addr = self.sock.recvfrom(buffer_size)
      logger.info(f"Received packet: {data.hex()} from {addr}")
      return data
    except Exception as e:
      logger.error(f"Failed to receive packet: {e}")
      return None
