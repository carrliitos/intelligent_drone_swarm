import socket
import threading
import time
import struct
from pathlib import Path

from command_packet import CommandPacket
from utils import logger
from utils import context

directory = context.get_context(__file__)
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_name}.log")

class UDPConnection:
  CMD_REQUEST_TOC = 0x10  # Command for requesting TOC from ESP32
  CMD_KEEP_ALIVE = 0x20   # Command for keeping the connection alive

  def __init__(self, app_ip, app_port, drone_port):
    self.app_ip = app_ip
    self.app_port = app_port
    self.drone_port = drone_port
    self.timer = None
    self.cmd_pckt = CommandPacket()
    self.toc_received = threading.Event()  # Event to signal TOC readiness

  def open_connection(self):
    """Create and bind the UDP socket."""
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind(('', self.app_port))
    logger.info(f"UDP socket created and bound to {self.app_ip}:{self.app_port}")
    
    # Start idling process to keep connection alive
    threading.Thread(target=self._idle, daemon=True).start()

    # Request TOC from drone
    self.request_toc()

    return self

  def close_connection(self):
    """Close the UDP socket."""
    self.sock.close()
    logger.info("UDP socket closed.")

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
      logger.info(f"Received {len(data)} bytes from {addr}: {data.hex()}")
      return data
    except socket.timeout:
      logger.warning("Timeout while waiting for packet.")
      return None
    except Exception as e:
      logger.error(f"Failed to receive packet: {e}")
      return None

  def request_toc(self):
    """Request the TOC (Table of Contents) from the drone."""
    logger.info("Requesting TOC from drone...")
    self.send_packet(struct.pack("<B", self.CMD_REQUEST_TOC))

    # Wait for TOC to be received before proceeding
    if not self.toc_received.wait(timeout=5):  
      logger.error("TOC download timed out!")
      raise TimeoutError("Failed to receive TOC from drone.")

  def _idle(self):
    """Sends a zero-thrust command to keep the drone connection alive."""
    packet = self.cmd_pckt.build(0.0, 0.0, 0.0, int(0.0))
    self.send_packet(packet)
    time.sleep(0.025)  # Wait 25ms before sending next idle command
    self._idle()  # Recursively call itself
