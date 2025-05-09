import socket
import threading
import time
from command_packet import CommandPacket

class UDPConnection:
  def __init__(self, app_ip, app_port, drone_port):
    self.app_ip = app_ip
    self.app_port = app_port
    self.drone_port = drone_port
    self.command_packet = CommandPacket()
    self.running = False
    self.sock = None
    self.idle_thread = None

  def _connect(self):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind(('', self.app_port))

    self.running = True
    self.idle_thread = threading.Thread(target=self._idle_loop)
    self.idle_thread.start()

  def _idle_loop(self):
    """Continuously sends a zero-setpoint command to keep the drone active."""
    data = self.command_packet.build_zero_point_command()
    self._send_data(data)

  def _send_data(self, data):
    while self.running:
      self.sock.sendto(data, (self.app_ip, self.drone_port))
      time.sleep(0.05)  # Send command every 50 ms

  def connect(self):
    self._connect()
    return self
    print("We're connected.")

  def shutdown(self):
    """Stops the idle loop and cleans up the socket."""
    self.running = False
    if self.idle_thread:
      self.idle_thread.join()
    if self.sock:
      self.sock.close()
