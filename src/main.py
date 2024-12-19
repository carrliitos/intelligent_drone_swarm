import logging
import time
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.udpdriver import UdpDriver
from cflib.utils import uri_helper

logging.basicConfig(level=logging.DEBUG)

DRONE_URI = uri_helper.uri_from_env(default="udp://192.168.43.42:2390")
APP_PORT = 2399 # App's listening port

class ESPDroneApp:
  def __init__(self, uri):
    self.uri = uri
    self.driver = UdpDriver()
    self.driver.debug = True  # Enable debug mode for detailed logs
    self.connected = False

  def connect(self):
    """Connect to the ESP-Drone using UdpDriver."""
    try:
      self.driver.connect(
        self.uri,
        linkQualityCallback=None,
        linkErrorCallback=None
      )
      self.connected = True
      logging.info(f"Connected to {self.uri}")
    except Exception as e:
      logging.error(f"Failed to connect: {e}")

  def disconnect(self):
    """Disconnect from the ESP-Drone."""
    if self.connected:
      self.driver.close()
      self.connected = False
      logging.info("Disconnected from the drone.")

  def send_command(self, roll, pitch, yawrate, thrust):
    """Send motor command to the ESP-Drone."""
    # Construct CRTP header and data
    port = 3  # Commander port
    channel = 0
    header = (port << 4) | channel

    thrust_bytes = thrust.to_bytes(2, byteorder="little", signed=False)
    # Prepare packet data
    data = (
      int(roll * 100),
      int(pitch * 100),
      int(yawrate * 100),
    ) + tuple(thrust_bytes)

    # Create CRTP packet
    packet = CRTPPacket(header, data)
    logging.debug(f"Port={port}, Channel={channel}, Header={header}")

    # Send packet using UdpDriver
    try:
      self.driver.send_packet(packet)
      logging.info(f"Sent command: roll={roll}, pitch={pitch}, yawrate={yawrate}, thrust={thrust}")
    except Exception as e:
      logging.error(f"Failed to send command: {e}")

  def _ramp_motors(self):
    """Ramp motors up and down."""
    thrust = 20000
    thrust_step = 500
    thrust_mult = 1
    roll, pitch, yawrate = 0, 0, 0

    try:
      while thrust >= 20000:
        self.send_command(roll, pitch, yawrate, thrust)
        time.sleep(0.1)

        if thrust >= 25000:
          thrust_mult = -1
        thrust += thrust_step * thrust_mult

      # Stop motors
      self.send_command(0, 0, 0, 0)
      logging.info("Motor ramping complete.")
    except Exception as e:
      logging.error(f"Error during motor ramping: {e}")

  def run(self):
    """Run the motor ramp example."""
    self.connect()
    if self.connected:
      self._ramp_motors()
      self.disconnect()


if __name__ == "__main__":
  drone = ESPDroneApp(DRONE_URI)
  drone.run()
