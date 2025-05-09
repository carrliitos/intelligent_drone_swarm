import ubinascii
import network
import espnow

DRONE_MAC = b'\x60\x55\xf9\xda\x18\x06'  # Update with actual drone MAC

# Set up WLAN in STA mode
w0 = network.WLAN(network.STA_IF)
w0.active(True)

# Initialize ESP-NOW
e = espnow.ESPNow()
e.active(True)
e.add_peer(DRONE_MAC)

e.send(DRONE_MAC, b"ping")
host, msg = e.recv(timeout_ms=1000)
if msg == b"pong":
  print("Drone is alive!")
