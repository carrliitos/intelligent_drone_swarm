import network, socket, time
from machine import Timer
import drone

# Constructing a four-axis object, headless way
d = drone.DRONE(flightmode=0)

# Place the four axes horizontally, wait for the calibration to pass and the blue light will stay on.
while True:
  # Print calibration information
  print(d.read_cal_data())
  # Calibration passed
  if d.read_calibrated():
    print(d.read_cal_data())
    break
  time.sleep_ms(100)
