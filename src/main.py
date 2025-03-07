import time
from esp_drone_udp import UDPConnection
from command import Command

# Define connection parameters
APP_IP = "192.168.43.42"  # App IP address
APP_PORT = 2399       # App port for sending/receiving
DRONE_PORT = 2390     # ESP-Drone's listening port

def main():
  # Configuration for thrust control
  THRUST_START = 0     # Starting thrust value
  THRUST_LIMIT = 60000   # Maximum thrust limit
  THRUST_STEP = 2000   # Increment per step
  THRUST_DELAY = 0.1   # Delay between each step in seconds

  connection = None

  try:
    connection = UDPConnection(APP_IP, APP_PORT, DRONE_PORT).open_connection()

    drone_command = Command(
      connection, 
      THRUST_START, 
      THRUST_LIMIT, 
      THRUST_STEP, 
      THRUST_DELAY
    )

    print("Starting thrust control...")
    drone_command.gradual_thrust_increase()  # Run thrust control
  except KeyboardInterrupt:
    print("Operation interrupted by user.")
  except Exception as e:
    print(f"An error occurred: {e}")
  finally:
    if connection:
      connection.close_connection()  # Close socket connection if open

if __name__ == "__main__":
  main()
