import sys
from esp_drone_udp import UDPConnection
from command import Command

def main():
  app_ip = "192.168.43.41" # App IP address
  app_port = 2399          # App port for sending/receiving
  drone_port = 2390        # ESP-Drone's listening port

  drone = UDPConnection(app_ip, app_port, drone_port)
  try:
    drone.connect()

    command = Command(drone)
    command.flight_test()

    drone.idle_thread.join()
  except KeyboardInterrupt:
    print("Operation interrupted by user.")
  except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
  finally:
    drone.shutdown()

if __name__ == '__main__':
  main()
