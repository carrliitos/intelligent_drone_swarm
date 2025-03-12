import time
import os
from pathlib import Path
import threading
from threading import Thread

from esp_drone_udp import UDPConnection
from command import Command
from drone_log import DroneLogger  # Import logging class
from toc_handler import TOCHandler    # Import TOC handling for telemetry variables
from utils import logger
from utils import context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
main_logger = logger.setup_logger(
  logger_name, 
  f"{directory}/logs/{logger_file_name}.log"
)

# Define connection parameters
APP_IP = "192.168.43.42"  # App IP address
APP_PORT = 2399           # App port for sending/receiving
DRONE_PORT = 2390         # ESP-Drone's listening port

def telemetry_listener(drone_logger):
  """Continuously fetch telemetry data."""
  while True:
    drone_logger.process_log_packet(drone_logger.udp_connection.receive_packet())

def main():
  # Configuration for thrust control
  THRUST_START = 0   # Starting thrust value
  THRUST_LIMIT = 60000   # Maximum thrust limit
  THRUST_STEP = 2000   # Increment per step
  THRUST_DELAY = 0.1   # Delay between each step in seconds

  connection = None

  try:
    connection = UDPConnection(APP_IP, APP_PORT, DRONE_PORT).open_connection()
    toc_handler = TOCHandler(connection)
    toc_handler.ready.wait(timeout=5)
    drone_logger = DroneLogger(connection, toc_handler)

    drone_logger.add_variable("pm.vbatMV", period_ms=100)
    drone_logger.add_variable("pwm.m1_pwm", period_ms=100)
    drone_logger.add_variable("pwm.m2_pwm", period_ms=100)
    drone_logger.add_variable("pwm.m3_pwm", period_ms=100)
    drone_logger.add_variable("pwm.m4_pwm", period_ms=100)

    drone_logger.start_logging()

    telemetry_thread = threading.Thread(target=telemetry_listener, args=(drone_logger,))
    telemetry_thread.daemon = True
    telemetry_thread.start()

    drone_command = Command(
      connection, 
      THRUST_START, 
      THRUST_LIMIT, 
      THRUST_STEP, 
      THRUST_DELAY
    )

    main_logger.info("Starting thrust control...")
    drone_command_thread = threading.Thread(target=drone_command.gradual_thrust_increase)
    drone_command_thread.start()
    drone_command_thread.join()

    drone_logger.stop_logging()
  except KeyboardInterrupt:
    main_logger.debug("Operation interrupted by user.")
  except Exception as e:
    main_logger.error(f"An error occurred: {e}")
  except TimeoutError as t:
    main_logger.error(f"TOC download failed. The drone may not be responding: {t}")
  finally:
    if connection:
      connection.close_connection()  # Close socket connection if open

if __name__ == "__main__":
  main()
