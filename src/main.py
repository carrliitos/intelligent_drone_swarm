import time
from esp_drone_udp import close_connection
from command import gradual_thrust_increase

def main():
  try:
    print("Starting thrust control...")
    gradual_thrust_increase()  # Run thrust control
  except KeyboardInterrupt:
    print("Operation interrupted by user.")
  finally:
    close_connection()  # Close socket connection

if __name__ == "__main__":
  main()
