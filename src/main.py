import time
from esp_drone_udp import close_connection, receive_packet, send_log_request
from command import gradual_thrust_increase

def fetch_and_print_logs():
  """Fetches and prints log data for rxRate and txRate."""
  # Request rxRate log
  send_log_request('rxRate')
  time.sleep(0.1)  # Allow time for the drone to respond
  rx_response = receive_packet()
  if rx_response:
    # Extract and print rxRate (assuming 2-byte uint16_t at index 1 and 2)
    rx_rate = int.from_bytes(rx_response[1:3], 'little')
    print(f"rxRate: {rx_rate}")

  # Request txRate log
  send_log_request('txRate')
  time.sleep(0.1)  # Allow time for the drone to respond
  tx_response = receive_packet()
  if tx_response:
    # Extract and print txRate (assuming 2-byte uint16_t at index 1 and 2)
    tx_rate = int.from_bytes(tx_response[1:3], 'little')
    print(f"txRate: {tx_rate}")

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
