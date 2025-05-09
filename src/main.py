import time
import network

from command import gradual_thrust_increase

def fetch_and_print_logs():
  """Fetches and prints log data for rxRate and txRate."""
  # Request rxRate log
  send_log_request('rxRate')
  time.sleep(0.1)
  rx_response = receive_packet()
  if rx_response:
    # Extract and print rxRate
    rx_rate = int.from_bytes(rx_response[1:3], 'little')
    print(f"rxRate: {rx_rate}")

  # Request txRate log
  send_log_request('txRate')
  time.sleep(0.1)
  tx_response = receive_packet()
  if tx_response:
    # Extract and print txRate
    tx_rate = int.from_bytes(tx_response[1:3], 'little')
    print(f"txRate: {tx_rate}")

def main():
  try:
    print("Starting thrust control...")
    time.sleep(3)
    gradual_thrust_increase()  # Run thrust control
  except KeyboardInterrupt:
    print("Operation interrupted by user.")

if __name__ == "__main__":
  main()
