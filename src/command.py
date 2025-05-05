import time
import pygame
from esp_drone_udp import send_command, send_log_request, receive_packet

# Configuration for thrust control
THRUST_START = 0       # Starting thrust value
THRUST_LIMIT = 60000   # Maximum thrust limit
THRUST_STEP = 100      # Increment per step
THRUST_DELAY = 0.05    # Delay between each step in seconds

def gradual_thrust_increase():
  """Gradually increases thrust to the specified limit."""
  thrust = THRUST_START
  try:
    print(f"Gradually increasing thrust to {THRUST_LIMIT}...")

    # Gradually increase thrust
    while thrust <= THRUST_LIMIT:
      send_command(0.0, 0.0, 0.0, thrust)  # Sending only thrust commands
      print(f"Thrust: {thrust}")
      thrust += THRUST_STEP
      time.sleep(THRUST_DELAY)

    # Maintain max thrust for a short time
    print("Holding max thrust...")
    for _ in range(10):
      send_command(0.0, 0.0, 0.0, THRUST_LIMIT)
      time.sleep(THRUST_DELAY)

    # Reduce thrust back to 0 gradually
    print("Reducing thrust to 0...")
    while thrust >= 0:
      send_command(0.0, 0.0, 0.0, thrust)
      print(f"Thrust: {thrust}")
      thrust -= THRUST_STEP
      time.sleep(THRUST_DELAY)

  except KeyboardInterrupt:
    print("Thrust control interrupted by user.")

def pygame_test():
  """
  Interactive PyGame controller for ESP32 drone thrust and orientation.
  Keys:
    W/S     -> Increase/Decrease Thrust
    ←/→     -> Roll Left/Right
    ↑/↓     -> Pitch Up/Down
    A/D     -> Yaw Left/Right
    G       -> Run gradual thrust increase sequence
    Backspace   -> Exit control loop
  """
  done = False
  thrust = THRUST_START
  roll = 0.0
  pitch = 0.0
  yaw = 0.0

  print("In pygame function")
  pygame.init()
  screen = pygame.display.set_mode((600, 600))
  pygame.display.set_caption("ESP32-Drone Flight Controls")
  font = pygame.font.SysFont("monospace", 16)

  try:
    while not done:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          pygame.quit()
          exit()
        if event.type == pygame.KEYDOWN:
          if event.key == pygame.K_BACKSPACE:
            done = True
          elif event.key == pygame.K_g:
            print("Triggered gradual thrust sequence")
            gradual_thrust_increase()

      keys = pygame.key.get_pressed()

      if keys[pygame.K_w]:
        thrust = min(thrust + THRUST_STEP, THRUST_LIMIT)
      if keys[pygame.K_s]:
        thrust = max(THRUST_START, thrust - THRUST_STEP)
      if keys[pygame.K_LEFT]:
        roll += -0.001
      if keys[pygame.K_RIGHT]:
        roll += 0.001
      if keys[pygame.K_UP]:
        pitch += 0.05
      if keys[pygame.K_DOWN]:
        pitch += -0.05
      if keys[pygame.K_a]:
        yaw += -0.001
      if keys[pygame.K_d]:
        yaw += 0.001

      send_command(roll, pitch, yaw, thrust)

      screen.fill((0, 0, 0))
      instructions = [
        "ESP32 Drone Control",
        "=======================================",
        "W/S        | Increase/Decrease Thrust",
        "←/→        | Roll Left/Right",
        "↑/↓        | Pitch Up/Down",
        "A/D        | Yaw Left/Right",
        "G          | Gradual Thrust Increase",
        "Backspace  | Exit",
        "",
        f"Current Thrust Limit: {THRUST_LIMIT}",
        f"Current Thrust Step: {THRUST_STEP}",
        "",
        f"Roll   : {roll:.3f}",
        f"Pitch  : {pitch:.3f}",
        f"Yaw  : {yaw:.3f}",
        f"Thrust : {thrust}"
      ]
      for i, line in enumerate(instructions):
        text = font.render(line, True, (255, 255, 255))
        screen.blit(text, (20, 20 + i * 25))

      pygame.display.flip()
      time.sleep(THRUST_DELAY)

  finally:
    print("Reducing thrust to 0...")
    time.sleep(1)
    while thrust >= 0:
      send_command(0.0, 0.0, 0.0, thrust)
      thrust -= int(THRUST_STEP / 2)
      time.sleep(THRUST_DELAY)
    send_command(0.0, 0.0, 0.0, 0)
    pygame.quit()

def fetch_logs():
  """Fetch and display logs for rxRate and txRate."""
  # Request rxRate
  send_log_request('rxRate')
  time.sleep(0.1)
  rx_response = receive_packet()
  if rx_response:
    # Extract rxRate (assuming 2-byte uint16_t at index 1 and 2)
    rx_rate = int.from_bytes(rx_response[1:3], 'little')
    print(f"rxRate: {rx_rate}")

  # Request txRate
  send_log_request('txRate')
  time.sleep(0.1)
  tx_response = receive_packet()
  if tx_response:
    # Extract txRate (assuming 2-byte uint16_t at index 1 and 2)
    tx_rate = int.from_bytes(tx_response[1:3], 'little')
    print(f"txRate: {tx_rate}")
