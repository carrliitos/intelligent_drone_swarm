import time
import os
import pandas as pd
import sys
import pygame
import threading
from pathlib import Path
from collections import deque

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

from utils import logger, context
from drone_connection import DroneConnection
from drone_log import DroneLogs
from swarm_command import SwarmCommand

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class Command:
  def __init__(self, 
               drone: DroneConnection, 
               drone_logger: DroneLogs, 
               swarm: SwarmCommand | None = None,
               takeoff_alt: float = 0.5):
    """
    Handles thrust commands for the drone.

    :param drone: Instance of DroneConnection.
    :param drone_logger: Instance of DroneLogs.
    :param swarm: Instance of SwarmCommand.
    """
    self.drone = drone
    self.scf = drone._scf # SyncCrazyflie
    self.drone_logger = drone_logger
    self.swarm = swarm

    self.window = 10
    self.period_ms = 500
    self.threshold = 0.001
    self.max_wait_sec = 10.0  # hard cap per CF

    self.mc = None
    self.manual_active = False
    self.swarm_active = False
    self._l_was_down = False      # for 'L' edge detection
    self._g_was_down = False      # for 'G' edge detection (enter manual)
    self._s_was_down = False      # for 'S' edge (enter swarm manual)
    self.speed_xy = 0.50          # m/s
    self.speed_z  = 0.50          # m/s
    self.yaw_rate = 90.0          # deg/s
    self.takeoff_alt = takeoff_alt # m

    self.ibvs_enable_event = threading.Event()   # pressed KP1 -> set; pressed again -> clear
    self.ibvs_enabled = False

  def _wait_for_param_download(self):
    logger.info("Waiting for parameters to be downloaded.")
    while not self.drone._cf.param.is_updated:
      time.sleep(1.0)
    logger.info(f"Parameters downloaded for {self.drone._cf.link_uri}")
    time.sleep(2.0)

  def _wait_for_position_estimator(self):
    cf = self.drone._cf
    logger.info(f"{cf.link_uri}: Waiting for estimator to find position...")

    log_config = LogConfig(name='Kalman Variance', period_in_ms=self.period_ms)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_history = [1000.0] * self.window
    var_y_history = [1000.0] * self.window
    var_z_history = [1000.0] * self.window

    start = time.time()
    with SyncLogger(self.drone._scf, log_config) as synclogger:
      for _, data, _ in synclogger:
        var_x_history.append(float(data['kalman.varPX'])); var_x_history.pop(0)
        var_y_history.append(float(data['kalman.varPY'])); var_y_history.pop(0)
        var_z_history.append(float(data['kalman.varPZ'])); var_z_history.pop(0)

        min_x, max_x = min(var_x_history), max(var_x_history)
        min_y, max_y = min(var_y_history), max(var_y_history)
        min_z, max_z = min(var_z_history), max(var_z_history)

        if ((max_x - min_x) < self.threshold and
            (max_y - min_y) < self.threshold and
            (max_z - min_z) < self.threshold):
          logger.info(f"[{cf.link_uri}] Estimator stable in {time.time()-start:.2f}s")
          return True

        if time.time() - start > self.max_wait_sec:
          logger.warning(f"{cf.link_uri}: Estimator not stable by {self.max_wait_sec}s "
                         f"(Δx={max_x-min_x:.4f}, Δy={max_y-min_y:.4f}, Δz={max_z-min_z:.4f})")
          return False

  def _reset_estimator(self):
    self._wait_for_param_download()

    cf = self.drone._cf
    t0 = time.time()

    cf.param.set_value('stabilizer.estimator', '2')  # EKF (Kalman)
    cf.param.set_value('stabilizer.controller', '1') # PID controller
    cf.param.set_value('commander.enHighLevel', '1') # High-Level commander on

    try:
      flow = cf.param.get_value('deck.bcFlow2')
      logger.info(f"{cf.link_uri}: Flow deck detected: {flow}")
    except Exception:
      pass

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.4)  # small stillness pause helps

    ok = self._wait_for_position_estimator()

    if not ok:
      logger.error("Estimators not stable. Exiting...")
      time.sleep(1.0)
      sys.exit()

  def _hover(self):
    mc = MotionCommander(self.scf)

    try:
      logger.info("Taking off now!")
      time.sleep(1.0)
      mc.take_off(0.25, velocity=1.0)
      time.sleep(5.0)
    finally:
      logger.info("Landing...")
      mc.land()

  def _go(self, keys):
    """
    Manual/teleop control, entirely self-contained.
    Call this every frame with pygame.key.get_pressed().
    Uses MotionCommander continuously (no low-level setpoints).
    """
    g_down = keys[pygame.K_g]
    if not self.manual_active:
      if g_down and not self._g_was_down:
        # First G press == enter manual mode
        if self.mc is None:
          self.mc = MotionCommander(self.scf)
        # If already flying via another mode, this will raise; so only take_off if not flying
        try:
          self.mc.take_off(self.takeoff_alt, velocity=0.4)
        except Exception as e:
          # Already flying? That's fine; continue.
          pass
        self.manual_active = True
        self.ibvs_enable_event.set()
      self._g_was_down = g_down
      # Not in manual yet, nothing else to do this frame
      return
    # Already in manual
    self._g_was_down = g_down

    l_down = keys[pygame.K_l]
    if l_down and not self._l_was_down:
      # Land once on edge
      try:
        self.mc.land()
      finally:
        self.mc = None
        self.manual_active = False
        self.ibvs_enable_event.clear()
      self._l_was_down = l_down
      return
    self._l_was_down = l_down

    moved = False

    # Planar
    if keys[pygame.K_UP]:
      self.mc.start_forward(self.speed_xy); moved = True
    elif keys[pygame.K_DOWN]:
      self.mc.start_back(self.speed_xy);  moved = True
    elif keys[pygame.K_LEFT]:
      self.mc.start_left(self.speed_xy);  moved = True
    elif keys[pygame.K_RIGHT]:
      self.mc.start_right(self.speed_xy);   moved = True

    # Yaw
    elif keys[pygame.K_a]:
      self.mc.start_turn_left(self.yaw_rate);  moved = True
    elif keys[pygame.K_d]:
      self.mc.start_turn_right(self.yaw_rate); moved = True

    # Vertical
    elif keys[pygame.K_r]:
      self.mc.start_up(self.speed_z);   moved = True
    elif keys[pygame.K_f]:
      self.mc.start_down(self.speed_z);   moved = True

    # Neutral hover if nothing is pressed
    if not moved:
      self.mc.stop()

  def _go_swarm(self, keys):
    """
    Broadcast same control scheme to the entire swarm.
    """
    if not self.swarm:
      return
    self.swarm.broadcast_go(_KeySnapshot(keys))

  def pygame(self):
    """
    Interactive PyGame controller for drone thrust and orientation.
    """
    done = False

    logger.info("In pygame function")
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Drone Flight Controls")
    font = pygame.font.SysFont("monospace", 16)

    try:
      logger.info("Resetting estimators.")
      self._reset_estimator()
      time.sleep(1.0)
      self.drone_logger.start_logging()

      while not done:
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            pygame.quit()
            exit()
          if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_BACKSPACE:
              done = True
            if event.key == pygame.K_h:
              self._hover()

        keys = pygame.key.get_pressed()
        mods = pygame.key.get_mods()

        # Single-drone MANUAL (G to enter, L to land/exit)
        g_down = keys[pygame.K_g]
        if g_down and not self._g_was_down and not self.manual_active and not self.swarm_active:
          if self.mc is None:
            self.mc = MotionCommander(self.scf)
          try:
            self.mc.take_off(self.takeoff_alt, velocity=0.4)
          except Exception:
            pass
          self.manual_active = True

        self._g_was_down = g_down

        # SWARM MANUAL (S to enter, K to land/exit)
        s_down = keys[pygame.K_s]
        if s_down and not self._s_was_down and self.swarm and not self.manual_active and not self.swarm_active:
          # Release single-drone link if it's part of the swarm set
          try:
            if hasattr(self.drone, "_cf") and hasattr(self.drone._cf, "link_uri"):
              if getattr(self.swarm, "uris", ()) and (self.drone._cf.link_uri in self.swarm.uris):
                logger.info(f"Releasing primary link {self.drone._cf.link_uri} for swarm…")
                self.drone._cf.close_link()
                time.sleep(0.2)
          except Exception as e:
            logger.warning(f"Unable to release primary link: {e}")

          # ensure swarm links are open & ready
          logger.info("Opening swarm links...")
          self.swarm.open()

          logger.info("Entering SWARM manual (takeoff all)...")
          self.swarm.enter_manual()

          self.swarm_active = True
          self.ibvs_enable_event.set()
        self._s_was_down = s_down

        # LAND/EXIT: single(L) vs swarm(K)
        if self.manual_active and keys[pygame.K_l]:
          try:
            self.mc.land()
          finally:
            self.mc = None
            self.manual_active = False
            self.ibvs_enable_event.clear()

        if self.swarm_active and keys[pygame.K_k]:
          try:
            self.swarm.land()
          finally:
            self.swarm_active = False
            self.ibvs_enable_event.clear()
            # Reconnect primary link so single-drone controls work again
            try:
              if hasattr(self.drone, "_cf") and hasattr(self.drone, "link_uri"):
                logger.info(f"Reconnecting primary link {self.drone.link_uri}…")
                self.drone._cf.open_link(self.drone.link_uri)
                # restore estimator state for single-drone path
                self._reset_estimator()
            except Exception as e:
              logger.warning(f"Failed to reconnect primary link: {e}")

        # Per-frame control
        if self.swarm_active:
          self._go_swarm(keys)
        else:
          self._go(keys)

        screen.fill((0, 0, 0))

        instructions = [
          "Drone Control",
          "=======================================",
          "H           | Autonomous Hover (one-shot, single)",
          "G           | Single-Drone Manual (take off)",
          "L           | Single-Drone Land / Exit Manual",
          "S           | SWARM Manual (take off all)",
          "K           | SWARM Land / Exit Manual",
          "Arrow Keys  | Move (Forward, Back, Left, Right)",
          "A / D       | Yaw left / right",
          "R / F       | Altitude up / down",
          "Backspace   | Exit program"
        ]

        for i, line in enumerate(instructions):
          text = font.render(line, True, (255, 255, 255))
          screen.blit(text, (20, 20 + i * 25))

        pygame.display.flip()
        time.sleep(0.01)

    finally:
      logger.info("Stopping logging and closing connection to drone.")
      self.drone_logger.stop_logging()
      self.drone._cf.commander.send_stop_setpoint()
      self.drone._cf.commander.send_notify_setpoint_stop()
      pygame.quit()

  def follow_target_ibvs(self, 
                         detector, 
                         stop_event,
                         start_event=None,
                         desired_area_px=1000, # time for 25mm tag
                         min_conf_frames=2,    # require N consecutive frame
                         loop_hz=20,
                         use_vertical=False):
    """
    IBVS is PASSIVE: it only runs while:
      - start_event is set (G/S pressed), AND
      - (self.manual_active or self.swarm_active) is True.
    No auto-takeoff, no auto-hover/land here; PyGame owns flight state.
    """
    # Takeoff
    if self.mc is None:
      self.mc = MotionCommander(self.scf)

    dt = 1.0 / loop_hz
    halfW = halfH = None

    # Minimal PD gains
    yaw_kp, yaw_kd = 100.0, 15.0 # deg/s per unit normalized pixel
    fwd_kp, fwd_kd = 0.50, 0.10  # m/s per unit distance error
    vrt_kp, vrt_kd = 0.40, 0.10  # m/s per unit vertical error

    # Clamps and headbands
    yaw_max, vx_max, vz_max = 180.0, 0.30, 0.25
    db_px, db_dist = 0.02, 0.02

    # derivative memory
    ex_prev = ed_prev = ey_prev = None

    while not stop_event.is_set():
      t0 = time.time()

      # gate: only do someting while G/S flight modes are active
      if (start_event is None or not start_event.is_set()) or not (self.manual_active or self.swarm_active):
        # stay idle; do not hover or land here
        logger.debug(f"IBVS idle: event={start_event.is_set() if start_event else None}, "
                     f"manual={self.manual_active}, swarm={self.swarm_active}")
        time.sleep(dt)
        continue

      frame, res = detector.read()
      if frame is None:
        time.sleep(dt)
        continue

      tgt = vision.primary_target(res, frame.shape)
      if not tgt:
        # no target? then do nothing (PyGame continues to command)
        logger.debug("IBVS: no target detected this frame")
        time.sleep(dt)
        continue

      cx, cy, area, W, H = tgt
      logger.debug(f"IBVS target: cx={cx:.1f}, cy={cy:.1f}, area={area:.0f}")

      if halfW is None: halfW, halfH = W / 2.0, H / 2.0

      # normalized errors
      ex = (cx - halfW) / halfW          # [-1,1], horizontal pixel error → yaw
      ey = (cy - halfH) / halfH          # vertical pixel error → vz (optional)
      ed = (desired_area_px - max(area, 1.0)) / max(desired_area_px, 1.0)

      # deadbands
      if abs(ex) < db_px: ex = 0.0
      if abs(ey) < db_px: ey = 0.0
      if abs(ed) < db_dist: ed = 0.0

      # finite diffs (derivative)
      ex_d = 0.0 if ex_prev is None else (ex - ex_prev) / dt
      ed_d = 0.0 if ed_prev is None else (ed - ed_prev) / dt
      ey_d = 0.0 if ey_prev is None else (ey - ey_prev) / dt
      ex_prev, ed_prev, ey_prev = ex, ed, ey

      # PD to commands + clamp
      yaw_rate = max(-yaw_max, min(yaw_max, (yaw_kp  *ex) + (yaw_kd * ex_d))) # deg/s
      vx = max(-vx_max, min(vx_max, (fwd_kp * ed) + (fwd_kd * ed_d)))         # m/s
      vz = 0.0 if not use_vertical else max(-vz_max, min(vz_max, (vrt_kp * ey) + (vrt_kd * ey_d)))

      # send setpoint (vy=0 for simplicity)
      self.mc.start_linear_motion(vx, 0.0, vz, rate_yaw=yaw_rate)

      # pacing
      elapsed = time.time() - t0
      if elapsed < dt:
        time.sleep(dt - elapsed)

class _KeySnapshot:
  """
  Inner class adapter for SwarmCommand so that SwarmCommand doesn't depend on pygame.
  """
  def __init__(self, keys):
    import pygame
    self.up = keys[pygame.K_UP]
    self.down = keys[pygame.K_DOWN]
    self.left = keys[pygame.K_LEFT]
    self.right = keys[pygame.K_RIGHT]
    self.a = keys[pygame.K_a]
    self.d = keys[pygame.K_d]
    self.r = keys[pygame.K_r]
    self.f = keys[pygame.K_f]

