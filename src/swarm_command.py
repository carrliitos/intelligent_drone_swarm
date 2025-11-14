import time
import os 
import math
import sys
from pathlib import Path
from typing import Iterable, Dict, List, Tuple
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env") 
import contextlib

import cflib.crtp
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

from utils import logger as _log, context, helpers

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = _log.setup_logger(
  logger_name=logger_name, 
  log_file=f"{directory}/logs/{logger_file_name}.log", 
  log_level=os.getenv("LOG_LEVEL")
)

WINDOW = 10
PERIOD_MS = 500
THRESHOLD = 0.001
MAX_WAIT_SEC = 10.0  # hard cap per CF

class SwarmCommand:
  def __init__(self, uris: Iterable[str], takeoff_alt: float = 0.25):
    """
    Wraps a cflib Swarm for "manual" MotionCommander-style broadcast control
    """
    self.uris = tuple(uris)
    self.factory = CachedCfFactory(rw_cache='./cache')
    self.swarm: Swarm | None = None
    self.mcs: Dict[str, MotionCommander] = {}
    self.takeoff_alt = takeoff_alt
    self.speed_xy = 0.50
    self.speed_z  = 1.00
    self.yaw_rate = 90.0

    # Formation parameters
    self.form_spacing_m = helpers._f_clamped(os.getenv("FORM_SPACING_M"), default=0.40, lo=0.25, hi=2.0)
    self.form_alt_m = helpers._f_clamped(os.getenv("FORM_ALT_M"), default=0.30, lo=0.15, hi=1.5)
    self.form_vel_mps = helpers._f_clamped(os.getenv("FORM_VEL_MPS"), default=0.20, lo=0.05, hi=0.8)
    self.form_dwell_s = helpers._f_clamped(os.getenv("FORM_DWELL_S"), default=3.0, lo=0.0, hi=10.0)
    self.form_return_to_origin = helpers._b(os.getenv("FORM_RETURN_TO_ORIGIN"), default=False)
    self.form_dry_run = helpers._b(os.getenv("FORM_DRY_RUN"), default=False)
    self.form_timeout_s = helpers._f_clamped(os.getenv("FORM_TIMEOUT_S"), default=15.0, lo=5.0, hi=60.0)

    logger.info(f"Formation config: spacing={self.form_spacing_m:.2f}m "
                f"alt={self.form_alt_m:.2f}m vel={self.form_vel_mps:.2f}m/s "
                f"dwell={self.form_dwell_s:.1f}s return={self.form_return_to_origin} "
                f"dry_run={self.form_dry_run}")

  def _ordered_uris(self) -> List[str]:
    """Return a stable sorted list of URIs for deterministic formation layout."""
    return sorted(self.uris)

  @staticmethod
  def _formation_offsets(shape: str, n: int, spacing: float) -> List[Tuple[float, float]]:
    """
    Generate (x, y) offsets for N drones in the specified formation shape.
    All formations are centered around (0, 0).

    - line: horizontal line centered at origin
    - triangle: equilateral triangle rows (1, 2, 3, ...)
    - square: grid ⌈√N⌉ × ⌈√N⌉ centered

    Returns a list of (x, y) tuples, one per drone.
    """
    offsets = []

    if shape == "line":
      # Horizontal line: x = (i - (n-1)/2) * spacing, y = 0
      for i in range(n):
        x = (i - (n - 1) / 2.0) * spacing
        y = 0.0
        offsets.append((x, y))
    elif shape == "triangle":
      # Equilateral triangle: rows 1, 2, 3, ...
      # Row spacing (vertical): spacing * sqrt(3)/2 for equilateral
      row_spacing = spacing * math.sqrt(3) / 2.0
      drone_idx = 0
      row = 0

      while drone_idx < n:
        row_count = row + 1  # row 0 has 1 drone, row 1 has 2, etc.
        for col in range(row_count):
          if drone_idx >= n:
            break
          # Center each row horizontally
          x = (col - (row_count - 1) / 2.0) * spacing
          y = -row * row_spacing  # negative to grow downward
          offsets.append((x, y))
          drone_idx += 1
        row += 1
    elif shape == "square":
      # Grid: ⌈√N⌉ × ⌈√N⌉
      side = math.ceil(math.sqrt(n))
      drone_idx = 0

      for row in range(side):
        for col in range(side):
          if drone_idx >= n:
            break
          # Center the grid around (0, 0)
          x = (col - (side - 1) / 2.0) * spacing
          y = (row - (side - 1) / 2.0) * spacing
          offsets.append((x, y))
          drone_idx += 1
    else:
      logger.warning(f"Unknown formation shape '{shape}'; defaulting to line.")
      return SwarmCommand._formation_offsets("line", n, spacing)

    return offsets

  def _execute_formation(self, shape: str):
    """
    Execute a formation maneuver for all drones in the swarm.

    Steps:
    1. Compute offsets for the given shape
    2. For each drone (in parallel):
       a. Ensure at formation altitude
       b. Move to offset position
       c. Dwell
       d. Return to origin
    """
    if self.swarm is None:
      logger.error("Swarm not open; cannot execute formation.")
      return

    ordered = self._ordered_uris()
    n = len(ordered)
    offsets = self._formation_offsets(shape, n, self.form_spacing_m)

    logger.info(f"=== Formation '{shape}' START ===")
    logger.info(f"Parameters: N={n}, spacing={self.form_spacing_m:.2f}m, "
                f"alt={self.form_alt_m:.2f}m, vel={self.form_vel_mps:.2f}m/s, "
                f"dwell={self.form_dwell_s:.1f}s, return={self.form_return_to_origin}, "
                f"dry_run={self.form_dry_run}")
    logger.info(f"Ordered URIs: {ordered}")
    logger.info(f"Computed offsets (x, y): {offsets}")

    if self.form_dry_run:
      logger.info("[DRY-RUN] No actual movement will occur.")
      for uri, (dx, dy) in zip(ordered, offsets):
        logger.info(f"[DRY-RUN] {uri} -> offset=({dx:.3f}, {dy:.3f})")
      logger.info("=== Formation DRY-RUN COMPLETE ===")
      return

    # Map URIs to offsets
    uri_to_offset = dict(zip(ordered, offsets))

    def _cf_formation_task(scf):
      """Per-drone formation task."""
      uri = scf.cf.link_uri
      mc = self.mcs.get(uri)

      if mc is None:
        logger.warning(f"{uri}: No MotionCommander available; skipping.")
        return

      dx, dy = uri_to_offset.get(uri, (0.0, 0.0))
      logger.info(f"{uri}: Starting formation task -> offset=({dx:.3f}, {dy:.3f})")

      try:
        # Move to offset position
        distance = math.hypot(dx, dy)
        if distance > 0.01:  # only move if offset is significant
          duration = distance / self.form_vel_mps
          # Cap duration to reasonable limits
          duration = helpers._clamp(duration, 0.5, self.form_timeout_s)

          # Compute velocity components
          vx = (dx / distance) * self.form_vel_mps if distance > 0 else 0.0
          vy = (dy / distance) * self.form_vel_mps if distance > 0 else 0.0

          logger.info(f"{uri}: Moving to offset ({dx:.3f}, {dy:.3f}) "
                      f"at v=({vx:.3f}, {vy:.3f}) for {duration:.2f}s")

          mc.start_linear_motion(vx, vy, 0.0)
          time.sleep(duration)
          mc.stop()

          logger.info(f"{uri}: Reached target offset.")
        else:
          logger.info(f"{uri}: Offset negligible; staying at origin.")

        # Dwell
        if self.form_dwell_s > 0:
          logger.info(f"{uri}: Dwelling for {self.form_dwell_s:.1f}s")
          time.sleep(self.form_dwell_s)

        # Return to origin
        if self.form_return_to_origin and distance > 0.01:
          logger.info(f"{uri}: Returning to origin")
          vx_ret = (-dx / distance) * self.form_vel_mps if distance > 0 else 0.0
          vy_ret = (-dy / distance) * self.form_vel_mps if distance > 0 else 0.0

          mc.start_linear_motion(vx_ret, vy_ret, 0.0)
          time.sleep(duration)  # same duration for return
          mc.stop()

          logger.info(f"{uri}: Returned to origin.")

        logger.info(f"{uri}: Formation task complete.")

      except Exception as e:
        logger.error(f"{uri}: Formation task failed: {e}", exc_info=True)
        try:
          mc.stop()
        except:
          pass

    # Execute in parallel
    try:
      self.swarm.parallel_safe(_cf_formation_task)
      logger.info("=== Formation COMPLETE ===")
    except Exception as e:
      logger.error(f"Formation execution failed: {e}", exc_info=True)

  @staticmethod
  def _wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
      time.sleep(1.0)

  @staticmethod
  def _wait_for_position_estimator(scf):
    cf = scf.cf
    logger.info(f"{cf.link_uri}: Waiting for estimator to find position...")

    log_config = LogConfig(name='Kalman Variance', period_in_ms=PERIOD_MS)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_history = [1000.0] * WINDOW
    var_y_history = [1000.0] * WINDOW
    var_z_history = [1000.0] * WINDOW

    start = time.time()
    with SyncLogger(scf, log_config) as sync_logger:
      for _, data, _ in sync_logger:
        var_x_history.append(float(data['kalman.varPX'])); var_x_history.pop(0)
        var_y_history.append(float(data['kalman.varPY'])); var_y_history.pop(0)
        var_z_history.append(float(data['kalman.varPZ'])); var_z_history.pop(0)

        min_x, max_x = min(var_x_history), max(var_x_history)
        min_y, max_y = min(var_y_history), max(var_y_history)
        min_z, max_z = min(var_z_history), max(var_z_history)

        if ((max_x - min_x) < THRESHOLD and
            (max_y - min_y) < THRESHOLD and
            (max_z - min_z) < THRESHOLD):
          logger.info(f"[{cf.link_uri}] Estimator stable in {time.time()-start:.2f}s")
          return True

        if time.time() - start > MAX_WAIT_SEC:
          logger.warning(f"{cf.link_uri}: Estimator not stable by {MAX_WAIT_SEC}s "
                         f"(Δx={max_x-min_x:.4f}, Δy={max_y-min_y:.4f}, Δz={max_z-min_z:.4f})")
          return False

  @staticmethod
  def _reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.estimator', '2')   # EKF
    cf.param.set_value('stabilizer.controller', '1')  # PID
    cf.param.set_value('commander.enHighLevel', '1')  # HL commander
    try:
      flow = cf.param.get_value('deck.bcFlow2')
      logger.info(f"{cf.link_uri}: Flow deck detected: {flow}")
    except Exception:
      pass

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.4)

    ok = SwarmCommand._wait_for_position_estimator(scf)

    if not ok:
      logger.error("Estimators not stable. Exiting...")
      time.sleep(1.0)
      sys.exit()

  @staticmethod
  def _arm(scf):
    scf.cf.platform.send_arming_request(True)
    time.sleep(0.5)

  @staticmethod
  def _light_check(scf, delay=0.1):
    cf = scf.cf
    logger.info(f"[{cf.link_uri}]: Light check!")
    time.sleep(1.0)

    GREEN = 138

    for _ in range(20):
      cf.param.set_value('led.bitmask', GREEN)
      time.sleep(delay)
      cf.param.set_value('led.bitmask', 0)
      time.sleep(delay)

    logger.info(f"[{cf.link_uri}]: Light check complete.")
    time.sleep(1.0)

  def open(self):
    if self.swarm is not None:
      return

    logger.info("In Swarm...")

    cflib.crtp.init_drivers(enable_debug_driver=False)
    self.swarm = Swarm(self.uris, factory=self.factory)
    self.swarm.open_links()
    try:
      self.swarm.parallel_safe(self._light_check)
      logger.info("Resetting estimators...")
      self.swarm.parallel_safe(self._reset_estimator)
      logger.info("Waiting for swarm parameters to be downloaded...")
      self.swarm.parallel_safe(self._wait_for_param_download)
      logger.info("Arming swarm...")
      self.swarm.parallel_safe(self._arm)
    except Exception as e:
      logger.error(f"Swarm bring-up failed: {e}", exc_info=True)
      # Make sure we don’t leave half-open links
      with contextlib.suppress(Exception):
        self.swarm.close_links()
      self.swarm = None
      raise

    # Create MotionCommander per drone
    logger.info("Creating MotionCommander per drone...")
    for uri, scf in self.swarm._cfs.items():
      self.mcs[uri] = MotionCommander(scf)

  def close(self):
    if self.swarm is None:
      return

    try:
      # Stop/land before close
      for mc in self.mcs.values():
        try: mc.stop()
        except: pass
      self.swarm.close_links()
    finally:
      self.swarm = None
      self.mcs.clear()

  def enter_manual(self):
    logger.info("Taking off...")
    time.sleep(1.0)
    # Takeoff all
    for mc in self.mcs.values():
      try:
        mc.take_off(self.takeoff_alt, velocity=0.4)
      except Exception:
        # Already flying is fine
        pass

  def land(self):
    logger.info("Landing...")
    time.sleep(1.0)
    for mc in self.mcs.values():
      try: mc.land()
      except: pass

  def _apply_move(self, fn_name: str, *args):
    for mc in self.mcs.values():
      getattr(mc, fn_name)(*args)

  def _stop_all(self):
    for mc in self.mcs.values():
      try: mc.stop()
      except: pass

  def broadcast_go(self, keys):
    moved = False
    # planar
    if keys.up:
      self._apply_move('start_forward', self.speed_xy); moved = True
    elif keys.down:
      self._apply_move('start_back', self.speed_xy); moved = True
    elif keys.left:
      self._apply_move('start_left', self.speed_xy); moved = True
    elif keys.right:
      self._apply_move('start_right', self.speed_xy); moved = True
    # yaw
    elif keys.a:
      self._apply_move('start_turn_left', self.yaw_rate); moved = True
    elif keys.d:
      self._apply_move('start_turn_right', self.yaw_rate); moved = True
    # vertical
    elif keys.r:
      self._apply_move('start_up', self.speed_z); moved = True
    elif keys.f:
      self._apply_move('start_down', self.speed_z); moved = True

    if not moved:
      self._stop_all()

  def form_line(self):
    """Arrange drones in a straight line formation."""
    logger.info("Executing LINE formation...")
    self._execute_formation("line")

  def form_triangle(self):
    """Arrange drones in a triangle formation."""
    logger.info("Executing TRIANGLE formation...")
    self._execute_formation("triangle")

  def form_square(self):
    """Arrange drones in a square formation."""
    logger.info("Executing SQUARE formation...")
    self._execute_formation("square")

  def form_oscillate(self, 
                     distance_m: float = 0.5, 
                     sets: int = 5, 
                     pause_s: float = 1.0):
    """
    Simple "formation" where every drone moves:

      forward distance_m  -> pause
      backward distance_m -> pause

    That sequence is 1 set. We repeat for `sets` sets.

    Movement is along the drone's X axis (same as forward/back).
    """
    if self.swarm is None:
      logger.error("Swarm not open; cannot execute oscillation.")
      return

    # Sanity on sets
    try:
      sets = max(1, int(sets))
    except Exception:
      sets = 5

    def _cf_oscillate_task(scf):
      uri = scf.cf.link_uri
      mc = self.mcs.get(uri)
      if mc is None:
        logger.warning(f"{uri}: No MotionCommander instance for oscillation.")
        return

      # Use formation velocity if available, otherwise manual XY speed
      v = getattr(self, "form_vel_mps", None) or self.speed_xy
      v = max(0.05, min(v, 0.8))  # keep it in a sane range
      seg_t = distance_m / v  # time to move distance_m at speed v

      logger.info(f"{uri}: Starting oscillation d={distance_m:.2f}m v={v:.2f}m/s "
                  f"sets={sets} pause={pause_s:.2f}s")

      try:
        for i in range(sets):
          # FORWARD
          logger.info(f"{uri}: Oscillate set {i+1}/{sets} - forward")
          mc.start_linear_motion(v, 0.0, 0.0)
          time.sleep(seg_t)
          mc.stop()

          # PAUSE
          if pause_s > 0:
            logger.info(f"{uri}: Pause after forward ({pause_s:.2f}s)")
            time.sleep(pause_s)

          # BACKWARD
          logger.info(f"{uri}: Oscillate set {i+1}/{sets} - backward")
          mc.start_linear_motion(-v, 0.0, 0.0)
          time.sleep(seg_t)
          mc.stop()

          # PAUSE
          if pause_s > 0 and i < sets - 1:
            logger.info(f"{uri}: Pause after backward ({pause_s:.2f}s)")
            time.sleep(pause_s)

      except Exception as e:
        logger.error(f"{uri}: Oscillation task failed: {e}", exc_info=True)
      finally:
        try:
          mc.stop()
        except Exception:
          pass

      logger.info(f"{uri}: Oscillation task complete.")

    try:
      # Stop any prior motion first
      self._stop_all()
      logger.info("Executing OSCILLATE formation...")
      self.swarm.parallel_safe(_cf_oscillate_task)
      logger.info("=== Oscillate formation COMPLETE ===")
    except Exception as e:
      logger.error(f"Oscillate formation failed: {e}", exc_info=True)

  def form_spin(self,
                duration_s: float = 5.0,
                angle_deg: float = 90.0,
                rate_dps: float = 100.0):
    """
    Formation where each drone spins in place (yawing left) for ~duration_s.

    Uses MotionCommander.turn_left(angle_deg, rate_dps) in a loop:
      - angle_deg: degrees per command (default 90°)
      - rate_dps: yaw rate in deg/s (default 100°/s)

    We compute how many turns to fit roughly into `duration_s`.
    """
    if self.swarm is None:
      logger.error("Swarm not open; cannot execute spin formation.")
      return

    # Clamp duration for safety
    duration_s = max(0.5, min(duration_s, 30.0))

    def _cf_spin_task(scf):
      uri = scf.cf.link_uri
      mc = self.mcs.get(uri)
      if mc is None:
        logger.warning(f"{uri}: No MotionCommander instance for spin.")
        return

      # Avoid divide-by-zero if someone passes a weird rate
      rate = max(1.0, abs(rate_dps))
      angle = float(angle_deg)
      t_per_turn = abs(angle) / rate # Approx time for one turn command (sec)
      n_turns = max(1, int(round(duration_s / t_per_turn))) # How many turns to approximate duration_s

      logger.info(f"{uri}: Starting spin formation: duration={duration_s:.2f}s "
                  f"angle={angle:.1f}° rate={rate:.1f}°/s ~{n_turns} turns")

      try:
        for i in range(n_turns):
          logger.info(f"{uri}: Spin turn {i+1}/{n_turns}")
          # Block until this turn is done
          mc.turn_left(angle, rate)
      except Exception as e:
        logger.error(f"{uri}: Spin task failed: {e}", exc_info=True)
      finally:
        try:
          mc.stop()
        except Exception:
          pass

      logger.info(f"{uri}: Spin formation complete.")

    try:
      self._stop_all()
      logger.info("Executing SPIN formation...")
      self.swarm.parallel_safe(_cf_spin_task)
      logger.info("=== Spin formation COMPLETE ===")
    except Exception as e:
      logger.error(f"Spin formation failed: {e}", exc_info=True)
