import io
import math
import time
import os
import sys
import pygame
import threading
import importlib
import warnings
import json
import cv2
import pandas as pd
import numpy as np
import datetime as _dt
from pathlib import Path
from collections import deque
from dataclasses import dataclass, asdict
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env") 

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

from utils import context, helpers, logger as _logger
from drone_connection import DroneConnection
from drone_log import DroneLogs
from swarm_command import SwarmCommand
import vision

root = Path(context.get_context(__file__)).resolve()
logs = root / "logs"
data = root / "data"
logs.mkdir(parents=True, exist_ok=True)
name = Path(__file__).stem
stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%d_%H%M%S")
log_file = logs / f"{name}.log"

logger = _logger.setup_logger(
  logger_name=name,
  log_file=str(log_file),
  log_level=os.getenv("LOG_LEVEL")
)

# Multi-window support (PyGame 2.x / SLD2)
MULTIWIN = False
try:
  from pygame._sdl2.video import Window, Renderer, Texture
  _HAS_REN_COPY = hasattr(Renderer, "copy")
  MULTIWIN = True
except Exception:
  MULTIWIN = False

def _has_multiwin():
  return MULTIWIN

@dataclass
class Waypoint:
  ts: str
  mx: int
  my: int
  fx: int | None = None
  fy: int | None = None
  X: float | None = None
  Y: float | None = None
  Z: float | None = None
  frame_idx: int | None = None

def _utc_iso():
  # ISO8601 with milliseconds and Z
  now = _dt.datetime.utcnow().replace(tzinfo=_dt.timezone.utc)
  return now.isoformat(timespec="milliseconds").replace("+00:00","Z")

def deprecated(reason: str = ""):
  """
  Usage:
    @deprecated("use _map_click() + self._vision_click instead")
  """
  def decorator(func):
    def wrapper(*args, **kwargs):
      warnings.warn(
        f"[DEPRECATED] {func.__name__} is deprecated. {reason}",
        category=DeprecationWarning,
        stacklevel=2
      )
      return func(*args, **kwargs)
    wrapper.__name__ = func.__name__
    wrapper.__doc__ = func.__doc__
    return wrapper
  return decorator

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
    self.speed_xy = 0.25          # m/s
    self.speed_z  = 0.25          # m/s
    self.yaw_rate = 90.0          # deg/s
    self.takeoff_alt = takeoff_alt # m

    self.ibvs_enable_event = threading.Event()   # pressed KP1 -> set; pressed again -> clear
    self.ibvs_enabled = False

    # targets for telemetry-based controller
    self.x_ref = None
    self.y_ref = None
    self.z_ref = None
    self.yaw_ref_deg = None

    self._vision_click = None
    self._vision_toggle = None
    self._vision_clear = None
    self._p_was_down = False

    # Waypoints config (env/CLI)
    self.wp_enabled = helpers._b(os.getenv("WAYPOINT_LOGGING"), default=True)
    self.wp_hud = helpers._b(os.getenv("WAYPOINT_HUD"), default=True)
    self.wp_Z = helpers._f(os.getenv("WAYPOINT_Z"), default=0.0)
    override = (os.getenv("WAYPOINT_LOG_FILE") or "").strip()
    self.wp_path = Path(override) if override else (data / f"waypoints-{stamp}.jsonl")

    # Click-to-Go controller config (env/CLI)
    self.c2g_rate_hz = helpers._f(os.getenv("CLICK2GO_RATE_HZ"), default=25.0)
    self.c2g_tol_m = helpers._f(os.getenv("CLICK2GO_TOL_M"), default=0.06)
    self.c2g_max_speed = helpers._f(os.getenv("CLICK2GO_MAX_SPEED"), default=0.50)
    self.c2g_slew = helpers._f(os.getenv("CLICK2GO_VEL_SLEW"), default=0.80)  # m/s per second (per axis)
    self.c2g_kx = helpers._f(os.getenv("CLICK2GO_KX"), default=0.8)
    self.c2g_ky = helpers._f(os.getenv("CLICK2GO_KY"), default=0.8)
    self.c2g_tag_lossN = int(helpers._f(os.getenv("CLICK2GO_TAG_LOSS_FRAMES"), default=10))
    self.c2g_n_consec = int(helpers._f(os.getenv("CLICK2GO_N_CONSEC"), default=4))
    self.c2g_timeout_s = helpers._f(os.getenv("CLICK2GO_TIMEOUT_S"), default=12.0)

    # Click-to-Go runtime state
    self._c2g_thread = None
    self._c2g_stop = threading.Event()
    self._c2g_active = False
    self._c2g_last = {"fx": None, "fy": None, "t": 0.0}
    self._c2g_debounce_s = 0.4

    # Back-and-forth motion test config (single-drone)
    self.bf_target_m = helpers._f(os.getenv("BF_TARGET_M"), default=0.25)
    self.bf_dwell_s = helpers._f(os.getenv("BF_DWELL_S"), default=3.0)
    self.bf_cycles = int(helpers._f(os.getenv("BF_CYCLES"), default=1))
    self.bf_tol_m = helpers._f(os.getenv("BF_TOL_M"), default=0.10)
    # Pixels-per-meter for converting image-plane displacements to approximate 
    # meters. If <= 0, units are "pixels".
    self.bf_px_per_m = helpers._f(os.getenv("BF_PX_PER_M"), default=0.0)

    # Back-and-forth runtime state (shared with HUD)
    self._bf_thread: threading.Thread | None = None
    self._bf_running = threading.Event()
    self._bf_status: dict | None = None

    # waypoints (thread-safe)
    self._waypoints: list[Waypoint] = []
    self._wp_lock = threading.Lock()
    self._latest_frame_idx = 0

    # Manual target verification (user-driven flight)
    self.target_mode = True
    self.target_tol_px = helpers._f(os.getenv("TARGET_TOL_PX"), default=0.6 * helpers._i(os.getenv("GRID_STEP_PX"), default=30))
    self._targets = []
    self._target_status = None  # {"ts", "total", "hit", "ok"} or None

  @property
  def detector(self):
    """
    Clunky AF but back-compat shim: allow code to use `self.detector` safely.
    """
    return self._get_detector()

  @staticmethod
  def _sat(x, lo, hi):
    return max(lo, min(hi, x))

  @staticmethod
  def _wrap_pi(a):
    # wrap to [-pi, pi)
    return (a + math.pi) % (2.0 * math.pi) - math.pi

  @staticmethod
  def _project_with_pose(u_px: int, v_px: int,
                         K, D,
                         rvec, tvec):
    """
    Project pixel (u,v) to the Z=0 plane of the current marker frame defined by (rvec,tvec).
    Returns (-1, X, Y) where X,Y are in meters in that marker frame. (-1) = unknown id.
    """
    K = np.asarray(K, dtype=float)
    D = np.zeros((5, 1), dtype=float) if D is None else np.asarray(D, dtype=float)
    rvec = np.asarray(rvec, dtype=float).reshape(3, 1)
    tvec = np.asarray(tvec, dtype=float).reshape(3, 1)

    R, _ = cv2.Rodrigues(rvec)              # marker->camera
    n = R @ np.array([[0.0], [0.0], [1.0]]) # plane normal in camera frame
    p0 = tvec
    uv = np.array([[[float(u_px), float(v_px)]]], dtype=np.float32)
    und = cv2.undistortPoints(uv, K, D, P=K)
    ray = np.array([[und[0, 0, 0]], [und[0, 0, 1]], [1.0]], dtype=np.float64)
    ray = ray / np.linalg.norm(ray)
    denom = float((n.T @ ray).item())
    numer = float((n.T @ p0).item())

    if not (math.isfinite(denom) and abs(denom) > 1e-6 and math.isfinite(numer)):
      return None

    s = numer / denom

    if s <= 0:
      return None

    Pc = s * ray
    Xm = R.T @ (Pc - tvec) # camera->marker

    return (-1, float(Xm[0, 0]), float(Xm[1, 0]))

  def _get_detector(self):
    import importlib, sys
    return getattr(sys.modules.get('main') or importlib.import_module('main'), "detector", None)

  def _append_waypoint(self, mx: int, my: int, fx: int | None, fy: int | None):
    """
    Create a Waypoint entry, attempt pixel -> marker-plane projection using the 
    Detector's pose estimate, and append the result to memory and the JSONL log.

    Parameters
    ----------
    mx : int
        The x-coordinate of the mouse click in the PyGame window (in screen pixels).
    my : int
        The y-coordinate of the mouse click in the PyGame window (in screen pixels).
    fx : int | None
        The corresponding x-coordinate in the detector’s native OpenCV frame, 
        derived from (mx, my) via viewport mapping. None if the click occurred 
        outside the video region or could not be mapped.
    fy : int | None
        The corresponding y-coordinate in the detector’s native OpenCV frame, 
        derived from (mx, my). None if unavailable.

    Notes
    -----
    - The function uses `DetectorRT.pixel_to_marker_xy()` to project the pixel 
      (fx, fy) into the Z=0 plane of the nearest detected ArUco marker.
    - The returned (X, Y) are expressed in that marker’s local coordinate frame
      in meters, not in a global world frame.
    - Z is assigned from the configured constant `WAYPOINT_Z`, typically the 
      desired flight altitude in meters.
    - The resulting waypoint is stored in memory (`self._waypoints`) and appended 
      as a JSONL record to `self.wp_path`.
    - If the click cannot be projected (e.g., no pose, no nearby marker), X/Y/Z 
      remain None and a `_why_null_world` reason is logged.
    """
    frame_idx = None

    try:
      main_mod = sys.modules.get('main') or importlib.import_module('main')

      with main_mod._latest_frame_lock:
        frame_idx = getattr(main_mod, "_latest_frame_idx", None)
    except Exception:
      frame_idx = None

    X = Y = Z = None
    reason = None
    try:
      if fx is None or fy is None:
        reason = "no_frame_coords"
      else:
        det = self._get_detector()

        if not det:
          reason = "no_detector"
          logger.warning(reason)
        elif not getattr(det, "_do_pose", False):
          reason = "pose_disabled"
          logger.warning(reason)
        else:
          res = det.latest()[1] if det else None

          if not res or res.get("rvecs") is None or res.get("tvecs") is None:
            reason = "no_pose_in_results"
            logger.warning(reason)
          else:
            out = det.pixel_to_marker_xy(fx, fy, res)

            if out is None:
              reason = "no_near_marker"
              logger.warning(reason)
            else:
              _, X, Y = out
              Z = float(self.wp_Z)
              reason = None
    except Exception:
      # Soft-fail: keep None world coords
      pass

    wp = Waypoint(ts=_utc_iso(),
                  mx=int(mx), my=int(my),
                  fx=(int(fx) if fx is not None else None),
                  fy=(int(fy) if fy is not None else None),
                  X=X, 
                  Y=Y, 
                  Z=Z,
                  frame_idx=(int(frame_idx) if frame_idx is not None else None))

    logger.info(f"Selected Waypoints={wp}")

    with self._wp_lock:
      self._waypoints.append(wp)

    if self.wp_enabled:
      try:
        rec = asdict(wp)
        if reason is not None:
          rec["_why_null_world"] = reason
        with open(self.wp_path, "a", encoding="utf-8") as f:
          f.write(json.dumps(rec, separators=(",", ":")) + "\n")
      except Exception as e:
        logger.warning(f"Waypoint log write failed: {e}")

  def _update_target_hits_from_vision(self):
    """
    In target_mode, use the latest ArUco centroid to mark targets as 'hit'
    when the drone comes within target_tol_px in the image plane.
    Updates self._target_status for HUD/overlay.
    """
    if not self.target_mode:
      return

    # No targets: no PASS/FAIL status
    if not self._targets:
      self._target_status = None
      return

    det = self._get_detector()
    if det is None:
      return

    try:
      frame, res = det.latest()
    except Exception as e:
      logger.debug(f"target verify: detector.latest() failed: {e}")
      return

    if frame is None or not res:
      return

    tgt = vision.primary_target(res, frame.shape)
    if not tgt:
      return

    cx, cy, area, W, H = tgt

    # Use the full detector frame size here (not the marker box dims)
    fh, fw = frame.shape[:2]
    frame_w, frame_h = float(fw), float(fh)
    tol = float(self.target_tol_px)
    hit_count = 0
    total = 0

    with self._wp_lock:
      total = len(self._targets)
      for t in self._targets:

        if t.get("hit"):
          hit_count += 1
          continue

        # Prefer detector-frame pixels if available, since cx,cy are in the
        # same coordinate system. Fall back to normalized clicks only if
        # fx,fy were never recorded (older logs, etc.).
        fx = t.get("fx")
        fy = t.get("fy")
        if fx is not None and fy is not None:
          tx = float(fx)
          ty = float(fy)
        else:
          rx = t.get("rx")
          ry = t.get("ry")
          if rx is None or ry is None:
            # Nothing to compare against
            continue

          # Map normalized click back into detector pixels using full frame size
          tx = float(rx) * max(1.0, frame_w - 1.0)
          ty = float(ry) * max(1.0, frame_h - 1.0)

        dx = float(cx) - tx
        dy = float(cy) - ty
        d = math.hypot(dx, dy)

        if d <= tol:
          t["hit"] = True
          hit_count += 1

    ok = (total > 0 and hit_count == total)

    self._target_status = {
      "ts": time.time(),
      "total": total,
      "hit": hit_count,
      "ok": ok,
    }

  def start_click_to_go(self, fx: int, fy: int):
    """
    Start/replace a closed-loop click-to-go controller that recomputes the metric
    error (X,Y) from the *current* detector pose each tick and drives until ‖e‖ < tol.

    fx, fy: click coordinates in detector frame pixels (e.g., 1280x720)
    """
    now = time.time()

    # Debounce: same click within debounce window? ignore
    if (self._c2g_last["fx"], self._c2g_last["fy"]) == (fx, fy) and (now - self._c2g_last["t"]) < self._c2g_debounce_s:
      logger.debug(f"click2go: debounced duplicate click ({fx},{fy})")
      return

    # If active and the new point is the same (±3 px), ignore
    if self._c2g_active and self._c2g_last["fx"] is not None:
      if abs(self._c2g_last["fx"] - fx) <= 3 and abs(self._c2g_last["fy"] - fy) <= 3:
        logger.debug("click2go: same target; controller already running")
        return

    # Stop any existing controller
    try:
      if self._c2g_thread and self._c2g_thread.is_alive():
        self._c2g_stop.set()
        self._c2g_thread.join(timeout=0.5)
    except Exception:
      pass

    self._c2g_stop = threading.Event()
    t = threading.Thread(target=self._run_click_to_go,
                         args=(int(fx), int(fy), self._c2g_stop),
                         daemon=True)

    self._c2g_thread = t
    self._c2g_active = True
    self._c2g_last = {"fx": fx, "fy": fy, "t": now}
    t.start()
    logger.info(f"click2go: started controller for click frame=({fx},{fy})")

  def _run_click_to_go(self, fx: int, fy: int, stop_event: threading.Event):
    det = self._get_detector()
    if det is None:
      logger.warning("click2go: no detector; aborting.")
      self._c2g_active = False
      return

    # Create MotionCommander if needed (will only move if already flying)
    mover = getattr(self, "mc", None)
    if mover is None:
      try:
        self.mc = MotionCommander(self.scf)
        mover = self.mc
      except Exception as e:
        logger.debug(f"click2go: could not create MotionCommander: {e}")

    hz = max(5.0, float(self.c2g_rate_hz))
    dt = 1.0 / hz
    vmax = float(self.c2g_max_speed)
    slew = float(self.c2g_slew)
    tol = float(self.c2g_tol_m)
    kx, ky = float(self.c2g_kx), float(self.c2g_ky)
    lossN = int(self.c2g_tag_lossN)
    needN = int(self.c2g_n_consec)
    timeout_s = float(self.c2g_timeout_s)

    consec_ok = 0
    loss_ctr  = 0
    prev_vx = prev_vy = 0.0
    t0 = time.time()

    def _slew(prev, target):
      step = slew * dt
      dv = target - prev

      if dv >  step: 
        return prev + step

      if dv < -step: 
        return prev - step

      return target

    stop_reason = "unknown"
    while not stop_event.is_set():
      if (time.time() - t0) > timeout_s:
        logger.info("click2go: timeout; stopping.")
        break

      # Latest vision results
      try:
        _, res = det.latest()
      except Exception:
        res = None

      if res is None:
        loss_ctr += 1
        try:
          if mover and getattr(mover, "_is_flying", False):
            mover.start_linear_motion(0.0, 0.0, 0.0)
        except Exception:
          pass
        if loss_ctr >= lossN:
          stop_reason = "loss"
          logger.info("click2go: tag loss threshold reached; abort.")
          break
        time.sleep(dt)
        continue

      # Recompute metric target *now* from the stored click point.
      # 1) Try with full results (ids/corners/rvecs/tvecs) -> nearest marker plane
      # 2) If missing pose this frame, fall back to cached (K, rvec, tvec) and project directly in that marker’s frame.
      out = None
      try:
        if res:
          out = det.pixel_to_marker_xy(fx, fy, res)
      except Exception:
        out = None

      if not out:
        pose = getattr(det, "latest_pose", lambda: None)()

        if pose is None:
          loss_ctr += 1
          time.sleep(dt)
          continue
        K, rvec, tvec = pose

        try:
          out = self._project_with_pose(fx, fy, K, det.dist_coeffs, rvec, tvec)
        except Exception:
          out = None

      if not out:
        loss_ctr += 1
        time.sleep(dt)
        continue

      loss_ctr = 0
      _, ex, ey = out    # meters in marker frame (marker center ~ (0,0))
      err = math.hypot(ex, ey)

      if err < tol:
        consec_ok += 1
        if consec_ok >= needN:
          stop_reason = f"success (‖e‖={err:.3f} m)"
          logger.info(f"click2go: reached target ‖e‖={err:.3f} m")
          break
      else:
        consec_ok = 0

      # Map to MotionCommander body frame (reuse established sign flip)
      vx = -(kx * ex)
      vy = -(ky * ey)

      # Clamp
      mag = math.hypot(vx, vy)
      if mag > vmax and mag > 1e-6:
        s = vmax / mag
        vx *= s; vy *= s

      # Slew
      vx = _slew(prev_vx, vx)
      vy = _slew(prev_vy, vy)
      prev_vx, prev_vy = vx, vy

      # Command (keep altitude fixed)
      try:
        if mover and getattr(mover, "_is_flying", False):
          mover.start_linear_motion(vx, vy, 0.0)
      except Exception as e:
        logger.debug(f"click2go: motion command failed: {e}")

      time.sleep(dt)

    # Stop & clear
    try:
      if mover and getattr(mover, "_is_flying", False):
        mover.stop()
    except Exception:
      pass
    self._c2g_active = False
    logger.info(f"click2go: controller stopped. reason={stop_reason}")

  def _draw_waypoints_on_video(self, surface: "pygame.Surface", last_draw: dict, color=(255,200,80)):
    """
    Draw small markers + indices for each waypoint onto the already-scaled video surface.
    last_draw contains mapping info for scaling preview pixels -> target rect.
    """
    if not self.wp_hud:
      return

    tw, th, fw, fh = last_draw["tw"], last_draw["th"], last_draw["fw"], last_draw["fh"]

    if tw <= 0 or th <= 0 or fw <= 0 or fh <= 0:
      return

    with self._wp_lock:
      for idx, wp in enumerate(self._waypoints, start=1):
        if wp.fx is None or wp.fy is None:  # need frame coords for overlay
          continue

        sx = int(round((wp.fx / max(1, fw-1)) * tw))
        sy = int(round((wp.fy / max(1, fh-1)) * th))
        pygame.draw.circle(surface, color, (sx, sy), max(2, th // 200))

        # tiny index label
        lbl = pygame.font.SysFont("monospace", 12).render(str(idx), True, color)
        surface.blit(lbl, (sx + 4, sy + 2))

  def _draw_targets_on_video(self, surface: "pygame.Surface", last_draw: dict, color=(220, 60, 60)):
    """Draw manual target cells (from B-mode) as persistent red rectangles.

    Each target stores:
      - rx, ry: normalized click coordinates in [0,1]×[0,1] in the video region
      - fx, fy: detector-frame pixels (for vision-based hit checks / logging)

    We map (rx, ry) into the current preview size (tw, th), then snap to the
    nearest GRID_STEP_PX cell so clicked cells line up with the OpenCV grid.
    Targets marked as "hit" are skipped so they visually disappear once the
    drone has flown over them.
    """
    # Only draw while the manual grid game is active
    if not self.target_mode:
      return

    tw = last_draw.get("tw", 0)
    th = last_draw.get("th", 0)
    fw = last_draw.get("fw", 0)
    fh = last_draw.get("fh", 0)

    if tw <= 0 or th <= 0:
      return

    step_px = helpers._i(os.getenv("GRID_STEP_PX"), default=30)
    if step_px <= 0:
      step_px = 30

    with self._wp_lock:
      for t in self._targets:
        if t.get("hit"):
          continue

        rx = t.get("rx")
        ry = t.get("ry")

        if rx is None or ry is None:
          # Backwards compat: reconstruct normalized coords from stored frame pixels
          fx = t.get("fx")
          fy = t.get("fy")
          if fx is None or fy is None or fw <= 0 or fh <= 0:
            continue
          rx = float(fx) / max(1.0, float(fw) - 1.0)
          ry = float(fy) / max(1.0, float(fh) - 1.0)

        # Map normalized click back into the current preview surface
        sx = int(round(float(rx) * tw))
        sy = int(round(float(ry) * th))

        # Snap to grid cell
        gx = (sx // step_px) * step_px
        gy = (sy // step_px) * step_px

        rect = pygame.Rect(gx, gy, step_px, step_px)
        pygame.draw.rect(surface, color, rect, width=2)

  def set_vision_hooks(self, on_click=None, on_toggle=None, on_clear=None):
    """
    Inject lightweight vision callbacks so PyGame can interact with the detector.
    """
    self._vision_click = on_click
    self._vision_toggle = on_toggle
    self._vision_clear = on_clear

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
      time.sleep(5.0)
      sys.exit()

  def _start_back_and_forth(self):
    """
    Launch the single-drone back-and-forth routine in a background thread.
    Vision is verification-only; this does not close the control loop.
    """
    # Avoid re-entrancy
    if self._bf_thread and self._bf_thread.is_alive():
      logger.info("Back-and-forth routine already running; ignoring trigger.")
      return

    # Do not interfere with active swarm mode (single-drone demo is incompatible with swarm)
    if self.swarm_active:
      logger.warning("Back-and-forth ignored: swarm mode already active.")
      return

    # Defensive: require an attached Crazyflie and an active detector
    if getattr(self, "scf", None) is None:
      logger.warning("Back-and-forth ignored: no Crazyflie connection.")
      return

    det = self._get_detector()
    if det is None:
      logger.warning("Back-and-forth ignored: vision/detector is not initialized.")
      return

    self._bf_running.set()
    self._bf_thread = threading.Thread(target=self._run_back_and_forth,
                                       name="back_forth",
                                       daemon=True)
    self._bf_thread.start()

  def _run_back_and_forth(self):
    """
    Perform an open-loop back-and-forth motion and log vision-based verification.
    """
    det = self._get_detector()
    if det is None:
      logger.warning("Back-and-forth aborted: detector unavailable.")
      self._bf_running.clear()
      return

    target_m = float(self.bf_target_m)
    dwell_s = float(self.bf_dwell_s)
    cycles = max(1, int(self.bf_cycles))
    tol_m = float(self.bf_tol_m)
    px_per_m = float(self.bf_px_per_m)

    logger.info(f"Back-and-forth: starting routine "
                f"(cycles={cycles}, target={target_m:.3f}, dwell={dwell_s:.1f}s, "
                f"tol={tol_m:.3f}, px_per_m={px_per_m:.1f})")

    # Ensure MotionCommander
    if self.mc is None:
      try:
        self.mc = MotionCommander(self.scf)
      except Exception as e:
        logger.error(f"Back-and-forth: cannot create MotionCommander: {e}")
        self._bf_running.clear()
        return

    # Try takeoff to configured hover height; if already flying this is a no-op/raises
    try:
      self.mc.take_off(self.takeoff_alt, velocity=0.4)
    except Exception as e:
      logger.debug(f"Back-and-forth: take_off skipped or failed: {e}")

    def _get_centroid():
      """
      Return (cx, cy) of the primary ArUco marker in the latest frame, or None.
      """
      try:
        frame, res = det.latest()
      except Exception as e:
        logger.debug(f"Back-and-forth: detector.latest() failed: {e}")
        return None

      if frame is None or not res:
        return None

      tgt = vision.primary_target(res, frame.shape)

      if not tgt:
        return None

      cx, cy, area, W, H = tgt
      return (cx, cy)

    def _wait_for_tag(timeout_s: float = 2.0):
      """
      Wait briefly for at least one centroid sample; return the first seen or None.
      """
      deadline = time.time() + max(0.0, timeout_s)
      last = None

      while time.time() < deadline and self._bf_running.is_set():
        p = _get_centroid()

        if p:
          last = p
          break
        time.sleep(0.05)

      return last

    def _eval_leg(cycle_idx: int, leg: str, p0, p1):
      """
      Compute displacement and PASS/FAIL for a single forward/backward leg.
      """
      disp_px = None
      disp_m = None
      ok = False

      if p0 is not None and p1 is not None:
        dx = float(p1[0] - p0[0])
        dy = float(p1[1] - p0[1])
        disp_px = math.hypot(dx, dy)

        if px_per_m > 0.0:
          disp_m = disp_px / px_per_m
          err = abs(disp_m - target_m)
          ok = err <= tol_m
        else:
          # Treat target/tolerance in the same arbitrary "pixel units"
          err = abs(disp_px - target_m)
          ok = err <= tol_m

      # Update HUD-visible status
      self._bf_status = {
        "ts": time.time(),
        "cycle": cycle_idx,
        "leg": leg,
        "disp_px": disp_px,
        "disp_m": disp_m,
        "target": target_m,
        "tol": tol_m,
        "ok": ok,
      }

      if disp_px is None:
        logger.warning(f"Back-and-forth: cycle={cycle_idx} leg={leg}: no vision data; marking FAIL.")
      else:
        msg = (f"Back-and-forth: cycle={cycle_idx} leg={leg} disp_px={disp_px:.1f}")

        if disp_m is not None:
          msg += f" disp_m={disp_m:.3f}"

        msg += f" target={target_m:.3f} tol={tol_m:.3f} PASS={ok}"
        logger.info(msg)

    try:
      for cycle in range(1, cycles + 1):
        if not self._bf_running.is_set():
          break

        # FORWARD leg
        p0 = _wait_for_tag()
        try:
          logger.info(f"Back-and-forth: cycle={cycle} forward leg: +{target_m:.3f} m.")
          # Use distance-based MotionCommander move; open-loop w.r.t. vision
          self.mc.forward(target_m)
        except Exception as e:
          logger.error(f"Back-and-forth: forward command failed: {e}")

        end_time = time.time() + dwell_s
        p1 = None
        samples = 0

        while time.time() < end_time and self._bf_running.is_set():
          p = _get_centroid()

          if p:
            p1 = p
            samples += 1
          time.sleep(0.1)

        if p1 is None:
          p1 = _get_centroid()

        _eval_leg(cycle, "forward", p0, p1)
        logger.debug(f"Back-and-forth: cycle={cycle} forward leg samples={samples}")

        if not self._bf_running.is_set():
          break

        # BACKWARD leg
        p0b = _wait_for_tag()

        try:
          logger.info(f"Back-and-forth: cycle={cycle} backward leg: -{target_m:.3f} m.")
          self.mc.back(target_m)
        except Exception as e:
          logger.error(f"Back-and-forth: backward command failed: {e}")

        end_time = time.time() + dwell_s
        p1b = None
        samples_b = 0

        while time.time() < end_time and self._bf_running.is_set():
          p = _get_centroid()

          if p:
            p1b = p
            samples_b += 1
          time.sleep(0.1)

        if p1b is None:
          p1b = _get_centroid()

        _eval_leg(cycle, "backward", p0b, p1b)
        logger.debug(f"Back-and-forth: cycle={cycle} backward leg samples={samples_b}")

      logger.info("Back-and-forth: routine complete.")
    finally:
      # Attempt a gentle land at the end of the routine so the demo is self-contained.
      try:
        if self.mc is not None:
          logger.info("Back-and-forth: landing at end of routine.")
          self.mc.land()
          self.mc = None
      except Exception as e:
        logger.debug(f"Back-and-forth: landing failed or skipped: {e}")
      self._bf_running.clear()

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
          self.mc.take_off(self.takeoff_alt, velocity=1.0)
        except Exception as e:
          # Already flying? That's fine; continue.
          pass
        self.manual_active = True
        self.ibvs_enable_event.set()
        
        # Initialize telemetry targets at current pose
        try:
          self.x_ref = self.drone_logger.get_stateEstimate_x()
          self.y_ref = self.drone_logger.get_stateEstimate_y()
          self.z_ref = max(0.15, float(self.drone_logger.get_range_zrange() or self.takeoff_alt))
          self.yaw_ref_deg = float(self.drone.get_stateEstimate_yaw())
        except Exception:
          pass

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
    clock = pygame.time.Clock()

    logger.info("In pygame function")
    pygame.init()
    font = pygame.font.SysFont("monospace", 14)
    video_size = (960, 520)
    console_size = (520, 600)

    if _has_multiwin():
      # two real OS windows
      video_win = Window("Drone View", size=video_size, resizable=True)
      console_win = Window("Drone Console", size=console_size, resizable=True)

      video_ren = Renderer(video_win)
      console_ren = Renderer(console_win)

      # For text, we render to a Surface then to a Texture per frame
      logger.info("Dual-window mode enabled (SDL2).")
    else:
      # Fallback: single window with a top video area + bottom console panel
      screen = pygame.display.set_mode((600, 600), pygame.RESIZABLE)
      pygame.display.set_caption("Drone Flight Controls")
      logger.info("Single-window fallback (no _sdl2).")

    # Track video placement/dims for mapping PyGame clicks -> frame pixels
    last_draw = { "x0": 0, "y0": 0, "tw": 0, "th": 0, "fw": 0, "fh": 0 }
    last_click_pg = None   # (mx,my) in PyGame window coords
    last_click_cv = None   # (fx,fy) in OpenCV frame coords

    # helper: map PyFame click -> detector frame (fx, fy)
    def _map_click(mx: int, my: int):
      """
      Return (fx, fy) in detector source pixels, or None if outside video.
      """
      x0, y0 = last_draw["x0"], last_draw["y0"]
      tw, th = last_draw["tw"], last_draw["th"]

      if tw <= 0 or th <= 0:
        return None

      # only count clicks inside the drawn video rectangle
      if not (x0 <= mx < x0 + tw and y0 <= my < y0 + th):
        return None

      # Normalize within the letterboxed video rectangle
      rx = (mx - x0) / float(tw)
      ry = (my - y0) / float(th)
      rx = min(max(rx, 0.0), 1.0)
      ry = min(max(ry, 0.0), 1.0)

      try:
        main_mod = sys.modules.get('main') or importlib.import_module('main')
        with main_mod._latest_frame_lock:
          src_wh = getattr(main_mod, "_latest_src_wh", None)
      except Exception:
        src_wh = None

      if not src_wh:
        return None

      fw, fh = src_wh  # (width, height) in detector.read() frame

      return (int(round(rx * (fw - 1))), int(round(ry * (fh - 1))))

    # Helper for creating textures from surfaces in _sdl2 mode
    def _blit_texture(ren: "Renderer", surf: "pygame.Surface"):
      try:
        tex = Texture.from_surface(ren, surf)
      except Exception:
        try:
          ren.clear()
          ren.present()
        except Exception:
          pass
        return

      try:
        ren.clear()

        if '_HAS_REN_COPY' in globals() and _HAS_REN_COPY:
          ren.copy(tex)
        else:
          tex.draw()

        ren.present()
      except Exception:
        try:
          ren.present()
        except Exception:
          pass
      finally:
        try:
          tex.destroy()
        except Exception:
          pass

    @deprecated("Use _map_click() + self._vision_click instead.")
    def _send_click_to_detector(mx, my):
      """
      [DEPRECATED] Legacy click mapping. 
      Replaced by _map_click() for pixel-accurate coordinate translation.
      """
      nonlocal last_click_pg, last_click_cv

      x0, y0, tw, th, fw, fh = (last_draw["x0"], last_draw["y0"], last_draw["tw"], last_draw["th"], last_draw["fw"], last_draw["fh"])
      if tw <= 0 or th <= 0 or fw <= 0 or fh <= 0:
        return
      if not (x0 <= mx < x0 + tw and y0 <= my < y0 + th):
        return

      rx = (mx - x0) / float(tw)
      ry = (my - y0) / float(th)
      # Store for on-screen debug (normalized + preview pixels)
      last_click_pg = (mx, my)
      last_click_cv = (int(round(rx * fw)), int(round(ry * fh)))

      try:
        if self._vision_click:
          # prefer the normalized method if its available
          det = self._get_detector()
          if det and hasattr(det, "set_click_point_normalized"):
            det.set_click_point_normalized(rx, ry)
          else:
            # Fallback: best-effort pixel mapping against detector/native size if exposed
            src_w, src_h = fw, fh
            try:
              main_mod = sys.modules.get('main') or importlib.import_module('main')
              with main_mod._latest_frame_lock:
                src_w, src_h = getattr(main_mod, "_latest_src_wh", (fw, fh))
                if not src_w or not src_h:
                  src_w, src_h = fw, fh
            except Exception as e:
              pass
            fx = int(round(rx * src_w))
            fy = int(round(ry * src_h))
            self._vision_click(fx, fy)
        else:
          det = self._get_detector()
          if det:
            if hasattr(det, "set_click_point_normalized"):
              det.set_click_point_normalized(rx, ry)
            else:
              # Same fallback logic as above
              src_w, src_h = fw, fh
              try:
                main_mod = sys.modules.get('main') or importlib.import_module('main')
                with main_mod._latest_frame_lock:
                  src_w, src_h = getattr(main_mod, "_latest_src_wh", (fw, fh))
              except Exception as e:
                pass
              det.set_click_point(int(round(rx * src_w)), int(round(ry * src_h)))
      except Exception as e:
        logger.debug(f"Failed to deliver click to detector: {e}")
      logger.info(f"Click: pygame=({mx},{my})  norm=({rx:.3f},{ry:.3f}) rect=({x0},{y0},{tw}x{th}) preview=({fw}x{fh})")

    try:
      # logger.info("Resetting estimators.")
      # self._reset_estimator()
      time.sleep(1.0)
      self.drone_logger.start_logging()
      logger.info("GO!!!!")

      while not done:
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            done = True
            break

          # left-click on the PyGame video -> waypoint  target (or click-to-go)
          if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            mapped = _map_click(mx, my)
            if not mapped:
              logger.debug("Click ignored (outside video).")
            else:
              fx, fy = mapped
              # track for HUD
              last_click_pg = (mx, my)
              last_click_cv = (fx, fy)

              # normalized click within the drawn video rect (for robust target drawing)
              x0, y0 = last_draw["x0"], last_draw["y0"]
              tw, th = last_draw["tw"], last_draw["th"]
              if tw > 0 and th > 0:
                rx = (mx - x0) / float(tw)
                ry = (my - y0) / float(th)
                rx = max(0.0, min(1.0, rx))
                ry = max(0.0, min(1.0, ry))
              else:
                rx = ry = None

              det = self._get_detector()
              try:
                if det:
                  if rx is not None and ry is not None and hasattr(det, "set_click_point_normalized"):
                    det.set_click_point_normalized(rx, ry)
                  else:
                    det.set_click_point(fx, fy)

                if self._vision_click:
                  self._vision_click(fx, fy)
              except Exception as e:
                logger.debug(f"on_click failed: {e}")

              # Log waypoint and then register a manual target (no click-to-go for the demo)
              try:
                self._append_waypoint(mx, my, fx, fy)

                if self.target_mode and fx is not None and fy is not None:
                  with self._wp_lock:
                    self._targets.append({
                      "fx": int(fx),
                      "fy": int(fy),
                      "rx": float(rx) if rx is not None else None,
                      "ry": float(ry) if ry is not None else None,
                      "hit": False,
                    })
                  msg = f"Target added at frame coords ({fx}, {fy})"
                  if rx is not None and ry is not None:
                    msg += f" norm=({rx:.3f},{ry:.3f})"
                  logger.info(msg)
              except Exception as e:
                logger.debug(f"waypoint/target append failed: {e}")

          elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_BACKSPACE:
              done = True

            if event.key == pygame.K_h:
              self._hover()

            if event.key == pygame.K_1 and self.swarm:
              try:
                self.swarm.form_line()
              except Exception as e:
                logger.warning(f"Swarm formation call failed (line): {e}")

            if event.key == pygame.K_2 and self.swarm:
              try:
                self.swarm.form_triangle()
              except Exception as e:
                logger.warning(f"Swarm formation call failed (triangle): {e}")

            if event.key == pygame.K_3 and self.swarm:
              try:
                self.swarm.form_square()
              except Exception as e:
                logger.warning(f"Swarm formation call failed (square): {e}")

            if event.key == pygame.K_4 and self.swarm:
              try:
                self.swarm.form_oscillate()
              except Exception as e:
                logger.warning(f"Swarm formation call failed (oscillate): {e}")

            if event.key == pygame.K_5 and self.swarm:
              try:
                self.swarm.form_spin()
              except Exception as e:
                logger.warning(f"Swarm formation call failed (spin): {e}")

            # X = back-and-forth demo (single-drone or swarm oscillate)
            if event.key == pygame.K_x:
              logger.info("Back-and-forth demo: trigger requested (X).")
              try:
                # If we're in swarm manual mode, run a two-drone oscillation using the same
                # distance / dwell / cycles config as the single-drone demo.
                if self.swarm_active and self.swarm:
                  distance = float(self.bf_target_m)
                  sets = max(1, int(self.bf_cycles))
                  pause = float(self.bf_dwell_s)
                  logger.info(f"Back-and-forth: executing SWARM oscillate demo (distance={distance:.3f}m, sets={sets}, pause={pause:.1f}s)")
                  try:
                    self.swarm.form_oscillate(distance_m=distance, sets=sets, pause_s=pause)
                  except Exception as e_sw:
                    logger.warning(f"Swarm back-and-forth (oscillate) failed: {e_sw}")
                else:
                  # Single-drone back-and-forth verification demo.
                  if self.swarm_active:
                    logger.warning("Back-and-forth (X) ignored: swarm mode active but no swarm object.")
                    return
                  # If currently in manual mode, drop out so the routine has exclusive control.
                  if self.manual_active:
                    logger.info("Back-and-forth: exiting manual mode before starting routine.")
                    self.manual_active = False
                    self.ibvs_enable_event.clear()

                  self._start_back_and_forth()
              except Exception as e:
                logger.warning(f"Back-and-forth demo trigger failed (X): {e}")

            # Manual target verification: toggle grid game mode (user-driven flight)
            if event.key == pygame.K_b:
              self.target_mode = not self.target_mode

              with self._wp_lock:
                self._targets = []

              self._target_status = None
              state = "ENABLED" if self.target_mode else "DISABLED"
              logger.info(f"Manual target verification mode {state} (B).")

            # Vision overlay hotkeys
            if event.key == pygame.K_v:
              try:
                if self._vision_toggle:
                  self._vision_toggle()
                else:
                  det = self._get_detector()
                  if det:
                    det.toggle_delta()
                logger.info("Toggled click-delta overlay (V).")
              except Exception as e:
                logger.debug(f"Toggle failed: {e}")
            if event.key == pygame.K_c:
              try:
                if self._vision_clear:
                  self._vision_clear()
                else:
                  det = self._get_detector()
                  if det:
                    det.clear_click()
                logger.info("Cleared click-delta point (C).")
              except Exception as e:
                logger.debug(f"Clear failed: {e}")

        keys = pygame.key.get_pressed()
        mods = pygame.key.get_mods()
        p_down = keys[pygame.K_p]

        if p_down and not self._p_was_down:
          try:
            if self.detector is not None:
              state = self.detector.toggle_record()
              if state:
                logger.info("Recording: ON (press P to stop)")
              else:
                logger.info("Recording: OFF")
            else:
              logger.warning("Recording toggle ignored: detector not initialized.")
          except Exception as e:
            logger.warning(f"Recording toggle failed: {e}")
        self._p_was_down = p_down

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
          self.ibvs_enable_event.set()

        self._g_was_down = g_down

        # SWARM MANUAL (S to enter, K to land/exit)
        s_down = keys[pygame.K_s]
        if s_down and not self._s_was_down and self.swarm and not self.manual_active and not self.swarm_active:
          # Release single-drone link if it's part of the swarm set
          try:
            if hasattr(self.drone, "_cf") and hasattr(self.drone._cf, "link_uri"):
              # stop logging cleanly  before closing the link to avoid LogEntry if errors
              try:
                self.drone_logger.stop_logging()
              except Exception as e:
                logger.warning(f"Could not stop logging before swarm: {e}")

              if getattr(self.swarm, "uris", ()) and (self.drone._cf.link_uri in self.swarm.uris):
                logger.info(f"Releasing primary link {self.drone._cf.link_uri} for swarm…")
                self.drone._cf.close_link()
                time.sleep(0.2)
          except Exception as e:
            logger.warning(f"Unable to release primary link: {e}")

          # ensure swarm links are open & ready
          logger.info("Opening swarm links...")
          try:
            self.swarm.open()
            logger.info("Entering SWARM manual (takeoff all)...")
            self.swarm.enter_manual()
            self.swarm_active = True
            self.ibvs_enable_event.set()
          except Exception as e:
            logger.error(f"Failed to enter swarm: {e}")
            # Attempt to restore single-drone mode so the app keeps running
            try:
              self.drone._cf.open_link(self.drone.link_uri)
              self._reset_estimator()
              self.drone_logger.start_logging()
            except Exception as e2:
              logger.warning(f"Recovery to single-drone failed: {e2}")
            # continue the loop; do NOT re-raise
            continue

          # initialize targets from current primary link telemetry if available
          try:
            self.x_ref = self.drone_logger.get_stateEstimate_x()
            self.y_ref = self.drone_logger.get_stateEstimate_y()
            self.z_ref = max(0.15, float(self.drone_logger.get_range_zrange() or self.takeoff_alt))
            self.yaw_ref_deg = float(self.drone.get_stateEstimate_yaw())
          except Exception:
            pass

        self._s_was_down = s_down

        # LAND/EXIT: single(L) vs swarm(K)
        if self.manual_active and keys[pygame.K_l]:
          try:
            self.mc.land()
          finally:
            self.mc = None
            self.manual_active = False
            self.ibvs_enable_event.clear()
            self.x_ref = self.y_ref = self.z_ref = self.yaw_ref_deg = None

        if self.swarm_active and keys[pygame.K_k]:
          try:
            self.swarm.land()
          finally:
            self.swarm_active = False
            self.ibvs_enable_event.clear()
            self.x_ref = self.y_ref = self.z_ref = self.yaw_ref_deg = None

            # Reconnect primary link so single-drone controls work again
            try:
              if hasattr(self.drone, "_cf") and hasattr(self.drone, "link_uri"):
                logger.info(f"Reconnecting primary link {self.drone.link_uri}...")
                self.drone._cf.open_link(self.drone.link_uri)
                self._reset_estimator()
                # re-start logging now that single-drone link is back
                self.drone_logger.start_logging()
            except Exception as e:
              logger.warning(f"Failed to reconnect/restart logging: {e}")

        # Per-frame control
        if self.swarm_active:
          self._go_swarm(keys)
        else:
          self._go(keys)

        # RENDER VIDEO WINDOW
        frame_rgb = None
        try:
          main_mod = sys.modules.get('main') or importlib.import_module('main')
          with main_mod._latest_frame_lock:
            if main_mod._latest_frame_np is not None:
              frame_rgb = main_mod._latest_frame_np.copy()
        except Exception as e:
          logger.debug(f"preview copy failed: {e}")

        # Per-frame manual target verification update (vision-only, no control)
        try:
          self._update_target_hits_from_vision()
        except Exception as e:
          logger.debug(f"target verify: update failed: {e}")

        if _has_multiwin():
          # size of the video window
          vw, vh = video_win.size
          # Compute the target size preserving the aspect ratio
          if frame_rgb is not None:
            fh, fw = frame_rgb.shape[:2]
            scale = min(vw / fw, vh / fh)
            tw, th = int(fw * scale), int(fh * scale)
            surf = pygame.image.frombuffer(frame_rgb.tobytes(), (fw, fh), "RGB")

            if (tw, th) != (fw, fh):
              surf = pygame.transform.smoothscale(surf, (tw, th))

            # letterbox center
            x0 = (vw - tw) // 2
            y0 = (vh - th) // 2

            # build a target surface matching window size to center-blit
            canvas = pygame.Surface((vw, vh))
            canvas.fill((0,0,0))

            # Save mapping for both click-mapping and overlays
            last_draw.update({"x0": x0, "y0": y0, "tw": tw, "th": th, "fw": fw, "fh": fh})

            # Draw waypoint markers and manual targets directly on the scaled video
            if self.wp_hud:
              self._draw_waypoints_on_video(surf, last_draw)
            self._draw_targets_on_video(surf, last_draw)

            # Center-blit the annotated video surface into the window canvas
            canvas.blit(surf, (x0, y0))

            # Manual target verification PASS/FAIL tint (multi-window video)
            status = self._target_status
            if status and status.get("ok") is not None:
              r, g, b = ((0, 120, 0) if status["ok"] else (120, 0, 0))
              tint = pygame.Surface((tw, th), pygame.SRCALPHA)
              tint.fill((r, g, b, 60))  # low alpha so image is still visible
              canvas.blit(tint, (x0, y0))

            # Back-and-forth PASS/FAIL overlay tint (multi-window video)
            bf = self._bf_status
            if bf and bf.get("ok") is not None:
              r, g, b = ((0, 120, 0) if bf.get("ok") else (120, 0, 0))
              tint = pygame.Surface((tw, th), pygame.SRCALPHA)
              tint.fill((r, g, b, 60))  # low alpha so image is still visible
              canvas.blit(tint, (x0, y0))

            _blit_texture(video_ren, canvas)

            # Save for click mapping
            last_draw.update({"x0": x0, "y0": y0, "tw": tw, "th": th, "fw": fw, "fh": fh})
          else:
            # clear if no frame yet
            video_ren.clear()
            video_ren.present()
        else:
          # Single-window fallback layout
          screen.fill((0, 0, 0))
          SCREEN_W, SCREEN_H = screen.get_size()
          INFO_H = max(140, SCREEN_H // 4) # bottom info strip height
          VIDEO_W = SCREEN_W
          VIDEO_H = SCREEN_H - INFO_H # top area reserved for video

          if frame_rgb is not None:
            fh, fw = frame_rgb.shape[:2]
            scale = min(VIDEO_W / fw, VIDEO_H / fh)
            tw, th = int(fw * scale), int(fh * scale)
            surf = pygame.image.frombuffer(frame_rgb.tobytes(), (fw, fh), "RGB")

            if (tw, th) != (fw, fh):
              surf = pygame.transform.smoothscale(surf, (tw, th))

            x0 = (VIDEO_W - tw) // 2
            y0 = (VIDEO_H - th) // 2

            # Save mapping for this frame so clicks and overlays share geometry
            last_draw.update({"x0": x0, "y0": y0, "tw": tw, "th": th, "fw": fw, "fh": fh})

            # Overlay markers directly on scaled video region
            if self.wp_hud:
              self._draw_waypoints_on_video(surf, last_draw)
            self._draw_targets_on_video(surf, last_draw)

            # Blit the annotated video to the main screen
            screen.blit(surf, (x0, y0))

            # Manual target verification PASS/FAIL tint (single-window video)
            status = self._target_status
            if status and status.get("ok") is not None:
              r, g, b = ((0, 120, 0) if status["ok"] else (120, 0, 0))
              tint = pygame.Surface((tw, th), pygame.SRCALPHA)
              tint.fill((r, g, b, 60))
              screen.blit(tint, (x0, y0))

            # Back-and-forth verification PASS/FAIL overlay tint (single-window video)
            bf = self._bf_status
            if bf and bf.get("ok") is not None:
              r, g, b = ((0, 120, 0) if bf.get("ok") else (120, 0, 0))
              tint = pygame.Surface((tw, th), pygame.SRCALPHA)
              tint.fill((r, g, b, 60))
              screen.blit(tint, (x0, y0))

        instructions = [
          "Drone Control",
          "=======================================",
          "Single-Drone Commands:",
          "  H           | Autonomous Hover (one-shot)",
          "  G           | Take Off (Manual Mode)",
          "  L           | Land / Exit Manual",
          "  B           | Toggle Target Mode (manual grid challenge)",
          "  X           | Back-and-Forth Motion Test",
          "Swarm Commands:",
          "  S           | SWARM Take Off (All Drones)",
          "  K           | SWARM Land / Exit",
          "  1           | LINE Formation",
          "  2           | TRIANGLE Formation",
          "  3           | SQUARE Formation",
          "  4           | OSCILLATION Formation",
          "  5           | FORWARD OSCILLATION Formation",
          "Movement (Manual/Swarm):",
          "  Arrow Keys  | Forward, Back, Left, Right",
          "  A / D       | Yaw Left / Right",
          "  R / F       | Altitude Up / Down",
          "Vision (if enabled):",
          "  Left Click  | Add Target Cell (manual grid game)",
          "  (Target Mode: video tints green when all clicked targets are reached, red otherwise)",
          "Exit:",
          "  Backspace   | Quit Program"
        ]

        # RENDER CONSOLE WINDOW
        pad_x, pad_y = 16, 12
        clock = pygame.time.Clock()

        try:
          main_mod = sys.modules.get('main') or importlib.import_module('main')
          with main_mod._latest_frame_lock:
            ids_list = list(dict.fromkeys(getattr(main_mod, "_latest_ids", [])))
          ids_list.sort()
        except Exception:
          ids_list = []

        ids_text = "IDs: " + (", ".join(map(str, ids_list)) if ids_list else "—")

        if _has_multiwin():
          cw, ch = console_win.size
          panel = pygame.Surface((cw, ch), pygame.SRCALPHA)
          panel.fill((18, 18, 22, 255))
          x, y = pad_x, pad_y

          txt_ids = font.render(ids_text, True, (120, 200, 255))
          panel.blit(txt_ids, (x, y))
          y += txt_ids.get_height() + 10

          if last_click_pg or last_click_cv:
            if last_click_pg:
              t = font.render(f"Click (PyGame):  ({last_click_pg[0]}, {last_click_pg[1]})", True, (180, 220, 180))
              panel.blit(t, (x, y))
              y += t.get_height() + 4
            if last_click_cv:
              t = font.render(f"Click (Frame):   ({last_click_cv[0]}, {last_click_cv[1]})", True, (180, 220, 180))
              panel.blit(t, (x, y))
              y += t.get_height() + 8

          # Simple HUD count
          try:
            with self._wp_lock:
              n_wp = len(self._waypoints)

            t = font.render(f"Waypoints: {n_wp}", True, (230, 230, 230))
            panel.blit(t, (x, y))
            y += t.get_height() + 8
          except Exception: 
            pass

          # Manual target verification HUD
          try:
            status = self._target_status
            if status:
              mode = "ON" if self.target_mode else "OFF"
              line1 = f"Targets mode: {mode}"
              t = font.render(line1, True, (230, 230, 180))
              panel.blit(t, (x, y))
              y += t.get_height() + 4

              total = int(status.get("total") or 0)
              hit = int(status.get("hit") or 0)
              ok = status.get("ok")
              suffix = ""
              if ok is True:
                suffix = " [PASS]"
              elif ok is False:
                suffix = " [FAIL]"
              line2 = f"  Targets hit: {hit}/{total}{suffix}"
              t2 = font.render(line2, True, (200, 200, 200))
              panel.blit(t2, (x, y))
              y += t2.get_height() + 8
          except Exception:
            pass

          # Back-and-forth verification HUD
          try:
            bf = self._bf_status

            if bf:
              status = "PASS" if bf.get("ok") else "FAIL"
              cycle = bf.get("cycle")
              leg = bf.get("leg")
              disp_px = bf.get("disp_px")
              disp_m = bf.get("disp_m")
              line1 = f"Back&Forth: cycle {cycle} leg {leg} [{status}]"
              t = font.render(line1, True, (230, 230, 180))
              panel.blit(t, (x, y))
              y += t.get_height() + 4

              if disp_px is not None:
                if disp_m is not None:
                  line2 = (f"  disp_px={disp_px:.1f}  "
                           f"disp_m={disp_m:.2f} m  "
                           f"(target={self.bf_target_m:.2f}, tol={self.bf_tol_m:.2f})")
                else:
                  line2 = (f"  disp_px={disp_px:.1f}  "
                           f"(target={self.bf_target_m:.2f}, tol={self.bf_tol_m:.2f})")
                t2 = font.render(line2, True, (200, 200, 200))
                panel.blit(t2, (x, y))
                y += t2.get_height() + 6

          except Exception:
            pass

          for line in instructions:
            t = font.render(line, True, (230, 230, 230)); panel.blit(t, (x, y)); y += t.get_height() + 6
            if y > ch - 8: 
              break
          _blit_texture(console_ren, panel)
        else:
          # Single-window bottom panel
          SCREEN_W, SCREEN_H = screen.get_size()
          INFO_H = max(140, SCREEN_H // 4)
          info_y = SCREEN_H - INFO_H
          panel = pygame.Surface((SCREEN_W, INFO_H), pygame.SRCALPHA)
          panel.fill((18, 18, 22, 220))
          screen.blit(panel, (0, info_y))
          x, y = pad_x, info_y + pad_y

          # Manual target verification HUD
          try:
            status = self._target_status
            if status:
              mode = "ON" if self.target_mode else "OFF"
              line1 = f"Targets mode: {mode}"
              t = font.render(line1, True, (230, 230, 180))
              screen.blit(t, (x, y))
              y += t.get_height() + 4

              total = int(status.get("total") or 0)
              hit = int(status.get("hit") or 0)
              ok = status.get("ok")
              suffix = ""

              if ok is True:
                suffix = " [PASS]"
              elif ok is False:
                suffix = " [FAIL]"

              line2 = f"  Targets hit: {hit}/{total}{suffix}"
              t2 = font.render(line2, True, (200, 200, 200))
              screen.blit(t2, (x, y))
              y += t2.get_height() + 8
          except Exception:
            pass

          txt_ids = font.render(ids_text, True, (120, 200, 255))
          screen.blit(txt_ids, (x, y))
          y += txt_ids.get_height() + 10 

          if last_click_pg or last_click_cv:
            if last_click_pg:
              t = font.render(f"Click (PyGame):  ({last_click_pg[0]}, {last_click_pg[1]})", True, (180, 220, 180))
              screen.blit(t, (x, y))
              y += t.get_height() + 4
            if last_click_cv:
              t = font.render(f"Click (Frame):   ({last_click_cv[0]}, {last_click_cv[1]})", True, (180, 220, 180))
              screen.blit(t, (x, y))
              y += t.get_height() + 8

          # Simple HUD count
          try:
            with self._wp_lock:
              n_wp = len(self._waypoints)

            t = font.render(f"Waypoints: {n_wp}", True, (230, 230, 230))
            screen.blit(t, (x, y))
            y += t.get_height() + 8
          except Exception: 
            pass

          # Back-and-forth verification HUD
          try:
            bf = self._bf_status
            if bf:
              status = "PASS" if bf.get("ok") else "FAIL"
              cycle = bf.get("cycle")
              leg = bf.get("leg")
              disp_px = bf.get("disp_px")
              disp_m = bf.get("disp_m")
              line1 = f"Back&Forth: cycle {cycle} leg {leg} [{status}]"
              t = font.render(line1, True, (230, 230, 180))
              screen.blit(t, (x, y))
              y += t.get_height() + 4

              if disp_px is not None:
                if disp_m is not None:
                  line2 = (f"  disp_px={disp_px:.1f}  "
                           f"disp_m={disp_m:.2f} m  "
                           f"(target={self.bf_target_m:.2f}, tol={self.bf_tol_m:.2f})")
                else:
                  line2 = (f"  disp_px={disp_px:.1f}  "
                           f"(target={self.bf_target_m:.2f}, tol={self.bf_tol_m:.2f})")
                t2 = font.render(line2, True, (200, 200, 200))
                screen.blit(t2, (x, y))
                y += t2.get_height() + 6

          except Exception:
            pass

          for line in instructions:
            t = font.render(line, True, (230, 230, 230))
            screen.blit(t, (x, y))
            y += t.get_height() + 6
            if y > SCREEN_H - 8:  # prevent overflow
              break

          pygame.display.flip()

        clock.tick(60)
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

      frame, res = detector.latest()
      if frame is None or not res:
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

  def follow_target_servo(self, detector, stop_event, start_event=None, 
                          loop_hz=50, gains=None, vision_yaw_alpha=0.05, 
                          forward_nudge_alpha=0.03):
    """
    Telemetry-primary position/velocity servo with optional slow vision drift correction.
    Uses stateEstimate/ToF for control; vision only nudges references when confident.
    """
    if self.mc is None:
      self.mc = MotionCommander(self.scf)


    # default gains
    if gains is None:
      gains = dict(Kpx=0.6, Kdx=0.2,
                   Kpy=0.6, Kdy=0.2,
                   Kpz=1.0, Kiz=0.2,
                   Kpyaw=2.0, Kdyaw=0.3)

    Kpx, Kdx = gains["Kpx"], gains["Kdx"]
    Kpy, Kdy = gains["Kpy"], gains["Kdy"]
    Kpz, Kiz = gains["Kpz"], gains["Kiz"]
    Kpyaw, Kdyaw = gains["Kpyaw"], gains["Kdyaw"]

    iz = 0.0
    dt = 1.0 / float(loop_hz)
    max_vxy, max_vz, max_rate = 0.30, 0.25, 180.0  # m/s, m/s, deg/s
    last_t = time.time()

    while not stop_event.is_set():
      t0 = time.time()

      # only run while the manual/swarm flight mode is active
      if (start_event is None or not start_event.is_set()) or not (self.manual_active or self.swarm_active):
        time.sleep(dt)
        continue

      # gate on estimator health
      try:
        ok_est = (self.drone_logger.get_kalman_varPX() < 1.0 and
                  self.drone_logger.get_kalman_varPY() < 1.0 and 
                  self.drone_logger.get_kalman_varPZ() < 1.0)
      except Exception:
        ok_est = True

      if not ok_est:
        self.mc.stop()
        time.sleep(dt)
        continue

      # read telemetry snapshot
      try:
        x = float(self.drone_logger.get_stateEstimate_x())
        y = float(self.drone_logger.get_stateEstimate_y())
        zt = float(self.drone_logger.get_range_zrange() or self.drone_logger.get_stateEstimate_z())
        vx = float(self.drone_logger.get_stateEstimate_vx())
        vy = float(self.drone_logger.get_stateEstimate_vy())
        yaw_deg = float(self.drone_logger.get_stateEstimate_yaw())
        yawrate_dps = float(self.drone_logger.get_gyro_z())
      except Exception:
        time.sleep(dt)
        continue

      # initialize refs if missing
      if self.x_ref is None:
        self.x_ref = x 
        self.y_ref = y
        self.z_ref = max(0.15, zt)
        self.yaw_ref_deg = yaw_deg

      # slow vision drift correction (yaw & gentle forward pull) -- might delete later (shoutout JCole)
      try:
        frame, res = detector.latest() if detector else (None, {})
        tgt = vision.primary_target(res, frame.shape) if (frame is not None and res) else None

        if tgt:
          cx, cy, area, W, H = tgt
          ex_px = (cx - (W / 2.0)) / max(1.0, (W / 2.0)) # normalize the horizontal offset to [-1, 1]
          # nudge yaw reference a little toward the tag
          self.yaw_ref_deg += float(vision_yaw_alpha * ex_px * 30.0) # up to +/- 30 degrees scaled by ex

          # forward nudge: pull towards the tag by shrinking (x_ref, y_ref) along heading
          # heuristic using area as inverse-distance proxy
          if area > 1.0 and forward_nudge_alpha > 0.0:
            # move a few cm towards the heading
            d_step = forward_nudge_alpha * 0.05 # meters per tick
            yaw_rad = math.radians(yaw_deg)
            self.x_ref += d_step * math.cos(yaw_rad)
            self.y_ref += d_step * math.sin(yaw_rad)
      except Exception:
        pass

      # errors in local frame
      ex = self.x_ref - x
      ey = self.y_ref - y
      ez = self.z_ref - zt
      eyaw = self._wrap_pi(math.radians(self.yaw_ref_deg) - math.radians(yaw_deg)) # rad

      # PD/PI in world frame for x,y
      # PI for z using ToF
      # Still kinda buggy due to drop in FPS
      vx_g = (Kpx * ex) - (Kdx * vx)
      vy_g = (Kpy * ey) - (Kdy * vy)
      iz = self._sat(iz + (Kiz * ez * dt), -0.3, 0.3)
      vz = self._sat(Kpz * ez + iz, -max_vz, max_vz)

      # rotate to body frame for MotionCommander
      # reference: https://github.com/alireza787b/mavsdk_drone_show/blob/73a0587bbdeff18b647db582c0a89042247014fc/smart_swarm_src/utils.py#L10
      yaw = math.radians(yaw_deg)
      cy, sy = math.cos(yaw), math.sin(yaw)
      vx_b = cy * vx_g + sy * vy_g
      vy_b = -sy * vx_g + cy * vy_g

      # Yaw rate command (deg/s). 
      # yawrate_dps already in deg/s if using gyro_z scaled; else keep small D.
      rate_yaw = self._sat(math.degrees(Kpyaw * eyaw) - (Kdyaw * yawrate_dps), -max_rate, max_rate)

      # clamp the planar speeds
      vx_b = self._sat(vx_b, -max_vxy, max_vxy)
      vy_b = self._sat(vy_b, -max_vxy, max_vxy)

      # send it
      self.mc.start_linear_motion(vx_b, vy_b, vz, rate_yaw=rate_yaw)

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

