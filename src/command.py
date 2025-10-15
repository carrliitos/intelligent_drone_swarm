import io
import math
import time
import os
import pandas as pd
import sys
import pygame
import threading
import numpy as np
import importlib
import warnings
import json
import datetime as _dt
from pathlib import Path
from collections import deque
from dataclasses import dataclass, asdict
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env") 

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander

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
    self._use_commander = helpers._b(os.getenv("IBVS_USE_COMMANDER"), default=True)
    self.cmdr = Commander(self.scf.cf) if self._use_commander else None
    logger.info(f"IBVS backend: {'Commander' if self._use_commander else 'MotionCommander'}")
    # Absolute Z target (meters) for Commander.send_hover_setpoint
    self._z_target = float(takeoff_alt) if takeoff_alt is not None else 0.25
    self._z_min = float(os.getenv("IBVS_Z_MIN", "0.10"))
    self._z_max = float(os.getenv("IBVS_Z_MAX", "1.50"))

    try:
      self._z_sign = float(os.getenv("IBVS_Z_SIGN", "1"))
      if self._z_sign == 0.0: 
        self._z_sign = 1.0
    except Exception: 
      self._z_sign = 1.0

    self._ibvs_ref_px = None   # (fx, fy) clicked pixel
    self._ibvs_ref_area = None # desired area at click (keeps distance)
    self.click2go_tol_px = helpers._to_int(os.getenv("CLICK2GO_TOL_PX"), default=8) or 8

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

    # Waypoints config (env/CLI)
    self.wp_enabled = helpers._b(os.getenv("WAYPOINT_LOGGING"), default=True)
    self.wp_hud = helpers._b(os.getenv("WAYPOINT_HUD"), default=True)
    self.wp_Z = helpers._f(os.getenv("WAYPOINT_Z"), default=0.0)
    override = (os.getenv("WAYPOINT_LOG_FILE") or "").strip()
    self.wp_path = Path(override) if override else (data / f"waypoints-{stamp}.jsonl")

    # waypoints (thread-safe)
    self._waypoints: list[Waypoint] = []
    self._wp_lock = threading.Lock()
    self._latest_frame_idx = 0

  @staticmethod
  def _sat(x, lo, hi):
    return max(lo, min(hi, x))

  @staticmethod
  def _wrap_pi(a):
    # wrap to [-pi, pi)
    return (a + math.pi) % (2.0 * math.pi) - math.pi

  def _send_body_motion(self, vx: float, vy: float, vz: float, rate_yaw: float, dt: float):
    """
    Stream body-frame velocity + yaw-rate.
    If Commander is enabled: send_hover_setpoint(vx, vy, yawrate_deg, z_abs).
    We integrate vz into the absolute z-distance target.
    Fallback: MotionCommander.start_linear_motion(vx, vy, vz, rate_yaw).
    """
    try:
      if self._use_commander and self.cmdr is not None:
        # integrate Z target (clamped)
        if not math.isnan(vz) and abs(vz) > 1e-6:
          self._z_target = self._z_target + self._z_sign * float(vz) * float(dt)
          self._z_target = min(self._z_max, max(self._z_min, self._z_target))
          logger.debug(f"hover z_target={self._z_target:.2f} (vz={vz:.2f}, sign={self._z_sign})")
        self.cmdr.send_hover_setpoint(float(vx), float(vy), float(rate_yaw), float(self._z_target))
      else:
        if self.mc is None:
          self.mc = MotionCommander(self.scf, default_height=self.takeoff_alt or 0.25)
        self.mc.start_linear_motion(float(vx), float(vy), float(vz), rate_yaw=float(rate_yaw))
    except Exception as e:
      logger.debug(f"_send_body_motion failed: {e}")

  def _stop_body_motion(self):
    """Graceful stop depending on backend."""
    try:
      if self._use_commander and self.cmdr is not None:
        try:
          self.cmdr.send_stop_setpoint()
          self.cmdr.send_notify_setpoint_stop(0)
        except Exception:
          pass
      else:
        if self.mc:
          self.mc.stop()
    except Exception as e:
      logger.debug(f"_stop_body_motion failed: {e}")

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
      # Stream a zero-velocity hover setpoint
      self._send_body_motion(0.0, 0.0, 0.0, 0.0, 0.0)

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
    font = pygame.font.SysFont("monospace", 14)
    video_size = (960, 520)
    console_size = (520, 600)

    if _has_multiwin():
      logger.info("Has dual-window.")

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
      logger.info("Resetting estimators.")
      self._reset_estimator()
      time.sleep(1.0)
      self.drone_logger.start_logging()
      logger.info("GO!!!!")

      while not done:
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            done = True
            break
          # left-click anywhere on the stream -> send to DetectorRT
          if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            # Only accepting clicks inside the video window on dual-window mode
            if _has_multiwin():
              # MOUSEBUTTONDOWN carries window position already local
              mx, my = event.pos
              mapped = _map_click(mx, my)
              if mapped and self._vision_click:
                fx, fy = mapped
                self._ibvs_ref_px = (fx, fy)
                try:
                  det = self._get_detector()
                  res = det.latest()[1] if det else None
                  if res:
                    # grab the current area as “distance” target
                    tgt = det.primary_target(res, None) if hasattr(det, "primary_target") else None
                    if tgt:
                      _, _, area, *_ = tgt
                      self._ibvs_ref_area = float(area)
                except Exception:
                  pass
                logger.info(f"IBVS click-to-go: target set at (fx={fx:.1f}, fy={fy:.1f})")

                # Forward to detector (will draw crosshair + delta on next frame)
                try:
                  self._vision_click(fx, fy)
                except Exception as e:
                  logger.debug(f"on_click failed: {e}")

                # Log waypoint (no command dispatch)
                try:
                  self._append_waypoint(mx, my, fx, fy)
                except Exception as e:
                  logger.debug(f"waypoint append failed: {e}")
              else:
                logger.debug("Click ignored (outside video).")

          if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_BACKSPACE:
              done = True
            if event.key == pygame.K_h:
              self._hover()
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

                # Also stop any residual motion on cancel
                self._stop_body_motion()
                self._ibvs_ref_px = None
                self._ibvs_ref_area = None
                logger.info("IBVS click-to-go: target cleared.")
              except Exception as e:
                logger.debug(f"Clear failed: {e}")

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
            canvas.blit(surf, (x0, y0))

            # Draw waypoint markers onto the video area
            if self.wp_hud:
              self._draw_waypoints_on_video(surf, last_draw)

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
            screen.blit(surf, (x0, y0))

            # overlay markers directly on scaled video region
            if self.wp_hud:
              self._draw_waypoints_on_video(surf, last_draw)

            last_draw.update({"x0": x0, "y0": y0, "tw": tw, "th": th, "fw": fw, "fh": fh})

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
          "V           | Toggle click-delta overlay (vision)",
          "C           | Clear click-delta point (vision)",
          "Backspace   | Exit program"
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

          for line in instructions:
            t = font.render(line, True, (230, 230, 230))
            screen.blit(t, (x, y))
            y += t.get_height() + 6
            if y > SCREEN_H - 8:  # prevent overflow
              break

          pygame.display.flip()
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

    k_lin_p, k_lin_d = 0.60, 0.12 # IBVS pixel -> body velocity gains (click-to-go)

    # Clamps and headbands
    yaw_max, vx_max, vz_max = 180.0, 0.30, 0.25
    db_px, db_dist = 0.02, 0.02

    vx_max = helpers._to_float(os.getenv("CLICK2GO_MAX_SPEED"), default=vx_max) or vx_max
    loss_N = helpers._to_int(os.getenv("CLICK2GO_TAG_LOSS_FRAMES"), default=10) or 10
    err_alpha = helpers._to_float(os.getenv("CLICK2GO_ERR_LPF"), default=0.35) or 0.35
    vel_slew = helpers._to_float(os.getenv("CLICK2GO_VEL_SLEW"), default=0.8) or 0.8  # m/s^2

    # state for filters/derivatives
    ex_f = 0.0
    ey_f = 0.0
    vx_cmd = 0.0
    vy_cmd = 0.0
    last_valid_t = None
    lost_frames = 0

    # derivative memory
    ex_prev = ed_prev = ey_prev = None

    while not stop_event.is_set():
      t0 = time.time()

      # gate: only do something while G/S flight modes are active
      if (start_event is None or not start_event.is_set()) or not (self.manual_active or self.swarm_active):
        # keep motors alive with a neutral hover setpoint
        vx_cmd = 0.0
        vy_cmd = 0.0
        last_valid_t = None
        ex_prev = ed_prev = ey_prev = None
        time.sleep(dt)
        continue

      frame, res = detector.latest()
      if frame is None or not res:
        lost_frames += 1
        if self._ibvs_ref_px is not None and lost_frames >= loss_N:
          self._send_body_motion(0.0, 0.0, 0.0, 0.0, dt)
          logger.info(f"IBVS click-to-go: paused (tag loss {lost_frames} frames).")
        time.sleep(dt)
        continue

      tgt = vision.primary_target(res, frame.shape)
      if not tgt:
        lost_frames += 1
        if self._ibvs_ref_px is not None and lost_frames >= loss_N:
          self._stop_body_motion()
          logger.info(f"IBVS click-to-go: paused (no marker {lost_frames} frames).")
        time.sleep(dt)
        continue
      else:
        lost_frames = 0

      cx, cy, area, W, H = tgt
      logger.debug(f"IBVS target: cx={cx:.1f}, cy={cy:.1f}, area={area:.0f}")

      if halfW is None: halfW, halfH = W / 2.0, H / 2.0

      # normalized errors
      rx, ry = (self._ibvs_ref_px if self._ibvs_ref_px else (halfW, halfH))
      ex = (cx - rx) / halfW          # horizontal pixel error → yaw/forward via your IBVS
      ey = (cy - ry) / halfH          # vertical pixel error (often ignored if you hold Z)

      ed = 0.0
      if self._ibvs_ref_area is not None:
        ed = (self._ibvs_ref_area - max(area, 1.0)) / max(self._ibvs_ref_area, 1.0)

      # deadbands
      if abs(ex) < db_px: ex = 0.0
      if abs(ey) < db_px: ey = 0.0
      if abs(ed) < db_dist: ed = 0.0

      # low-pass error and compute derivatives with actual time since last detection
      ex_f = (1.0 - err_alpha) * ex_f + err_alpha * ex
      ey_f = (1.0 - err_alpha) * ey_f + err_alpha * ey
      now = time.time()

      if last_valid_t is None:
        dt_valid = dt
      else:
        dt_valid = max(1e-3, now - last_valid_t)

      ex_d = 0.0 if ex_prev is None else (ex_f - ex_prev) / dt_valid
      ed_d = 0.0 if ed_prev is None else (ed - ed_prev) / dt_valid
      ey_d = 0.0 if ey_prev is None else (ey_f - ey_prev) / dt_valid
      last_valid_t = now
      ex_prev, ed_prev, ey_prev = ex_f, ed, ey_f

      # If no active click target, hold station (keep sending zeros to preserve cadence)
      if self._ibvs_ref_px is None:
        self._send_body_motion(0.0, 0.0, 0.0, 0.0, dt)
        elapsed = time.time() - t0
        if elapsed < dt:
          time.sleep(dt - elapsed)
        continue

      # If no active click target, hold station (no IBVS motion)
      if self._ibvs_ref_px is None:
        self._stop_body_motion()
        elapsed = time.time() - t0
        if elapsed < dt: time.sleep(dt - elapsed)
        continue

      # Click-to-go mapping: keep yaw fixed; drive with lateral body velocities
      yaw_rate = 0.0

      # Pixel up/down (−ey_f) -> forward/back; pixel left/right (−ex_f) -> strafe
      vx_des = self._sat(k_lin_p * (-ey_f) + k_lin_d * (-(ey_d)), -vx_max, vx_max)
      vy_des = self._sat(k_lin_p * (-ex_f) + k_lin_d * (-(ex_d)), -vx_max, vx_max)

      # slew-limit toward desired velocity to prevent thrust puffs
      max_step = vel_slew * dt_valid
      vx_cmd = vx_cmd + self._sat(vx_des - vx_cmd, -max_step, +max_step)
      vy_cmd = vy_cmd + self._sat(vy_des - vy_cmd, -max_step, +max_step)
      vx_b = vx_cmd
      vy_b = vy_cmd
      vz = 0.0 if not use_vertical else self._sat((vrt_kp * ey_f) + (vrt_kd * ey_d), -vz_max, vz_max)
      logger.debug(f"IBVS cmd: vx={vx_b:.2f} vy={vy_b:.2f} tol={self.click2go_tol_px}px ex={ex:.3f} ey={ey:.3f}")

      # Stop when the pixel is inside tolerance of the clicked target
      if self._ibvs_ref_px is not None:
        if abs(cx - rx) < self.click2go_tol_px and abs(cy - ry) < self.click2go_tol_px:
          # neutral hover instead of STOP to prevent thrust drop
          self._send_body_motion(0.0, 0.0, 0.0, 0.0, dt)
          self._ibvs_ref_px = None
          self._ibvs_ref_area = None
          logger.info("IBVS click-to-go: reached target; hovering.")
          time.sleep(0.1)
          continue

      logger.info("Sending body motion: " \
                 f"[vx_b={vx_b:.2f} | vy_b={vy_b:.2f} | vz={vz:.2f} | "
                 f"yaw_rate={yaw_rate:.2f} | dt={dt_valid:.3f}]")
      self._send_body_motion(vx_b, vy_b, vz, yaw_rate, dt)

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
        self._stop_body_motion()
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
      self._send_body_motion(vx_b, vy_b, vz, rate_yaw, dt)

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

