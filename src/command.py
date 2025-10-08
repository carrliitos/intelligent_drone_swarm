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
from pathlib import Path
from collections import deque
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env") 

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

from utils import logger, context
from drone_connection import DroneConnection
from drone_log import DroneLogs
from swarm_command import SwarmCommand
import vision

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(
  logger_name=logger_name, 
  log_file=f"{directory}/logs/{logger_file_name}.log", 
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

  @staticmethod
  def _sat(x, lo, hi):
    return max(lo, min(hi, x))

  @staticmethod
  def _wrap_pi(a):
    # wrap to [-pi, pi)
    return (a + math.pi) % (2.0 * math.pi) - math.pi

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

    def _get_detector():
      import importlib, sys
      return getattr(sys.modules.get('main') or importlib.import_module('main'), "detector", None)

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
      last_click_g = (mx, my)
      last_click_cv = (int(round(rx * fw)), int(round(ry * fh)))

      try:
        if self._vision_click:
          # prefer the normalized method if its available
          det = _get_detector()
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
          det = _get_detector()
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
      logger.info("GO!!!!\n")

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
                # Forward to detector (will draw crosshair + delta on next frame)
                try:
                  self._vision_click(fx, fy)
                except Exception as e:
                  logger.debug(f"on_click failed: {e}")
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
                  det = _get_detector()
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
                  det = _get_detector()
                  if det:
                    det.clear_click()
                logger.info("Cleared click-delta point (C).")
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

