from pathlib import Path
import time
import sys
import logging
from typing import Optional, Tuple, Dict, Any, List, Union
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env")

import cv2
import numpy as np
import os
import threading
from contextlib import contextmanager

from utils import logger, context, helpers

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(
  logger_name=logger_name,
  log_file=f"{directory}/logs/{logger_file_name}.log",
  log_level=os.getenv("LOG_LEVEL")
)

def _logfmt(event: str, level: int = logging.INFO, **kv):
  """
  Log a single-line event in logfmt style: event=foo k=v k2="v v" 
  """
  parts = [f"event={event}"]
  for k, v in kv.items():
    if isinstance(v, str):
      if (" " in v) or ('"' in v):
        v = '"' + v.replace('"', '\\"') + '"'
    parts.append(f"{k}={v}")
  logger.log(level, " ".join(parts))

class _Timer:
  """
  Context manager to log an event's duration in ms.
  """
  def __init__(self, event: str, **kv):
    self.event = event
    self.kv = kv
  def __enter__(self):
    self.t0 = time.perf_counter()
    return self
  def __exit__(self, exc_type, exc, tb):
    dt_ms = int((time.perf_counter() - self.t0) * 1000)
    _logfmt(self.event, duration_ms=dt_ms, **self.kv)

class _RateLimiter:
  def __init__(self):
    self._last = {}
  def allow(self, key: str, every_sec: float = 5.0):
    now = time.time()
    last = self._last.get(key, 0.0)
    if (now - last) >= every_sec:
      self._last[key] = now
      return True
    return False

_rate = _RateLimiter()

# env-based tuning for sampled diagnostics and warning rate limits
LOG_IDS_EVERY = helpers._to_int(os.getenv("LOG_IDS_EVERY"), 60) or 60
LOG_FRAME_SAMPLE_EVERY = helpers._to_int(os.getenv("LOG_FRAME_SAMPLE_EVERY"), 60) or 60
LOG_WARN_RATELIMIT = helpers._to_float(os.getenv("LOG_WARN_RATELIMIT"), 3.0) or 3.0

@contextmanager
def suppress_stderr():
  """
  Temporarily silence libjpeg MJPG warnings written to stderr during cap.read().
  Keep the scope as small as possible since this is process-wide.
  """
  try:
    _logfmt("stderr_suppress_enter", level=logging.DEBUG)
    fd = sys.stderr.fileno()
    saved = os.dup(fd)
    try:
      with open(os.devnull, "w") as devnull:
        os.dup2(devnull.fileno(), fd)
      yield
    finally:
      os.dup2(saved, fd)
      os.close(saved)
  finally:
    _logfmt("stderr_suppress_exit", level=logging.DEBUG)

class DetectorRT:
  """
  Real-time ArUco detection and pose estimation.
  """

  _singleton_open = threading.Lock()
  _is_open = False

  _DICT_NAME_TO_ENUM = {
    "DICT_4X4_50":   cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":  cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":  cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
  }

  def __init__(
    self,
    dictionary: Optional[str] = None,
    camera: Optional[Union[int, str]] = None,
    width: Optional[Union[int, str]] = None,
    height: Optional[Union[int, str]] = None,
    fps: Optional[Union[int, str]] = None,
    calib_path: Optional[str] = None,
    marker_length_m: Optional[Union[float, str]] = None,
    window_title: Optional[str] = None,
    draw_axes: bool = True,
    allowed_ids: Optional[Union[List[int], List[str], str]] = None,
    # Draw grid
    draw_grid: bool = True,
    min_brightness: Optional[Union[int, float, str]] = None,
    grid_step_px: Optional[Union[int, str]] = None,
    grid_color: Optional[Union[str, Tuple[int, int, int], List[int]]] = None,
    grid_thickness: Optional[Union[int, str]] = None,
    draw_rule_of_thirds: bool = False,
    draw_crosshair: bool = False,
    capture_cells: bool = False,
    # State knobs (let caller pass)
    fps_counter: Optional[Union[int, str]] = None,
    fps_display: Optional[Union[int, float, str]] = None
  ):
    # dictionary
    raw_dict = dictionary or "4x4_1000"
    self.dictionary_name = helpers._normalize_dict_name(raw_dict)

    # camera source: support either int index or URL
    # - If camera looks like an int (e.g., "0"), use local webcam index
    # - Otherwise treat as a literal source
    self.camera_source_raw = camera
    self.camera_url: Optional[str] = None
    self.camera_index: Optional[int] = None
    try:
      if camera is None:
        self.camera_index = 0
      else:
        self.camera_index = int(str(camera))
    except (TypeError, ValueError):
      self.camera_index = None
      self.camera_url = str(camera) if camera else None

    # image settings (None means "do not force-set"; set only if not None)
    self.width = helpers._to_int(width, default=None)
    self.height = helpers._to_int(height, default=None)
    self.fps = helpers._to_int(fps, default=None)

    self.window_title = window_title or "ArUco Detector"
    self.draw_axes = bool(draw_axes)

    # Low-latency controls
    self.stream_backend = (os.getenv("STREAM_BACKEND") or "auto").lower()  # auto|ffmpeg|gstreamer
    self.low_latency = bool(int(os.getenv("STREAM_LOW_LATENCY", "1")))
    self.flush_grabs = helpers._to_int(os.getenv("STREAM_FLUSH_GRABS"), default=0) or 0

    # rate-limit heavy processing
    self.max_proc_fps = helpers._to_float(os.getenv("MAX_PROC_FPS"), default=0.0) or 0.0
    self._last_proc_ts = 0.0

    ids = helpers._to_ids(allowed_ids)
    self.allowed_ids = set(ids) if ids else None

    self.capture_cells = bool(capture_cells)

    # thresholds & counters
    self.min_brightness = helpers._to_float(min_brightness, default=2.0)
    self._fps_counter = helpers._to_int(fps_counter, default=0) or 0
    self._fps_display = helpers._to_float(fps_display, default=0.0) or 0.0

    # click-to-delta feature
    self._click_pt = None
    self._click_lock = threading.Lock()

    # normalize click (u, v) in [0,1] x [0,1]; if set, takes precedence
    self._click_norm = None

    # env toggles: 1=enabled, 0=disabled
    self._delta_enabled = bool(int(os.getenv("CLICK_DELTA_ENABLED", "1")))
    self._metric_enabled = bool(int(os.getenv("CLICK_DELTA_METRIC", "1")))

    # grid defaults
    self.draw_grid = bool(draw_grid)
    self.grid_step_px = helpers._to_int(grid_step_px, default=50) or 50
    self.grid_color = helpers._to_bgr_tuple(grid_color, default=(0, 255, 0))  # BGR
    self.grid_thickness = helpers._to_int(grid_thickness, default=1) or 1
    self.draw_rule_of_thirds = bool(draw_rule_of_thirds)
    self.draw_crosshair = bool(draw_crosshair)

    # Pose estimation
    self.marker_length_m = helpers._to_float(marker_length_m, default=None)
    self.camera_matrix = None
    self.dist_coeffs = None
    self._do_pose = False
    if calib_path is not None and self.marker_length_m is not None:
      self.camera_matrix, self.dist_coeffs = helpers._load_calibration(calib_path)
      self._do_pose = True

    # cache last full detection result so control can tolerate pose gaps
    self._last_results = None 

    # OpenCV ArUco Setup
    if self.dictionary_name not in self._DICT_NAME_TO_ENUM:
      raise ValueError(f"Unsupported dictionary: '{raw_dict}'. "
                       f"Choose one of: {list(self._DICT_NAME_TO_ENUM.keys())}")
    self.dict_enum = self._DICT_NAME_TO_ENUM[self.dictionary_name]
    self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dict_enum)
    self.params = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)

    # State
    self.cap = None
    self._fps_prev = time.time()
    self.last_results: Dict[str, Any] = {}

    # Occupied-grid overlay
    self.highlight_occupied: bool = True
    self.occupied_color: Tuple[int, int, int] = (40, 40, 200)  # BGR
    self.occupied_alpha: float = 0.75
    self._occupied_cells = set()

    # Locks
    self._cap_lock = threading.Lock()
    self._latest_frame = None
    self._latest_lock = threading.Lock()

    # Session stats
    self.stats = {
      "frames": 0,
      "detect_frames": 0,
      "reopen_count": 0,
      "read_failures": 0,
      "ema_fps": 0.0
    }
    self._ema_alpha = 0.2  # EMA smoothing for FPS

    # Init echo
    _logfmt(
      "detector_init",
      dictionary=self.dictionary_name,
      camera_index=self.camera_index,
      req_width=self.width, req_height=self.height, req_fps=self.fps,
      window_title=self.window_title,
      draw_axes=self.draw_axes,
      allowed_ids=("none" if self.allowed_ids is None else len(self.allowed_ids)),
      draw_grid=self.draw_grid, grid_step=self.grid_step_px,
      rule_of_thirds=self.draw_rule_of_thirds, crosshair=self.draw_crosshair,
      min_brightness=self.min_brightness,
      do_pose=self._do_pose, marker_len_m=self.marker_length_m
    )

  def _point_to_cell(self, x: float, y: float, w: int, h: int):
    """
    Maps the pixel (x, y) to (row, col) in the grid.
    """
    step = max(1, self.grid_step_px)
    col = int(np.clip(x, 0, w - 1)) // step
    row = int(np.clip(y, 0, h - 1)) // step
    return (row, col)

  def _mark_occupied(self, frame: np.ndarray):
    """
    Alpha-fill currently occupied cell and then draw the grid lines on top of it.
    """
    if not (self.draw_grid and self.highlight_occupied and self._occupied_cells):
      self._overlay_grid(frame)  # default to just drawing the grid and overlays
      return

    h, w = frame.shape[:2]
    step = max(1, self.grid_step_px)

    # Draw fills on an overlay
    overlay = frame.copy()
    for (row, col) in self._occupied_cells:
      x0, y0 = col * step, row * step
      x1, y1 = min(x0 + step - 1, w - 1), min(y0 + step - 1, h - 1)
      cv2.rectangle(overlay, (x0, y0), (x1, y1), self.occupied_color, thickness=cv2.FILLED)
    # Blend overlay to frame
    cv2.addWeighted(overlay, self.occupied_alpha, frame, 1.0 - self.occupied_alpha, 0, frame)

    self._overlay_grid(frame)

  def _overlay_grid(self, frame: np.ndarray):
    h, w = frame.shape[:2]

    if self.draw_grid and self.grid_step_px > 0:
      step = self.grid_step_px
      # vertical lines
      for x in range(step, w, step):
        cv2.line(frame, (x, 0), (x, h), self.grid_color, self.grid_thickness, cv2.LINE_AA)
      # Horizontal lines
      for y in range(step, h, step):
        cv2.line(frame, (0, y), (w, y), self.grid_color, self.grid_thickness, cv2.LINE_AA)

    # Rule of thirds
    if self.draw_rule_of_thirds:
      x1, x2 = w // 3, (2 * w) // 3
      y1, y2 = h // 3, (2 * h) // 3
      cv2.line(frame, (x1, 0), (x1, h), self.grid_color, self.grid_thickness, cv2.LINE_AA)
      cv2.line(frame, (x2, 0), (x2, h), self.grid_color, self.grid_thickness, cv2.LINE_AA)
      cv2.line(frame, (0, y1), (w, y1), self.grid_color, self.grid_thickness, cv2.LINE_AA)
      cv2.line(frame, (0, y2), (w, y2), self.grid_color, self.grid_thickness, cv2.LINE_AA)

    if self.draw_crosshair:
      cx, cy = w // 2, h // 2
      size = min(w, h) // 20
      cv2.line(frame, (cx - size, cy), (cx + size, cy), self.grid_color, self.grid_thickness + 1, cv2.LINE_AA)
      cv2.line(frame, (cx, cy - size), (cx, cy + size), self.grid_color, self.grid_thickness + 1, cv2.LINE_AA)

  def latest_pose(self):
    """
    Return (K, rvec, tvec) for the most recent pose, or None if unavailable.
    """
    res = getattr(self, "_last_results", None) or {}
    rvecs = res.get("rvecs")
    tvecs = res.get("tvecs")

    if rvecs is None or tvecs is None or len(rvecs) == 0:
      return None

    # Use first detected marker’s pose (matches your other helpers)
    K = self.camera_matrix
    rvec = np.asarray(rvecs[0]).reshape(3, 1)
    tvec = np.asarray(tvecs[0]).reshape(3, 1)

    if K is None:
      return None

    return (K, rvec, tvec)

  def set_click_point(self, x: int, y: int):
    with self._click_lock:
      self._click_pt = (int(x), int(y))
      self._click_norm = None  # pixel click overrides normalized for this session

  def clear_click(self):
    with self._click_lock:
      self._click_pt = None
      self._click_norm = None

  def set_click_point_normalized(self, u: float, v: float):
    """
    Store a normlized click coordinate (u,v) in [0,1], independent of frame size.
    The actual pixel  is resolved at overlay time using the current frame wxh.
    """
    u = float(max(0.0, min(1.0, u)))
    v = float(max(0.0, min(1.0, v)))
    with self._click_lock:
      self._click_norm = (u, v)

  def toggle_delta(self, enabled: Optional[bool] = None):
    if enabled is None:
      self._delta_enabled = not self._delta_enabled
    else:
      self._delta_enabled = bool(enabled)

  def register_mouse_callback(self):
    """
    Register an OpenCV mouse callback on this detector's window title.
    Note: The caller must ensure a window exists with this exact title.
    """
    try:
      cv2.setMouseCallback(self.window_title, self._on_mouse)
    except Exception as e:
      _logfmt("mouse_callback_register_error", level=logging.WARNING, err=str(e))

  def _on_mouse(self, event, x, y, flags, userdata):
    if event == cv2.EVENT_LBUTTONDOWN:
      self.set_click_point(x, y)
      _logfmt("click_capture", x=x, y=y)

  def _nearest_marker_to_point(self, corners, ids, pt):
    if corners is None or ids is None or len(ids) == 0 or pt is None:
      return None
    # Build list of allowed markers
    pts = []
    xs, ys = pt
    for i, c in enumerate(corners):
      if c is None:
        continue
      mid = int(ids[i][0]) if isinstance(ids[i], (list, np.ndarray)) else int(ids[i])
      if self.allowed_ids is not None and mid not in self.allowed_ids:
        continue
      center = np.mean(c[0], axis=0)  # (x,y)
      dx = float(xs - center[0])
      dy = float(ys - center[1])
      dist2 = dx*dx + dy*dy
      pts.append((dist2, i, mid, float(center[0]), float(center[1])))
    if not pts:
      return None
    pts.sort(key=lambda t: t[0])
    _, idx, mid, cx, cy = pts[0]
    return idx, mid, cx, cy

  def _overlay_click_delta(self, frame: np.ndarray, results: Dict[str, Any]):
    if not self._delta_enabled:
      return

    # REsolve click: normalized will take precedence, else use pixel if present
    with self._click_lock:
      click_px = None
      if self._click_norm is not None:
        u, v = self._click_norm
        h, w = frame.shape[:2]
        x = int(round(u * (w - 1)))
        y = int(round(v * (h - 1)))
        click_px = (x, y)
      elif self._click_pt is not None:
        click_px = tuple(self._click_pt)
      else:
        click_px = None

    if click_px is None:
      return

    click = click_px
    click_x, click_y = click

    corners = results.get("corners")
    ids = results.get("ids")
    rvecs = results.get("rvecs")
    tvecs = results.get("tvecs")

    chosen = self._nearest_marker_to_point(corners, ids, click)
    if chosen is None:
      # No target; draw click but no delta
      cv2.drawMarker(frame, (click_x, click_y), (200, 200, 255), cv2.MARKER_CROSS, 12, 2)
      cv2.putText(frame, "No marker", (click_x+8, click_y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,255), 1, cv2.LINE_AA)
      return

    idx, mid, cx, cy = chosen
    dx_px = int(round(click_x - cx))
    dy_px = int(round(click_y - cy))

    # Draw visuals
    cv2.drawMarker(frame, (click_x, click_y), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
    cv2.circle(frame, (int(round(cx)), int(round(cy))), 4, (0, 255, 255), -1, cv2.LINE_AA)
    cv2.line(frame, (int(round(cx)), int(round(cy))), (click_x, click_y), (255, 255, 255), 1, cv2.LINE_AA)

    # Build label
    label = f"ID {mid}  dpx=({dx_px:d},{dy_px:d})"
    metric_txt = ""
    if self._metric_enabled and self._do_pose and rvecs is not None and tvecs is not None:
      try:
        # camera intrinsics
        K = self.camera_matrix
        D = self.dist_coeffs if self.dist_coeffs is not None else np.zeros((5,1))

        # Marker pose for chosen idx (marker -> camera)
        rvec = rvecs[idx].reshape(3,1)
        tvec = tvecs[idx].reshape(3,1)
        R,_ = cv2.Rodrigues(rvec)

        # Plane in camera frame: n·(p - p0) = 0, with p0 = tvec, n = R*[0,0,1]
        n = R @ np.array([[0.0],[0.0],[1.0]])
        p0 = tvec

        # Ray from camera origin through pixel
        uv = np.array([[[float(click_x), float(click_y)]]], dtype=np.float32)
        undist = cv2.undistortPoints(uv, K, D, P=K)
        xn = undist[0,0,0]
        yn = undist[0,0,1]
        ray_dir = np.array([[xn],[yn],[1.0]], dtype=np.float64)
        ray_dir = ray_dir / np.linalg.norm(ray_dir)

        # Per Numpy 1.25+: explicitly extract scalars from 1x1 arrays
        denom = float((n.T @ ray_dir).item())
        numer = float((n.T @ p0).item())
        # Per Numpy 1.25+: near-parallel ray/plane or non-finite values
        if np.isfinite(denom) and abs(denom) > 1e-6 and np.isfinite(numer):
          s = numer / denom
          Pc = s * ray_dir  # intersection in camera frame
          # Convert to marker frame: X_m = R^T (Pc - tvec)
          Xm = R.T @ (Pc - tvec)
          dX = float(Xm[0,0])
          dY = float(Xm[1,0])
          metric_txt = f"  dM=({dX:.3f}m,{dY:.3f}m)"
        else:
          metric_txt = "  dM=(n/a)"
      except Exception as e:
        metric_txt = "  dM=(n/a)"
        _logfmt("metric_delta_error", level=logging.DEBUG, err=str(e))

    cv2.putText(frame, label + metric_txt, (click_x + 8, click_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2, cv2.LINE_AA)
    _logfmt("click_delta", id=int(mid), x=click_x, y=click_y, cx=f"{cx:.1f}", cy=f"{cy:.1f}", dx_px=dx_px, dy_px=dy_px, metric=metric_txt.strip())

  def open(self):
    """
    Open the camera and then do basic capture settings.
    """
    with self._singleton_open:
      if DetectorRT._is_open:
        logger.warning("DetectorRT.open() called but camera is already open; ignoring.")
        return

      def _try_open_index(idx, width, height, fps, fourcc=None):
        cap = cv2.VideoCapture(int(idx))
        if not cap.isOpened():
          return None

        try:
          if fourcc is not None:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

          if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))

          if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

          if fps:
            cap.set(cv2.CAP_PROP_FPS, int(fps))

          cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # reduce latency for webcams too
          cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        except Exception:
          pass
        return cap

      def _try_open_url(url):
        # For IP streams (DroidCam), we avoid forcing width/height/fps—
        # the producer decides. Use a small buffer to cut latency.
        backend = None
        if self.stream_backend == "ffmpeg":
          backend = cv2.CAP_FFMPEG
        elif self.stream_backend == "gstreamer":
          # GStreamer pipeline: https://gstreamer.freedesktop.org/documentation/tutorials/basic/dynamic-pipelines.html?gi-language=c
          pipeline = (f"souphttpsrc location={url} is-live=true ! "
                       "jpegdec ! videoconvert ! "
                       "appsink drop=true max-buffers=1 sync=false")
          cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
          return cap if cap.isOpened() else None

        cap = cv2.VideoCapture(url, backend) if backend is not None else cv2.VideoCapture(url)

        if not cap.isOpened():
          return None

        try:
          if self.low_latency:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # tiny queue
        except Exception:
          pass
        return cap

      if self.camera_url:
        _logfmt("camera_open_begin", source="url", url=self.camera_url)
      else:
        _logfmt("camera_open_begin", source="index", camera_index=self.camera_index)

      if self.camera_url:
        self.cap = _try_open_url(self.camera_url)
        if self.cap is None:
          _logfmt("camera_open_failed", level=logging.ERROR, reason="url_open_failed")
          raise RuntimeError("Camera open failed (URL).")
      else:
        attempts = [
          (self.width, self.height, self.fps, 'MJPG'), # requested (e.g., 1280x720@30 MJPG)
          (1280, 720, 30, 'MJPG'),                     # UVC default profile
          (640, 480, 30, 'MJPG'),                      # lower-res MJPG (keeps JPEG decode path)
          (640, 480, 30, None),                        # fallback
          (640, 480, 30, 'YUYV'),                      # uncompressed fallback (full FPS at VGA)
        ]

        self.cap = None
        attempt_no = 0

        for (w, h, fps, fcc) in attempts:
          attempt_no += 1
          _logfmt("camera_try", attempt=attempt_no, width=w, height=h, fps=fps, fourcc=(fcc or "RAW"))
          cap = _try_open_index(self.camera_index or 0, w, h, fps, fcc)
          if cap is None:
            _logfmt("camera_try_failed_open", attempt=attempt_no, reason="cap_not_open", level=logging.WARNING)
            continue

          # Warm-up + “not black” check
          ok_count, t0 = 0, time.time()
          while time.time() - t0 < 2.0:
            with suppress_stderr():
              ok, frm = cap.read()
            if ok and frm is not None and frm.mean() > self.min_brightness:
              ok_count += 1
              if ok_count >= 3:
                self.cap = cap
                self.width, self.height, self.fps = w, h, fps
                _logfmt("camera_try_success", attempt=attempt_no, warmup_frames=ok_count)
                break
            time.sleep(0.01)

          if self.cap is not None:
            break

          _logfmt("camera_try_failed_warmup", attempt=attempt_no, warmup_ok=ok_count, level=logging.WARNING)
          cap.release()

        if self.cap is None:
          logger.error("Could not open camera after multiple attempts.")
          _logfmt("camera_open_failed", level=logging.ERROR)
          raise RuntimeError("Camera open failed.")

      got_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
      got_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
      got_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
      fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
      fcc_text = "".join([chr((fourcc >> (8*i)) & 0xFF) for i in range(4)]).strip()
      logger.info(f"Camera opened at {got_w}x{got_h} @ {got_fps:.1f} FPS ({fcc_text or 'RAW'})")
      _logfmt("camera_open_ok", got_w=got_w, got_h=got_h, got_fps=got_fps, fourcc=fcc_text or "RAW")

      DetectorRT._is_open = True

  def latest(self):
    """
    Return (frame, results) of the most recent processed frame without 
    reading the camera.
    """
    with self._latest_lock:
      frame = None if self._latest_frame is None else self._latest_frame.copy()
    return frame, self.last_results

  def release(self):
    """
    Release camera and destroy any open windows, then log a session summary.
    """
    if self.cap is not None:
      with self._cap_lock:
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # AUTO
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        self.cap.release()
      self.cap = None
    cv2.destroyAllWindows()
    DetectorRT._is_open = False

    # Session summary
    frames = max(1, self.stats["frames"])
    det_rate = self.stats["detect_frames"] / frames
    _logfmt(
      "session_summary",
      frames=self.stats["frames"],
      detect_frames=self.stats["detect_frames"],
      detect_rate=f"{det_rate:.3f}",
      read_failures=self.stats["read_failures"],
      reopen_count=self.stats["reopen_count"],
      avg_fps=f"{self.stats['ema_fps']:.1f}"
    )

  def read(self):
    """
    Grab a frame from the camera, run detection (and pose if possible),
    draw overlays, and return (annotated_frame, results_dict).

    results_dict keys:
      - 'ids': np.ndarray shape (N,1) or None
      - 'corners': list of N arrays (1,4,2) or []
      - 'rvecs': np.ndarray (N,1,3) if pose, else None
      - 'tvecs': np.ndarray (N,1,3) if pose, else None
      - 'dist_m': list of N floats (Euclidean distances) if pose, else []
      - 'fps': float (smoothed)
    """
    if self.cap is None:
      logger.error("Call open() before read().")
      raise RuntimeError("Call open() before read().")

    if self.max_proc_fps > 0.0:
      now_ts = time.time()
      min_dt = 1.0 / max(1e-6, float(self.max_proc_fps))

      if (now_ts - self._last_proc_ts) < min_dt:
        with self._latest_lock:
          frame = None if self._latest_frame is None else self._latest_frame.copy()
        return (frame, self.last_results if frame is not None else {})

      self._last_proc_ts = now_ts

    # Always try to read the freshest frame; flush a few with grab() then retrieve once.
    with self._cap_lock:
      with suppress_stderr():
        if self.flush_grabs > 0:
          # Drain N stale frames (cheap, decode happens on retrieve())
          for _ in range(self.flush_grabs):
            self.cap.grab()

          ok = self.cap.grab()
          frame = None

          if ok:
            ok, frame = self.cap.retrieve()
        else:
          ok, frame = self.cap.read()

    if not ok or frame is None:
      self.stats["read_failures"] += 1
      if _rate.allow("cam_read_fail", every_sec=LOG_WARN_RATELIMIT):
        _logfmt("camera_read_fail", level=logging.WARNING)

      try:
        self.release()
        self.stats["reopen_count"] += 1
        _logfmt("camera_reopen", count=self.stats["reopen_count"])
        self.open()
        with self._cap_lock:
          with suppress_stderr():
            ok, frame = self.cap.read()
      except Exception as e:
        logger.exception("Reopen failed")  # includes traceback
        _logfmt("camera_reopen_failed", level=logging.ERROR, error=type(e).__name__)
        return None, {}
      if not ok or frame is None:
        if _rate.allow("cam_read_fail_after_reopen", every_sec=max(5.0, LOG_WARN_RATELIMIT)):
          _logfmt("camera_read_fail_after_reopen", level=logging.ERROR)
        return None, {}

    corners, ids, _ = self.detector.detectMarkers(frame)
    corners, ids = helpers._filter_known(corners, ids, self.allowed_ids)
    rvecs = tvecs = None
    dists: List[float] = []
    if not self.capture_cells:
      # Reset the occupied cells per frame
      self._occupied_cells = set()

    if ids is not None and len(ids) > 0:
      # Draw the detected boundaries and their IDs
      cv2.aruco.drawDetectedMarkers(frame, corners, ids)

      # Compute centers and map to grid cells
      h, w = frame.shape[:2]
      for corner in corners:
        # corner shape: (1, 4, 2) -> take the mean over the 4 corner points
        center = np.mean(corner[0], axis=0) # x, y
        cell = self._point_to_cell(center[0], center[1], w, h)
        self._occupied_cells.add(cell)

      if self._do_pose:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 
                                                              self.marker_length_m, 
                                                              self.camera_matrix, 
                                                              self.dist_coeffs)
        if self.draw_axes:
          for rvec, tvec in zip(rvecs, tvecs):
            cv2.drawFrameAxes(frame, 
                              self.camera_matrix, 
                              self.dist_coeffs,
                              rvec,
                              tvec,
                              (self.marker_length_m * 0.5))
            dists.append(float(np.linalg.norm(tvec)))
        else:
          dists = [float(np.linalg.norm(t)) for t in tvecs]

    # FPS overlay
    self._fps_counter += 1
    now = time.time()
    if now - self._fps_prev >= 0.5:
      self._fps_display = self._fps_counter / (now - self._fps_prev)
      self._fps_prev = now
      self._fps_counter = 0

    self.stats["frames"] += 1
    if ids is not None and len(ids) > 0:
      self.stats["detect_frames"] += 1

    if self._fps_display > 0:
      if self.stats["ema_fps"] == 0.0:
        self.stats["ema_fps"] = float(self._fps_display)
      else:
        self.stats["ema_fps"] = (
          0.2 * float(self._fps_display) + 0.8 * self.stats["ema_fps"]
        )

    # Sampled diagnostics
    if (self.stats["frames"] % LOG_FRAME_SAMPLE_EVERY) == 0:
      bright = float(frame.mean()) if frame is not None else -1.0
      id_list = [] if ids is None else list(map(int, ids.flatten().tolist()))
      _logfmt(
        "frame_sample",
        level=logging.DEBUG,
        fps=f"{self._fps_display:.1f}",
        mean_brightness=f"{bright:.2f}",
        ids=len(id_list),
        ids_list=",".join(map(str, id_list)) if id_list else "-"
      )

    # FPS overlay on frame
    cv2.putText(frame,
                f"{self._fps_display:.1f} FPS",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA)

    results = {
      "ids": ids,
      "corners": corners if corners is not None else {},
      "rvecs": rvecs,
      "tvecs": tvecs,
      "dist_m": dists,
      "fps": self._fps_display
    }

    # Single-tag pose snapshot
    if self._do_pose and rvecs is not None and len(rvecs) > 0:
      results["K"] = self.camera_matrix
      results["rvec"] = rvecs[0].reshape(3, 1)
      results["tvec"] = tvecs[0].reshape(3, 1)

    # Overlays
    self._overlay_grid(frame)
    self._overlay_click_delta(frame, results)
    self._mark_occupied(frame) # Highlight the occupied cells and then draw grid

    # Update public + cached results (pose snapshot included above if available)
    self.last_results = results
    self._last_results = results

    with self._latest_lock:
      self._latest_frame = frame.copy()   # small cost; safe for readers

    # Occasional detection summary (every LOG_IDS_EVERY frames)
    if (self.stats["frames"] % LOG_IDS_EVERY) == 0:
      _logfmt("detect_summary",
              n_ids=int(len(ids)) if (ids is not None) else 0,
              do_pose=self._do_pose,
              min_dist_m=(min(dists) if dists else -1),
              max_dist_m=(max(dists) if dists else -1))

    return frame, results

  def process_frame(self, frame: np.ndarray):
    """
    Run detection (and pose) on a provided frame.
    Returns results dict (same schema as read()) and modifies the frame 
    in-place with overlays.
    """
    corners, ids, _ = self.detector.detectMarkers(frame)
    corners, ids = helpers._filter_known(corners, ids, self.allowed_ids)
    rvecs = tvecs = None
    dists: List[float] = []
    if not self.capture_cells:
      # Reset the occupied cells per frame
      self._occupied_cells = set()

    if ids is not None and len(ids) > 0:
      cv2.aruco.drawDetectedMarkers(frame, corners, ids)
      # centers -> occupied grid cells
      h, w = frame.shape[:2]
      for corner in corners:
        center = np.mean(corner[0], axis=0)
        cell = self._point_to_cell(center[0], center[1], w, h)
        self._occupied_cells.add(cell)

      if self._do_pose:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 
                                                              self.marker_length_m, 
                                                              self.camera_matrix, 
                                                              self.dist_coeffs)
        if self.draw_axes:
          for rvec, tvec in zip(rvecs, tvecs):
            cv2.drawFrameAxes(frame, 
                              self.camera_matrix, 
                              self.dist_coeffs, 
                              rvec, 
                              tvec, 
                              (self.marker_length_m * 0.5))
            dists.append(float(np.linalg.norm(tvec)))
        else:
          dists = [float(np.linalg.norm(t)) for t in tvecs]

    self._overlay_grid(frame)
    self._mark_occupied(frame)

    return {
      "ids": ids,
      "corners": corners if corners is not None else [],
      "rvecs": rvecs,
      "tvecs": tvecs,
      "dist_m": dists,
    }

  def pixel_to_marker_xy(self, x: int, y: int, results: Optional[Dict[str, Any]] = None) -> Optional[Tuple[int, float, float]]:
    """
    Project pixel (x,y) to the Z=0 plane of the nearest detected marker and
    return (marker_id, X, Y) in that marker's local frame. Requires pose.
    Returns None if intrinsics/pose are unavailable or no suitable marker.
    """
    if self.camera_matrix is None:
      return None

    res = results if results is not None else self.last_results

    if not res:
      return None

    corners = res.get("corners")
    ids = res.get("ids")
    rvecs = res.get("rvecs")
    tvecs = res.get("tvecs")

    if corners is None or ids is None or rvecs is None or tvecs is None:
      return None

    chosen = self._nearest_marker_to_point(corners, ids, (x, y))

    if chosen is None:
      return None

    idx, mid, cx, cy = chosen

    try:
      # intrinsics
      K = self.camera_matrix
      D = self.dist_coeffs if self.dist_coeffs is not None else np.zeros((5,1))

      # marker pose (marker -> camera)
      rvec = rvecs[idx].reshape(3,1)
      tvec = tvecs[idx].reshape(3,1)
      R,_ = cv2.Rodrigues(rvec)
      n = R @ np.array([[0.0],[0.0],[1.0]])  # marker plane normal in camera frame
      p0 = tvec                              # a point on the plane (marker origin)

      uv = np.array([[[float(x), float(y)]]], dtype=np.float32)
      und = cv2.undistortPoints(uv, K, D, P=K)
      ray = np.array([[und[0,0,0]],[und[0,0,1]],[1.0]], dtype=np.float64)
      ray = ray / np.linalg.norm(ray)
      denom = float((n.T @ ray).item())
      numer = float((n.T @ p0).item())

      if not (np.isfinite(denom) and abs(denom) > 1e-6 and np.isfinite(numer)):
        return None

      s = numer / denom
      Pc = s * ray           # intersection in camera frame
      Xm = R.T @ (Pc - tvec) # to marker frame

      return int(mid), float(Xm[0,0]), float(Xm[1,0])
    except Exception as e:
      logger.error(f"No pixel_to_marker_xy: {e}")
      return None

def primary_target(results, frame_shape):
  """
  Return (cx, cy, area_px, W, H) of the first detected marker, or None.
  """
  corners = results.get("corners")
  if corners is None or len(corners) == 0:
    return None
  # corners[i]: shape (1, 4, 2); take mean for centroid & polygon area
  c = corners[0][0]             # (4,2)
  cx, cy = float(c[:,0].mean()), float(c[:,1].mean())
  # approximate area by contour area (pixel^2)
  area = float(cv2.contourArea(c.astype(np.float32)))
  H, W = frame_shape[:2]
  return (cx, cy, area, W, H)
