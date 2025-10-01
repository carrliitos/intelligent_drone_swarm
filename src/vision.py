from pathlib import Path
import time
import sys
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

@contextmanager
def suppress_stderr():
  """
  Temporarily silence libjpeg MJPG warnings written to stderr during cap.read().
  Keep the scope as small as possible since this is process-wide.
  """
  fd = sys.stderr.fileno()
  saved = os.dup(fd)
  try:
    with open(os.devnull, "w") as devnull:
      os.dup2(devnull.fileno(), fd)
    yield
  finally:
    os.dup2(saved, fd)
    os.close(saved)

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

    # camera index
    cam_idx = helpers._to_int(camera, default=0)
    if cam_idx is None:
      cam_idx = 0
    self.camera_index = cam_idx

    # image settings (None means "do not force-set"; set only if not None)
    self.width = helpers._to_int(width, default=None)
    self.height = helpers._to_int(height, default=None)
    self.fps = helpers._to_int(fps, default=None)

    self.window_title = window_title or "ArUco Detector"
    self.draw_axes = bool(draw_axes)

    ids = helpers._to_ids(allowed_ids)
    self.allowed_ids = set(ids) if ids else None

    self.capture_cells = bool(capture_cells)

    # thresholds & counters
    self.min_brightness = helpers._to_float(min_brightness, default=2.0)
    self._fps_counter = helpers._to_int(fps_counter, default=0) or 0
    self._fps_display = helpers._to_float(fps_display, default=0.0) or 0.0

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
      self._overlay_grid(frame) # default to just drawing the grid and overlays and stuff
      return

    h, w = frame.shape[:2]
    step = max(1, self.grid_step_px)

    # Draw fills on an overlay
    overlay = frame.copy()
    for (row, col) in self._occupied_cells:
      x0, y0 = col * step, row * step
      x1, y1 = min(x0 + step - 1, w - 1), min(y0 + step - 1, h - 1)
      cv2.rectangle(overlay, (x0, y0), (x1, y1), self.occupied_color, thickness=cv2.FILLED)
    # Blend overaly to frame
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

    # Rule of thirds: https://web.cecs.pdx.edu/~fliu/papers/ism2011.pdf
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

  def open(self):
    """
    Open the camera and then do basic capture settings.
    """
    with self._singleton_open:
      if DetectorRT._is_open:
        logger.warning("DetectorRT.open() called but camera is already open; ignoring.")
        return

      def _try_open(width, height, fps, fourcc=None):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not cap.isOpened():
          return None

        if fourcc is not None:
          cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # low latency
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

        # Prefer AUTO exposure that V4L2 actually honors (0.75 commonly maps to AUTO)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        return cap

      attempts = [
        (self.width, self.height, self.fps, 'MJPG'), # requested (1280x720@30 MJPG)
        (1280, 720, 30, 'MJPG'),                     # UVC default profile
        (640, 480, 30, 'MJPG'),                      # lower-res MJPG (keeps JPEG decode path)
        (640, 480, 30, None),                        # fallback
        (640, 480, 30, 'YUYV'),                      # uncompressed fallback (full FPS at VGA)
      ]

      self.cap = None

      for (w, h, fps, fcc) in attempts:
        cap = _try_open(w, h, fps, fcc)
        if cap is None:
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
              break
          time.sleep(0.01)

        if self.cap is not None:
          break
        cap.release()

      if self.cap is None:
        logger.error("Could not open camera after multiple attempts.")
        raise RuntimeError("Camera open failed.")

      got_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
      got_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
      got_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
      fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
      fcc_text = "".join([chr((fourcc >> (8*i)) & 0xFF) for i in range(4)]).strip()
      logger.info(f"Camera opened at {got_w}x{got_h} @ {got_fps:.1f} FPS ({fcc_text or 'RAW'})")

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
    Release camera and detroy and open windows.
    """
    if self.cap is not None:
      with self._cap_lock:
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # AUTO
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        self.cap.release()
      self.cap = None
    cv2.destroyAllWindows()
    DetectorRT._is_open = False

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

    with self._cap_lock:
      with suppress_stderr():
        ok, frame = self.cap.read()
    if not ok or frame is None:
      logger.warning("Camera read failed; attempting one-shot reopen…")
      try:
        self.release()
        self.open()
        with self._cap_lock:
          with suppress_stderr():
            ok, frame = self.cap.read()
      except Exception as e:
        logger.error(f"Reopen failed: {e}")
        return None, {}
      if not ok or frame is None:
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

    cv2.putText(frame, 
                f"{self._fps_display:.1f} FPS", 
                (10, frame.shape[0]- 10), 
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

    self._overlay_grid(frame)
    self._mark_occupied(frame) # Highlight the occupied cells and then draw grid
    self.last_results = results

    with self._latest_lock:
      self._latest_frame = frame.copy()   # small cost; safe for readers
    self.last_results = results

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

def primary_target(results, frame_shape):
  """
  Return (cx, cy, area_px) of the first detected marker, or None.
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
