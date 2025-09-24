from pathlib import Path
import time
import sys
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np
import os

from utils import logger, context

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")

class DetectorRT:
  """
  Real-time ArUco detection and pose estimation (**CURRENTLY OPTIONAL**).
  """

  _DICT_NAME_TO_ENUM = {
    "4x4_50":   cv2.aruco.DICT_4X4_50,
    "4x4_100":  cv2.aruco.DICT_4X4_100,
    "4x4_250":  cv2.aruco.DICT_4X4_250,
    "4x4_1000": cv2.aruco.DICT_4X4_1000
  }

  def __init__(
    self,
    dictionary: str = "4x4_1000",
    camera: int = 2, # USB-connected camera
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    calib_path: Optional[str] = None,
    marker_length_m: Optional[float] = None,
    window_title: str = "Intelligent Drone Swarm",
    draw_axes: bool = True,
    # Draw grid
    draw_grid: bool = True,
    grid_step_px: int = 40,
    grid_color: Tuple[int, int, int] = (60, 220, 60),
    grid_thickness: int = 1,
    draw_rule_of_thirds: bool = False,
    draw_crosshair: bool = False
  ):
    self.dictionary_name = dictionary
    if dictionary not in self._DICT_NAME_TO_ENUM:
      raise ValueError(f"Unsupported dictionary: '{dictionary}'. "
                       f"Choose one of: {list(self._DICT_NAME_TO_ENUM.keys())}")

    self.dict_enum = self._DICT_NAME_TO_ENUM[dictionary]
    self.camera_index = camera
    self.width = width
    self.height = height
    self.fps = fps
    self.window_title = window_title
    self.draw_axes = draw_axes

    # Pose estimation stuff (**CURRENTLY OPTIONAL: Still needs calibration file.**)
    self.marker_length_m = marker_length_m
    self.camera_matrix = None
    self.dist_coeffs = None
    self._do_pose = False
    if calib_path is not None and marker_length_m is not None:
      self.camera_matrix, self.dist_coeffs = self._load_calibration(calib_path)
      self._do_pose = True

    # OpenCV ArUco
    self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dict_enum)
    self.params = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)

    # State
    self.cap = None
    self._fps_prev = time.time()
    self._fps_counter = 0
    self._fps_display = 0.0
    self.last_results: Dict[str, Any] = {}

    self.draw_grid = draw_grid
    self.grid_step_px = grid_step_px
    self.grid_color = grid_color
    self.grid_thickness = grid_thickness
    self.draw_rule_of_thirds = draw_rule_of_thirds
    self.draw_crosshair = draw_crosshair

    # Occupy grid for the detected ArUco marker
    self.highlight_occupied: bool = True
    self.occupied_color: Tuple[int, int, int] = (40, 40, 200) # BGR (red-ish(?))
    self.occupied_alpha: float = 0.75                         # 0..1 fill opacity
    self._occupied_cells = set()                              # A set of {(row, col), ...}

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
    self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
    if not self.cap.isOpened():
      logger.error("Could not open camera.")
      raise RuntimeError("Camera open failed.")

    ### IMPORTANT: Specific to C920 at 720p/1080p ###
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # Request resolution/FPS
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Reduce Latency
    self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1) # Ensure RGB conversion

    # Hard toggle exposure to known-good AUTO state
    # V4L2 expects '3' for auto. Some OpenCV builds also accept 0.75
    self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual (kick)
    self.cap.set(cv2.CAP_PROP_EXPOSURE, -4) # safe-ish dummy (ignored in AUTO)
    self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # AUTO (V4L2)
    self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)

    # Warm-up: grab a few frames so auto-exposure converges
    for _ in range(10):
      ok, _ = self.cap.read()
      if not ok:
        break

    # Sanity checks
    got_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
    logger.info(f"Camera opened at {got_w}x{got_h} @ {got_fps:.1f} FPS (MJPG)")

  def release(self):
    """
    Release camera and detroy and open windows.
    """
    if self.cap is not None:
      # Put camera back to AUTO so the next run doesn’t start “dark”
      self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # AUTO
      self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
      self.cap.release()
      self.cap = None
    cv2.destroyAllWindows()

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

    ok, frame = self.cap.read()
    if not ok or frame is None:
      return None, {}

    corners, ids, _ = self.detector.detectMarkers(frame)
    rvecs = tvecs = None
    dists: List[float] = []
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

    return frame, results

  def process_frame(self, frame: np.ndarray):
    """
    Run detection (and pose) on a provided frame.
    Returns results dict (same schema as read()) and modifies the frame 
    in-place with overlays.
    """
    corners, ids, _ = self.detector.detectMarkers(frame)
    rvecs = tvecs = None
    dists: List[float] = []
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

  @staticmethod
  def _load_calibration(npz_path: str):
    calib_p = Path(npz_path)
    if not calib_p.exists():
      raise FileNotFoundError(f"Calibration file not found: {calib_p}")
    data = np.load(str(calib_p))
    return data["camera_matrix"], data["dist_coeffs"]

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
