from pathlib import Path
import time
import sys
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np

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
    width: int = 1280,
    height: int = 720,
    fps: int = 60,
    calib_path: Optional[str] = None,
    marker_length_m: Optional[str] = None,
    window_title: str = "Intelligent Drone Swarm",
    draw_axes: bool = True
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

  def open(self):
    """
    Open the camera and then do basic capture settings.
    """
    self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_ANY)
    if not self.cap.isOpened():
      logger.error("Could not open camera.")
      raise RuntimeError("Camera open failed.")

    # idk if this actuall does anything -- i just saw this on some tutorial
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)

  def release(self):
    """
    Release camera and detroy and open windows.
    """
    if self.cap is not None:
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

    corner, ids, _ = self.detector.detectMarkers(frame)
    rvecs = tvecs = None
    dists: List[float] = []

    if ids is not None and len(ids) > 0:
      # Draw the detected boundaries and their IDs
      cv2.aruco.drawDetectedMarkers(frame, corner, ids)

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

  def process_frame(self, frame: np.ndarray):
    pass

  @staticmethod
  def _load_calibration(npz_path: str):
    calib_p = Path(npz_path)
    if not calib_p.exists():
      raise FileNotFoundError(f"Calibration file not found: {calib_p}")
    data = np.load(str(calib_p))
    return data["camera_matrix"], data["dist_coeffs"]
