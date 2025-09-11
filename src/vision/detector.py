from pathlib import Path
import time
import sys
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np

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
    pass

  def release(self):
    pass

  def read(self):
    pass

  def process_frame(self, frame: np.ndarray):
    pass

  @staticmethod
  def _load_calibration(npz_path: str):
    calib_p = Path(npz_path)
    if not calib_p.exists():
      raise FileNotFoundError(f"Calibration file not found: {calib_p}")
    data = np.load(str(calib_p))
    return data["camera_matrix"], data["dist_coeffs"]
