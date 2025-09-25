import os
import sys
import glob
import hashlib
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np
import context

# ---------------- USER SETTINGS ----------------
DICT = cv2.aruco.DICT_4X4_1000
SQUARES_X = 5      # columns (number of squares)
SQUARES_Y = 7      # rows (number of squares)
SQUARE_LENGTH_M = 0.025  # 25 mm
MARKER_LENGTH_M = 0.015  # 15 mm

MIN_VALID_VIEWS = 15
FLAGS = cv2.CALIB_RATIONAL_MODEL

directory = context.get_context(os.path.abspath(__file__))
CALIB_IMAGES_GLOB = f"{directory}/utils/calibration_imgs/*.jpg"
OUT_FILE = f"{directory}/utils/calibration_imgs/cam_param.npz"
# ------------------------------------------------

logging.basicConfig(
  level=logging.INFO,
  format="%(levelname)s %(message)s"
)
LOG = logging.getLogger("charuco_calib")


@dataclass
class CharucoData:
  corners: List[np.ndarray]
  ids: List[np.ndarray]
  image_size: Tuple[int, int]  # (w, h)
  used_images: int


def _sha1_of_paths(paths: List[str]) -> str:
  h = hashlib.sha1()
  for p in paths:
    h.update(Path(p).read_bytes())
  return h.hexdigest()[:12]


def _build_detector(aruco_dict_id: int) -> Tuple[cv2.aruco.ArucoDetector, cv2.aruco.CharucoBoard]:
  aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
  dparams = cv2.aruco.DetectorParameters()
  dparams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

  dparams.cornerRefinementWinSize = 5
  dparams.cornerRefinementMaxIterations = 50
  dparams.cornerRefinementMinAccuracy = 0.01

  board = cv2.aruco.CharucoBoard((int(SQUARES_X), int(SQUARES_Y)), 
                                 float(SQUARE_LENGTH_M),
                                 float(MARKER_LENGTH_M),
                                 aruco_dict)
  return cv2.aruco.ArucoDetector(aruco_dict, dparams), board


def _collect_charuco_data(img_paths: List[str],
                          detector: cv2.aruco.ArucoDetector,
                          board: cv2.aruco.CharucoBoard) -> CharucoData:
  all_corners: List[np.ndarray] = []
  all_ids: List[np.ndarray] = []
  image_size: Optional[Tuple[int, int]] = None
  used = 0

  for path in img_paths:
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
      LOG.warning("Unreadable image skipped: %s", path)
      continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if image_size is None:
      image_size = gray.shape[1], gray.shape[0]  # (w, h)

    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
      continue

    # refine markers against the board
    cv2.aruco.refineDetectedMarkers(
      image=gray,
      board=board,
      detectedCorners=corners,
      detectedIds=ids,
      rejectedCorners=None
    )

    ok, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
      markerCorners=corners,
      markerIds=ids,
      image=gray,
      board=board
    )
    if ok and ch_ids is not None and len(ch_ids) > 8:
      all_corners.append(ch_corners)
      all_ids.append(ch_ids)
      used += 1

  if image_size is None:
    raise SystemExit("No readable images found.")

  return CharucoData(all_corners, all_ids, image_size, used)


def calibrate_charuco() -> None:
  img_paths = sorted(glob.glob(CALIB_IMAGES_GLOB))
  if not img_paths:
    raise SystemExit(f"No images found matching {CALIB_IMAGES_GLOB}")

  LOG.info("Found %d calibration images", len(img_paths))

  detector, board = _build_detector(DICT)
  data = _collect_charuco_data(img_paths, detector, board)

  if len(data.corners) < MIN_VALID_VIEWS:
    raise SystemExit(f"Not enough valid detections for calibration "
                     f"({len(data.corners)} < {MIN_VALID_VIEWS}). Capture more varied images.")

  LOG.info("Using %d valid views for calibration", len(data.corners))

  rms, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
    charucoCorners=data.corners,
    charucoIds=data.ids,
    board=board,
    imageSize=data.image_size,
    cameraMatrix=None,
    distCoeffs=None,
    flags=FLAGS
  )

  # Save rich metadata for reproducibility & runtime checks
  meta = {
    "camera_matrix": camera_matrix,
    "dist_coeffs": dist_coeffs,
    "image_width": int(data.image_size[0]),
    "image_height": int(data.image_size[1]),
    "rms": float(rms),
    "squares_x": int(SQUARES_X),
    "squares_y": int(SQUARES_Y),
    "square_len_m": float(SQUARE_LENGTH_M),
    "marker_len_m": float(MARKER_LENGTH_M),
    "dict": int(DICT),
    "flags": int(FLAGS),
    "num_views": int(len(data.corners)),
    "files_used": int(data.used_images),
    "dataset_sha1_12": _sha1_of_paths(img_paths),
  }

  Path(OUT_FILE).parent.mkdir(parents=True, exist_ok=True)
  np.savez(OUT_FILE, **meta)

  LOG.info("Saved %s", OUT_FILE)
  LOG.info("Resolution: %dx%d", meta["image_width"], meta["image_height"])
  LOG.info("RMS reprojection error: %.4f", meta["rms"])

if __name__ == "__main__":
  try:
    calibrate_charuco()
  except SystemExit as e:
    LOG.error(str(e))
    sys.exit(1)
  except Exception:
    LOG.exception("Unexpected error")
    sys.exit(2)
