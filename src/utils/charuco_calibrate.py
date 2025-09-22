import glob
import cv2
import numpy as np
from pathlib import Path
import os
import context

directory = context.get_context(os.path.abspath(__file__))

# ---------------- USER SETTINGS ----------------
# Match these to how you printed your board
DICT = cv2.aruco.DICT_4X4_1000
SQUARES_X = 5            # number of chessboard squares along X (columns)
SQUARES_Y = 7            # number of chessboard squares along Y (rows)
SQUARE_LENGTH_M = 0.03   # square length in meters (e.g., 0.03 = 30 mm)
MARKER_LENGTH_M = 0.015  # marker length in meters (e.g., 0.015 = 15 mm)

CALIB_IMAGES = f"{directory}/utils/calibration_imgs/*.jpg"
OUT_FILE = f"{directory}/utils/calibration_imgs/cam_param.npz"
# ------------------------------------------------


def main():
  aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
  board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH_M, MARKER_LENGTH_M, aruco_dict)

  img_paths = sorted(glob.glob(CALIB_IMAGES))
  if not img_paths:
    raise SystemExit(f"No images found matching {CALIB_IMAGES}")

  print(f"Found {len(img_paths)} calibration images")

  all_corners = []
  all_ids = []
  image_size = None

  detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

  for path in img_paths:
    img = cv2.imread(path)
    if img is None:
      continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if image_size is None:
      image_size = gray.shape[::-1]  # (width, height)

    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
      continue

    cv2.aruco.refineDetectedMarkers(
      image=gray,
      board=board,
      detectedCorners=corners,
      detectedIds=ids,
      rejectedCorners=None
    )

    ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
      markerCorners=corners,
      markerIds=ids,
      image=gray,
      board=board
    )

    if ret and ch_ids is not None and len(ch_ids) > 8:
      all_corners.append(ch_corners)
      all_ids.append(ch_ids)

  if len(all_corners) < 8:
    raise SystemExit("Not enough valid detections for calibration. Capture more varied images.")

  print(f"Using {len(all_corners)} valid views for calibration")

  flags = (cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_TANGENT_DIST)

  rms, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
    charucoCorners=all_corners,
    charucoIds=all_ids,
    board=board,
    imageSize=image_size,
    cameraMatrix=None,
    distCoeffs=None,
    flags=flags
  )

  np.savez(
    OUT_FILE,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    image_width=image_size[0],
    image_height=image_size[1],
    rms=rms,
    squares_x=SQUARES_X,
    squares_y=SQUARES_Y,
    square_len_m=SQUARE_LENGTH_M,
    marker_len_m=MARKER_LENGTH_M,
    dict=DICT
  )

  print(f"Saved {OUT_FILE}")
  print(f"RMS reprojection error: {rms:.4f}")


if __name__ == "__main__":
  main()
