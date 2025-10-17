#!/usr/bin/env python3
"""
Cross-platform camera smoke test for DetectorRT.

- Loads settings from .env (dictionary, camera, width/height/fps, overlays, etc.).
- Picks a sensible OpenCV backend per OS (AVFOUNDATION on macOS, DSHOW on Windows, V4L2 on Linux).
- Prints live FPS and (if available) the first detected marker's pose.
- Warns if runtime capture size != calibration size.
- Cleanly restores cv2.VideoCapture on exit.
- Quit with 'q' or ESC.
"""

import os
import sys
import time
import platform
from pathlib import Path

import cv2
import numpy as np
from dotenv import load_dotenv

from utils import context, helpers, logger as _logger
from vision import DetectorRT

directory = context.get_context(os.path.abspath(__file__))
load_dotenv(dotenv_path=f"{directory}/config/.env")

logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
log = _logger.setup_logger(
  logger_name=logger_name,
  log_file=logger_file,
  log_level=os.getenv("LOG_LEVEL")
)


def _select_backend() -> int:
  """
  Choose a reasonable OpenCV capture backend based on the OS.
  """
  sysname = platform.system()
  if sysname == "Darwin":
    return cv2.CAP_AVFOUNDATION
  if sysname == "Windows":
    return cv2.CAP_DSHOW
  # Linux and everything else
  return cv2.CAP_V4L2


def _monkeypatch_videocapture():
  """
  Monkey-patch cv2.VideoCapture so that when an integer index is passed,
  we force the OS-specific backend; strings/URLs are passed through unchanged.
  Returns a callable that restores the original VideoCapture.
  """
  orig_videocapture = cv2.VideoCapture
  backend = _select_backend()

  def capture_format(index, *args, **kwargs):
    if isinstance(index, int):
      return orig_videocapture(index, backend)

    return orig_videocapture(index, *args, **kwargs)

  cv2.VideoCapture = capture_format

  def restore():
    cv2.VideoCapture = orig_videocapture

  return restore


def _parse_camera_env(val: str | None):
  """
  CAMERA_CONNECTION may be an integer index ('0', '1', ...), a device path,
  or a URL. Convert to int when it’s numeric; otherwise return as string/None.
  """
  if val is None:
    return None

  try:
    return int(val)
  except (TypeError, ValueError):
    return val


def main():
  # Allow env to drive everything, but still benefit from platform backend handling
  restore_vc = _monkeypatch_videocapture()

  # Build DetectorRT from env
  det = DetectorRT(
    dictionary=os.getenv("ARUCO_DICTIONARY"),
    camera=_parse_camera_env(os.getenv("CAMERA_CONNECTION", "0")),
    width=helpers._i(os.getenv("CAMERA_WIDTH", "640")),
    height=helpers._i(os.getenv("CAMERA_HEIGHT", "480")),
    fps=helpers._i(os.getenv("CAMERA_FPS", "30")),
    calib_path=os.getenv("CALIB_PATH"),
    marker_length_m=helpers._f(os.getenv("MARKER_LENGTH_M", "0.05")),
    window_title=os.getenv("WINDOW_TITLE", "Camera Feed"),
    draw_axes=helpers._b(os.getenv("DRAW_AXES", "false")),
    allowed_ids=helpers._ids(os.getenv("ALLOWED_IDS")),
    draw_grid=helpers._b(os.getenv("DRAW_GRID", "false")),
    min_brightness=helpers._f(os.getenv("BRIGHTNESS_VALUE", "0")),
    grid_step_px=helpers._i(os.getenv("GRID_STEP_PX", "80")),
    grid_color=os.getenv("GRID_COLOR", "0,255,0"),
    grid_thickness=helpers._i(os.getenv("GRID_THICKNESS", "1")),
    draw_rule_of_thirds=helpers._b(os.getenv("DRAW_RULE_OF_THIRDS", "false")),
    draw_crosshair=helpers._b(os.getenv("DRAW_CROSSHAIR", "false")),
    capture_cells=helpers._b(os.getenv("CAPTURE_CELLS", "false")),
    fps_counter=helpers._i(os.getenv("FPS_COUNTER", "30")),
    fps_display=helpers._f(os.getenv("FPS_DISPLAY", "0.5")),
  )

  try:
    det.open()
    log.info("Camera opened. Press 'q' or ESC to quit.")

    # Sanity: warn if runtime size != calibration size
    try:
      # Prefer CALIB_PATH if it points directly to cam_param.npz; otherwise fall back to repo default
      calib_file = os.getenv("CALIB_PATH")
      if calib_file and calib_file.endswith(".npz"):
        calib_npz = calib_file
      else:
        calib_npz = f"{directory}/src/utils/calibration_imgs/cam_param.npz"

      with np.load(calib_npz) as X:
        cal_w, cal_h = int(X["image_width"]), int(X["image_height"])

      cap_w = int(det.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
      cap_h = int(det.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

      if (cap_w, cap_h) != (cal_w, cal_h):
        log.warning(f"Capture {cap_w}x{cap_h} != calibrated {cal_w}x{cal_h}. "
                     "Match the resolution or recalibrate for best accuracy.")
    except Exception as e:
      log.debug(f"Skipping calibration size check: {e}")

    # Display loop
    max_w, max_h = 960, 540  # preview bounding box
    while True:
      frame, results = det.read()
      if frame is None:
        log.error("No frame captured from camera.")
        break

      # FPS (from DetectorRT)
      fps = results.get("fps", 0.0)
      sys.stdout.write(f"\rFPS: {fps:4.1f}")
      sys.stdout.flush()

      # First marker pose summary
      ids = results.get("ids")
      tvecs = results.get("tvecs")
      if ids is not None and tvecs is not None and len(ids) > 0:
        id0 = int(ids[0, 0])
        tx, ty, tz = tvecs[0, 0].astype(float)
        xy = (tx * tx + ty * ty) ** 0.5
        dist = (tx * tx + ty * ty + tz * tz) ** 0.5
        log.debug(f"id={id0}  x={tx:.3f}  y={ty:.3f}  z={tz:.3f} m  | xy={xy:.3f}  dist={dist:.3f}")

      # Fit preview into a box while keeping aspect
      h, w = frame.shape[:2]
      scale = min(max_w / w, max_h / h, 1.0)
      display = cv2.resize(frame, (int(w * scale), int(h * scale))) if scale < 1.0 else frame

      title = os.getenv("WINDOW_TITLE", "Camera Feed")
      cv2.imshow(title, display)

      key = cv2.waitKey(1) & 0xFF
      if key in (27, ord("q")):  # ESC or 'q'
        break

  except KeyboardInterrupt:
    log.info("Exiting per user (Ctrl+C).")
  finally:
    try:
      sys.stdout.write("\n")
      sys.stdout.flush()
    except Exception:
      pass

    det.release()
    cv2.destroyAllWindows()
    restore_vc()
    log.info("Camera released and VideoCapture restored.")

if __name__ == "__main__":
  main()
