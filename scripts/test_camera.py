#!/usr/bin/env python3

"""
This script opens the camera using DetectorRT and prints live FPS to the console for testing/debugging
camera connectivity and performance.
"""

import time
import sys
import os
import platform
import cv2
import logging
from pathlib import Path
from dotenv import load_dotenv
load_dotenv(dotenv_path="config/.env") 

from utils import context, helpers, logger as _logger
from vision import DetectorRT

directory = context.get_context(os.path.abspath(__file__))
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger_file = f"{directory}/logs/{logger_file_name}.log"
logger = _logger.setup_logger(logger_name=logger_name, log_file=logger_file, log_level=os.getenv("LOG_LEVEL"))

def test_camera(camera_index=0):
  """
  MacOS uses AVFOUNDATION as the backend for camera capture. DetectorRT assumes default video
  capture of DSHOW or V4L2. caputre_format function created to allow for creating script on mac. feel free to
  remove in future for a simplified script.
  """
  orig_videocapture = cv2.VideoCapture

  def capture_format(index, *args, **kwargs):
    backend = None
    if platform.system() == "Darwin": # MacOS
      backend = cv2.CAP_AVFOUNDATION
    elif platform.system() == "Windows": #
      backend = cv2.CAP_DSHOW
    else:
      backend = cv2.CAP_V4L2
    if isinstance(index, int):
      return orig_videocapture(index, backend)
    return orig_videocapture(index, *args, **kwargs)

  cv2.VideoCapture = capture_format
  detector = DetectorRT(camera=camera_index, width=640, height=480, fps=30)

  try:
    detector.open()
    logging.info(f"Camera {camera_index} opened. Press 'q' to quit.")

    while True:
      frame, results = detector.read()
      if frame is None:
        logging.error("\nISSUE: no frame captured from camera")
        break

      fps = results.get("fps", 0.0)
      logging.info(f"\rFPS: {fps:.1f}", end="", flush=True)
      cv2.imshow("Camera Feed", frame)

      # quit on 'q'
      if cv2.waitKey(1) == ord('q'):
        break
  except KeyboardInterrupt:
    logging.info("\nExiting per user.")
  finally:
    detector.release()
    cv2.destroyAllWindows()
    logging.info("Camera released")
    cv2.VideoCapture = orig_videocapture

if __name__ == "__main__":
  test_camera()
