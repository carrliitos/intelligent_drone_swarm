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

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), "..")
SRC_PATH = os.path.join(PROJECT_ROOT, "src")
sys.path.extend([PROJECT_ROOT, SRC_PATH])
os.environ["LOG_LEVEL"] = "INFO"
logging.warning = logging.WARNING

from src.vision import DetectorRT

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
        print(f"Camera {camera_index} opened. Press 'q' to quit.")

        while True:
            frame, results = detector.read()
            if frame is None:
                print("\nISSUE: no frame captured from camera")
                break

            fps = results.get("fps", 0.0)
            print(f"\rFPS: {fps:.1f}", end="", flush=True)
            cv2.imshow("Camera Feed", frame)

            # quit on 'q'
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nExiting per user.")

    finally:
        detector.release()
        cv2.destroyAllWindows()
        print("Camera released")
        cv2.VideoCapture = orig_videocapture

if __name__ == "__main__":
    test_camera(0)
