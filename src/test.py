# quick_test_detector.py
from vision import DetectorRT

if __name__ == "__main__":
  # det = DetectorRT(dictionary="4x4_1000", camera=2)
  det = DetectorRT(dictionary="4x4_1000", camera=2, draw_grid=False, grid_step_px=50, draw_rule_of_thirds=True)
  det.open()
  try:
    while True:
      frame, results = det.read()
      # show window if available
      import cv2
      if frame is not None:
        cv2.imshow("Aruco Test", frame)
        if (cv2.waitKey(1) & 0xFF) == 27: # ESC to exit
          break
  finally:
    det.release()
