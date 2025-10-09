import cv2
import numpy as np
from typing import Optional

def _mm_to_px(mm: float, dpi: int) -> int:
  return int(round(mm / 25.4 * dpi))

def generate_repeated_aruco_board(
  dictionary_name: str,
  marker_id: int,
  cols: int,
  rows: int,
  marker_size_px: Optional[int] = None,   # e.g., 300
  spacing_px: int = 20,                   # gap between markers (pixels)
  border_px: int = 60,                    # outer white border (pixels)
  marker_size_mm: Optional[float] = None, # e.g., 40.0
  spacing_mm: float = 3.0,
  border_mm: float = 10.0,
  dpi: int = 300,
  output_filename: str = "aruco_board.png"
):
  """
  Create a board with the same ArUco marker repeated in a grid.

  Args:
    dictionary_name: e.g. 'DICT_4X4_100', 'DICT_5X5_1000', 'DICT_6X6_250', etc.
    marker_id: ID to draw for every cell.
    cols, rows: grid dimensions (e.g., 5, 7).
    marker_size_px: marker side length in pixels (used if *_mm is None).
    spacing_px: gap between markers in pixels.
    border_px: outer border in pixels.
    marker_size_mm: marker side length in millimeters (if provided, used instead of px).
    spacing_mm: gap between markers in millimeters.
    border_mm: outer border in millimeters.
    dpi: dots per inch for mm→px conversion.
    output_filename: output image path.
  """
  aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
  max_id = aruco_dict.bytesList.shape[0] - 1

  if not (0 <= marker_id <= max_id):
    raise ValueError(f"marker_id {marker_id} out of range for {dictionary_name} (max {max_id}).")

  if marker_size_mm is not None:
    marker_size_px = _mm_to_px(marker_size_mm, dpi)
    spacing_px = _mm_to_px(spacing_mm, dpi)
    border_px = _mm_to_px(border_mm, dpi)
  elif marker_size_px is None:
    raise ValueError("Provide either marker_size_px or marker_size_mm.")

  W = cols * marker_size_px + (cols - 1) * spacing_px + 2 * border_px
  H = rows * marker_size_px + (rows - 1) * spacing_px + 2 * border_px
  canvas = np.full((H, W), 255, dtype=np.uint8)  # white background

  marker_img = np.zeros((marker_size_px, marker_size_px), dtype=np.uint8)
  cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px, marker_img, 1)

  for r in range(rows):
    for c in range(cols):
      y = border_px + r * (marker_size_px + spacing_px)
      x = border_px + c * (marker_size_px + spacing_px)
      canvas[y:y+marker_size_px, x:x+marker_size_px] = marker_img

  cv2.imwrite(output_filename, canvas)
  print(f"Saved {cols}x{rows} board of ID {marker_id} to {output_filename} ({W}x{H}px).")

if __name__ == "__main__":
  dict_name = "DICT_4X4_1000"
  marker_id = 77
  out_file = f"aruco_board_{dict_name}_id{marker_id}.png"

  generate_repeated_aruco_board(
    dictionary_name=dict_name,
    marker_id=marker_id,
    cols=5,
    rows=7,
    marker_size_mm=35.0,
    spacing_mm=3.0, # mm gaps
    border_mm=10.0, # mm outer border
    dpi=300,
    output_filename=out_file
  )
