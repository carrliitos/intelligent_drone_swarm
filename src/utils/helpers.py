from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List, Union
import numpy as np

def _b(s, default=False):
  return (s or "").strip().lower() in {"1","true","yes","on"}

def _i(s, default=None):
  try: return int(s) if s is not None else default
  except: return default

def _f(s, default=None):
  try: return float(s) if s is not None else default
  except: return default

def _ids(s, default=None):
  if not s:
    return default
  try:
    parts = [p.strip() for p in s.split(",") if p.strip() != ""]
    return [int(p) for p in parts]
  except Exception:
    return default

def _to_int(val: Optional[Union[str, int]], default: Optional[int] = None) -> Optional[int]:
  try:
    if val is None: return default
    return int(val)
  except (ValueError, TypeError):
    return default

def _to_float(val: Optional[Union[str, float]], default: Optional[float] = None) -> Optional[float]:
  try:
    if val is None: return default
    return float(val)
  except (ValueError, TypeError):
    return default

def _to_bgr_tuple(val: Optional[Union[str, Tuple[int,int,int], List[int]]], default: Tuple[int,int,int] = (0,255,0)) -> Tuple[int,int,int]:
  try:
    if val is None:
      return default
    if isinstance(val, str):
      parts = [int(x.strip()) for x in val.split(",")]
      if len(parts) == 3:
        return (parts[0], parts[1], parts[2])
      return default
    if isinstance(val, (list, tuple)) and len(val) == 3:
      return (int(val[0]), int(val[1]), int(val[2]))
    return default
  except Exception:
    return default

def _to_ids(val: Optional[Union[str, List[int], List[str]]]) -> Optional[List[int]]:
  if val is None:
    return None
  if isinstance(val, list):
    out = []
    for v in val:
      try: out.append(int(v))
      except Exception: pass
    return out or None
  if isinstance(val, str):
    parts = [p.strip() for p in val.split(",") if p.strip() != ""]
    out = []
    for p in parts:
      try: out.append(int(p))
      except Exception: pass
    return out or None
  return None

def _normalize_dict_name(name: str) -> str:
  """
  Accept variants like '4x4_1000', 'DICT_4X4_1000', '4X4_1000' and normalize to 'DICT_4X4_1000'.
  """
  s = name.strip().upper()
  if not s.startswith("DICT_"):
    s = "DICT_" + s
  return s

def _load_calibration(npz_path: str):
  calib_p = Path(npz_path)
  if not calib_p.exists():
    raise FileNotFoundError(f"Calibration file not found: {calib_p}")
  data = np.load(str(calib_p))
  return data["camera_matrix"], data["dist_coeffs"]

def _filter_known(corners, ids, allowed: Optional[set]):
  """
  Keep only markers whose ID is in 'allowed'. Preserves correspondence between
  corners[i] and ids[i]. Returns (corners_filt, ids_filt or None).
  """
  if ids is None or len(ids) == 0 or allowed is None:
    return corners, ids
  keep_corners = []
  keep_ids = []
  # ids is shape (N,1); flatten to iterate
  for corner, mid in zip(corners, ids.flatten()):
    if mid in allowed:
      keep_corners.append(corner)
      keep_ids.append(mid)
  if not keep_ids:
    return [], None
  ids_out = np.array(keep_ids, dtype=np.int32).reshape(-1, 1)
  return keep_corners, ids_out
