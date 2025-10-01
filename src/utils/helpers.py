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
