from __future__ import annotations
import numpy as np
from scipy.interpolate import splev, splprep

def smooth_path(pts, n_out=200, smoothing=0.0):
    """Fit B-spline through pts (N,2), resample to n_out points. Raises ValueError if <4 unique pts."""
    pts = np.array(pts, dtype=np.float64)
    diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], diffs > 1e-9])
    pts = pts[keep]
    if len(pts) < 4:
        raise ValueError(f"smooth_path requires >= 4 unique points, got {len(pts)}")
    k = min(3, len(pts) - 1)
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=smoothing, k=k)
    xs, ys = splev(np.linspace(0.0, 1.0, n_out), tck)
    return np.column_stack([xs, ys])
