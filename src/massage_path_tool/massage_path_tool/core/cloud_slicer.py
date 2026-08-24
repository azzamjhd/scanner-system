"""cloud_slicer.py — Z-height extraction from a 3D point cloud at (x, y).

All coordinates in metres (base_link frame). Uses scipy KD-Tree for nearest-neighbour Z.

No longer does proportional Y interpolation — that's handled by path_translator
using bilinear anchor interpolation.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial import KDTree


@dataclass
class SliceResult:
    x_m: float
    y_m: float
    z_m: float
    n_slice_pts: int


class CloudSlicer:
    def __init__(self, x_tol_m: float = 0.005) -> None:
        self._tol = float(x_tol_m)

    def query_z(self, cloud_xyz: np.ndarray, x_m: float, y_m: float) -> Optional[float]:
        """Return Z height at (x_m, y_m) using nearest-neighbour in an X-slice."""
        mask = np.abs(cloud_xyz[:, 0] - x_m) <= self._tol
        sl = cloud_xyz[mask]
        if len(sl) < 2:
            return None
        tree = cKDTree(sl[:, :2])
        _, idx = tree.query([x_m, y_m], k=1)
        return float(sl[idx, 2])

    def query(
        self, cloud_xyz: np.ndarray, x_m: float, y_ratio: float
    ) -> Optional[SliceResult]:
        """Legacy method kept for compatibility."""
        mask = np.abs(cloud_xyz[:, 0] - x_m) <= self._tol
        sl = cloud_xyz[mask]
        if len(sl) < 2:
            return None
        y_vals = sl[:, 1]
        y_min = float(np.percentile(y_vals, 2))
        y_max = float(np.percentile(y_vals, 98))
        if (y_max - y_min) < 1e-6:
            return None
        r = float(np.clip(y_ratio, -0.2, 1.2))
        y_target = y_min + r * (y_max - y_min)
        y_target = float(np.clip(y_target, float(y_vals.min()), float(y_vals.max())))
        tree = cKDTree(sl[:, :2])
        _, idx = tree.query([x_m, y_target], k=1)
        z_target = float(sl[idx, 2])
        return SliceResult(x_m=float(x_m), y_m=y_target,
                           z_m=z_target, n_slice_pts=int(len(sl)))
