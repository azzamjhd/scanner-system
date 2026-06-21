"""cloud_slicer.py — 2.5D proportional body-slice + KD-Tree Z extraction.

All coordinates in metres (base_link frame). Uses scipy KD-Tree for nearest-neighbour Z.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import numpy as np
from scipy.spatial import cKDTree


@dataclass
class SliceResult:
    x_m: float        # X_motor (metres, passed through)
    y_m: float        # proportional Y target (metres)
    z_m: float        # KD-Tree nearest-neighbour Z depth (metres)
    n_slice_pts: int  # diagnostic: points in the X slice


class CloudSlicer:
    def __init__(self, x_tol_m: float = 0.005) -> None:
        self._tol = float(x_tol_m)

    def query(
        self,
        cloud_xyz: np.ndarray,
        x_m: float,
        y_ratio: float,
    ) -> Optional[SliceResult]:
        # 1. X slice
        mask = np.abs(cloud_xyz[:, 0] - x_m) <= self._tol
        sl = cloud_xyz[mask]
        if len(sl) < 2:
            return None

        # 2. Body Y bounds
        y_min, y_max = float(sl[:, 1].min()), float(sl[:, 1].max())
        if (y_max - y_min) < 1e-6:
            return None

        # 3. Proportional Y
        r = float(np.clip(y_ratio, -0.2, 1.2))
        y_target = y_min + r * (y_max - y_min)
        y_target = float(np.clip(y_target, y_min, y_max))

        # 4. KD-Tree Z
        tree = cKDTree(sl[:, :2])
        _, idx = tree.query([x_m, y_target], k=1)
        z_target = float(sl[idx, 2])

        return SliceResult(x_m=float(x_m), y_m=float(y_target),
                           z_m=z_target, n_slice_pts=int(len(sl)))
