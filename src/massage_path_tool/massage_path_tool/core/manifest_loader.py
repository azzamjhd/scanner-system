"""manifest_loader.py — manifest.json → X_motor_mm lookup.
Maps stitched-image pixel Y (y_stitch, 0=head at top) → gantry X position
in mm via simple linear interpolation using the stitched image height.

The stitched image is built by stacking frames collected as the gantry moves
from head (X_min) to feet (X_max) along the body.  y_stitch=0 → X_min (head),
y_stitch=stitched_H → X_max (feet).
"""
from __future__ import annotations
import numpy as np


class ManifestLoader:
    def __init__(self, manifest: dict, stitched_H: int) -> None:
        _ = manifest.get('resolved_pixels_per_mm')  # kept for compatibility
        frames = manifest.get('frames')
        if not frames:
            raise ValueError("manifest 'frames' array is empty or missing")
        xs = np.array([f['x_mm'] for f in frames], dtype=np.float64)
        self._x_min = float(xs.min())
        self._x_max = float(xs.max())
        self._stitched_H = int(stitched_H)

    def y_stitch_to_motor_mm(self, y_stitch: float) -> float:
        """Linear map: y_stitch=0 (head) → X_min, y_stitch=H (feet) → X_max."""
        alpha = float(y_stitch) / max(self._stitched_H, 1)
        x_mm = self._x_min + alpha * (self._x_max - self._x_min)
        return float(np.clip(x_mm, self._x_min, self._x_max))

    @property
    def x_range_mm(self) -> tuple[float, float]:
        return self._x_min, self._x_max
