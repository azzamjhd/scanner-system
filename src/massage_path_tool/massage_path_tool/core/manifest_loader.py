"""manifest_loader.py — manifest.json → X_motor_mm lookup.
Maps stitched-image pixel Y (y_stitch) → gantry X position in mm using
the uniform pixels_per_mm scale recorded in the manifest.
"""
from __future__ import annotations
import numpy as np


class ManifestLoader:
    def __init__(self, manifest: dict, frame_H: int) -> None:
        ppm = manifest.get('resolved_pixels_per_mm')
        if not ppm or ppm <= 0.0:
            raise ValueError(
                f"manifest 'resolved_pixels_per_mm' is {ppm!r}, must be > 0"
            )
        frames = manifest.get('frames')
        if not frames:
            raise ValueError("manifest 'frames' array is empty or missing")
        xs = np.array([f['x_mm'] for f in frames], dtype=np.float64)
        self._ppm = float(ppm)
        self._x_min = float(xs.min())
        self._x_max = float(xs.max())
        self._frame_H = int(frame_H)

    def y_stitch_to_motor_mm(self, y_stitch: float) -> float:
        x_mm = self._x_min + (float(y_stitch) - self._frame_H / 2.0) / self._ppm
        return float(np.clip(x_mm, self._x_min, self._x_max))

    @property
    def x_range_mm(self) -> tuple[float, float]:
        return self._x_min, self._x_max
