"""Cubic Bezier curve interpolation through control points.

Converts a sequence of control points into a smooth piecewise cubic Bezier
curve using Catmull-Rom → cubic Bezier conversion. The curve passes exactly
through every control point (interpolating), with C1 continuity.
"""
from __future__ import annotations

import numpy as np


def catmull_rom_to_bezier(pts: np.ndarray, tension: float = 0.5) -> list[np.ndarray]:
    """Convert N control points to (N-1) cubic Bezier segments.

    Each segment is a 4x2 array: [P0, P1, P2, P3] where P0 and P3 are the
    anchor points (on the curve) and P1, P2 are the control handles.

    pts: shape (N, 2) float, N >= 2.
    tension: 0 = loose, 0.5 = default, 1 = tight.
    Returns: list of (4, 2) arrays, one per segment.
    """
    if len(pts) < 2:
        raise ValueError(f"Need >= 2 points for Bezier, got {len(pts)}")

    pts = np.array(pts, dtype=np.float64)
    n = len(pts)
    segments = []

    # For Catmull-Rom, we need neighbors. Clamp boundaries by duplicating.
    padded = np.vstack([pts[0], pts, pts[-1]])

    for i in range(n - 1):
        p0 = padded[i + 1]      # current anchor
        p3 = padded[i + 2]      # next anchor
        # Tangents from neighbors
        t1 = (padded[i + 2] - padded[i]) * tension
        t2 = (padded[i + 3] - padded[i + 1]) * tension
        p1 = p0 + t1 / 3.0
        p2 = p3 - t2 / 3.0
        segments.append(np.array([p0, p1, p2, p3]))

    return segments


def sample_bezier_segments(segments: list[np.ndarray], n_total: int = 200) -> np.ndarray:
    """Uniformly sample n_total points across all Bezier segments.

    segments: list of (4, 2) arrays from catmull_rom_to_bezier().
    Returns: shape (n_total, 2) float64.
    """
    if not segments:
        return np.zeros((0, 2), dtype=np.float64)

    n_seg = len(segments)
    # Allocate samples proportionally to chord length of each segment
    lengths = []
    for seg in segments:
        # Approximate length by sampling 10 points
        t = np.linspace(0, 1, 10)
        samples = _eval_cubic_bezier(seg, t)
        lengths.append(np.sum(np.linalg.norm(np.diff(samples, axis=0), axis=1)))

    total_len = sum(lengths)
    if total_len < 1e-9:
        # All segments degenerate — just return control points
        return np.vstack([seg[0] for seg in segments] + [segments[-1][-1]])

    # Distribute n_total samples across segments.
    # Each internal boundary point is shared between two segments, so:
    # total_points = sum(counts) - (n_seg - 1)  [deduplicated boundaries]
    # We want total_points ≈ n_total, so sum(counts) ≈ n_total + n_seg - 1
    target_sum = n_total + n_seg - 1
    counts = []
    for L in lengths:
        counts.append(max(2, int(round(target_sum * L / total_len))))
    # Adjust last segment to hit exactly target_sum
    counts[-1] += target_sum - sum(counts)
    counts[-1] = max(2, counts[-1])

    samples = []
    for i, (seg, count) in enumerate(zip(segments, counts)):
        t = np.linspace(0, 1, count)
        seg_samples = _eval_cubic_bezier(seg, t)
        if i > 0:
            # Skip first point — it's the same as the last point of previous segment
            seg_samples = seg_samples[1:]
        samples.append(seg_samples)

    return np.vstack(samples)


def _eval_cubic_bezier(ctrl: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Evaluate cubic Bezier at parameter values t.

    ctrl: shape (4, 2) — [P0, P1, P2, P3]
    t: shape (M,) in [0, 1]
    Returns: shape (M, 2)
    """
    p0, p1, p2, p3 = ctrl
    u = 1 - t
    # B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
    return (
        np.outer(u**3, p0) +
        np.outer(3 * u**2 * t, p1) +
        np.outer(3 * u * t**2, p2) +
        np.outer(t**3, p3)
    )


def bezier_through_points(pts: np.ndarray, n_out: int = 200, tension: float = 0.5) -> np.ndarray:
    """Smooth interpolating curve through control points.

    pts: shape (N, 2) float, N >= 2.
    n_out: total number of output samples.
    tension: Catmull-Rom tension (0.5 = default).
    Returns: shape (n_out, 2) float64 — smooth curve passing through all pts.
    """
    if len(pts) < 2:
        raise ValueError(f"bezier_through_points needs >= 2 points, got {len(pts)}")
    segments = catmull_rom_to_bezier(pts, tension)
    return sample_bezier_segments(segments, n_out)
