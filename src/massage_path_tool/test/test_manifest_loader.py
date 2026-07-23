"""Tests for ManifestLoader — y_stitch pixel → X_motor_mm via linear mapping.

The stitched image runs head (y=0) → feet (y=H), which maps to
gantry X_min (head) → X_max (feet).
"""
import pytest
from massage_path_tool.core.manifest_loader import ManifestLoader


def _manifest(xs=None):
    if xs is None:
        xs = [100.0, 110.0, 120.0, 130.0, 140.0]
    return {
        'resolved_pixels_per_mm': 2.0,
        'frames': [{'file': f'frame_{i:04d}.png', 'x_mm': x} for i, x in enumerate(xs)],
    }


def test_y_zero_maps_to_x_min_head():
    m = ManifestLoader(_manifest(), stitched_H=100)
    # y=0 (top of stitched image = head) → X_min = 100.0
    assert abs(m.y_stitch_to_motor_mm(0.0) - 100.0) < 0.01


def test_y_half_maps_to_midpoint():
    m = ManifestLoader(_manifest(xs=[100.0, 200.0]), stitched_H=100)
    # y=50.0 → halfway → 150.0
    assert abs(m.y_stitch_to_motor_mm(50.0) - 150.0) < 0.01


def test_y_full_maps_to_x_max_feet():
    m = ManifestLoader(_manifest(xs=[100.0, 200.0]), stitched_H=100)
    # y=100.0 (bottom of stitched image = feet) → X_max = 200.0
    assert abs(m.y_stitch_to_motor_mm(100.0) - 200.0) < 0.01


def test_y_proportional_to_x():
    m = ManifestLoader(_manifest(xs=[100.0, 120.0]), stitched_H=100)
    # y=60 → alpha=0.6 → 100 + 0.6*20 = 112.0
    assert abs(m.y_stitch_to_motor_mm(60.0) - 112.0) < 0.01


def test_raises_on_missing_frames():
    with pytest.raises(ValueError):
        ManifestLoader({'resolved_pixels_per_mm': 1.0}, stitched_H=100)


def test_clamps_to_x_range():
    m = ManifestLoader(_manifest(xs=[100.0, 150.0]), stitched_H=100)
    # y below 0 → clamps to X_min
    assert abs(m.y_stitch_to_motor_mm(-9999.0) - 100.0) < 0.01
    # y above H → clamps to X_max
    assert abs(m.y_stitch_to_motor_mm(99999.0) - 150.0) < 0.01


def test_x_range():
    m = ManifestLoader(_manifest(xs=[50.0, 100.0, 200.0]), stitched_H=100)
    lo, hi = m.x_range_mm
    assert lo == pytest.approx(50.0)
    assert hi == pytest.approx(200.0)
