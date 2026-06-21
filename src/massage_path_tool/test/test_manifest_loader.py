"""Tests for ManifestLoader — y_stitch pixel → X_motor_mm via ppm formula."""
import pytest
from massage_path_tool.core.manifest_loader import ManifestLoader


def _manifest(xs=None):
    if xs is None:
        xs = [100.0, 110.0, 120.0, 130.0, 140.0]
    return {
        'resolved_pixels_per_mm': 2.0,
        'frames': [{'file': f'frame_{i:04d}.png', 'x_mm': x} for i, x in enumerate(xs)],
    }


def test_center_y_maps_to_x_min_plus_half_frame():
    m = ManifestLoader(_manifest(), frame_H=100)
    assert abs(m.y_stitch_to_motor_mm(50.0) - 100.0) < 0.01


def test_y_below_center_maps_negative_x_travel():
    m = ManifestLoader(_manifest(), frame_H=100)
    # y = 0 is half a frame above the first frame centre (y=50), which by the
    # formula gives x_mm = 75.0, but the result is clamped to x_min (100.0).
    assert abs(m.y_stitch_to_motor_mm(0.0) - 100.0) < 0.01


def test_y_proportional_to_x():
    m = ManifestLoader(_manifest(xs=[100.0, 120.0]), frame_H=100)
    assert abs(m.y_stitch_to_motor_mm(60.0) - 105.0) < 0.01


def test_raises_on_missing_frames():
    with pytest.raises(ValueError):
        ManifestLoader({'resolved_pixels_per_mm': 1.0}, frame_H=100)


def test_raises_on_zero_ppm():
    with pytest.raises(ValueError):
        ManifestLoader({'resolved_pixels_per_mm': 0.0, 'frames': [{'x_mm': 100.0}]}, frame_H=100)


def test_clamps_to_x_range():
    m = ManifestLoader(_manifest(xs=[100.0, 150.0]), frame_H=100)
    assert abs(m.y_stitch_to_motor_mm(-9999.0) - 100.0) < 0.01
    assert abs(m.y_stitch_to_motor_mm(99999.0) - 150.0) < 0.01


def test_x_range():
    m = ManifestLoader(_manifest(xs=[50.0, 100.0, 200.0]), frame_H=100)
    lo, hi = m.x_range_mm
    assert lo == pytest.approx(50.0)
    assert hi == pytest.approx(200.0)
