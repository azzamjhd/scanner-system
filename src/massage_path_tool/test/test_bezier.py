import numpy as np
import pytest

from massage_path_tool.core.bezier import (
    catmull_rom_to_bezier,
    sample_bezier_segments,
    bezier_through_points,
)


def test_catmull_rom_two_points():
    pts = np.array([[0, 0], [100, 0]], dtype=float)
    segs = catmull_rom_to_bezier(pts)
    assert len(segs) == 1
    assert segs[0].shape == (4, 2)
    # Endpoints should match input points
    np.testing.assert_allclose(segs[0][0], [0, 0], atol=1e-6)
    np.testing.assert_allclose(segs[0][3], [100, 0], atol=1e-6)


def test_catmull_rom_three_points():
    pts = np.array([[0, 0], [50, 50], [100, 0]], dtype=float)
    segs = catmull_rom_to_bezier(pts)
    assert len(segs) == 2
    for seg in segs:
        assert seg.shape == (4, 2)
    # Segment endpoints should connect
    np.testing.assert_allclose(segs[0][3], segs[1][0], atol=1e-6)


def test_sample_shape():
    pts = np.array([[0, 0], [50, 50], [100, 0]], dtype=float)
    segs = catmull_rom_to_bezier(pts)
    samples = sample_bezier_segments(segs, n_total=100)
    assert samples.shape == (100, 2)
    assert samples.dtype == np.float64


def test_bezier_through_points_interpolates():
    """The curve should pass very close to the control points."""
    pts = np.array([[0, 0], [50, 100], [100, 0], [150, 50]], dtype=float)
    curve = bezier_through_points(pts, n_out=500)
    # Check that the curve passes near each control point
    for p in pts:
        dists = np.linalg.norm(curve - p, axis=1)
        assert dists.min() < 2.0, f"Point {p} not near curve (min dist={dists.min():.2f})"


def test_bezier_through_points_endpoints_exact():
    """First and last points should be on the curve exactly."""
    pts = np.array([[10, 20], [50, 80], [90, 30]], dtype=float)
    curve = bezier_through_points(pts, n_out=100)
    np.testing.assert_allclose(curve[0], pts[0], atol=1e-6)
    np.testing.assert_allclose(curve[-1], pts[-1], atol=1e-6)


def test_bezier_through_points_few_points():
    pts = np.array([[0, 0], [10, 10]], dtype=float)
    curve = bezier_through_points(pts, n_out=50)
    assert curve.shape == (50, 2)
    np.testing.assert_allclose(curve[0], pts[0], atol=1e-6)
    np.testing.assert_allclose(curve[-1], pts[-1], atol=1e-6)


def test_bezier_through_points_raises_on_single():
    with pytest.raises(ValueError):
        bezier_through_points(np.array([[0, 0]]), n_out=10)
