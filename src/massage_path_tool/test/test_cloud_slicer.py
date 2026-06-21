# test/test_cloud_slicer.py
import numpy as np
import pytest
from massage_path_tool.core.cloud_slicer import CloudSlicer, SliceResult


def flat_cloud(x_c=0.05, n=300, y_lo=0.10, y_hi=0.50, z=0.80):
    rng = np.random.default_rng(42)
    xs = rng.normal(x_c, 0.002, n).astype(np.float32)
    ys = rng.uniform(y_lo, y_hi, n).astype(np.float32)
    zs = np.full(n, z, dtype=np.float32)
    return np.column_stack([xs, ys, zs])


def test_returns_sliceresult():
    c = CloudSlicer(x_tol_m=0.01)
    r = c.query(flat_cloud(), x_m=0.05, y_ratio=0.5)
    assert isinstance(r, SliceResult)
    assert hasattr(r, 'x_m')
    assert hasattr(r, 'y_m')
    assert hasattr(r, 'z_m')
    assert hasattr(r, 'n_slice_pts')


def test_y_ratio_0_maps_to_y_min():
    c = CloudSlicer(x_tol_m=0.01)
    r = c.query(flat_cloud(), x_m=0.05, y_ratio=0.0)
    assert r is not None
    assert r.y_m <= 0.12  # near y_lo=0.10


def test_y_ratio_1_maps_to_y_max():
    c = CloudSlicer(x_tol_m=0.01)
    r = c.query(flat_cloud(), x_m=0.05, y_ratio=1.0)
    assert r is not None
    assert r.y_m >= 0.48  # near y_hi=0.50


def test_z_from_flat_cloud():
    c = CloudSlicer(x_tol_m=0.01)
    r = c.query(flat_cloud(z=0.80), x_m=0.05, y_ratio=0.5)
    assert r is not None
    assert abs(r.z_m - 0.80) < 0.001


def test_returns_none_when_slice_empty():
    c = CloudSlicer(x_tol_m=0.001)
    far_away = np.array([[5.0, 0.3, 0.8]], dtype=np.float32)
    r = c.query(far_away, x_m=0.05, y_ratio=0.5)
    assert r is None


def test_returns_none_when_slice_too_thin():
    c = CloudSlicer(x_tol_m=0.01)
    thin = np.array([[0.05, 0.30, 0.80]], dtype=np.float32)
    r = c.query(thin, x_m=0.05, y_ratio=0.5)
    assert r is None
