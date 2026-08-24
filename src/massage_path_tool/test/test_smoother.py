import numpy as np
import pytest
from massage_path_tool.core.smoother import smooth_path

def _line(n=20):
    return np.column_stack([np.linspace(0,100,n), np.linspace(0,200,n)])

def test_output_shape_default():
    assert smooth_path(_line()).shape == (200, 2)

def test_output_shape_custom_n():
    assert smooth_path(_line(), n_out=50).shape == (50, 2)

def test_endpoints_preserved():
    r = smooth_path(_line(20), n_out=100)
    np.testing.assert_allclose(r[0], [0.0, 0.0], atol=1.0)
    np.testing.assert_allclose(r[-1], [100.0, 200.0], atol=1.0)

def test_too_few_points_raises():
    with pytest.raises(ValueError):
        smooth_path(np.array([[0,0],[1,1],[2,2]], dtype=float))

def test_noisy_line():
    rng = np.random.default_rng(42)
    pts = _line(40) + rng.normal(0, 3.0, (40, 2))
    assert smooth_path(pts, n_out=100, smoothing=50.0).shape == (100, 2)
