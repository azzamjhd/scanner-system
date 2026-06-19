import numpy as np
import pytest
from massage_path_tool.core.homography import UNIT_QUAD,apply_homography,compute_h_norm,compute_h_proj,quad_aspect_ratio,validate_quad
SRC = np.float32([[100,50],[300,50],[300,250],[100,250]])
def test_h_norm_maps_src_to_unit():
    H = compute_h_norm(SRC)
    assert np.allclose(apply_homography(H,SRC),UNIT_QUAD,atol=1e-5)
def test_h_proj_maps_unit_to_src():
    H = compute_h_proj(SRC)
    assert np.allclose(apply_homography(H,UNIT_QUAD),SRC,atol=1e-4)
def test_roundtrip():
    Hn=compute_h_norm(SRC);Hp=compute_h_proj(SRC)
    pt=np.float32([[200,150]])
    assert np.allclose(apply_homography(Hp,apply_homography(Hn,pt)),pt,atol=1e-4)
def test_apply_empty():
    H=compute_h_norm(SRC)
    r=apply_homography(H,np.zeros((0,2),dtype='float32'))
    assert r.shape==(0,2)
def test_validate_good():
    pts=np.float32([[0,0],[200,0],[200,100],[0,100]])
    assert validate_quad(pts,quad_aspect_ratio(pts)).valid
def test_validate_bad_aspect():
    pts=np.float32([[0,0],[200,0],[200,100],[0,100]])
    assert not validate_quad(pts,ref_aspect_ratio=5.0).valid
def test_validate_zero_ref_skips_ar():
    pts=np.float32([[0,0],[200,0],[200,100],[0,100]])
    assert validate_quad(pts,ref_aspect_ratio=0.0).valid
def test_quad_ar_rectangle():
    pts=np.float32([[0,0],[200,0],[200,100],[0,100]])
    assert abs(quad_aspect_ratio(pts)-2.0)<1e-5
