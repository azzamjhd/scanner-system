"""Tests for path_translator translate() — using new linear y_stitch→X_motor_mm.

ManifestLoader now maps y_stitch in stitched-image coords to gantry X via
simple linear interpolation: y=0 (head / top) → X_min, y=H (feet / bottom) → X_max.
"""
import json, os, numpy as np, pytest
import open3d as o3d

SESSION_V2 = {
    'schema_version': 2,
    'source_image': '',
    'calibration_aspect_ratio': 1.4,
    'anchor_landmarks': {
        'left_shoulder':  {'px': 100.0, 'py': 200.0, 'u': 0.0, 'v': 0.0},
        'right_shoulder': {'px': 500.0, 'py': 205.0, 'u': 1.0, 'v': 0.0},
        'right_hip':      {'px': 480.0, 'py': 700.0, 'u': 1.0, 'v': 1.0},
        'left_hip':       {'px': 150.0, 'py': 695.0, 'u': 0.0, 'v': 1.0},
    },
    'discrete_points': [
        {'id': 'p1', 'label': 'p1', 'u': 0.5, 'v': 0.4, 'x_stitch': 300.0, 'y_stitch': 380.0},
    ],
    'paths': [
        {'id': 'path1', 'label': 'path1', 'points': [
            {'u': 0.4, 'v': 0.3, 'x_stitch': 240.0, 'y_stitch': 340.0},
            {'u': 0.6, 'v': 0.5, 'x_stitch': 360.0, 'y_stitch': 420.0},
        ]},
    ],
}

MANIFEST = {
    'resolved_pixels_per_mm': 2.0,
    'frame_count': 5,
    'frames': [
        {'file': 'frame_0000.png', 'x_mm': 100.0},
        {'file': 'frame_0001.png', 'x_mm': 110.0},
        {'file': 'frame_0002.png', 'x_mm': 120.0},
        {'file': 'frame_0003.png', 'x_mm': 130.0},
        {'file': 'frame_0004.png', 'x_mm': 140.0},
    ],
}


def make_synthetic_pcd(path: str) -> None:
    rng = np.random.default_rng(0)
    n = 8000
    xs = rng.uniform(0.095, 0.145, n).astype(np.float32)
    ys = rng.uniform(0.10, 0.50, n).astype(np.float32)
    zs = np.full(n, 0.80, dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack([xs, ys, zs]))
    o3d.io.write_point_cloud(path, pcd)


@pytest.fixture
def tmp_files(tmp_path):
    sf = str(tmp_path / 'session.json')
    mf = str(tmp_path / 'manifest.json')
    pf = str(tmp_path / 'scan.pcd')
    of = str(tmp_path / 'out.json')
    with open(sf, 'w') as f: json.dump(SESSION_V2, f)
    with open(mf, 'w') as f: json.dump(MANIFEST, f)
    make_synthetic_pcd(pf)
    return sf, mf, pf, of, 480  # 480 = stitched_H


def test_cli_produces_output_file(tmp_files):
    sf, mf, pf, of, sh = tmp_files
    from massage_path_tool.core.path_translator import translate
    translate(session_path=sf, manifest_path=mf, pcd_path=pf, output_path=of, stitched_H=sh, x_tol_mm=5.0)
    assert os.path.exists(of)


def test_output_has_correct_structure(tmp_files):
    sf, mf, pf, of, sh = tmp_files
    from massage_path_tool.core.path_translator import translate
    translate(session_path=sf, manifest_path=mf, pcd_path=pf, output_path=of, stitched_H=sh, x_tol_mm=5.0)
    with open(of) as f: out = json.load(f)
    assert out['frame'] == 'base_link'
    assert 'discrete_points' in out
    assert 'paths' in out
    assert 'skipped' in out


def test_discrete_point_translated(tmp_files):
    sf, mf, pf, of, sh = tmp_files
    from massage_path_tool.core.path_translator import translate
    translate(session_path=sf, manifest_path=mf, pcd_path=pf, output_path=of, stitched_H=sh, x_tol_mm=5.0, method="proportional_table")
    with open(of) as f: out = json.load(f)
    assert len(out['discrete_points']) == 1
    p = out['discrete_points'][0]
    assert p['id'] == 'p1'
    # y_stitch=380, stitched_H=480 → alpha=0.7917
    # x_mm = 100 + 0.7917*40 = 131.67mm → 0.1317m
    assert 0.12 < p['x'] < 0.14
    # body x_stitch min=240, max=360, span=120
    # ratio = (300-240)/120 = 0.5
    # y_min ≈ 0.108, y_max ≈ 0.492 → y ≈ 0.300
    assert 0.25 < p['y'] < 0.35
    # z ≈ 0.80 (all PCD points at z=0.80)
    assert 0.75 < p['z'] < 0.85


def test_bilinear_anchor_fallback(tmp_files):
    sf, mf, pf, of, sh = tmp_files
    from massage_path_tool.core.path_translator import translate
    translate(session_path=sf, manifest_path=mf, pcd_path=pf, output_path=of, stitched_H=sh, x_tol_mm=5.0, method="bilinear_anchor")
    with open(of) as f: out = json.load(f)
    assert len(out['discrete_points']) == 1
    assert out['method'] == 'bilinear_anchor'


def test_path_translated(tmp_files):
    sf, mf, pf, of, sh = tmp_files
    from massage_path_tool.core.path_translator import translate
    translate(session_path=sf, manifest_path=mf, pcd_path=pf, output_path=of, stitched_H=sh, x_tol_mm=5.0)
    with open(of) as f: out = json.load(f)
    assert len(out['paths']) == 1
    assert len(out['paths'][0]['waypoints']) == 2


def test_v1_session_points_skipped(tmp_files):
    sf, mf, pf, of, sh = tmp_files
    with open(sf) as f: s = json.load(f)
    for dp in s['discrete_points']:
        dp.pop('x_stitch', None); dp.pop('y_stitch', None)
    for p in s['paths']:
        for pt in p['points']:
            pt.pop('x_stitch', None); pt.pop('y_stitch', None)
    s['schema_version'] = 1
    with open(sf, 'w') as f: json.dump(s, f)
    from massage_path_tool.core.path_translator import translate
    translate(session_path=sf, manifest_path=mf, pcd_path=pf, output_path=of, stitched_H=sh, x_tol_mm=5.0)
    with open(of) as f: out = json.load(f)
    assert len(out['discrete_points']) == 0
    assert len(out['skipped']) >= 1
