#!/usr/bin/env python3
"""path_translator.py — translate 2D massage session → 3D waypoints file.

Usage:
    python -m massage_path_tool.core.path_translator \\
        --session  massage_session.json \\
        --manifest scan_session_<ts>/manifest.json \\
        --pcd      scan_YYYYMMDD_HHMMSS.pcd \\
        --frame-h  480 \\
        --output   trajectories_3d.json \\
        --x-tol-mm 5.0

No ROS dependency — runs with plain python3.
"""
from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path
from typing import Optional
import numpy as np
import open3d as o3d
from massage_path_tool.core.manifest_loader import ManifestLoader
from massage_path_tool.core.cloud_slicer import CloudSlicer
from massage_path_tool.io.session import load_session


def _load_pcd(pcd_path: str) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.shape[0] == 0:
        raise ValueError(f"PCD file is empty: {pcd_path}")
    return pts


def _y_ratio(y_stitch: float, anchors: dict) -> float:
    if not anchors:
        return 0.5
    y_shoulder = min(
        anchors.get('left_shoulder',  {}).get('py', float('inf')),
        anchors.get('right_shoulder', {}).get('py', float('inf')),
    )
    y_hip = max(
        anchors.get('left_hip',  {}).get('py', 0.0),
        anchors.get('right_hip', {}).get('py', 0.0),
    )
    span = y_hip - y_shoulder
    if span < 1.0:
        return 0.5
    return float(np.clip((y_stitch - y_shoulder) / span, -0.2, 1.2))


def translate(
    session_path: str,
    manifest_path: str,
    pcd_path: str,
    output_path: str,
    frame_H: int = 480,
    x_tol_mm: float = 5.0,
) -> dict:
    session = load_session(session_path)
    with open(manifest_path) as f:
        manifest = json.load(f)
    cloud = _load_pcd(pcd_path)
    anchors = session.get('anchor_landmarks', {})
    loader = ManifestLoader(manifest, frame_H=frame_H)
    slicer = CloudSlicer(x_tol_m=x_tol_mm / 1000.0)
    out_discrete: list[dict] = []
    out_paths: list[dict] = []
    skipped: list[dict] = []

    def _translate_point(eid: str, label: str, x_stitch, y_stitch) -> Optional[dict]:
        if x_stitch is None or y_stitch is None:
            skipped.append({'id': eid, 'reason': 'no x_stitch/y_stitch (v1 session — re-author)'})
            return None
        x_mm = loader.y_stitch_to_motor_mm(float(y_stitch))
        x_m  = x_mm / 1000.0
        yr   = _y_ratio(float(y_stitch), anchors)
        res  = slicer.query(cloud, x_m=x_m, y_ratio=yr)
        if res is None:
            skipped.append({'id': eid, 'reason': f'no cloud slice at X={x_m:.3f}m (0 pts in ±{x_tol_mm:.0f}mm window)'})
            return None
        return {'id': eid, 'label': label, 'x': round(res.x_m, 6), 'y': round(res.y_m, 6), 'z': round(res.z_m, 6), 'n_slice_pts': res.n_slice_pts}

    for dp in session.get('discrete_points', []):
        r = _translate_point(dp.get('id', '?'), dp.get('label', '?'), dp.get('x_stitch'), dp.get('y_stitch'))
        if r:
            out_discrete.append(r)
    for path in session.get('paths', []):
        waypoints = []
        for i, pt in enumerate(path.get('points', [])):
            sub_id = f"{path.get('id', '?')}[{i}]"
            r = _translate_point(sub_id, sub_id, pt.get('x_stitch'), pt.get('y_stitch'))
            if r:
                waypoints.append({'x': r['x'], 'y': r['y'], 'z': r['z'], 'n_slice_pts': r['n_slice_pts']})
        out_paths.append({'id': path.get('id', '?'), 'label': path.get('label', '?'), 'waypoints': waypoints})
    output = {
        'frame': 'base_link',
        'units': 'metres',
        'source': {'session': str(Path(session_path).resolve()), 'manifest': str(Path(manifest_path).resolve()), 'pcd': str(Path(pcd_path).resolve())},
        'x_tolerance_mm': x_tol_mm,
        'discrete_points': out_discrete,
        'paths': out_paths,
        'skipped': skipped,
    }
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)
    print(f"[path_translator] Done: {len(out_discrete)} points, {sum(len(p['waypoints']) for p in out_paths)} waypoints across {len(out_paths)} paths, {len(skipped)} skipped → {output_path}")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description='Translate 2D massage session → 3D waypoints JSON (no ROS required)')
    parser.add_argument('--session',  required=True, help='massage_session.json')
    parser.add_argument('--manifest', required=True, help='scan_image_recorder_node manifest.json')
    parser.add_argument('--pcd',      required=True, help='scan_YYYYMMDD.pcd (binary, base_link frame)')
    parser.add_argument('--frame-h', type=int, default=480, help='source camera frame height in px (default 480)')
    parser.add_argument('--output',   default='trajectories_3d.json')
    parser.add_argument('--x-tol-mm', type=float, default=5.0, help='X slice half-window in mm (default 5)')
    args = parser.parse_args()
    try:
        translate(session_path=args.session, manifest_path=args.manifest, pcd_path=args.pcd, output_path=args.output, frame_H=args.frame_h, x_tol_mm=args.x_tol_mm)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
