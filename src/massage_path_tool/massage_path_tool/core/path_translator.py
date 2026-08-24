#!/usr/bin/env python3
"""path_translator.py — translate 2D massage session → 3D waypoints file.

Method:
  1. Extract 3D body-edge Y positions of the 4 anchors (L/R-Shoulder, L/R-Hip)
     from the point cloud using percentile-based body-edge detection.
  2. For each waypoint, use bilinear interpolation between the 4 body-edge anchors:
       - X (longitudinal / gantry motor): computed from y_stitch via manifest ppm.
       - Y (lateral): bilinear-interpolated u coordinate between body-edge Y.
       - Z (height): KD-Tree nearest-neighbour at (x, y).

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
from scipy.spatial import cKDTree
from massage_path_tool.core.manifest_loader import ManifestLoader
from massage_path_tool.io.session import load_session


def _load_pcd(pcd_path: str) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.shape[0] == 0:
        raise ValueError(f"PCD file is empty: {pcd_path}")
    return pts


def _get_body_edge_y(cloud: np.ndarray, x_m: float, side: str, x_tol_m: float) -> Optional[float]:
    """Return Y of body edge (left=positive or right=negative) at given X.

    Filters out table top points (Z <= 0.015m) to get accurate body boundaries,
    and uses 2nd/98th percentiles to reject noise/outliers.
    """
    mask = (np.abs(cloud[:, 0] - x_m) <= x_tol_m) & (cloud[:, 2] > 0.015)
    sl = cloud[mask]
    if len(sl) < 5:
        # Fallback to no Z-filter if slice is empty (e.g. neck or pillow)
        mask_fallback = np.abs(cloud[:, 0] - x_m) <= x_tol_m
        sl = cloud[mask_fallback]
        if len(sl) < 5:
            return None
    y_vals = sl[:, 1]
    if side == "left":
        return float(np.percentile(y_vals, 98))
    else:
        return float(np.percentile(y_vals, 2))


def _z_at_xy(cloud: np.ndarray, x_m: float, y_m: float, x_tol_m: float) -> Optional[float]:
    """Return Z at (x_m, y_m) using KD-Tree nearest-neighbour in an X-slice."""
    mask = np.abs(cloud[:, 0] - x_m) <= x_tol_m
    sl = cloud[mask]
    if len(sl) < 2:
        return None
    tree = cKDTree(sl[:, :2])
    _, idx = tree.query([x_m, y_m], k=1)
    return float(sl[idx, 2])


class BilinearMapper:
    """Bilinear interpolation between 4 body-edge anchor positions.

    Given the stitched-image (px, py) of each anchor and their 3D Y positions
    at the body edge, map any (x_stitch, y_stitch) → Y (metres, lateral).
    """

    def __init__(self, anchors_2d: dict, y_left_sh: float, y_right_sh: float,
                 y_left_hip: float, y_right_hip: float):
        # Image pixel coords
        self.lsh_px = anchors_2d["left_shoulder"]["px"]
        self.lsh_py = anchors_2d["left_shoulder"]["py"]
        self.rsh_px = anchors_2d["right_shoulder"]["px"]
        self.rsh_py = anchors_2d["right_shoulder"]["py"]
        self.lhip_px = anchors_2d["left_hip"]["px"]
        self.lhip_py = anchors_2d["left_hip"]["py"]
        self.rhip_px = anchors_2d["right_hip"]["px"]
        self.rhip_py = anchors_2d["right_hip"]["py"]

        # 3D body-edge Y at shoulder/hip X levels
        self.y_left_sh = y_left_sh
        self.y_right_sh = y_right_sh
        self.y_left_hip = y_left_hip
        self.y_right_hip = y_right_hip

        # Vertical spans
        self.py_span = (abs(self.lhip_py - self.lsh_py) + abs(self.rhip_py - self.rsh_py)) / 2.0

    def map_y(self, x_stitch: float, y_stitch: float) -> float:
        """Map (x_stitch, y_stitch) → Y (metres, lateral direction)."""
        # Normalized V between shoulder and hip in image
        v = float(np.clip((y_stitch - self.lsh_py) / self.py_span, 0.0, 1.0))

        # Left/right anchor px at this V (bilinear in image space)
        left_px = self.lsh_px + v * (self.lhip_px - self.lsh_px)
        right_px = self.rsh_px + v * (self.rhip_px - self.rsh_px)
        span_px = right_px - left_px
        if span_px <= 0.0:
            span_px = 1.0  # fallback

        # u coordinate between left/right body edge in image
        u = float(np.clip((x_stitch - left_px) / span_px, 0.0, 1.0))

        # Same V in 3D (shoulder-to-hip)
        y_left = self.y_left_sh + v * (self.y_left_hip - self.y_left_sh)
        y_right = self.y_right_sh + v * (self.y_right_hip - self.y_right_sh)

        # u=0 → left body edge (positive Y), u=1 → right body edge (negative Y)
        return y_left + u * (y_right - y_left)


def translate(
    session_path: str,
    manifest_path: str,
    pcd_path: str,
    output_path: str,
    stitched_H: int = 2402,
    x_tol_mm: float = 5.0,
    method: str = "proportional_table",
) -> dict:
    session = load_session(session_path)
    with open(manifest_path) as f:
        manifest = json.load(f)
    cloud = _load_pcd(pcd_path)
    anchors_2d = session.get("anchor_landmarks", {})
    loader = ManifestLoader(manifest, stitched_H=stitched_H)
    x_tol_m = x_tol_mm / 1000.0

    # ── 1. Setup mapper ────────────────────────────────────────────
    # For proportional_table: use global body bounds computed from the entire point cloud (Z > 0.015m)
    body_pts = cloud[cloud[:, 2] > 0.015]
    if len(body_pts) < 10:
        body_pts = cloud  # fallback to entire cloud if empty
    y_min_global = float(np.percentile(body_pts[:, 1], 2))
    y_max_global = float(np.percentile(body_pts[:, 1], 98))

    # Compute the horizontal (x_stitch) span the body actually occupies in the image
    # so lateral (Y) mapping uses the real body bounds, not the full 640px canvas.
    _xs_all: list[float] = []
    for dp in session.get("discrete_points", []):
        if dp.get("x_stitch") is not None:
            _xs_all.append(float(dp["x_stitch"]))
    for path in session.get("paths", []):
        for pt in path.get("points", []):
            if pt.get("x_stitch") is not None:
                _xs_all.append(float(pt["x_stitch"]))
    body_x_min = float(min(_xs_all)) if _xs_all else 0.0
    body_x_max = float(max(_xs_all)) if _xs_all else 640.0
    _x_span = body_x_max - body_x_min

    bilinear = None
    if method == "bilinear_anchor":
        # Required anchor keys
        anchor_info = {
            "left_shoulder": {"side": "left"},
            "right_shoulder": {"side": "right"},
            "left_hip": {"side": "left"},
            "right_hip": {"side": "right"},
        }

        body_edges = {}  # session_key -> y_body_edge
        for session_key, info in anchor_info.items():
            a = anchors_2d.get(session_key, {})
            if "px" not in a or "py" not in a:
                print(f"[path_translator] WARNING: missing anchor {session_key}")
                break
            x_m = loader.y_stitch_to_motor_mm(a["py"]) / 1000.0
            y_edge = _get_body_edge_y(cloud, x_m, info["side"], x_tol_m)
            if y_edge is None:
                print(f"[path_translator] WARNING: cannot find body edge for {session_key} at X={x_m:.3f}")
                break
            body_edges[session_key] = y_edge
            print(f"[path_translator]   {session_key}: X={x_m:.3f}, Y_edge={y_edge:.4f}")

        has_all_edges = len(body_edges) == 4
        if has_all_edges:
            bilinear = BilinearMapper(
                anchors_2d,
                body_edges["left_shoulder"], body_edges["right_shoulder"],
                body_edges["left_hip"], body_edges["right_hip"],
            )
            print(f"[path_translator] Using bilinear anchor interpolation")
        else:
            print(f"[path_translator] WARNING: falling back to proportional_table method")
            method = "proportional_table"
    def _translate_proportional_table(eid: str, label: str, x_s: float, y_s: float) -> Optional[dict]:
        x_m = loader.y_stitch_to_motor_mm(float(y_s)) / 1000.0
        # Normalise x_stitch within actual body horizontal span, then map
        # x_stitch=body_left_edge → y_max (person's left = positive Y)
        # x_stitch=body_right_edge → y_min (person's right = negative Y)
        ratio = (float(x_s) - body_x_min) / _x_span if _x_span > 0 else 0.5
        ratio = float(np.clip(ratio, 0.0, 1.0))
        y_m = y_max_global - ratio * (y_max_global - y_min_global)
        z_m = _z_at_xy(cloud, x_m, y_m, x_tol_m)
        if z_m is None:
            return None
        # We still count slice points if slice is non-empty for diagnostics
        mask = np.abs(cloud[:, 0] - x_m) <= x_tol_m
        n_pts = int(np.sum(mask))
        return {"id": eid, "label": label, "x": round(x_m, 6),
                "y": round(y_m, 6), "z": round(z_m, 6), "n_slice_pts": n_pts}

    out_discrete: list[dict] = []
    out_paths: list[dict] = []
    skipped: list[dict] = []

    def _translate_bilinear(eid: str, label: str, x_s: float, y_s: float) -> Optional[dict]:
        x_m = loader.y_stitch_to_motor_mm(float(y_s)) / 1000.0
        y_m = bilinear.map_y(float(x_s), float(y_s))
        z_m = _z_at_xy(cloud, x_m, y_m, x_tol_m)
        if z_m is None:
            return None
        return {"id": eid, "label": label, "x": round(x_m, 6),
                "y": round(y_m, 6), "z": round(z_m, 6), "n_slice_pts": 0}

    def _translate_legacy(eid: str, label: str, x_s: float, y_s: float, u: float) -> Optional[dict]:
        from massage_path_tool.core.cloud_slicer import CloudSlicer
        slicer = CloudSlicer(x_tol_m=x_tol_m)
        x_m = loader.y_stitch_to_motor_mm(float(y_s)) / 1000.0
        res = slicer.query(cloud, x_m, float(u))
        if res is None:
            return None
        return {"id": eid, "label": label, "x": round(res.x_m, 6),
                "y": round(res.y_m, 6), "z": round(res.z_m, 6),
                "n_slice_pts": res.n_slice_pts}

    # ── 2. Translate discrete points ───────────────────────────────
    for dp in session.get("discrete_points", []):
        eid = dp.get("id", "?")
        label = dp.get("label", "?")
        x_s = dp.get("x_stitch")
        y_s = dp.get("y_stitch")

        if method == "proportional_table":
            if x_s is None or y_s is None:
                skipped.append({"id": eid, "reason": "no x_stitch/y_stitch"})
                continue
            r = _translate_proportional_table(eid, label, x_s, y_s)
        elif method == "bilinear_anchor" and bilinear is not None:
            if x_s is None or y_s is None:
                skipped.append({"id": eid, "reason": "no x_stitch/y_stitch"})
                continue
            r = _translate_bilinear(eid, label, x_s, y_s)
        else:
            u = dp.get("u")
            if x_s is None or y_s is None or u is None:
                skipped.append({"id": eid, "reason": "no x_stitch/y_stitch/u"})
                continue
            r = _translate_legacy(eid, label, x_s, y_s, u)

        if r:
            out_discrete.append(r)
        else:
            x_mm = loader.y_stitch_to_motor_mm(float(y_s)) if y_s else 0
            skipped.append({"id": eid, "reason": f"no Z at X={x_mm/1000:.3f}"})

    # ── 3. Translate path waypoints ────────────────────────────────
    for path in session.get("paths", []):
        waypoints = []
        for i, pt in enumerate(path.get("points", [])):
            sub_id = f"{path.get('id', '?')}[{i}]"
            x_s = pt.get("x_stitch")
            y_s = pt.get("y_stitch")

            if method == "proportional_table":
                if x_s is None or y_s is None:
                    continue
                r = _translate_proportional_table(sub_id, sub_id, x_s, y_s)
            elif method == "bilinear_anchor" and bilinear is not None:
                if x_s is None or y_s is None:
                    continue
                r = _translate_bilinear(sub_id, sub_id, x_s, y_s)
            else:
                u = pt.get("u")
                if x_s is None or y_s is None or u is None:
                    continue
                r = _translate_legacy(sub_id, sub_id, x_s, y_s, u)

            if r:
                waypoints.append({"x": r["x"], "y": r["y"], "z": r["z"], "n_slice_pts": r.get("n_slice_pts", 0)})
        out_paths.append({"id": path.get("id", "?"), "label": path.get("label", "?"), "waypoints": waypoints})

    output = {
        "frame": "base_link",
        "units": "metres",
        "source": {
            "session": str(Path(session_path).resolve()),
            "manifest": str(Path(manifest_path).resolve()),
            "pcd": str(Path(pcd_path).resolve()),
        },
        "x_tolerance_mm": x_tol_mm,
        "method": method,
        "discrete_points": out_discrete,
        "paths": out_paths,
        "skipped": skipped,
    }
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(output, f, indent=2)
    print(
        f"[path_translator] Done: {len(out_discrete)} points, "
        f"{sum(len(p['waypoints']) for p in out_paths)} waypoints across "
        f"{len(out_paths)} paths, {len(skipped)} skipped → {output_path}"
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Translate 2D massage session → 3D waypoints JSON (no ROS required)"
    )
    parser.add_argument("--session", required=True, help="massage_session.json")
    parser.add_argument("--manifest", required=True, help="scan_image_recorder_node manifest.json")
    parser.add_argument("--pcd", required=True, help="scan_YYYYMMDD.pcd (binary, base_link frame)")
    parser.add_argument("--stitched-h", type=int, default=2402, help="stitched image height in px (default 2402)")
    parser.add_argument("--output", default="trajectories_3d.json")
    parser.add_argument("--x-tol-mm", type=float, default=5.0, help="X slice half-window in mm (default 5)")
    parser.add_argument(
        "--method",
        choices=["proportional_table", "bilinear_anchor", "slice_percentile"],
        default="proportional_table",
        help="translation method (default: proportional_table)"
    )
    args = parser.parse_args()
    try:
        translate(
            session_path=args.session,
            manifest_path=args.manifest,
            pcd_path=args.pcd,
            output_path=args.output,
            stitched_H=args.stitched_h,
            x_tol_mm=args.x_tol_mm,
            method=args.method,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
