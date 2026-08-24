#!/usr/bin/env python3
"""
restitch_scan.py
================
Offline re-stitcher matching the live scan_image_recorder_node's compositing.
Canvas width = frame width (no horizontal accumulation). Y placement from
gantry X × pixels_per_mm. Center-strip compositing (one source per pixel).
"""

import argparse
import glob
import json
import os
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ----------------------------- frame loading -----------------------------

def load_session(session_dir: str) -> Tuple[List[np.ndarray], List[float], dict]:
    """Load frames + gantry X coords + full manifest from a session directory."""
    manifest_path = os.path.join(session_dir, 'manifest.json')
    with open(manifest_path) as fh:
        manifest = json.load(fh)

    imgs: List[np.ndarray] = []
    xs: List[float] = []
    for rec in manifest.get('frames', []):
        p = os.path.join(session_dir, rec['file'])
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f'  WARN: could not read {p}', file=sys.stderr)
            continue
        imgs.append(img)
        xs.append(float(rec['x_mm']))
    return imgs, xs, manifest


def load_glob(pattern: str) -> Tuple[List[np.ndarray], List[Optional[float]]]:
    paths = sorted(glob.glob(pattern))
    imgs = [cv2.imread(p, cv2.IMREAD_COLOR) for p in paths]
    imgs = [i for i in imgs if i is not None]
    return imgs, [None] * len(imgs)


# ----------------------------- phase correlation (diagnostics only) -----------------------------

def _hann(shape: Tuple[int, int]) -> np.ndarray:
    return cv2.createHanningWindow((shape[1], shape[0]), cv2.CV_32F)


def measure_shift(prev: np.ndarray,
                  curr: np.ndarray) -> Tuple[float, float, float]:
    """Return (dx, dy, confidence) mapping prev→curr via phase correlation."""
    pg = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY).astype(np.float32)
    cg = cv2.cvtColor(curr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    (dx, dy), response = cv2.phaseCorrelate(pg, cg, _hann(pg.shape))
    # phaseCorrelate returns shift of curr relative to prev:
    #   If curr's content is LOWER (gantry advanced), dy is NEGATIVE.
    # We report positive = "content moves down on canvas".
    return -dx, -dy, float(response)


# ----------------------------- compositing (matches live node) -----------------------------

def composite(imgs: List[np.ndarray],
              xs_mm: List[float],
              ppm: float,
              feather: int) -> np.ndarray:
    """Center-strip composite matching scan_image_recorder_node.

    * canvas_w = frame_w
    * py_starts[i] = (x_i - x_min) * ppm
    * Strip boundaries at midpoints between adjacent py_centers.
    * No overlap averaging — hard-cut strips, with optional feather.
    """
    _, frame_w = imgs[0].shape[:2]
    frame_h, frame_w = imgs[0].shape[:2]

    x_min = xs_mm[0]
    x_max = xs_mm[-1]
    travel_mm = x_max - x_min

    if travel_mm < 0.01:
        return imgs[-1]

    canvas_h = int(travel_mm * ppm) + frame_h
    canvas_w = frame_w

    # Canvas-Y of each frame's top edge and center
    py_starts = [int(round((x - x_min) * ppm)) for x in xs_mm]
    py_centers = [p + frame_h // 2 for p in py_starts]

    # Strip boundaries along canvas Y
    n = len(imgs)
    boundaries = [0]
    for i in range(1, n):
        boundaries.append((py_centers[i - 1] + py_centers[i]) // 2)
    boundaries.append(canvas_h)

    if feather > 0:
        # Weighted blend: each pixel can come from up to 2 frames (linear crossfade)
        result = np.zeros((canvas_h, canvas_w, 3), dtype=np.float64)
        wsum = np.zeros((canvas_h, canvas_w), dtype=np.float64)

        for i, img in enumerate(imgs):
            b_top = boundaries[i]
            b_bot = boundaries[i + 1]
            a_top = max(0, b_top - feather)
            a_bot = min(canvas_h, b_bot + feather)

            if a_bot <= a_top:
                continue

            ys = np.arange(a_top, a_bot, dtype=np.int32)
            src_ys = ys - py_starts[i]
            valid = (src_ys >= 0) & (src_ys < frame_h)
            if not valid.any():
                continue

            ys_v = ys[valid]
            src_v = src_ys[valid]

            # Feather weights: linear ramp over feather pixels from each boundary
            w_y = np.ones(len(ys_v), dtype=np.float64)
            if feather > 0:
                # Top ramp: canvas rows near b_top
                mask_top = ys_v < b_top + feather
                if mask_top.any():
                    w_y[mask_top] = (ys_v[mask_top] - (b_top - feather)) / (2.0 * feather)
                # Bottom ramp: canvas rows near b_bot
                mask_bot = ys_v > b_bot - feather
                if mask_bot.any():
                    w_y[mask_bot] = ((b_bot + feather) - ys_v[mask_bot]) / (2.0 * feather)
                np.clip(w_y, 0.0, 1.0, out=w_y)

            sy0 = int(src_v[0])
            sy1 = int(src_v[-1] + 1)
            cy0 = int(ys_v[0])
            cy1 = int(ys_v[-1] + 1)

            result[cy0:cy1, :] += img[sy0:sy1, :].astype(np.float64) * w_y[:, None, None]
            wsum[cy0:cy1, :] += w_y[:, None]

        mask = wsum > 1e-6
        out = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        out[mask] = np.clip(result[mask] / wsum[mask, None], 0, 255).astype(np.uint8)
        return out
    else:
        # Hard-cut: each pixel from exactly one frame (live node behaviour)
        out = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        for i, img in enumerate(imgs):
            b_top = boundaries[i]
            b_bot = boundaries[i + 1]

            src_top = b_top - py_starts[i]
            src_bot = b_bot - py_starts[i]
            src_top = max(0, src_top)
            src_bot = min(frame_h, src_bot)

            canvas_top = py_starts[i] + src_top
            canvas_bot = py_starts[i] + src_bot

            if src_bot <= src_top or canvas_bot <= canvas_top:
                continue

            out[canvas_top:canvas_bot, :] = img[src_top:src_bot, :]
        return out


# ----------------------------- restitch -----------------------------

def restitch(imgs: List[np.ndarray],
             xs_mm: List[float],
             ppm_prior: float,
             feather: int,
             verbose: bool = True) -> Tuple[np.ndarray, dict]:
    """Re-stitch offline, matching the live node's compositing exactly.

    Phase correlation runs only for diagnostics / scale validation.
    Frame placement is always gantry-driven (no dx accumulation).
    """
    n = len(imgs)
    if n == 1:
        return imgs[0], {'frames': 1}

    # Run phase correlation for diagnostics
    dys = []
    for i in range(1, n):
        _, dy, conf = measure_shift(imgs[i - 1], imgs[i])
        dy = max(0.0, dy)
        dys.append(dy)
        if verbose:
            prior_str = f' prior={abs(xs_mm[i] - xs_mm[i-1]) * ppm_prior:.1f}'
            print(f'  seam {i:3d}: dy={dy:7.2f}px conf={conf:.3f} [phasecorr]{prior_str}')

    # Composite — gantry-driven Y, no X shift
    out = composite(imgs, xs_mm, ppm_prior, feather)

    stats = {
        'frames': n,
        'canvas': [out.shape[1], out.shape[0]],
        'dy_mean': float(np.mean(dys)) if dys else 0.0,
        'dy_min': float(np.min(dys)) if dys else 0.0,
        'dy_max': float(np.max(dys)) if dys else 0.0,
    }
    return out, stats


# ----------------------------- main -----------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session_dir', nargs='?', help='scan_session_* directory')
    ap.add_argument('--frames', help='glob of frame images')
    ap.add_argument('-o', '--output', help='output PNG path')
    ap.add_argument('--feather', type=int, default=0,
                    help='feather ramp height in px per strip edge (default 0 = hard-cut)')
    args = ap.parse_args()

    ppm_prior = None
    if args.frames:
        imgs, xs = load_glob(args.frames)
        out_default = 'restitched.png'
    elif args.session_dir:
        imgs, xs, manifest = load_session(args.session_dir)
        out_default = os.path.join(args.session_dir, 'restitched.png')
        ppm_prior = manifest.get('resolved_pixels_per_mm')
    else:
        ap.error('provide a session_dir or --frames')

    if not imgs:
        print('No frames loaded.', file=sys.stderr)
        return 1

    if not xs or all(x is None for x in xs):
        print('No gantry X coordinates found. Cannot stitch without position data.', file=sys.stderr)
        return 1

    # At this point xs contains only valid floats
    xs_float: List[float] = [float(x) for x in xs if x is not None]
    if len(xs_float) != len(imgs):
        print('Mismatch between frame count and gantry position count.', file=sys.stderr)
        return 1

    if ppm_prior is None or ppm_prior <= 0:
        print('No valid pixels_per_mm in manifest. Cannot stitch without scale.', file=sys.stderr)
        return 1

    # Sort by gantry X (same as live node)
    pairs = sorted(zip(xs_float, imgs), key=lambda p: p[0])
    xs_sorted = [pair[0] for pair in pairs]
    imgs_sorted = [pair[1] for pair in pairs]

    print(f'Loaded {len(imgs_sorted)} frames '
          f'({imgs_sorted[0].shape[1]}x{imgs_sorted[0].shape[0]} px each); '
          f'ppm_prior={ppm_prior:.4f}')
    print(f'  gantry travel: {xs_sorted[0]:.1f} → {xs_sorted[-1]:.1f} mm '
          f'({abs(xs_sorted[-1] - xs_sorted[0]):.1f} mm)')

    out, stats = restitch(imgs_sorted, xs_sorted, ppm_prior,
                          feather=args.feather, verbose=True)

    out_path = args.output or out_default
    cv2.imwrite(out_path, out)
    print(f'\nWrote {out_path}  ({stats["canvas"][0]}x{stats["canvas"][1]} px)')
    print(f'Seam advance dy: mean={stats["dy_mean"]:.1f} '
          f'min={stats["dy_min"]:.1f} max={stats["dy_max"]:.1f} px')

    # --- SCALE VALIDATION ---
    result = out
    total_gantry_mm = abs(xs_sorted[-1] - xs_sorted[0])
    total_pixel_dy = stats['dy_mean'] * (len(imgs_sorted) - 1)
    measured_ppm = total_pixel_dy / total_gantry_mm if total_gantry_mm > 0 else 0

    print('\n=== SCALE VALIDATION (image dy vs gantry Δx) ===')
    print(f'  resolved ppm : {ppm_prior:.4f} px/mm  (used for canvas)')
    print(f'  measured ppm : {measured_ppm:.4f} px/mm  (from {len(imgs_sorted)-1} seams)')
    error_pct = ((measured_ppm - ppm_prior) / ppm_prior) * 100.0 if ppm_prior > 0 else 0
    print(f'  scale error  : {error_pct:+.1f} %')
    print(f'  canvas       : {stats["canvas"][0]}×{stats["canvas"][1]} px')

    if abs(error_pct) < 15.0:
        print('  VERDICT      : OK')
    else:
        print('  VERDICT      : SCALE MISMATCH EXCEEDS THRESHOLD '
              '(expected; depth parallax between body and floor)')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
