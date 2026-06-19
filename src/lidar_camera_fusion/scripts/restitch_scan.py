#!/usr/bin/env python3
"""
restitch_scan.py
================
Offline re-stitcher for scan_image_recorder_node sessions.

Why this exists
---------------
The live node composites with one global pixels_per_mm = fx / Z derived from a
single LiDAR median distance. A human body has 100-200 mm of depth relief, so
that single scale is wrong everywhere except at the assumed depth: features
near the camera (torso) advance faster than the canvas -> tearing; features far
from it (floor) advance slower -> duplicated/compressed banding. The live node
also butts thin hard-cut strips together, turning any residual mismatch into a
visible line every few pixels.

This tool fixes both:
  1. Per-seam registration. Each consecutive frame pair's vertical (and small
     horizontal) shift is MEASURED from image content via sub-pixel phase
     correlation, validated against the gantry-X prior, with an ORB/affine
     fallback. No single global scale -> parallax error is absorbed seam by seam.
  2. Feather blending. Overlap regions are linearly cross-faded instead of
     hard-cut, so residual sub-pixel mismatch is smeared out instead of drawn
     as a seam line.

Input: a session dir written by scan_image_recorder_node containing
frame_*.png + manifest.json. Falls back to lexical frame order + manifest x_mm
if present, or pure lexical order if no manifest.

Usage:
    restitch_scan.py SESSION_DIR [-o OUT.png] [--no-blend] [--feather N]
    restitch_scan.py --frames 'dir/frame_*.png' [-o OUT.png]
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

def load_session(session_dir: str) -> Tuple[List[np.ndarray], List[Optional[float]]]:
    manifest_path = os.path.join(session_dir, 'manifest.json')
    imgs: List[np.ndarray] = []
    xs: List[Optional[float]] = []
    if os.path.exists(manifest_path):
        with open(manifest_path) as fh:
            man = json.load(fh)
        for rec in man.get('frames', []):
            p = os.path.join(session_dir, rec['file'])
            img = cv2.imread(p, cv2.IMREAD_COLOR)
            if img is None:
                print(f'  WARN: could not read {p}', file=sys.stderr)
                continue
            imgs.append(img)
            xs.append(rec.get('x_mm'))
    else:
        paths = sorted(glob.glob(os.path.join(session_dir, 'frame_*.png')))
        for p in paths:
            img = cv2.imread(p, cv2.IMREAD_COLOR)
            if img is not None:
                imgs.append(img)
                xs.append(None)
    return imgs, xs


def load_glob(pattern: str) -> Tuple[List[np.ndarray], List[Optional[float]]]:
    paths = sorted(glob.glob(pattern))
    imgs = [cv2.imread(p, cv2.IMREAD_COLOR) for p in paths]
    imgs = [i for i in imgs if i is not None]
    return imgs, [None] * len(imgs)


# ----------------------------- registration -----------------------------

def _hann(shape: Tuple[int, int]) -> np.ndarray:
    return cv2.createHanningWindow((shape[1], shape[0]), cv2.CV_32F)


def measure_shift(prev_bgr: np.ndarray,
                  curr_bgr: np.ndarray,
                  x_prior_px: Optional[float]) -> Tuple[float, float, float, str]:
    """Return (dx, dy, confidence, method) mapping prev -> curr.

    dy>0 means curr's content sits LOWER (larger row index) than prev, i.e. the
    gantry advanced. Phase correlation first; validated against the x prior;
    ORB affine fallback; finally the prior itself.
    """
    pg = cv2.cvtColor(prev_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    cg = cv2.cvtColor(curr_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    win = _hann(pg.shape)
    (dx, dy), response = cv2.phaseCorrelate(pg, cg, win)
    # phaseCorrelate returns shift of curr relative to prev; a downward gantry
    # advance shows up as the content moving up in curr -> dy negative. We want
    # canvas advance (positive down), so invert.
    adv_dy = -dy
    adv_dx = -dx

    # Validate against the gantry prior when available: reject phase-corr if it
    # disagrees with physics by a large margin (wrap-around / repeated texture).
    if x_prior_px is not None and x_prior_px > 1.0:
        if abs(adv_dy - x_prior_px) > 0.6 * x_prior_px + 15.0:
            odx, ody, ok = _orb_shift(prev_bgr, curr_bgr)
            if ok and abs(ody - x_prior_px) <= 0.6 * x_prior_px + 15.0:
                return odx, ody, 0.5, 'orb'
            return 0.0, float(x_prior_px), 0.3, 'prior'

    if response < 0.05:
        odx, ody, ok = _orb_shift(prev_bgr, curr_bgr)
        if ok:
            return odx, ody, 0.5, 'orb'
        if x_prior_px is not None:
            return 0.0, float(x_prior_px), 0.3, 'prior'
    return adv_dx, adv_dy, float(response), 'phasecorr'


def _orb_shift(a: np.ndarray, b: np.ndarray) -> Tuple[float, float, bool]:
    try:
        ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
        gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create(nfeatures=1500)
        ka, da = orb.detectAndCompute(ga, None)
        kb, db = orb.detectAndCompute(gb, None)
        if da is None or db is None or len(ka) < 6 or len(kb) < 6:
            return 0.0, 0.0, False
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        m = bf.match(da, db)
        if len(m) < 6:
            return 0.0, 0.0, False
        pa = np.float32([ka[x.queryIdx].pt for x in m])
        pb = np.float32([kb[x.trainIdx].pt for x in m])
        M, inl = cv2.estimateAffinePartial2D(pa, pb, method=cv2.RANSAC)
        if M is None or inl is None or int(inl.sum()) < 6:
            return 0.0, 0.0, False
        # M maps a->b; gantry advance = -translation (content moves up as we go down)
        return -float(M[0, 2]), -float(M[1, 2]), True
    except cv2.error:
        return 0.0, 0.0, False


# ----------------------------- compositing -----------------------------

def composite(imgs: List[np.ndarray],
              offsets: List[Tuple[float, float]],
              feather: int,
              blend: bool) -> np.ndarray:
    """offsets[i] = cumulative (ox, oy) of frame i top-left on the canvas."""
    h, w = imgs[0].shape[:2]
    oxs = [o[0] for o in offsets]
    oys = [o[1] for o in offsets]
    min_x, min_y = min(oxs), min(oys)
    sh = [(ox - min_x, oy - min_y) for ox, oy in offsets]
    canvas_w = int(np.ceil(max(ox + w for ox, _ in sh)))
    canvas_h = int(np.ceil(max(oy + h for _, oy in sh)))

    acc = np.zeros((canvas_h, canvas_w, 3), np.float64)
    wsum = np.zeros((canvas_h, canvas_w), np.float64)

    for i, img in enumerate(imgs):
        ox = int(round(sh[i][0]))
        oy = int(round(sh[i][1]))
        ih, iw = img.shape[:2]
        # Per-frame weight: feathered ramp along Y (rows) so vertically adjacent
        # frames cross-fade in their overlap. Full weight in the body, ramping
        # to ~0 at top/bottom edges.
        if blend and feather > 0 and ih > 2 * feather:
            wy = np.ones(ih, np.float64)
            ramp = np.linspace(0.0, 1.0, feather, endpoint=False)
            wy[:feather] = ramp
            wy[-feather:] = ramp[::-1]
        else:
            wy = np.ones(ih, np.float64)
        wt = np.repeat(wy[:, None], iw, axis=1)

        y2 = min(oy + ih, canvas_h)
        x2 = min(ox + iw, canvas_w)
        ph, pw = y2 - oy, x2 - ox
        acc[oy:y2, ox:x2] += img[:ph, :pw].astype(np.float64) * wt[:ph, :pw, None]
        wsum[oy:y2, ox:x2] += wt[:ph, :pw]

    out = np.zeros((canvas_h, canvas_w, 3), np.uint8)
    mask = wsum > 1e-6
    out[mask] = np.clip(acc[mask] / wsum[mask, None], 0, 255).astype(np.uint8)
    return out


def restitch(imgs: List[np.ndarray],
             xs_mm: List[Optional[float]],
             ppm_prior: Optional[float],
             feather: int,
             blend: bool,
             verbose: bool = True) -> Tuple[np.ndarray, dict]:
    if len(imgs) == 1:
        return imgs[0], {'frames': 1, 'methods': {}}

    offsets: List[Tuple[float, float]] = [(0.0, 0.0)]
    methods = {}
    dys = []
    for i in range(1, len(imgs)):
        prior = None
        if (ppm_prior and xs_mm[i] is not None and xs_mm[i - 1] is not None):
            prior = abs(xs_mm[i] - xs_mm[i - 1]) * ppm_prior
        dx, dy, conf, method = measure_shift(imgs[i - 1], imgs[i], prior)
        # clamp dy to non-negative advance (monotonic scan); allow small dx
        dy = max(0.0, dy)
        dx = float(np.clip(dx, -0.15 * imgs[0].shape[1], 0.15 * imgs[0].shape[1]))
        offsets.append((offsets[-1][0] + dx, offsets[-1][1] + dy))
        methods[method] = methods.get(method, 0) + 1
        dys.append(dy)
        if verbose:
            print(f'  seam {i:3d}: dy={dy:7.2f}px dx={dx:6.2f}px '
                  f'conf={conf:.3f} [{method}]'
                  + (f' prior={prior:.1f}' if prior else ''))

    out = composite(imgs, offsets, feather, blend)
    stats = {
        'frames': len(imgs),
        'methods': methods,
        'dy_mean': float(np.mean(dys)) if dys else 0.0,
        'dy_min': float(np.min(dys)) if dys else 0.0,
        'dy_max': float(np.max(dys)) if dys else 0.0,
        'canvas': [out.shape[1], out.shape[0]],
    }
    return out, stats


# ----------------------------- main -----------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir', nargs='?', help='scan_session_* directory')
    ap.add_argument('--frames', help='glob of frame images instead of a session dir')
    ap.add_argument('-o', '--output', help='output PNG path')
    ap.add_argument('--feather', type=int, default=24,
                    help='feather ramp height in px per edge (default 24)')
    ap.add_argument('--no-blend', action='store_true', help='hard-cut, no feather')
    args = ap.parse_args()

    ppm_prior = None
    if args.frames:
        imgs, xs = load_glob(args.frames)
        out_default = 'restitched.png'
    elif args.session_dir:
        imgs, xs = load_session(args.session_dir)
        out_default = os.path.join(args.session_dir, 'restitched.png')
        mp = os.path.join(args.session_dir, 'manifest.json')
        if os.path.exists(mp):
            with open(mp) as fh:
                ppm_prior = json.load(fh).get('resolved_pixels_per_mm')
    else:
        ap.error('provide a session_dir or --frames')

    if not imgs:
        print('No frames loaded.', file=sys.stderr)
        return 1

    print(f'Loaded {len(imgs)} frames '
          f'({imgs[0].shape[1]}x{imgs[0].shape[0]} px each); '
          f'ppm_prior={ppm_prior}')
    out, stats = restitch(imgs, xs, ppm_prior,
                          feather=args.feather, blend=not args.no_blend)
    out_path = args.output or out_default
    cv2.imwrite(out_path, out)
    print(f'\nWrote {out_path}  ({stats["canvas"][0]}x{stats["canvas"][1]} px)')
    print(f'Registration methods: {stats["methods"]}')
    print(f'Seam advance dy: mean={stats["dy_mean"]:.1f} '
          f'min={stats["dy_min"]:.1f} max={stats["dy_max"]:.1f} px')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
