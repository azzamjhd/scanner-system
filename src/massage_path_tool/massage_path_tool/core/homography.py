from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

UNIT_QUAD = np.float32([[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]])

@dataclass
class QuadValidation:
    valid: bool
    aspect_ratio: float
    min_angle_deg: float
    max_angle_deg: float
    reason: str

def compute_h_norm(src_pts):
    H, _ = cv2.findHomography(src_pts.astype('float32'), UNIT_QUAD)
    return H

def compute_h_proj(live_pts):
    H, _ = cv2.findHomography(UNIT_QUAD, live_pts.astype('float32'))
    return H

def apply_homography(H, pts):
    if len(pts) == 0:
        return np.zeros((0,2),dtype='float32')
    out = cv2.perspectiveTransform(pts.reshape(-1,1,2).astype('float32'), H)
    return out.reshape(-1,2)

def quad_aspect_ratio(pts):
    top_w = float(np.linalg.norm(pts[1]-pts[0]))
    bot_w = float(np.linalg.norm(pts[2]-pts[3]))
    left_h = float(np.linalg.norm(pts[3]-pts[0]))
    right_h = float(np.linalg.norm(pts[2]-pts[1]))
    w = (top_w+bot_w)/2.0
    h = (left_h+right_h)/2.0
    return float(w/h) if h>1e-6 else 0.0

def _interior_angles_deg(pts):
    n = len(pts)
    angles = []
    for i in range(n):
        a = pts[(i-1)%n].astype(float)
        b = pts[i].astype(float)
        c = pts[(i+1)%n].astype(float)
        ba = a-b; bc = c-b
        denom = np.linalg.norm(ba)*np.linalg.norm(bc)+1e-9
        cos_a = np.dot(ba,bc)/denom
        angles.append(float(np.degrees(np.arccos(np.clip(cos_a,-1.0,1.0)))))
    return angles

def validate_quad(live_pts, ref_aspect_ratio, aspect_tolerance=0.15, min_angle_deg=45.0, max_angle_deg=135.0):
    ar = quad_aspect_ratio(live_pts)
    angles = _interior_angles_deg(live_pts)
    min_a, max_a = min(angles), max(angles)
    if ref_aspect_ratio > 1e-6:
        deviation = abs(ar-ref_aspect_ratio)/ref_aspect_ratio
        if deviation > aspect_tolerance:
            return QuadValidation(False,ar,min_a,max_a,f'Aspect ratio {ar:.2f} deviates {deviation*100:.0f}% from ref {ref_aspect_ratio:.2f}')
    if min_a < min_angle_deg:
        return QuadValidation(False,ar,min_a,max_a,f'Interior angle {min_a:.1f}deg < {min_angle_deg}deg')
    if max_a > max_angle_deg:
        return QuadValidation(False,ar,min_a,max_a,f'Interior angle {max_a:.1f}deg > {max_angle_deg}deg')
    return QuadValidation(True,ar,min_a,max_a,'')
