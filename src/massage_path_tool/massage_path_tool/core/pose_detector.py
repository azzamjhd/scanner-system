"""MediaPipe Pose wrapper — returns the 4-point anchor quad in pixel coords.

Uses the MediaPipe **Tasks** API (PoseLandmarker + .task model file), because
the installed mediapipe build (0.10.35) ships tasks-only — it has no
`mediapipe.solutions` module. This matches the approach already used by the
sibling `massage_perception` package.

Landmark lookup order: (11, 12, 23, 24) = L-shoulder, R-shoulder, L-hip, R-hip
Quad corner order after reindex: (11, 12, 24, 23) = TL, TR, BR, BL
The hip indices are swapped so the quad is convex (not self-intersecting):
shoulders on top, hips on the bottom.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import mediapipe as mp

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Indices into the 33-landmark pose to extract
_LOOKUP_IDS = (11, 12, 23, 24)
# Reorder to TL(11) TR(12) BR(24) BL(23) — convex quad
_QUAD_ORDER = (0, 1, 3, 2)

VISIBILITY_THRESHOLD = 0.7
# Lower threshold for prone/static images: prone MediaPipe visibility scores
# are systematically lower than upright poses.
VISIBILITY_THRESHOLD_STATIC = 0.3

_DEFAULT_MODEL = os.path.join(
    os.path.dirname(__file__), "models", "pose_landmarker_full.task"
)


@dataclass
class AnchorLandmarks:
    """Pixel coords of the 4 calibration anchor landmarks."""

    pts: np.ndarray         # shape (4, 2) float32, order TL/TR/BR/BL
    visibility: float       # mean visibility of the 4 landmarks
    valid: bool             # True if mean visibility >= threshold
    per_landmark_vis: list  # per-landmark visibility (lookup order) for debugging


class PoseDetector:
    """MediaPipe PoseLandmarker wrapper extracting only landmarks 11, 12, 23, 24.

    Args:
        model_path: path to a .task model file. Defaults to the bundled
            pose_landmarker_full.task.
        video_mode: if True, uses RunningMode.VIDEO for tracking on a stream;
            if False, uses RunningMode.IMAGE for single-frame detection.
    """

    def __init__(self, model_path: Optional[str] = None, video_mode: bool = True):
        self._model_path = model_path or _DEFAULT_MODEL
        self._video_mode = video_mode
        self._timestamp_ms = 0
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self._model_path),
            running_mode=(
                VisionRunningMode.VIDEO if video_mode else VisionRunningMode.IMAGE
            ),
            num_poses=1,
        )
        self._landmarker = PoseLandmarker.create_from_options(options)

    def detect(self, bgr_frame: np.ndarray) -> Optional[AnchorLandmarks]:
        """Stream detection (VIDEO mode). Use for the live webcam feed."""
        mp_image = self._to_mp_image(bgr_frame)
        if self._video_mode:
            self._timestamp_ms += 33  # ~30 FPS
            result = self._landmarker.detect_for_video(mp_image, self._timestamp_ms)
        else:
            result = self._landmarker.detect(mp_image)
        return self._extract(result, bgr_frame.shape, VISIBILITY_THRESHOLD)

    def detect_static(self, bgr_frame: np.ndarray) -> Optional[AnchorLandmarks]:
        """Single-image detection (IMAGE mode, highest accuracy).

        Use for reference images and stitched_*.png — not for live feed.
        Threshold lowered to 0.3 to accommodate prone visibility scores.
        Creates a one-shot IMAGE-mode landmarker so it works regardless of
        whether this instance was constructed in video mode.
        """
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self._model_path),
            running_mode=VisionRunningMode.IMAGE,
            num_poses=1,
        )
        with PoseLandmarker.create_from_options(options) as landmarker:
            mp_image = self._to_mp_image(bgr_frame)
            result = landmarker.detect(mp_image)
            return self._extract(result, bgr_frame.shape, VISIBILITY_THRESHOLD_STATIC)

    @staticmethod
    def _to_mp_image(bgr_frame: np.ndarray) -> mp.Image:
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        return mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    @staticmethod
    def _extract(result, shape, vis_threshold: float) -> Optional[AnchorLandmarks]:
        if not result.pose_landmarks:
            return None
        h, w = shape[:2]
        lm = result.pose_landmarks[0]  # first detected pose
        raw_pts, vis = [], []
        for idx in _LOOKUP_IDS:
            raw_pts.append([lm[idx].x * w, lm[idx].y * h])
            vis.append(float(lm[idx].visibility))
        raw_pts = np.array(raw_pts, dtype=np.float32)
        pts = raw_pts[list(_QUAD_ORDER)]  # reorder to TL/TR/BR/BL
        mean_vis = float(np.mean(vis))
        return AnchorLandmarks(
            pts=pts,
            visibility=mean_vis,
            valid=mean_vis >= vis_threshold,
            per_landmark_vis=vis,
        )

    def close(self) -> None:
        self._landmarker.close()
