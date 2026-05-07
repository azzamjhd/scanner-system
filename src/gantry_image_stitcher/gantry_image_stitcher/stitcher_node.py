#!/usr/bin/env python3

import json
import os

# numba (pulled in by stitching → largestinteriorrectangle) imports several
# attributes from coverage.types (Tracer, TShouldTraceFn, …) that coverage >=7
# removed.  Replacing the module with a subclass whose __getattr__ returns a
# harmless dummy unblocks numba for any current or future missing attribute.
import sys as _sys
import types as _pytypes

class _NumbaCompatCovTypes(_pytypes.ModuleType):
    def __getattr__(self, name):
        if name.startswith('__') and name.endswith('__'):
            raise AttributeError(name)  # let Python use normal dunder defaults
        return type  # harmless sentinel for non-dunder missing attrs (Tracer, TShouldTraceFn, …)

if 'coverage.types' in _sys.modules:
    _real_cov = _sys.modules['coverage.types']
    _stub_cov = _NumbaCompatCovTypes('coverage.types')
    _stub_cov.__dict__.update(
        {k: v for k, v in vars(_real_cov).items() if not k.startswith('__')}
    )
    _sys.modules['coverage.types'] = _stub_cov
else:
    _sys.modules['coverage.types'] = _NumbaCompatCovTypes('coverage.types')

import threading
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


@dataclass
class CaptureFrame:
    image: np.ndarray
    position_mm: float
    timestamp_ns: int
    file_path: str = ''


class StitcherNode(Node):
    def __init__(self) -> None:
        super().__init__('stitcher_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('position_topic', '/current_position')
        self.declare_parameter('capture_spacing_mm', 5.0)
        self.declare_parameter('min_motion_for_frame_mm', 0.5)
        self.declare_parameter('max_frame_buffer', 300)
        self.declare_parameter('preview_scale', 0.35)
        self.declare_parameter('preview_publish_period_ms', 250)
        self.declare_parameter('preview_max_frames', 10)
        self.declare_parameter('output_dir', '/tmp/gantry_stitcher')
        self.declare_parameter('pixels_per_mm', 0.0)
        self.declare_parameter('phase_corr_min_response', 0.1)
        self.declare_parameter('stitcher_detector', 'sift')
        self.declare_parameter('stitcher_nfeatures', 1000)
        self.declare_parameter('stitcher_confidence', 0.3)

        self.image_topic = self.get_parameter('image_topic').value
        self.position_topic = self.get_parameter('position_topic').value
        self.capture_spacing_mm = max(0.1, float(self.get_parameter('capture_spacing_mm').value))
        self.min_motion_for_frame_mm = max(0.0, float(self.get_parameter('min_motion_for_frame_mm').value))
        self.max_frame_buffer = max(10, int(self.get_parameter('max_frame_buffer').value))
        self.preview_scale = float(np.clip(float(self.get_parameter('preview_scale').value), 0.1, 1.0))
        self.preview_publish_period_ms = max(50, int(self.get_parameter('preview_publish_period_ms').value))
        self.preview_max_frames = max(2, int(self.get_parameter('preview_max_frames').value))
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.pixels_per_mm = max(0.0, float(self.get_parameter('pixels_per_mm').value))
        self.phase_corr_min_response = float(
            np.clip(float(self.get_parameter('phase_corr_min_response').value), 0.0, 1.0)
        )
        self.stitcher_detector = str(self.get_parameter('stitcher_detector').value)
        self.stitcher_nfeatures = max(100, int(self.get_parameter('stitcher_nfeatures').value))
        self.stitcher_confidence = float(
            np.clip(float(self.get_parameter('stitcher_confidence').value), 0.0, 1.0)
        )

        # Runtime state
        self._lock = threading.Lock()
        self.state = 'idle'  # idle | capturing | finalizing | error
        self.current_position_mm: Optional[float] = None
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_stamp_ns: int = 0
        self.session_start_position_mm: float = 0.0
        self.last_capture_position_mm: float = 0.0
        self.last_capture_bucket: int = -1
        self.frames: List[CaptureFrame] = []
        self.last_preview_image: Optional[np.ndarray] = None
        self.last_preview_publish_ns: int = 0
        self.session_id: str = ''
        self.session_frames_dir: str = ''

        # ROS interfaces
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.position_sub = self.create_subscription(Float32, self.position_topic, self.position_callback, 10)
        self.preview_pub = self.create_publisher(Image, '/stitcher/preview', 10)
        self.status_pub = self.create_publisher(String, '/stitcher/status', 10)

        self.start_srv = self.create_service(Trigger, '/stitcher/start_session', self.start_session_callback)
        self.stop_srv = self.create_service(Trigger, '/stitcher/stop_session', self.stop_session_callback)
        self.snapshot_srv = self.create_service(Trigger, '/stitcher/save_snapshot', self.save_snapshot_callback)

        self.preview_timer = self.create_timer(
            self.preview_publish_period_ms / 1000.0,
            self.publish_preview_timer_callback,
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._publish_status('idle')
        self.get_logger().info('Gantry stitcher node started')
        self.get_logger().info(
            f'Listening image_topic={self.image_topic}, position_topic={self.position_topic}, '
            f'capture_spacing_mm={self.capture_spacing_mm}'
        )

    # ----------------------------- Parameter callback -----------------------------

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        for param in params:
            name = param.name
            val = param.value

            if name == 'capture_spacing_mm':
                if float(val) <= 0.0:
                    return SetParametersResult(successful=False, reason='capture_spacing_mm must be > 0')
                new_spacing = float(val)
                with self._lock:
                    self.capture_spacing_mm = new_spacing
                    if self.state == 'capturing' and self.current_position_mm is not None:
                        travel = abs(self.current_position_mm - self.session_start_position_mm)
                        self.last_capture_bucket = int(np.floor(travel / new_spacing))
                self.get_logger().info(f'capture_spacing_mm updated to {new_spacing:.2f} mm')

            elif name == 'min_motion_for_frame_mm':
                if float(val) < 0.0:
                    return SetParametersResult(successful=False, reason='min_motion_for_frame_mm must be >= 0')
                with self._lock:
                    self.min_motion_for_frame_mm = float(val)

            elif name == 'preview_scale':
                v = float(val)
                if not (0.1 <= v <= 1.0):
                    return SetParametersResult(successful=False, reason='preview_scale must be in [0.1, 1.0]')
                with self._lock:
                    self.preview_scale = v

            elif name == 'preview_max_frames':
                if int(val) < 2:
                    return SetParametersResult(successful=False, reason='preview_max_frames must be >= 2')
                with self._lock:
                    self.preview_max_frames = int(val)

            elif name == 'max_frame_buffer':
                if int(val) < 10:
                    return SetParametersResult(successful=False, reason='max_frame_buffer must be >= 10')
                with self._lock:
                    self.max_frame_buffer = int(val)

            elif name == 'pixels_per_mm':
                if float(val) < 0.0:
                    return SetParametersResult(successful=False, reason='pixels_per_mm must be >= 0')
                with self._lock:
                    self.pixels_per_mm = float(val)

            elif name == 'phase_corr_min_response':
                v = float(val)
                if not (0.0 <= v <= 1.0):
                    return SetParametersResult(
                        successful=False, reason='phase_corr_min_response must be in [0.0, 1.0]'
                    )
                with self._lock:
                    self.phase_corr_min_response = v

            elif name == 'stitcher_detector':
                if str(val) not in ('sift', 'orb', 'brisk', 'akaze'):
                    return SetParametersResult(
                        successful=False, reason='stitcher_detector must be sift, orb, brisk, or akaze'
                    )
                with self._lock:
                    self.stitcher_detector = str(val)

            elif name == 'stitcher_nfeatures':
                if int(val) < 100:
                    return SetParametersResult(successful=False, reason='stitcher_nfeatures must be >= 100')
                with self._lock:
                    self.stitcher_nfeatures = int(val)

            elif name == 'stitcher_confidence':
                v = float(val)
                if not (0.0 <= v <= 1.0):
                    return SetParametersResult(
                        successful=False, reason='stitcher_confidence must be in [0.0, 1.0]'
                    )
                with self._lock:
                    self.stitcher_confidence = v

        return SetParametersResult(successful=True)

    # ----------------------------- ROS callbacks -----------------------------

    def position_callback(self, msg: Float32) -> None:
        with self._lock:
            self.current_position_mm = float(msg.data)

    def image_callback(self, msg: Image) -> None:
        frame = self._image_msg_to_bgr(msg)
        if frame is None:
            return

        with self._lock:
            self.latest_image = frame
            self.latest_image_stamp_ns = self._stamp_to_ns(msg)

            if self.state == 'capturing':
                self._maybe_capture_current_frame_locked()

    # ----------------------------- Service callbacks -----------------------------

    def start_session_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            if self.state == 'capturing':
                response.success = False
                response.message = 'Stitcher session already running'
                return response

            if self.current_position_mm is None:
                response.success = False
                response.message = f'No position received on {self.position_topic}'
                return response

            self.state = 'capturing'
            self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.session_start_position_mm = self.current_position_mm
            self.last_capture_position_mm = self.current_position_mm
            self.last_capture_bucket = -1
            self.frames = []
            self.last_preview_image = None
            self.last_preview_publish_ns = 0
            frames_dir = os.path.join(self.output_dir, self.session_id, 'frames')
            os.makedirs(frames_dir, exist_ok=True)
            self.session_frames_dir = frames_dir

            self._capture_current_frame_locked(force=True)
            self._publish_status('capturing')

            response.success = True
            response.message = (
                f'Session {self.session_id} started at position '
                f'{self.session_start_position_mm:.2f} mm'
            )
            self.get_logger().info(response.message)
            return response

    def stop_session_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            if self.state != 'capturing':
                response.success = False
                response.message = 'No active session to stop'
                return response
            self.state = 'finalizing'
            frames_copy = list(self.frames)
            session_id = self.session_id
            start_position = self.session_start_position_mm
            capture_spacing_snapshot = self.capture_spacing_mm
            frames_dir_snapshot = self.session_frames_dir

        t = threading.Thread(
            target=self._finalize_session,
            args=(frames_copy, session_id, start_position, capture_spacing_snapshot, frames_dir_snapshot),
            daemon=True,
        )
        t.start()

        response.success = True
        response.message = (
            f'Finalization started for session {session_id} with {len(frames_copy)} frames'
        )
        self.get_logger().info(response.message)
        return response

    def save_snapshot_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            frame = None if self.latest_image is None else self.latest_image.copy()
            session_id = self.session_id or datetime.now().strftime('%Y%m%d_%H%M%S')

        if frame is None:
            response.success = False
            response.message = 'No image available for snapshot'
            return response

        output_dir = os.path.join(self.output_dir, session_id)
        os.makedirs(output_dir, exist_ok=True)
        snapshot_path = os.path.join(output_dir, f'snapshot_{datetime.now().strftime("%H%M%S")}.png')
        cv2.imwrite(snapshot_path, frame)
        response.success = True
        response.message = f'Snapshot saved: {snapshot_path}'
        self.get_logger().info(response.message)
        return response

    # ----------------------------- Background finalization -----------------------------

    def _finalize_session(
        self,
        frames: List[CaptureFrame],
        session_id: str,
        start_position: float,
        capture_spacing_mm: float,
        frames_dir: str,
    ) -> None:
        try:
            output_dir = os.path.join(self.output_dir, session_id)
            os.makedirs(output_dir, exist_ok=True)

            file_paths = [f.file_path for f in frames if f.file_path and os.path.exists(f.file_path)]
            final_image = self._build_openstitching_stitch(file_paths)
            if final_image is None:
                self.get_logger().warn('OpenStitching failed, falling back to incremental phase-correlation')
                final_image = self._build_incremental_stitch(frames)
            final_path = os.path.join(output_dir, 'stitched_final.png')
            cv2.imwrite(final_path, final_image)

            manifest = {
                'session_id': session_id,
                'frame_count': len(frames),
                'capture_spacing_mm': capture_spacing_mm,
                'start_position_mm': start_position,
                'positions_mm': [f.position_mm for f in frames],
                'timestamps_ns': [f.timestamp_ns for f in frames],
                'frame_paths': [f.file_path for f in frames],
                'image_topic': self.image_topic,
                'position_topic': self.position_topic,
                'final_image_path': final_path,
            }
            manifest_path = os.path.join(output_dir, 'manifest.json')
            with open(manifest_path, 'w') as file:
                json.dump(manifest, file, indent=2)

            with self._lock:
                self.state = 'idle'
            self._publish_status('idle')
            self.get_logger().info(
                f'Session {session_id} finalized: {len(frames)} frames → {final_path}'
            )
        except Exception as exc:
            with self._lock:
                self.state = 'error'
            self._publish_status('error')
            self.get_logger().error(f'Finalization failed: {exc}')

    # ----------------------------- Capture logic -----------------------------

    def _maybe_capture_current_frame_locked(self) -> None:
        if self.current_position_mm is None:
            return
        if self.latest_image is None:
            return
        if len(self.frames) >= self.max_frame_buffer:
            return

        travel_mm = abs(self.current_position_mm - self.session_start_position_mm)
        bucket = int(np.floor(travel_mm / self.capture_spacing_mm))

        if bucket <= self.last_capture_bucket:
            return

        if abs(self.current_position_mm - self.last_capture_position_mm) < self.min_motion_for_frame_mm:
            return

        self._capture_current_frame_locked(force=False, bucket=bucket)

    def _capture_current_frame_locked(self, force: bool, bucket: Optional[int] = None) -> None:
        if self.latest_image is None or self.current_position_mm is None:
            return
        if len(self.frames) >= self.max_frame_buffer:
            return

        if bucket is None:
            bucket = 0

        frame = self.latest_image.copy()
        frame_index = len(self.frames)
        file_path = ''
        if self.session_frames_dir:
            file_path = os.path.join(self.session_frames_dir, f'frame_{frame_index:04d}.jpg')
            cv2.imwrite(file_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

        self.frames.append(
            CaptureFrame(
                image=frame,
                position_mm=float(self.current_position_mm),
                timestamp_ns=int(self.latest_image_stamp_ns),
                file_path=file_path,
            )
        )
        self.last_capture_position_mm = float(self.current_position_mm)
        self.last_capture_bucket = bucket if not force else max(self.last_capture_bucket, 0)

        self.get_logger().info(
            f'Captured frame #{len(self.frames)} at {self.current_position_mm:.2f} mm '
            f'(bucket={self.last_capture_bucket})'
        )

    # ----------------------------- Preview publishing -----------------------------

    def publish_preview_timer_callback(self) -> None:
        with self._lock:
            if self.state != 'capturing' or len(self.frames) == 0:
                return
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns - self.last_preview_publish_ns < self.preview_publish_period_ms * 1_000_000:
                return
            preview_frames = [f.image for f in self.frames[-self.preview_max_frames:]]

        try:
            preview = self._build_preview(preview_frames)
            if preview is None:
                return

            msg = self._bgr_to_image_msg(preview)
            self.preview_pub.publish(msg)
            with self._lock:
                self.last_preview_image = preview
                self.last_preview_publish_ns = int(self.get_clock().now().nanoseconds)
        except Exception as exc:
            self.get_logger().warn(f'Preview publish failed: {exc}')

    # ----------------------------- Stitching helpers -----------------------------

    def _build_preview(self, frames: List[np.ndarray]) -> Optional[np.ndarray]:
        if not frames:
            return None

        scaled = [
            cv2.resize(
                frame,
                None,
                fx=self.preview_scale,
                fy=self.preview_scale,
                interpolation=cv2.INTER_AREA,
            )
            for frame in frames
        ]

        if len(scaled) == 1:
            return scaled[0]

        panorama = self._stitch_with_opencv(scaled, cv2.Stitcher_SCANS)
        if panorama is not None:
            return panorama
        return self._concat_with_overlap(scaled, overlap_ratio=0.35)

    def _build_openstitching_stitch(self, file_paths: List[str]) -> Optional[np.ndarray]:
        """High-quality stitcher using AffineStitcher from the OpenStitching library.

        AffineStitcher is purpose-built for flat-surface linear scans (affine motion).
        Frames are read from disk so the stitcher gets clean, independently loaded images.
        """
        if not file_paths:
            return None

        try:
            from stitching import AffineStitcher
        except ImportError:
            self.get_logger().warn("'stitching' package not installed — run: pip install stitching")
            return None

        with self._lock:
            detector = self.stitcher_detector
            nfeatures = self.stitcher_nfeatures
            confidence = self.stitcher_confidence

        try:
            stitcher = AffineStitcher(
                detector=detector,
                nfeatures=nfeatures,
                confidence_threshold=confidence,
            )
            result = stitcher.stitch(file_paths)
            if result is None or result.size == 0:
                return None
            self.get_logger().info(
                f'OpenStitching succeeded: {result.shape[1]}×{result.shape[0]} px'
            )
            return result
        except Exception as exc:
            self.get_logger().warn(f'OpenStitching failed: {exc}')
            return None

    def _build_incremental_stitch(self, frames: List[CaptureFrame]) -> np.ndarray:
        """Fast O(N) incremental stitcher using phase correlation between consecutive frames."""
        if not frames:
            raise RuntimeError('No frames captured for final stitch')
        if len(frames) == 1:
            return frames[0].image

        total = len(frames)
        offsets: List[Tuple[float, float]] = [(0.0, 0.0)]

        for i in range(1, total):
            prev_gray = cv2.cvtColor(frames[i - 1].image, cv2.COLOR_BGR2GRAY).astype(np.float32)
            curr_gray = cv2.cvtColor(frames[i].image, cv2.COLOR_BGR2GRAY).astype(np.float32)

            (dx, dy), response = cv2.phaseCorrelate(prev_gray, curr_gray)

            if response < self.phase_corr_min_response:
                orb_dx, orb_dy, ok = self._estimate_translation_orb(frames[i - 1].image, frames[i].image)
                if ok:
                    dx, dy = orb_dx, orb_dy
                else:
                    pos_dx, pos_dy, ok = self._estimate_translation_from_position(frames[i - 1], frames[i])
                    if ok:
                        dx, dy = pos_dx, pos_dy
                    else:
                        dx, dy = 0.0, 0.0

            prev_ox, prev_oy = offsets[-1]
            offsets.append((prev_ox + dx, prev_oy + dy))
            self._publish_status('finalizing', progress=i, total=total - 1)

        frame_h, frame_w = frames[0].image.shape[:2]

        xs = [ox for ox, _ in offsets]
        ys = [oy for _, oy in offsets]
        min_x = min(xs)
        min_y = min(ys)

        shifted = [(ox - min_x, oy - min_y) for ox, oy in offsets]

        canvas_w = int(np.ceil(max(ox + frame_w for ox, _ in shifted)))
        canvas_h = int(np.ceil(max(oy + frame_h for _, oy in shifted)))

        if canvas_w * canvas_h > 200_000_000:
            self.get_logger().warn(
                f'Canvas too large ({canvas_w}×{canvas_h}), falling back to overlap concatenation'
            )
            return self._concat_with_overlap([f.image for f in frames], overlap_ratio=0.35)

        canvas_sum = np.zeros((canvas_h, canvas_w, 3), dtype=np.float64)
        canvas_weight = np.zeros((canvas_h, canvas_w), dtype=np.uint8)

        for i, frame_obj in enumerate(frames):
            ox = int(round(shifted[i][0]))
            oy = int(round(shifted[i][1]))
            img = frame_obj.image
            h, w = img.shape[:2]
            x2 = min(ox + w, canvas_w)
            y2 = min(oy + h, canvas_h)
            canvas_sum[oy:y2, ox:x2] += img[:y2 - oy, :x2 - ox].astype(np.float64)
            canvas_weight[oy:y2, ox:x2] += 1

        mask = canvas_weight > 0
        result = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        result[mask] = (canvas_sum[mask] / canvas_weight[mask, np.newaxis]).astype(np.uint8)
        return result

    def _estimate_translation_orb(
        self, img_a: np.ndarray, img_b: np.ndarray
    ) -> Tuple[float, float, bool]:
        try:
            gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
            gray_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)
            orb = cv2.ORB_create(nfeatures=500)
            kp_a, des_a = orb.detectAndCompute(gray_a, None)
            kp_b, des_b = orb.detectAndCompute(gray_b, None)
            if des_a is None or des_b is None or len(kp_a) < 4 or len(kp_b) < 4:
                return 0.0, 0.0, False
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des_a, des_b)
            if len(matches) < 4:
                return 0.0, 0.0, False
            pts_a = np.float32([kp_a[m.queryIdx].pt for m in matches])
            pts_b = np.float32([kp_b[m.trainIdx].pt for m in matches])
            M, inliers = cv2.estimateAffinePartial2D(pts_a, pts_b, method=cv2.RANSAC)
            if M is None or inliers is None or int(inliers.sum()) < 4:
                return 0.0, 0.0, False
            return float(M[0, 2]), float(M[1, 2]), True
        except Exception:
            return 0.0, 0.0, False

    def _estimate_translation_from_position(
        self, frame_a: CaptureFrame, frame_b: CaptureFrame
    ) -> Tuple[float, float, bool]:
        with self._lock:
            ppm = self.pixels_per_mm
        if ppm <= 0.0:
            return 0.0, 0.0, False
        dx = (frame_b.position_mm - frame_a.position_mm) * ppm
        return dx, 0.0, True

    def _stitch_with_opencv(self, frames: List[np.ndarray], mode: int) -> Optional[np.ndarray]:
        try:
            stitcher = cv2.Stitcher_create(mode)
            status, pano = stitcher.stitch(frames)
            if status == cv2.Stitcher_OK:
                return pano
            return None
        except Exception:
            return None

    def _concat_with_overlap(self, frames: List[np.ndarray], overlap_ratio: float) -> np.ndarray:
        output = frames[0]
        overlap_ratio = float(np.clip(overlap_ratio, 0.0, 0.9))

        for frame in frames[1:]:
            h = min(output.shape[0], frame.shape[0])
            output_crop = output[:h, :]
            frame_crop = frame[:h, :]
            overlap_px = int(min(output_crop.shape[1], frame_crop.shape[1]) * overlap_ratio)
            non_overlap_part = frame_crop[:, overlap_px:]
            output = np.hstack([output_crop, non_overlap_part])
        return output

    # ----------------------------- Image conversion -----------------------------

    def _image_msg_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ('bgr8', 'rgb8'):
            channels = 3
            if msg.step < msg.width * channels:
                self.get_logger().error(
                    f'Invalid image step for {encoding}: step={msg.step}, width={msg.width}'
                )
                return None
            row_data = data.reshape((msg.height, msg.step))
            img = row_data[:, :msg.width * channels].reshape((msg.height, msg.width, channels))
            if encoding == 'rgb8':
                img = img[:, :, ::-1]
            return np.ascontiguousarray(img)

        if encoding == 'mono8':
            if msg.step < msg.width:
                self.get_logger().error(
                    f'Invalid image step for mono8: step={msg.step}, width={msg.width}'
                )
                return None
            row_data = data.reshape((msg.height, msg.step))
            gray = row_data[:, :msg.width].reshape((msg.height, msg.width))
            return np.ascontiguousarray(np.stack((gray, gray, gray), axis=2))

        self.get_logger().warn(
            f'Unsupported encoding {msg.encoding}. Use bgr8/rgb8/mono8 topics.'
        )
        return None

    def _bgr_to_image_msg(self, image: np.ndarray) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'stitcher'
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = image.shape[1] * image.shape[2]
        msg.data = np.ascontiguousarray(image).tobytes()
        return msg

    def _stamp_to_ns(self, msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _publish_status(self, state: str, progress: Optional[int] = None, total: Optional[int] = None) -> None:
        if state == 'finalizing' and progress is not None and total is not None:
            data = json.dumps({'state': 'finalizing', 'progress': progress, 'total': total})
        else:
            data = state
        status_msg = String()
        status_msg.data = data
        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
