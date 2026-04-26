#!/usr/bin/env python3

import json
import os
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


@dataclass
class CaptureFrame:
    image: np.ndarray
    position_mm: float
    timestamp_ns: int


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

        self.image_topic = self.get_parameter('image_topic').value
        self.position_topic = self.get_parameter('position_topic').value
        self.capture_spacing_mm = max(0.1, float(self.get_parameter('capture_spacing_mm').value))
        self.min_motion_for_frame_mm = max(0.0, float(self.get_parameter('min_motion_for_frame_mm').value))
        self.max_frame_buffer = max(10, int(self.get_parameter('max_frame_buffer').value))
        self.preview_scale = float(np.clip(float(self.get_parameter('preview_scale').value), 0.1, 1.0))
        self.preview_publish_period_ms = max(50, int(self.get_parameter('preview_publish_period_ms').value))
        self.preview_max_frames = max(2, int(self.get_parameter('preview_max_frames').value))
        self.output_dir = str(self.get_parameter('output_dir').value)

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

        self._publish_status('idle')
        self.get_logger().info('Gantry stitcher node started')
        self.get_logger().info(
            f'Listening image_topic={self.image_topic}, position_topic={self.position_topic}, '
            f'capture_spacing_mm={self.capture_spacing_mm}'
        )

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

        try:
            output_dir = os.path.join(self.output_dir, session_id)
            os.makedirs(output_dir, exist_ok=True)

            final_image = self._build_final_stitch([f.image for f in frames_copy])
            final_path = os.path.join(output_dir, 'stitched_final.png')
            cv2.imwrite(final_path, final_image)

            manifest = {
                'session_id': session_id,
                'frame_count': len(frames_copy),
                'capture_spacing_mm': self.capture_spacing_mm,
                'start_position_mm': start_position,
                'positions_mm': [f.position_mm for f in frames_copy],
                'timestamps_ns': [f.timestamp_ns for f in frames_copy],
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

            response.success = True
            response.message = (
                f'Session {session_id} finalized with {len(frames_copy)} frames. '
                f'Output: {final_path}'
            )
            self.get_logger().info(response.message)
            return response
        except Exception as exc:
            with self._lock:
                self.state = 'error'
                self._publish_status('error')
            response.success = False
            response.message = f'Failed to finalize stitch: {exc}'
            self.get_logger().error(response.message)
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
        self.frames.append(
            CaptureFrame(
                image=frame,
                position_mm=float(self.current_position_mm),
                timestamp_ns=int(self.latest_image_stamp_ns),
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

    def _build_final_stitch(self, frames: List[np.ndarray]) -> np.ndarray:
        if not frames:
            raise RuntimeError('No frames captured for final stitch')
        if len(frames) == 1:
            return frames[0]

        pano = self._stitch_with_opencv(frames, cv2.Stitcher_SCANS)
        if pano is not None:
            return pano

        pano = self._stitch_with_opencv(frames, cv2.Stitcher_PANORAMA)
        if pano is not None:
            return pano

        self.get_logger().warn('OpenCV stitcher failed, using overlap concatenation fallback')
        return self._concat_with_overlap(frames, overlap_ratio=0.35)

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

    def _publish_status(self, state: str) -> None:
        status_msg = String()
        status_msg.data = state
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
