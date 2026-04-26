import os
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import mediapipe as mp
import cv2
import yaml

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

MODEL_PATH = os.path.join(os.path.dirname(__file__), 'models', 'pose_landmarker_full.task')

DEFAULT_CONFIG_PATH = os.path.join(
    get_package_share_directory('massage_perception'), 'config', 'massage_points.yaml')

# MediaPipe BlazePose 33-landmark skeleton connections
POSE_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,7),(0,4),(4,5),(5,6),(6,8),
    (9,10),(11,12),(11,13),(13,15),(15,17),(15,19),(15,21),
    (17,19),(12,14),(14,16),(16,18),(16,20),(16,22),(18,20),
    (11,23),(12,24),(23,24),(23,25),(24,26),(25,27),(26,28),
    (27,29),(28,30),(29,31),(30,32),(27,31),(28,32),
]

MASSAGE_POINT_COLOR = (0, 165, 255)  # orange (BGR)


def load_massage_config(path: str) -> dict:
    """Load and parse the massage points YAML config."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('massage_points', {})


def get_massage_points(landmarks, config: dict) -> dict:
    """Compute massage point positions from landmark weighted sums + offsets."""
    points = {}
    for name, definition in config.items():
        x, y = 0.0, 0.0
        for comp in definition.get('components', []):
            idx = comp['landmark']
            w   = comp.get('weight', 1.0)
            x  += w * landmarks[idx].x
            y  += w * landmarks[idx].y
        offset = definition.get('offset', {})
        x += offset.get('x', 0.0)
        y += offset.get('y', 0.0)
        points[name] = (x, y)
    return points


class MassagePoseNode(Node):
    def __init__(self):
        super().__init__('massage_pose_node')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._fps_last_time = time.monotonic()
        self._fps = 0.0

        # Publishers
        self.pub_landmarks   = self.create_publisher(PoseArray, 'pose_landmarks', 10)
        self.pub_massage_pts = self.create_publisher(PoseArray, 'massage_points',  10)

        # Config
        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
        self._config_path  = self.get_parameter('config_path').get_parameter_value().string_value
        self._config_mtime = 0.0
        self._massage_cfg  = {}
        self._reload_config()

        # MediaPipe landmarker
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=VisionRunningMode.LIVE_STREAM,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            result_callback=self._pose_result_callback,
        )
        self.landmarker = PoseLandmarker.create_from_options(options)

        self.declare_parameter('show_viewer', True)
        self.create_timer(0.03, self._display_callback)  # ~30 FPS viewer
        self.create_timer(1.0,  self._config_watch)      # config hot-reload
        self.get_logger().info('Massage Pose Node has been started')
        self.get_logger().info(f'Config file: {self._config_path}')

    # ------------------------------------------------------------------
    # Config loading / hot-reload
    # ------------------------------------------------------------------

    def _reload_config(self):
        try:
            self._massage_cfg  = load_massage_config(self._config_path)
            self._config_mtime = os.path.getmtime(self._config_path)
            self.get_logger().info(
                f'Loaded massage config: {list(self._massage_cfg.keys())}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')

    def _config_watch(self):
        """Reload config if the file has changed on disk."""
        try:
            mtime = os.path.getmtime(self._config_path)
            if mtime != self._config_mtime:
                self.get_logger().info('Config file changed — reloading...')
                self._reload_config()
        except Exception as e:
            self.get_logger().error(f'Config watch error: {e}')

    # ------------------------------------------------------------------
    # MediaPipe callback
    # ------------------------------------------------------------------

    def _pose_result_callback(self, result: PoseLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        # FPS measured at the MediaPipe output rate
        now = time.monotonic()
        self._fps = 1.0 / max(now - self._fps_last_time, 1e-9)
        self._fps_last_time = now

        frame = cv2.cvtColor(output_image.numpy_view().copy(), cv2.COLOR_RGB2BGR)
        stamp = self.get_clock().now().to_msg()

        if result.pose_landmarks:
            lm = result.pose_landmarks[0]
            h, w = frame.shape[:2]

            # --- Publish landmarks ---
            lm_msg = PoseArray(header=Header(stamp=stamp, frame_id='camera'))
            for pt in lm:
                p = Pose()
                p.position.x, p.position.y, p.position.z = pt.x, pt.y, pt.z
                lm_msg.poses.append(p)
            self.pub_landmarks.publish(lm_msg)

            # --- Compute & publish massage points ---
            massage_pts = get_massage_points(lm, self._massage_cfg)
            mp_msg = PoseArray(header=Header(stamp=stamp, frame_id='camera'))
            for name, (x, y) in massage_pts.items():
                p = Pose()
                p.position.x, p.position.y, p.position.z = x, y, 0.0
                mp_msg.poses.append(p)
            self.pub_massage_pts.publish(mp_msg)

            # --- Draw skeleton ---
            for a, b in POSE_CONNECTIONS:
                if a < len(lm) and b < len(lm):
                    cv2.line(frame,
                             (int(lm[a].x * w), int(lm[a].y * h)),
                             (int(lm[b].x * w), int(lm[b].y * h)),
                             (0, 255, 0), 2)
            for pt in lm:
                cv2.circle(frame, (int(pt.x * w), int(pt.y * h)), 4, (0, 0, 255), -1)

            # Draw massage points
            for name, (x, y) in massage_pts.items():
                cx, cy = int(x * w), int(y * h)
                cv2.circle(frame, (cx, cy), 8, MASSAGE_POINT_COLOR, -1)
                cv2.putText(frame, name, (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            MASSAGE_POINT_COLOR, 1, cv2.LINE_AA)

        # Draw FPS overlay (always, even when no pose detected)
        cv2.putText(frame, f'MP FPS: {self._fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)

        with self._lock:
            self._latest_frame = frame

    def listener_callback(self, data):
        try:
            cv_image  = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            mp_image  = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            timestamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
            self.landmarker.detect_async(mp_image, timestamp_ms)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def _display_callback(self):
        show = self.get_parameter('show_viewer').get_parameter_value().bool_value
        if not show:
            cv2.destroyAllWindows()
            return
        with self._lock:
            frame = self._latest_frame
        if frame is not None:
            cv2.imshow('Massage Pose Detection', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.landmarker.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MassagePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
