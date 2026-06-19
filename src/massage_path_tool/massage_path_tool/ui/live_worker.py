import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap

from massage_path_tool.core.pose_detector import PoseDetector
from massage_path_tool.core.homography import (
    compute_h_proj,
    apply_homography,
    validate_quad,
)


class LiveWorker(QThread):
    """Capture webcam frames, run pose detection, validate the detected quad,
    and project saved UV paths back into pixel space."""

    frame_ready = pyqtSignal(object)              # emits a QPixmap of the raw video frame
    projection_ready = pyqtSignal(object, object)  # emits (anchor_pts np(4,2), projected_px np(N,2))
    pose_invalid = pyqtSignal(str)                # emits the validation reason string
    pose_lost = pyqtSignal()                      # emitted when no valid pose this frame

    def __init__(self, camera_index: int, session: dict, parent=None):
        super().__init__(parent)
        self._camera_index = camera_index
        self._session = session
        self._stop = False

    def run(self):
        detector = PoseDetector(video_mode=True)
        cap = cv2.VideoCapture(self._camera_index)
        cap.set(cv2.CAP_PROP_FPS, 30)

        ref_ar = self._session.get('calibration_aspect_ratio', 1.4)
        all_uvs = self._collect_uvs()

        try:
            while not self._stop and cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    break

                h, w, ch = frame.shape
                qimg = QImage(frame.data, w, h, ch * w, QImage.Format.Format_BGR888)
                pixmap = QPixmap.fromImage(qimg.copy())
                self.frame_ready.emit(pixmap)

                anchors = detector.detect(frame)
                if anchors is None or not anchors.valid:
                    self.pose_lost.emit()
                    continue

                val = validate_quad(anchors.pts, ref_ar)
                if not val.valid:
                    self.pose_invalid.emit(val.reason)
                    continue

                if len(all_uvs):
                    H = compute_h_proj(anchors.pts)
                    projected = apply_homography(H, all_uvs)
                    self.projection_ready.emit(anchors.pts, projected)
        finally:
            cap.release()
            detector.close()

    def _collect_uvs(self) -> np.ndarray:
        uvs = []
        for pt in self._session.get('discrete_points', []):
            uvs.append([pt['u'], pt['v']])
        for path in self._session.get('paths', []):
            for pt in path.get('points', []):
                uvs.append([pt['u'], pt['v']])
        if not uvs:
            return np.zeros((0, 2), dtype=np.float32)
        return np.array(uvs, dtype=np.float32)

    def stop(self):
        self._stop = True
        self.wait()
