import os

import numpy as np
import pytest

REF_IMAGE = "/home/azzam/Documents/ros2_ws/src/lidar_camera_fusion/massage_points_reference.png"


def test_import():
    try:
        from massage_path_tool.core.pose_detector import PoseDetector, AnchorLandmarks
        assert PoseDetector is not None
        assert AnchorLandmarks is not None
    except ImportError as e:
        pytest.skip(f"mediapipe not installed: {e}")


def test_black_frame_no_crash():
    try:
        from massage_path_tool.core.pose_detector import PoseDetector, AnchorLandmarks
    except ImportError as e:
        pytest.skip(f"mediapipe not installed: {e}")
    det = PoseDetector(video_mode=True)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    result = det.detect(frame)
    # Black frame → no pose → None. Must not raise.
    assert result is None or isinstance(result, AnchorLandmarks)
    det.close()


def test_detect_static_on_reference_image():
    """Real detection on the prone-body reference image."""
    try:
        import cv2
        from massage_path_tool.core.pose_detector import PoseDetector, AnchorLandmarks
    except ImportError as e:
        pytest.skip(f"deps not installed: {e}")
    if not os.path.exists(REF_IMAGE):
        pytest.skip(f"reference image not found: {REF_IMAGE}")
    img = cv2.imread(REF_IMAGE, cv2.IMREAD_COLOR)
    assert img is not None
    det = PoseDetector(video_mode=False)
    result = det.detect_static(img)
    det.close()
    # Print diagnostics for visibility assessment
    if result is not None:
        print(f"\nmean_vis={result.visibility:.3f} valid={result.valid}")
        print(f"per-landmark vis (11,12,23,24)={[f'{v:.3f}' for v in result.per_landmark_vis]}")
        print(f"pts (TL/TR/BR/BL)=\n{result.pts}")
        assert result.pts.shape == (4, 2)
        assert len(result.per_landmark_vis) == 4
    else:
        print("\nNO POSE DETECTED on reference image — manual anchor mode needed")
