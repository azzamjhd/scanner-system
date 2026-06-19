# massage_path_tool

A PyQt6 desktop application for authoring and projecting massage-point and massage-path trajectories onto a human body using 2D optical homography. It maps pixel coordinates drawn on a static reference image into a normalized $(u,v)$ space via MediaPipe Pose landmarks, then re-projects those saved trajectories onto a live video feed in real time.

---

## Table of Contents

1. [Overview](#overview)
2. [Three-Phase Workflow](#three-phase-workflow)
3. [Mathematical Model](#mathematical-model)
4. [UI Reference](#ui-reference)
5. [Installation](#installation)
6. [Build](#build)
7. [Usage](#usage)
8. [Session File Format](#session-file-format)
9. [Package Layout](#package-layout)
10. [Troubleshooting](#troubleshooting)

---

## Overview

The tool solves a practical problem: how to define massage points and paths on a reference photograph of a body, then track those same anatomical locations on a *different* patient (or the same patient in a different pose) in real time.

It does this by:

1. **Calibration** — detecting four anatomical landmarks (left shoulder, right shoulder, left hip, right hip) on the reference image using MediaPipe Pose.
2. **Normalization** — computing a homography matrix $H_{norm}$ that maps the quadrilateral formed by those four landmarks into a unit square $[0,1] \times [0,1]$.
3. **Authoring** — letting the user click discrete points or build smooth paths (via Catmull-Rom → cubic Bézier curves) inside that quadrilateral. All coordinates are stored as normalized $(u,v)$ values, decoupled from absolute pixel values.
4. **Live Projection** — on a live webcam feed, detecting the same four landmarks, computing the inverse homography $H_{proj}$, and projecting the saved $(u,v)$ trajectories back into pixel space.

The result is a set of massage points and paths that are **pose-invariant** — they track the patient's body as they move, stretch, or shift in the camera frame.

---

## Three-Phase Workflow

### Phase A — Author (reference image)

Use a high-quality reference photograph (e.g. `massage_points_reference.png`) to define where massage points and paths *should* be anatomically.

```bash
ros2 run massage_path_tool gui --image ~/Documents/ros2_ws/src/lidar_camera_fusion/massage_points_reference.png
```

**Steps:**
1. Click **Open Image** and load the reference photo.
2. Click **Detect Anchors** — MediaPipe Pose runs on the image and draws a magenta quadrilateral connecting the four landmarks.
3. If auto-detection fails (e.g. prone pose, partial occlusion), click **Set Anchors Manually** and click the four landmarks in order: left shoulder → right shoulder → right hip → left hip.
4. Click **Set Points** and click on the image to place discrete massage points (e.g. acupoints). A dialog asks for a label.
5. Click **Draw Path** and click to place control points along a desired trajectory (e.g. along the spine). Right-click or press **Enter** to finish the path. A dialog asks for a label. The path is automatically smoothed via a Catmull-Rom → cubic Bézier interpolating curve that passes exactly through every control point.
6. Click **Save Session** to write a JSON file containing all normalized $(u,v)$ coordinates.

### Phase B — Verify (stitched scan image)

After a 3D scan, the `scan_image_recorder_node` produces a stitched panorama image (e.g. `stitched_20260617_120000.png`). Load this image plus the session from Phase A to verify and fine-tune the paths on the actual patient geometry.

```bash
ros2 run massage_path_tool gui \
  --image ~/ros2_scans/stitched_20260617_120000.png \
  --session ~/ros2_scans/massage_session.json
```

The saved $(u,v)$ coordinates are re-projected onto the new image using the new image's detected (or manually clicked) anchor quadrilateral. You can adjust paths and re-save.

### Phase C — Live (webcam feed)

With a session loaded, click **Start Live Feed**. The app opens the webcam, runs MediaPipe Pose on every frame, and overlays the saved massage points and paths onto the live video in real time.

If the patient moves, the paths track their body. If the pose becomes invalid (e.g. the patient turns away, or the quadrilateral becomes too distorted), the screen flashes **"INVALID POSE GEOMETRY"** in red and projection is suspended until a valid pose is recovered.

---

## Mathematical Model

### Homography

A homography is a $3 \times 3$ projective transformation matrix $\mathbf{H}$ that maps points from one plane to another. It requires exactly 4 source points and 4 destination points (no three collinear).

**Forward transformation (pixel → normalized):**

Given the four detected anchor landmarks in pixel coordinates:

```
src_pts = np.float32([[x_11, y_11], [x_12, y_12], [x_24, y_24], [x_23, y_23]])
dst_pts = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])  # unit square
H_norm, _ = cv2.findHomography(src_pts, dst_pts)
```

For any pixel point $(x, y)$, the normalized coordinate is:

```
[u, v, w]^T = H_norm · [x, y, 1]^T
u = u/w,  v = v/w
```

**Inverse transformation (normalized → pixel):**

On a live frame, detect the four landmarks again to get `live_pts`. Then:

```
H_proj, _ = cv2.findHomography(dst_pts, live_pts)
[x, y, w]^T = H_proj · [u, v, 1]^T
x = x/w,  y = y/w
```

All coordinate mapping uses `cv2.perspectiveTransform` (vectorized over arrays of points).

### Bézier Curve Smoothing

Raw mouse clicks produce sparse, unevenly spaced control points. The tool converts these into a smooth interpolating curve using:

1. **Catmull-Rom spline** — an interpolating spline that passes exactly through every control point with C1 continuity.
2. **Conversion to cubic Bézier segments** — each Catmull-Rom segment is converted to a cubic Bézier curve with control handles derived from the tangent vectors.
3. **Uniform sampling** — the Bézier segments are sampled proportionally to their arc length, producing `n_out=200` evenly spaced points.

The `tension` parameter (default 0.5) controls how "tight" the curve is:
- `tension=0.0` — very loose, rounded curves
- `tension=0.5` — natural, balanced (default)
- `tension=1.0` — tight, almost straight lines between points

### Geometry Validation (Safety)

Before computing $H_{proj}$ on a live frame, the tool validates the detected quadrilateral:

| Check | Threshold | Failure Action |
|---|---|---|
| Aspect ratio vs. reference | ±15% | Flash "INVALID POSE GEOMETRY" |
| Internal angles | 45°–135° | Flash "INVALID POSE GEOMETRY" |
| Landmark visibility | avg < 0.7 | Pose lost (no overlay) |

This prevents projection when the patient is in an extreme pose, severely distorted by perspective, or partially occluded.

---

## UI Reference

### Toolbar

| Button | Action | Mode |
|---|---|---|
| **Open Image** | Load a reference or stitched image | — |
| **Detect Anchors** | Run MediaPipe Pose on the current image | — |
| **Set Anchors Manually** | Click 4 landmarks manually (fallback) | — |
| **Set Points** | Click to place discrete massage points | `point` |
| **Draw Path** | Click control points, right-click/Enter to finish | `path` |
| **View** | Pan and zoom only | `view` |
| **Save Session** | Write JSON with all $(u,v)$ data | — |
| **Load Session** | Load a previously saved JSON | — |
| **Start Live Feed** | Begin webcam projection | — |
| **Stop Live Feed** | Stop webcam and release camera | — |

### Mouse / Keyboard Shortcuts

| Input | Action |
|---|---|
| Left click (in `point` mode) | Place a discrete point |
| Left click (in `path` mode) | Add a control point to the current path |
| Right click (in `path` mode) | Finish the current path |
| Enter (in `path` mode) | Finish the current path |
| Escape (in `path` mode) | Cancel the current path |
| Scroll wheel | Zoom in/out |
| Drag (in `view` mode) | Pan the image |

### Visual Elements

| Element | Color | Meaning |
|---|---|---|
| Magenta dashed quadrilateral | — | Anchor landmark bounds (shoulders + hips) |
| Orange dots | — | Control points of a path being built |
| Orange dashed line | — | Polyline connecting control points |
| Cyan solid line | — | Final smoothed Bézier path |
| Orange circle + label | — | Discrete massage point |
| Red banner | — | "INVALID POSE GEOMETRY" — projection suspended |

---

## Installation

### System Dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-pyqt6 \
  libopencv-dev \
  python3-opencv \
  ros-jazzy-cv-bridge
```

### Python Dependencies

```bash
pip install --break-system-packages \
  numpy==1.26.4 \
  mediapipe \
  scipy
```

> **Note:** `mediapipe` requires `numpy<2`. The system may have `numpy 2.x` installed. If `mediapipe` fails to import with `AttributeError: module 'mediapipe' has no attribute 'solutions'`, reinstall numpy:
> ```bash
> pip install --break-system-packages "numpy<2" --force-reinstall
> ```

---

## Build

```bash
cd ~/Documents/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select massage_path_tool --symlink-install
source install/setup.bash
```

---

## Usage

### Launch the GUI

```bash
# Phase A — Author on reference image
ros2 run massage_path_tool gui --image ~/Documents/ros2_ws/src/lidar_camera_fusion/massage_points_reference.png

# Phase B — Verify on stitched scan
ros2 run massage_path_tool gui \
  --image ~/ros2_scans/stitched_20260617_120000.png \
  --session ~/ros2_scans/massage_session.json

# Phase C — Live (load session first, then click Start Live Feed)
ros2 run massage_path_tool gui --session ~/ros2_scans/massage_session.json
```

### Command-Line Arguments

| Argument | Description |
|---|---|
| `--image IMAGE` | Path to a reference or stitched image to load on startup |
| `--session SESSION` | Path to a saved JSON session to load on startup |

---

## Session File Format

Sessions are saved as JSON with the following schema:

```json
{
  "version": 1,
  "source_image": "/path/to/reference.png",
  "calibration_aspect_ratio": 0.508,
  "anchor_landmarks": {
    "left_shoulder":  {"px": 101.65, "py": 201.95, "u": 0.0, "v": 0.0},
    "right_shoulder": {"px": 289.94, "py": 205.82, "u": 1.0, "v": 0.0},
    "right_hip":      {"px": 245.08, "py": 481.38, "u": 1.0, "v": 1.0},
    "left_hip":       {"px": 148.07, "py": 480.87, "u": 0.0, "v": 1.0}
  },
  "discrete_points": [
    {"id": "p1", "label": "Jian_Jing_L", "u": 0.35, "v": 0.12},
    {"id": "p2", "label": "Jian_Jing_R", "u": 0.65, "v": 0.12}
  ],
  "paths": [
    {
      "id": "path1",
      "label": "spine_center",
      "points": [
        {"u": 0.50, "v": 0.05},
        {"u": 0.50, "v": 0.10},
        ...
      ]
    }
  ]
}
```

All coordinates are normalized $(u,v)$ in the unit square defined by the anchor landmarks. The `source_image` field records which image was used for calibration, but the data is image-agnostic — the same session can be re-projected onto any other image or live feed with valid anchor detection.

---

## Package Layout

```
massage_path_tool/
├── package.xml                          # ROS 2 ament_python package manifest
├── setup.py                             # setuptools entry point (console_scripts: gui)
├── setup.cfg
├── README.md                            # This file
├── resource/
│   └── massage_path_tool                # Package marker for ament_index
├── massage_path_tool/
│   ├── __init__.py
│   ├── main.py                          # Entry point: argparse + QApplication
│   ├── core/
│   │   ├── __init__.py
│   │   ├── pose_detector.py            # MediaPipe Tasks API wrapper (landmarks 11/12/23/24)
│   │   ├── homography.py               # cv2.findHomography + perspectiveTransform + quad validation
│   │   ├── bezier.py                   # Catmull-Rom → cubic Bézier converter + uniform sampler
│   │   └── smoother.py                 # B-spline alternative (scipy.interpolate.splprep) — legacy
│   │   └── models/
│   │       ├── pose_landmarker_full.task   # MediaPipe full model (heavy, accurate)
│   │       └── pose_landmarker_lite.task   # MediaPipe lite model (fast, less accurate)
│   ├── io/
│   │   ├── __init__.py
│   │   └── session.py                  # JSON load/save with schema validation
│   └── ui/
│       ├── __init__.py
│       ├── overlay.py                  # QGraphicsItem helpers (quad, points, paths, banner)
│       ├── canvas.py                   # QGraphicsView with 4 interaction modes
│       ├── main_window.py             # Toolbar, session state, mode wiring
│       └── live_worker.py            # QThread: webcam + pose detection + projection
└── test/
    ├── __init__.py
    ├── test_pose_detector.py           # MediaPipe detection on real reference image
    ├── test_homography.py              # H_norm, H_proj, roundtrip, validation
    ├── test_bezier.py                  # Catmull-Rom → Bézier conversion, interpolation, shape
    ├── test_smoother.py                # B-spline legacy tests
    └── test_session_io.py            # JSON roundtrip, schema validation
```

---

## Troubleshooting

### "No pose detected" or low visibility on reference image

**Cause:** MediaPipe Pose was trained primarily on upright/standing poses. Prone (face-down) detection works but may fail on some images.

**Fix:** Click **Set Anchors Manually** and click the four landmarks in order: left shoulder → right shoulder → right hip → left hip. The rest of the workflow is identical.

### "INVALID POSE GEOMETRY" flashes during live feed

**Cause:** The live quadrilateral's aspect ratio differs from the reference by >15%, or an internal angle is outside 45°–135°. This happens when the patient turns, bends, or moves too close/far from the camera.

**Fix:** Ask the patient to assume a neutral prone/supine pose facing the camera directly. The projection resumes automatically once geometry is valid.

### Paths look jagged or don't pass through clicked points

**Cause:** The old B-spline smoother (`smoother.py`) was an approximating spline — it did not pass through control points. The new default is Catmull-Rom → Bézier (`bezier.py`), which is interpolating.

**Fix:** This is the default behavior as of the latest version. If you loaded an old session, re-draw the path in `Draw Path` mode.

### `mediapipe` import error: `module 'mediapipe' has no attribute 'solutions'`

**Cause:** The installed `mediapipe` wheel is a tasks-only build (no `solutions` submodule). This package uses the **Tasks API** (`mediapipe.tasks.vision.PoseLandmarker`), not the legacy `solutions.pose` API.

**Fix:** No action needed — the code already uses the Tasks API. If you see this error in other scripts, update them to use `mediapipe.tasks.vision` or reinstall mediapipe from a wheel that includes `solutions`.

### Camera fails to open in live mode

**Cause:** The default camera index is `0` (`/dev/video0`). If another device (e.g. v4l2loopback, secondary webcam) occupies index 0, OpenCV may fail.

**Fix:** Edit `ui/live_worker.py` and change `camera_index=0` to the correct index (try `1`, `2`, etc.).

### PyQt6 vs. PyQt5 conflict

**Cause:** The ROS 2 workspace may have PyQt5 installed (e.g. for `manual_segmentation_node`). Both can coexist, but importing order matters.

**Fix:** This package uses PyQt6 exclusively. No known conflicts in the current workspace. If you see `ImportError` about `QtCore`, ensure `python3-pyqt6` is installed:
```bash
sudo apt install python3-pyqt6
```

---

## Dependencies

| Package | Purpose |
|---|---|
| `PyQt6` | GUI framework (QMainWindow, QGraphicsView, QThread) |
| `opencv-python` | `cv2.findHomography`, `cv2.perspectiveTransform`, webcam I/O |
| `mediapipe` | PoseLandmarker (Tasks API) for landmark detection |
| `numpy` (<2) | Vectorized coordinate math |
| `scipy` | `splprep` legacy smoother (optional) |
| `rclpy` | ROS 2 entry point only — no runtime ROS dependency |

---

## License

Same as the parent workspace (`lidar_camera_fusion`).
