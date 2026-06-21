# massage_path_tool

A PyQt6 desktop application for authoring and projecting massage-point and massage-path trajectories onto a human body using 2D optical homography, plus a pure-Python pipeline that translates those 2D paths into absolute 3D Cartesian waypoints for gantry actuation.

**Two subsystems in one package:**

1. **GUI Authoring/Projection** — map pixel coordinates drawn on a static reference image into normalized $(u,v)$ space via MediaPipe Pose landmarks, then re-project onto a live video feed. Pose-invariant.
2. **2D→3D Path Translator** — offline CLI (`massage_path_translator`) that reads the authored session plus a 3D point cloud and gantry scan manifest, computes gantry X motor positions and body-surface Z depths via proportional 2.5D slicing, and writes `trajectories_3d.json` for RViz visualisation or physical gantry execution.

---

## Table of Contents

1. [Overview](#overview)
2. [Three-Phase Workflow](#three-phase-workflow)
3. [2D→3D Translation Pipeline](#2d3d-translation-pipeline)
4. [Mathematical Model](#mathematical-model)
5. [Session File Format (v2)](#session-file-format-v2)
6. [Output: trajectories_3d.json](#output-trajectories_3djson)
7. [UI Reference](#ui-reference)
8. [Installation](#installation)
9. [Build](#build)
10. [Usage](#usage)
11. [Package Layout](#package-layout)
12. [Troubleshooting](#troubleshooting)
13. [Dependencies](#dependencies)

---

## Overview

The tool solves two related problems:

**Problem 1 (GUI):** How to define massage points and paths on a reference photograph of a body, then track those same anatomical locations on a *different* patient (or the same patient in a different pose) in real time.

**Problem 2 (Translator):** Given a set of 2D massage paths authored on a stitched panoramic image, how to compute the absolute 3D gantry coordinates ($X_{motor}$, $Y_{body}$, $Z_{surface}$) for each waypoint so a physical massage robot can execute the path.

The GUI handles Problem 1. The translator CLI handles Problem 2 — it is a **pure Python CLI with zero ROS dependencies** (ROS 2 is only needed for optional RViz visualisation via `marker_publisher_node`).

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
6. Click **Save Session** to write a JSON file. Session schema v2 now stores `x_stitch`/`y_stitch` (raw pixel coordinates on the stitched image) alongside $(u,v)$ — required for the 2D→3D translator.

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

## 2D→3D Translation Pipeline

```
 ┌─────────────────────┐   ┌──────────────────────┐   ┌──────────────────┐
 │  massage_session    │   │  manifest.json       │   │  scan.pcd        │
 │  (schema v2)        │   │  (scan session dir)  │   │  (base_link, m)  │
 └────────┬────────────┘   └──────────┬───────────┘   └────────┬─────────┘
          │                           │                        │
          └───────────────────────────┼────────────────────────┘
                                      │
                                      ▼
                         ┌──────────────────────────┐
                         │  massage_path_translator  │
                         │  (pure Python CLI, no ROS)│
                         └────────────┬─────────────┘
                                      │
                                      ▼
                         ┌──────────────────────────┐
                         │  trajectories_3d.json    │
                         │  (frame: base_link, m)   │
                         └────────────┬─────────────┘
                                      │
                                      ▼
                         ┌──────────────────────────────┐
                         │  marker_publisher_node       │
                         │  (optional ROS 2 node)       │
                         │  → MarkerArray on            │
                         │    /massage_trajectories_3d   │
                         └──────────────────────────────┘
```

### Input files

| File | Source | Format |
|------|--------|--------|
| `massage_session.json` | GUI save (schema v2) | JSON with `x_stitch`/`y_stitch` per point |
| `manifest.json` | `scan_image_recorder_node` output dir | JSON with `frames[].x_mm` + `resolved_pixels_per_mm` |
| `scan.pcd` | `scan_assembler_node` on `/scanner/stop` | Binary PCD (`x y z` float32, metres, `base_link` frame) |

### Key mapping

The stitcher (`scan_image_recorder_node`) places each camera frame's centre-row on a vertical canvas using:

```
canvas_y_center = (x_mm - x_min_mm) × ppm + frame_H / 2
```

Where `ppm = resolved_pixels_per_mm` (computed live from camera intrinsics and lidar distance).

**The reverse mapping** (y_stitch → X_motor_mm) is therefore linear:

```
X_motor_mm = x_min_mm + (y_stitch - frame_H / 2) / ppm
```

- `y_stitch` = pixel row in the stitched image (0 = top) — stored in session schema v2
- `x_min_mm` = min of all `frames[].x_mm` from manifest
- `frame_H` = source camera frame height (e.g. 480 px for C922 YUYV 640×480)
- `ppm` = resolved_pixels_per_mm from manifest (~1.06 px/mm typical)

No image file reading is required — `ppm`, `x_min_mm`, `x_max_mm`, and `frame_H` are all derived from manifest.json.

### Proportional Y (body-lateral)

The y_ratio maps a stitched-image Y pixel into a body-relative position [0, 1] using anchor landmarks:

```python
y_ratio = clip((y_stitch - y_shoulder_px) / (y_hip_px - y_shoulder_px), -0.2, 1.2)
```

- `0.0` = shoulder level (top of body, low Y index in image)
- `1.0` = hip level (bottom of body)
- Small slack beyond [0, 1] allows extremities (above shoulders, below hips)

### Cloud slicing (2.5D Z extraction)

For each (X_motor, y_ratio) pair:

1. **Slice** the point cloud: keep points where `|X - X_motor| ≤ x_tol_mm`
2. **Body Y bounds** in the slice: `[Y_min, Y_max]`
3. **Proportional Y target**: `Y_target = Y_min + y_ratio × (Y_max - Y_min)`
4. **KD-Tree nearest neighbour** on `(X, Y)` → extract `Z_target`
5. Result clamped to `[x_min, x_max]` motor bounds

If a slice contains fewer than 2 points, the waypoint is skipped and logged in the `skipped` array.

### Components

| Module | Purpose | ROS? |
|--------|---------|------|
| `core/manifest_loader.py` | Parse manifest.json, compute X_motor_mm from y_stitch | No |
| `core/cloud_slicer.py` | 2.5D slice + KD-Tree Z extraction | No |
| `core/path_translator.py` | CLI — orchestrates the full pipeline | No |
| `ros/marker_publisher_node.py` | Optional — publish trajectories as MarkerArray for RViz | Yes |

---

## Mathematical Model

### Homography (GUI subsystem)

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

### Bézier Curve Smoothing (GUI subsystem)

Raw mouse clicks produce sparse, unevenly spaced control points. The tool converts these into a smooth interpolating curve using:

1. **Catmull-Rom spline** — an interpolating spline that passes exactly through every control point with C1 continuity.
2. **Conversion to cubic Bézier segments** — each Catmull-Rom segment is converted to a cubic Bézier curve with control handles derived from the tangent vectors.
3. **Uniform sampling** — the Bézier segments are sampled proportionally to their arc length, producing `n_out=200` evenly spaced points.

The `tension` parameter (default 0.5) controls how "tight" the curve is:
- `tension=0.0` — very loose, rounded curves
- `tension=0.5` — natural, balanced (default)
- `tension=1.0` — tight, almost straight lines between points

### 2D→3D Mapping (translator subsystem)

```
y_stitch ──► ManifestLoader ──► X_motor_mm (gantry axis)
y_stitch ──► y_ratio()     ──► y_ratio [0, 1] (body-lateral)
x_motor_m  +  y_ratio ──► CloudSlicer ──► (Y_body_m, Z_surface_m)
```

Three inputs → three outputs per waypoint. No camera intrinsics, no cv2.projectPoints, no ROS.

### Geometry Validation (Safety)

Before computing $H_{proj}$ on a live frame, the tool validates the detected quadrilateral:

| Check | Threshold | Failure Action |
|---|---|---|
| Aspect ratio vs. reference | ±15% | Flash "INVALID POSE GEOMETRY" |
| Internal angles | 45°–135° | Flash "INVALID POSE GEOMETRY" |
| Landmark visibility | avg < 0.7 | Pose lost (no overlay) |

This prevents projection when the patient is in an extreme pose, severely distorted by perspective, or partially occluded.

---

## Session File Format (v2)

Sessions are saved as JSON with `schema_version: 2`. Version 1 (legacy) sessions load without error but points lacking `x_stitch`/`y_stitch` will be **skipped by the translator** — re-author with the v2 UI to enable 2D→3D translation.

### Example v2 Session

```json
{
  "schema_version": 2,
  "source_image": "/path/to/reference.png",
  "calibration_aspect_ratio": 0.508,
  "anchor_landmarks": {
    "left_shoulder":  {"px": 101.65, "py": 201.95, "u": 0.0, "v": 0.0},
    "right_shoulder": {"px": 289.94, "py": 205.82, "u": 1.0, "v": 0.0},
    "right_hip":      {"px": 245.08, "py": 481.38, "u": 1.0, "v": 1.0},
    "left_hip":       {"px": 148.07, "py": 480.87, "u": 0.0, "v": 1.0}
  },
  "discrete_points": [
    {
      "id": "p1",
      "label": "Jian_Jing_L",
      "u": 0.35, "v": 0.12,
      "x_stitch": 180.0, "y_stitch": 150.0
    },
    {
      "id": "p2",
      "label": "Jian_Jing_R", 
      "u": 0.65, "v": 0.12,
      "x_stitch": 420.0, "y_stitch": 148.0
    }
  ],
  "paths": [
    {
      "id": "path1",
      "label": "spine_center",
      "points": [
        {"u": 0.50, "v": 0.05, "x_stitch": 320.0, "y_stitch": 85.0},
        {"u": 0.50, "v": 0.10, "x_stitch": 320.0, "y_stitch": 120.0}
      ]
    }
  ]
}
```

### v1 vs v2

| Field | v1 | v2 |
|-------|----|----|
| `schema_version` | 1 | 2 |
| discrete points | `{id, label, u, v}` | `{id, label, u, v, x_stitch, y_stitch}` |
| path points | `{u, v}` | `{u, v, x_stitch, y_stitch}` |

The `x_stitch`/`y_stitch` fields store the raw pixel coordinates on the stitched image. These are set automatically by the GUI when authoring on a loaded image — no manual entry needed.

---

## Output: trajectories_3d.json

```json
{
  "frame": "base_link",
  "units": "metres",
  "source": {
    "session": "/abs/path/massage_session.json",
    "manifest": "/abs/path/manifest.json",
    "pcd": "/abs/path/scan.pcd"
  },
  "x_tolerance_mm": 5.0,
  "discrete_points": [
    {"id": "p1", "label": "p1", 
     "x": 0.123456, "y": 0.345678, "z": 0.821000, "n_slice_pts": 147}
  ],
  "paths": [
    {
      "id": "path1",
      "label": "path1",
      "waypoints": [
        {"x": 0.120000, "y": 0.320000, "z": 0.819000, "n_slice_pts": 132},
        {"x": 0.125000, "y": 0.340000, "z": 0.823000, "n_slice_pts": 118}
      ]
    }
  ],
  "skipped": [
    {"id": "p3", "reason": "no cloud slice at X=0.340m (0 pts in ±5mm window)"}
  ]
}
```

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
| **Save Session** | Write JSON with all $(u,v)$ + pixel data | — |
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
# GUI authoring tool
pip install --break-system-packages \
  numpy==1.26.4 \
  mediapipe \
  scipy

# 2D→3D translator
pip install --break-system-packages \
  open3d
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

### GUI — Author Massage Paths

```bash
# Phase A — Author on reference image
ros2 run massage_path_tool gui --image ~/Documents/ros2_ws/src/lidar_camera_fusion/massage_points_reference.png

# Phase B — Verify on stitched scan
ros2 run massage_path_tool gui \
  --image ~/ros2_scans/stitched_20260617_120000.png \
  --session ~/ros2_scans/massage_session.json

# Phase C — Live (load session, then click Start Live Feed)
ros2 run massage_path_tool gui --session ~/ros2_scans/massage_session.json
```

### CLI — Translate 2D Paths → 3D Waypoints

```bash
massage_path_translator \
  --session  massage_session.json \
  --manifest ~/ros2_scans/scan_session_20260617_124637/manifest.json \
  --pcd      ~/ros2_scans/body_20260617_124637.pcd \
  --frame-h  480 \
  --output   /tmp/trajectories_3d.json \
  --x-tol-mm 10.0
```

Or via `python -m`:

```bash
python -m massage_path_tool.core.path_translator \
  --session  massage_session.json \
  --manifest ~/ros2_scans/scan_session_20260617_124637/manifest.json \
  --pcd      ~/ros2_scans/body_20260617_124637.pcd \
  --frame-h  480 \
  --output   /tmp/trajectories_3d.json \
  --x-tol-mm 10.0
```

#### CLI Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--session` | (required) | v2 `massage_session.json` with `x_stitch`/`y_stitch` |
| `--manifest` | (required) | `scan_image_recorder_node` manifest.json |
| `--pcd` | (required) | Binary PCD, `base_link` frame, units metres |
| `--frame-h` | `480` | Source camera frame height in pixels (C922 = 480) |
| `--output` | `trajectories_3d.json` | Output file path |
| `--x-tol-mm` | `5.0` | X slice half-window in mm for cloud slicing |

### RViz — Visualise 3D Trajectories

```bash
source /opt/ros/jazzy/setup.bash && source ~/Documents/ros2_ws/install/setup.bash
ros2 run massage_path_tool marker_publisher_node \
  --ros-args -p trajectories_file:=/tmp/trajectories_3d.json
```

Then open RViz, add **MarkerArray** display, topic `/massage_trajectories_3d`, fixed frame `base_link`. Orange spheres = discrete points, cyan lines = paths.

---

## Package Layout

```
massage_path_tool/
├── package.xml                              # ROS 2 ament_python manifest
├── setup.py                                 # setuptools (gui, massage_path_translator, marker_publisher_node)
├── setup.cfg
├── README.md                                # This file
├── SHARING.md                               # Development sharing notes
├── resource/
│   └── massage_path_tool                    # Package marker for ament_index
├── massage_path_tool/
│   ├── __init__.py
│   ├── main.py                              # GUI entry point: argparse + QApplication
│   ├── core/
│   │   ├── __init__.py
│   │   ├── pose_detector.py                 # MediaPipe Tasks API (landmarks 11/12/23/24)
│   │   ├── homography.py                    # cv2.findHomography + perspectiveTransform + validation
│   │   ├── bezier.py                        # Catmull-Rom → cubic Bézier + sampler
│   │   ├── smoother.py                      # B-spline alternative (legacy)
│   │   ├── manifest_loader.py               # manifest.json → X_motor_mm via ppm formula
│   │   ├── cloud_slicer.py                  # 2.5D slice + KD-Tree Z extraction
│   │   ├── path_translator.py               # CLI: session + manifest + PCD → trajectories_3d.json
│   │   └── models/
│   │       ├── pose_landmarker_full.task
│   │       └── pose_landmarker_lite.task
│   ├── io/
│   │   ├── __init__.py
│   │   └── session.py                       # JSON load/save (schema v1/v2 compatible)
│   ├── ros/
│   │   ├── __init__.py
│   │   └── marker_publisher_node.py         # ROS 2: trajectories_3d.json → MarkerArray
│   └── ui/
│       ├── __init__.py
│       ├── overlay.py                       # QGraphicsItem helpers (quad, points, paths, banner)
│       ├── canvas.py                        # QGraphicsView with 4 interaction modes
│       ├── main_window.py                   # Toolbar, session state, mode wiring
│       └── live_worker.py                   # QThread: webcam + pose detection + projection
└── test/
    ├── __init__.py
    ├── test_pose_detector.py                # MediaPipe on real reference image
    ├── test_homography.py                   # H_norm, H_proj, roundtrip, validation
    ├── test_bezier.py                       # Catmull-Rom → Bézier conversion
    ├── test_smoother.py                     # B-spline legacy tests
    ├── test_session_io.py                   # JSON roundtrip, schema v1/v2, validation
    ├── test_manifest_loader.py              # y_stitch → X_motor_mm, ppm, clamps
    ├── test_cloud_slicer.py                 # 2.5D slice, Y ratio, KD-Tree, edge cases
    └── test_path_translator_cli.py          # Integration test: full 2D→3D pipeline
```

---

## Troubleshooting

### "No pose detected" or low visibility on reference image

**Cause:** MediaPipe Pose was trained primarily on upright/standing poses. Prone (face-down) detection works but may fail on some images.

**Fix:** Click **Set Anchors Manually** and click the four landmarks in order: left shoulder → right shoulder → right hip → left hip. The rest of the workflow is identical.

### "INVALID POSE GEOMETRY" flashes during live feed

**Cause:** The live quadrilateral's aspect ratio differs from the reference by >15%, or an internal angle is outside 45°–135°. This happens when the patient turns, bends, or moves too close/far from the camera.

**Fix:** Ask the patient to assume a neutral prone/supine pose facing the camera directly. The projection resumes automatically once geometry is valid.

### All points skipped by translator

**Cause:** The session file has `schema_version: 1` and points lack `x_stitch`/`y_stitch`. The translator cannot compute gantry X without pixel coordinates.

**Fix:** Re-author the session with the v2 UI (which stores pixel coords automatically), or write a migration script.

### Paths look jagged or don't pass through clicked points

**Cause:** The old B-spline smoother (`smoother.py`) was an approximating spline — it did not pass through control points. The new default is Catmull-Rom → Bézier (`bezier.py`), which is interpolating.

**Fix:** This is the default behavior as of the latest version. If you loaded an old session, re-draw the path in `Draw Path` mode.

### `mediapipe` import error

**Cause:** The installed `mediapipe` wheel is a tasks-only build (no `solutions` submodule). This package uses the **Tasks API** (`mediapipe.tasks.vision.PoseLandmarker`), not the legacy `solutions.pose` API.

**Fix:** No action needed — the code already uses the Tasks API.

### Camera fails to open in live mode

**Cause:** The default camera index is `0` (`/dev/video0`). If another device (e.g. v4l2loopback, secondary webcam) occupies index 0, OpenCV may fail.

**Fix:** Edit `ui/live_worker.py` and change `camera_index=0` to the correct index (try `1`, `2`, etc.).

### `open3d` import error in translator

**Fix:** Install open3d in the workspace Python environment:
```bash
pip install --break-system-packages open3d
```

---

## Dependencies

| Package | Purpose | Subsystem |
|---------|---------|-----------|
| `PyQt6` | GUI framework (QMainWindow, QGraphicsView, QThread) | GUI |
| `opencv-python` | `cv2.findHomography`, `cv2.perspectiveTransform`, webcam I/O | GUI |
| `mediapipe` | PoseLandmarker (Tasks API) for landmark detection | GUI |
| `numpy` (<2) | Vectorized coordinate math | Both |
| `scipy` | KD-Tree for Z extraction (`cKDTree`), legacy smoother | Translator |
| `open3d` | PCD file reading (`o3d.io.read_point_cloud`) | Translator |
| `rclpy` | ROS 2 node base (marker publisher only) | RViz |
| `visualization_msgs` | Marker/MarkerArray message definitions | RViz |
| `geometry_msgs` | Point message | RViz |

---

## License

Same as the parent workspace (`lidar_camera_fusion`).
