# Massage Scanning System — Sharing Guide

**Author:** Azzam  
**Target:** Ubuntu 24.04 (Noble) + ROS 2 Jazzy  
**Hardware:** RPLiDAR A1, Logitech C922, ESP32 (micro-ROS), gantry

---

## Quick start (for someone with the hardware)

```bash
# 1. Clone the workspace (it IS the source tree)
git clone https://github.com/azzamjhd/scanner-system.git ~/scanner_ws
cd ~/scanner_ws

# 2. One command to pull external deps, install everything, and build
./bootstrap.sh

# 3. Run
source install/setup.bash
ros2 launch lidar_camera_fusion full_system.launch.py
```

## What bootstrap.sh does

0. Installs `python3-vcstool` and runs `vcs import src < scanner_system.repos` to pull `rplidar_ros`
1. Installs ROS 2 Jazzy (if absent)
2. Runs `rosdep install` for all ROS-level dependencies
3. Installs system packages: PCL, OpenCV, NumPy, SciPy, Matplotlib, Tk, PyQt5, PyQtGraph, PyQt6
4. Creates a Python `.venv` with `numpy<2` + `mediapipe` (isolated — won't break system packages)
5. Patches `rplidar_ros` with the A1 `angle_max` fix (359° → 360°)
6. Builds the workspace with `colcon build --symlink-install`

## Package inventory

| Package | Type | What it does |
|---|---|---|
| `lidar_camera_fusion` | C++/Python hybrid | 3D scanner assembler, colorizer, body segmentation, GUI |
| `medical_scanner_pkg` | Python | Original scanner node + PyQt5 mission-control GUI |
| `gantry_image_stitcher` | Python | Distance-triggered camera frame capture + OpenCV stitching |
| `massage_perception` | Python | MediaPipe BlazePose landmark detection → massage points |
| `massage_path_tool` | Python | PyQt6 interactive massage path/path authoring tool |
| `rplidar_ros` | C++ | Slamtec RPLiDAR A1 driver (patched, ros2 branch) |

## Running without hardware

The scanner works in simulation mode for dev/testing:

```bash
ros2 launch lidar_camera_fusion full_system.launch.py \
  mcu_port:='' \
  lidar_port:=/dev/null \
  simulate_encoder:=true
```

## MediaPipe setup

MediaPipe requires `numpy<2`. The bootstrap creates an isolated venv at
`~/.scanner_venv`. Before running any MediaPipe node, source the venv:

```bash
source ~/.scanner_venv/bin/activate
```

This affects:
- `massage_perception` (pose_node)
- `massage_path_tool` (gui)

## Key topics and services

### Topics (pub/sub)
| Topic | Type | Description |
|---|---|---|
| `/scan` | `LaserScan` | RPLiDAR scan data |
| `/scanner/colored_cloud` | `PointCloud2` | 3D point cloud with RGB |
| `/scanner/assembled_cloud` | `PointCloud2` | Uncolored assembled cloud |
| `/scanner/scan_cloud` | `PointCloud2` | Single-ring scan |
| `/scanner/body_cloud` | `PointCloud2` | Bed-stripped body points |
| `/scanner/is_scanning` | `Bool` | Scanner state gate |
| `/scanner/segmented_regions` | `PointCloud2` | Per-region labeled points |
| `/camera/image_raw` | `Image` | Raw camera feed |
| `/camera/camera_info` | `CameraInfo` | Camera intrinsics |
| `/current_position` | `Float32` | Gantry position (mm) |
| `pose_landmarks` | `PoseArray` | 33 MediaPipe landmarks |
| `massage_points` | `PoseArray` | Derived massage target points |

### Services
| Service | Type | Description |
|---|---|---|
| `/scanner/start` | `Trigger` | Start scan accumulation |
| `/scanner/stop` | `Trigger` | Stop scan, save PCD + stitched image |
| `/scanner/clear_cloud` | `Trigger` | Clear accumulated cloud |
| `/scanner/start_cycle` | `Trigger` | Auto sweep (start→end) |
| `/scanner/interrupt` | `Trigger` | Abort sweep, save partial |
| `/scanner/open_segmenter` | `Trigger` | Open body-point segmenter GUI |
| `/stitcher/start_session` | `Trigger` | Start stitching capture |
| `/stitcher/stop_session` | `Trigger` | Finish stitch, save mosaic |

## Hardware wiring

| Device | Port | Node |
|---|---|---|
| ESP32 (micro-ROS) | `/dev/ttyUSB0` | `micro_ros_agent` |
| RPLiDAR A1 | `/dev/ttyUSB1` | `rplidar_ros` |
| Logitech C922 | `/dev/video0` | `v4l2_camera_node` |

## Camera calibration

Place your `.yaml` calibration file at:
```
~/.ros/camera_info/c922_pro_stream_webcam.yaml
```

Or set `camera_info_url` in the launch arguments (default: `file:///home/azzam/Documents/webcam_calibration.yaml`).

## Troubleshooting

**`colcon build` fails with `ModuleNotFoundError: catkin_pkg`**  
Build from the workspace root and force the system Python:
```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

**Camera node crashes with `Device or resource busy (16)`**  
Another process already holds `/dev/video0`. Kill the other camera launch.

**`import mediapipe` fails with `AttributeError`**  
You're on `numpy>=2`. Source the venv: `source ~/.scanner_venv/bin/activate`

**Subscriber callbacks never fire on sensor topics**  
QoS mismatch — the subscriber likely uses `RELIABLE` against a `BEST_EFFORT` camera topic. Set `Reliability Policy: Best Effort` in the display config.

**RViz Image display blank**  
Same QoS issue — set `Reliability Policy: Best Effort` in that display's `Topic:` block.
