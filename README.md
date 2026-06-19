# Smart Massage Machine — Scanning & Perception System

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com/)
[![Python](https://img.shields.io/badge/Python-3.12-yellow)](https://python.org/)

A ROS 2 Jazzy workspace for LiDAR–camera sensor fusion on a linear gantry, built
as the perception subsystem of a smart massage machine. Turns an RPLiDAR A1, a
USB camera, and encoder position feedback into colorized 3D point clouds with
body-part segmentation and massage-point mapping.

---

## System overview

```
ESP32 (micro-ROS) ─► /current_position (mm)
                               │
RPLiDAR A1 ──► /scan ─────────┤
                               ▼
USB camera ──► /image_raw ──► scan_assembler_node ──► /scanner/assembled_cloud
                  │                 + built-in gantry bridge (mm → m)
                  ▼                                │
          rectify_node                 cloud_colorizer_node
     (image_proc, rectified)                │ (rectified P)
                  │                         ▼
                  └────► /image_rect_color ─► /scanner/colored_cloud (XYZRGB)
                                                     │
                                           body_preprocess_node (RANSAC bed removal)
                                                     │
                                                     ▼
                                           /scanner/body_cloud
                                                     │
                                                     ▼
                                        manual_segmentation_node (polygon GUI)
                                                     │
                                                     ▼
                                        /scanner/segmented_regions
```

## Packages

| Package | Language | Purpose |
|---|---|---|
| `lidar_camera_fusion` | C++ / Python | Core scanner: assembler, colorizer, RANSAC bed removal, polygon segmentation, PyQt5 control panel |
| `medical_scanner_pkg` | Python | Legacy scanner node + mission-control GUI (being superseded by `lidar_camera_fusion`) |
| `gantry_image_stitcher` | Python | Distance-triggered camera capture → stitched panorama via OpenStitching |
| `massage_perception` | Python | MediaPipe BlazePose landmarks → derived massage points |
| `massage_path_tool` | Python | PyQt6 interactive path authoring with homography-based pose-invariant projection |
| `rplidar_ros` | C++ | RPLiDAR A1 driver (Slamtec, `ros2` branch) |

## Quick start

### Prerequisites

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy

### Clone and build

```bash
git clone https://github.com/azzamjhd/scanner-system.git ~/scanner_ws
cd ~/scanner_ws
./bootstrap.sh
source install/setup.bash
```

`bootstrap.sh` installs ROS 2 (if absent), all system deps, creates an isolated
Python venv for MediaPipe (`numpy<2`), patches `rplidar_ros` (A1 angle fix),
and runs `colcon build`.

### Run the full scanning pipeline

```bash
ros2 launch lidar_camera_fusion full_system.launch.py
```

### Without hardware (simulation)

```bash
ros2 launch lidar_camera_fusion full_system.launch.py \
  mcu_port:='' lidar_port:=/dev/null scan_mode:=Standard \
  simulate_encoder:=true
```

### Scan workflow

```bash
# Manual (two service calls)
ros2 service call /scanner/start std_srvs/srv/Trigger
ros2 topic pub /position std_msgs/msg/Float32 "{data: 300.0}" -1
ros2 service call /scanner/stop std_srvs/srv/Trigger
# Output: ~/ros2_scans/scan_<ts>.pcd, colored_<ts>.pcd, body_<ts>.pcd

# Automatic sweep
ros2 service call /scanner/start_cycle std_srvs/srv/Trigger
```

### MediaPipe nodes

MediaPipe requires `numpy<2`. Source the venv before running:

```bash
source ~/.scanner_venv/bin/activate
ros2 run massage_perception pose_node
```

## Hardware

| Device | Port | Interface |
|---|---|---|
| ESP32 (gantry control) | `/dev/ttyUSB0` | micro-ROS serial @ 115200 baud |
| RPLiDAR A1 | `/dev/ttyUSB1` | UART serial |
| USB camera (C922) | `/dev/video0` | V4L2, YUYV 640×480 |

Gantry constants: `STEPS_PER_MM = 80`, range ±2000 mm, speed 0.1–200 mm/s.

## Camera calibration

Place your calibration YAML at:
```
~/.ros/camera_info/c922_pro_stream_webcam.yaml
```

Or set via launch arg:
```bash
camera_info_url:=file:///path/to/calibration.yaml
```

The camera must be calibrated with **auto focus/exposure/WB off** — the
`full_system.launch.py` locks these via `v4l2-ctl` 4 s after startup.

## Sharing / bootstrap for others

See [`SHARING.md`](./SHARING.md) for the step-by-step guide to set up the system
from scratch on a fresh Ubuntu 24.04 machine.

## License

Apache-2.0 (custom packages). `rplidar_ros` carries its own BSD-2 license.
