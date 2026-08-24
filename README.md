# Smart Massage Machine — Scanning & Perception System

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com/)
[![Python](https://img.shields.io/badge/Python-3.12-yellow)](https://python.org/)

ROS 2 Jazzy workspace for LiDAR–camera sensor fusion on a linear gantry — the
perception subsystem of a smart massage machine. Turns an RPLiDAR A1, a USB
camera, and encoder position feedback into colorized 3D point clouds, then uses
homography-based projection to map massage paths onto a patient's body.

Two main packages:

| Package | Language | Purpose |
|---|---|---|
| **`lidar_camera_fusion`** | C++ / Python | Colorized 3D scanner: scan assembly, rectified colorization, RANSAC body extraction, polygon segmentation, `fusion_gui` control panel |
| **`massage_path_tool`** | Python | PyQt6 interactive path authoring — defines massage trajectories on a reference image, re-projects them onto live video via MediaPipe homography |

Other packages in the workspace (`medical_scanner_pkg`, `gantry_image_stitcher`,
`massage_perception`) are legacy/experimental and not part of the active stack.

---

## System pipeline

```
ESP32 (micro-ROS) ─► /current_position (mm)
                             │
RPLiDAR A1 ──► /scan ───────┤
                             ▼
USB camera ──► /image_raw ──► scan_assembler_node ──► /scanner/assembled_cloud
                  │                + built-in gantry bridge (mm → m)
                  ▼                                │
         rectify_node                    cloud_colorizer_node
    (image_proc, rectified)                   │ (projects with P)
                  │                           ▼
                  └──► /image_rect_color ────► /scanner/colored_cloud (XYZRGB)
                                                       │
                                              body_preprocess_node
                                          (RANSAC bed removal)
                                                       │
                                                       ▼
                                              /scanner/body_cloud
                                                       │
                                                       ▼
                                           manual_segmentation_node
                                           (PyQt polygon GUI)
                                                       │
                                                       ▼
                                           /scanner/segmented_regions
```

After scanning, load the stitched panorama into `massage_path_tool` to
author massage paths in normalized (u,v) coordinates, then re-project them
onto a live video feed.

---

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

`bootstrap.sh` installs ROS 2 (if absent), all system deps, creates an
isolated Python venv for MediaPipe (`numpy<2`), patches `rplidar_ros` (A1
angle fix), and runs `colcon build`.

### Full system

```bash
ros2 launch lidar_camera_fusion full_system.launch.py
```

Launches: micro-ROS agent → RPLiDAR → robot_state_publisher → v4l2_camera →
rectify_node → scan_assembler → cloud_colorizer → scan_image_recorder →
body_preprocess → manual_segmentation → scan_cycle_node → fusion_gui → rviz2.

Key launch arguments (defaults): `mcu_port` (/dev/ttyUSB0), `lidar_port`
(/dev/ttyUSB1), `camera_device` (/dev/video0), `scan_mode` (Boost),
`output_dir` (~/ros2_scans), `max_points` (500000), `publish_rate` (2.0). Sweep
range: `scan_start_mm` (0.0), `scan_end_mm` (300.0), `scan_speed_mm_s` (20.0).

### Running fusion_gui (control panel)

```bash
ros2 run lidar_camera_fusion fusion_gui
```

Single-window control: node status (2 Hz heartbeat), Start/Stop sweep, parameter
discovery for scan_assembler and colorizer, `/rosout` log viewer.

### Massage path authoring

```bash
# Phase A — author paths on a reference image
ros2 run massage_path_tool gui --image /path/to/reference.png

# Phase B — verify on a scan result
ros2 run massage_path_tool gui \
  --image ~/ros2_scans/stitched_20260617_120000.png \
  --session ~/ros2_scans/massage_session.json

# Phase C — live projection on webcam
# (click "Start Live Feed" from the GUI with a session loaded)
```

See [`massage_path_tool/README.md`](src/massage_path_tool/README.md) for the
full three-phase workflow.

### Without hardware

The fusion pipeline needs position feedback. Publish synthetic data:

```bash
# Terminal 1: fusion pipeline (no camera)
ros2 launch lidar_camera_fusion fusion.launch.py camera_device:=/dev/null

# Terminal 2: fake encoder feedback
ros2 topic pub /current_position std_msgs/msg/Float32 "{data: 50}" -r 20
```

### Scan workflow

```bash
# Manual
ros2 service call /scanner/start std_srvs/srv/Trigger
ros2 topic pub /position std_msgs/msg/Float32 "{data: 300.0}" -1
ros2 service call /scanner/stop std_srvs/srv/Trigger

# Automatic sweep
ros2 service call /scanner/start_cycle std_srvs/srv/Trigger
```

Output in `~/ros2_scans/`: `scan_<ts>.pcd` (XYZ), `colored_<ts>.pcd` (XYZRGB),
`body_<ts>.pcd` (segmented body), `regions_<ts>.pcd` (labeled),
`stitched_<ts>.png` (panorama).

---

## Hardware

| Device | Port | Interface |
|---|---|---|
| ESP32 (gantry) | `/dev/ttyUSB0` | micro-ROS serial @ 115200 baud |
| RPLiDAR A1 | `/dev/ttyUSB1` | UART serial |
| USB camera (C922) | `/dev/video0` | V4L2, YUYV 640×480 |

Gantry: `STEPS_PER_MM = 80`, range ±2000 mm, speed 0.1–200 mm/s.

## Camera calibration

Place calibration YAML at `~/.ros/camera_info/c922_pro_stream_webcam.yaml` or
set via `camera_info_url:=file:///path/to/calibration.yaml`.

Auto focus/exposure/WB are locked 4 s after startup via `v4l2-ctl` to keep
intrinsics stable during scans.

---

## Sharing / bootstrap for others

See [`SHARING.md`](./SHARING.md) for the step-by-step setup guide from a
fresh Ubuntu 24.04 install.

---

## License

Apache-2.0 (custom packages). `rplidar_ros` carries its own BSD-2 license.
