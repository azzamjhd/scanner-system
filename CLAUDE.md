# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cd ~/Documents/ros2_ws
source /opt/ros/jazzy/setup.bash

# Build all custom packages (rplidar_ros is excluded via .gitignore; it builds separately if present)
colcon build --symlink-install

# Build a single package
colcon build --packages-select medical_scanner_pkg
colcon build --packages-select gantry_image_stitcher
colcon build --packages-select massage_perception

# After every build, source the overlay
source install/setup.bash
```

## Run the full system

The tmuxinator config at `~/Documents/.tmuxinator.yml` starts everything:

```bash
tmuxinator start -p ~/Documents/.tmuxinator.yml
```

Three windows:
- **hardware** — micro-ROS agent (ESP32 on `/dev/ttyUSB0 @115200`) + RPLiDAR A1 (`/dev/ttyUSB1`)
- **scanner** — `scanner_3d_node` + `scanner_gui`
- **perception** — v4l2 camera (with auto-reconnect loop) + gantry stitcher

## Run nodes individually

```bash
# Scanner processing node
ros2 run medical_scanner_pkg scanner_3d_node

# GUI control panel
ros2 run medical_scanner_pkg scanner_gui

# With camera topic override
ros2 run medical_scanner_pkg scanner_gui --ros-args -p image_topic:=/camera/image_raw

# With stitcher integration
ros2 run medical_scanner_pkg scanner_gui --ros-args \
  -p enable_stitcher_integration:=true

# Gantry image stitcher
ros2 launch gantry_image_stitcher stitcher.launch.py

# Pose/massage-point detection
ros2 run massage_perception pose_node

# Camera (with auto-reconnect)
ros2 run v4l2_camera v4l2_camera_node --ros-args -r image_raw:=/camera/image_raw

# RPLiDAR A1
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

## Scan lifecycle (CLI)

```bash
ros2 service call /start_scan std_srvs/srv/Trigger
ros2 service call /stop_scan std_srvs/srv/Trigger          # saves .ply to output_dir
ros2 service call /clear_visualization std_srvs/srv/Trigger

# Gantry stitcher session
ros2 service call /stitcher/start_session std_srvs/srv/Trigger
ros2 service call /stitcher/stop_session  std_srvs/srv/Trigger
```

## Firmware (ESP32 micro-ROS)

The firmware lives outside this workspace (PlatformIO project). See `src/README_MICROROS_FIRMWARE.md` for pinout and constants. Key values: `STEPS_PER_MM = 80`, position clamp ±2000 mm, speed 0.1–200 mm/s, accel 1–400 mm/s².

The firmware publishes `/current_position` (Float32, mm) and subscribes to `/position`, `/speed`, `/acceleration`.

## Architecture

Three custom packages in `src/`:

```
medical_scanner_pkg/     — scanner node + GUI
gantry_image_stitcher/   — image capture & stitch node
massage_perception/      — MediaPipe pose + massage-point node
src/rplidar_ros/         — external (own .git, excluded from workspace git)
```

### `medical_scanner_pkg`

**`scanner_3d_node`** fuses RPLiDAR `LaserScan` with encoder position (`/current_position`) to build a 3D `PointCloud2`. Two axis modes: `axis=0` rotational-Y (angular position in degrees), `axis=1` linear-Z (position in mm). Supports `simulate_encoder=true` for testing without hardware. Saves scans on `/stop_scan` as timestamped `.ply` files. Downsamples to 50 k points for live RViz display.

**`scanner_gui_node`** is the PyQt5 desktop app. It has three layers:

- `_ROSBridge` — `QObject` with `pyqtSignal`s. Emitted from the ROS spin thread; Qt auto-queues them onto the GUI main thread (no explicit locking needed).
- `ScannerROSNode` — holds all publishers, subscribers, service clients. Callbacks emit bridge signals.
- `MissionControlWindow` — thin `QMainWindow` that owns a `MissionControlGUI` widget (from `mission_control_gui.py`) and wires all signals.

**`mission_control_gui.py`** is a self-contained `QWidget` with a dark mission-control theme. Key signals it emits: `start_scan_requested`, `stop_scan_requested`, `estop_requested`, `move_requested(target, speed, accel)`, `home_requested`, `lidar_settings_changed`, `scan_settings_changed`, `rviz_launch_requested`, `rviz_clear_requested`, `preset_loaded`. Public API for telemetry: `set_position(mm)`, `set_rpm(rpm)`, `set_points(n)`, `set_camera_image(QImage)`, `set_connected(bool)`.

> **Known PyQt5 quirk**: `QFontDatabase().hasFamily()` was removed in newer Qt5 builds. The `_families()` helper in `mission_control_gui.py` wraps `db.families()` (Qt5) with a `TypeError` fallback to `QFontDatabase.families()` (Qt6 static API). Do not revert to `hasFamily()`.

### `gantry_image_stitcher`

`StitcherNode` captures camera frames at configurable spacing (default 5 mm) keyed to `/current_position`. On `/stitcher/stop_session` it attempts OpenCV stitching (`Stitcher_SCANS` then `Stitcher_PANORAMA`) and falls back to overlapping horizontal concatenation. Output: `output_dir/<session_id>/stitched_final.png` + `manifest.json`.

### `massage_perception`

`MassagePoseNode` subscribes to `/camera/image_raw`, runs MediaPipe BlazePose (`LIVE_STREAM` mode), and publishes 33 skeleton landmarks on `pose_landmarks` and derived massage target points on `massage_points` (both `PoseArray`). Massage points are defined in `config/massage_points.yaml` as weighted sums of landmark indices + offsets. The config hot-reloads if the file changes on disk.

## ROS2 topic/service graph summary

| Topic | Direction | Type | Producer → Consumer |
|---|---|---|---|
| `/scan` | pub | `LaserScan` | rplidar_ros → scanner_3d_node |
| `/current_position` | pub | `Float32` | ESP32 firmware → scanner_3d_node, scanner_gui, stitcher_node |
| `/motor_speed` | pub | `Float32` | firmware → scanner_gui |
| `/position`, `/speed`, `/acceleration` | pub | `Float32` | scanner_gui → firmware |
| `/scanner/cmd` | pub | `Vector3` | scanner_gui → (legacy compat) |
| `/scanner/pointcloud` | pub | `PointCloud2` | scanner_3d_node → scanner_gui, RViz |
| `/camera/image_raw` | pub | `Image` | v4l2_camera → scanner_gui, stitcher, massage_perception |
| `/stitcher/preview` | pub | `Image` | stitcher → any |
| `/stitcher/status` | pub | `String` | stitcher → any |
| `/start_scan`, `/stop_scan`, `/clear_visualization` | srv | `Trigger` | scanner_gui → scanner_3d_node |
| `/stitcher/start_session`, `/stitcher/stop_session` | srv | `Trigger` | scanner_gui → stitcher_node |

## Hardware device assignment

| Device | Port | Node |
|---|---|---|
| ESP32 (micro-ROS) | `/dev/ttyUSB0` | `micro_ros_agent` |
| RPLiDAR A1 | `/dev/ttyUSB1` | `rplidar_ros` |
| USB camera | `/dev/video0` | `v4l2_camera_node` |

The v4l2 camera pane in tmuxinator uses a restart loop (`until [ -e /dev/video0 ]`) so that unplug/replug automatically restarts the node when the device reappears.
