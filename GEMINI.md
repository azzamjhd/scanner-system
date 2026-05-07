# GEMINI.md - Smart Massage Machine ROS 2 Workspace

Instructional context for Gemini CLI in `ros2_ws`.

## Project Overview
**Smart Massage Machine** is an academic capstone project. It uses a linear gantry with RPLiDAR A1 and an RGB camera to scan humans and generate 3D paths for an AR3 robot arm.

### Main Technologies
- **ROS 2 Jazzy**: Core middleware.
- **Python (ament_python)**: `medical_scanner_pkg`, `gantry_image_stitcher`, `massage_perception`.
- **C++ (ament_cmake)**: `lidar_camera_fusion`.
- **micro-ROS**: ESP32 gantry firmware.
- **OpenCV & MediaPipe**: Image stitching and body landmarks.
- **PCL**: Point cloud processing.
- **PyQt5**: Desktop dashboard.

## Core Architecture
- **Scanning**: RPLiDAR + Gantry position → 3D Point Cloud (`.ply`).
- **Vision**: Camera frames + Gantry position → Stitched Panorama.
- **Perception**: MediaPipe detects back/shoulder landmarks on panorama.
- **Path Gen**: 2D zones on panorama → 3D waypoints for robot arm.

## Building and Running

### Build Commands
```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select <package_name>

# Source workspace
source install/setup.bash
```

### Key Launch Commands
```bash
# Full bringup (hardware + scanner + perception)
tmuxinator start -p ~/Documents/.tmuxinator.yml

# Hardware bridge (LiDAR + ESP32)
ros2 launch medical_scanner_pkg scanner_bridge.launch.py

# Fusion nodes (Assembler + Colorizer)
ros2 launch lidar_camera_fusion fusion.launch.py
```

## Development Conventions

### Python Guidelines
- Follow PEP8. Use `flake8` and `pep257` (run via `colcon test`).
- ROS 2 Nodes: Use `rclpy` with standard `Node` inheritance.
- Prefer `QObject` with `pyqtSignal` for ROS-to-GUI communication (see `medical_scanner_pkg`).

### C++ Guidelines
- C++17 standard.
- Prefer `ament_target_dependencies` in `CMakeLists.txt`.
- Use `tf2_ros::Buffer` with `spin_thread=true` for time-sensitive lookups.

### Communication
- **Telemetry**: `/current_position` (Float32, mm) is the ground truth for gantry location.
- **Images**: `/camera/image_raw` (SensorDataQoS).
- **Clouds**: `/scanner/assembled_cloud` and `/scanner/colored_cloud`.

## Patient Data Handling
- **Location**: `~/.massage_machine/patients/` (Private, do not commit).
- **Structure**: `patient_id/sessions/session_id/` contains `.ply`, `.png`, and `.json` configs.
