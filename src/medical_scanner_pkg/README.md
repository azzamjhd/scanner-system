# medical_scanner_pkg

ROS 2 package for 3D scanning with LiDAR + encoder fusion, with a PyQt5 control GUI, firmware motion control integration, RViz visualization, and configurable camera feed.

## What is included

- `scanner_3d_node`  
  Builds a 3D point cloud from `LaserScan` + encoder position, publishes live `PointCloud2`, and saves scans to `.ply`.
- `scanner_gui`  
  Desktop control panel (PyQt5) for scan control, firmware-compatible motion commands, parameter sync, RViz launch, and live camera preview.
- `config/scanner_rviz.rviz`  
  RViz profile for viewing `/scanner/pointcloud`.
- `urdf/scanner.urdf`  
  Scanner model asset.

---

## 1. Prerequisites

1. ROS 2 workspace (colcon) and sourced ROS environment
2. LiDAR publishing `sensor_msgs/msg/LaserScan` (default topic: `/scan`)
3. Position feedback publishing `std_msgs/msg/Float32` (default topic: `/current_position`)  
   or run in simulation mode (`simulate_encoder:=true`)

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-sensor-msgs-py \
  ros-$ROS_DISTRO-v4l2-camera \
  python3-pyqt5 \
  python3-numpy
```

---

## 2. Build

From workspace root (example `~/ros2_ws`):

```bash
cd ~/ros2_ws
colcon build --packages-select medical_scanner_pkg
source install/setup.bash
```

---

## 3. Run scanner node

### Basic

```bash
ros2 run medical_scanner_pkg scanner_3d_node
```

### Typical custom run

```bash
ros2 run medical_scanner_pkg scanner_3d_node --ros-args \
  -p simulate_encoder:=false \
  -p angle_min:=0.0 \
  -p angle_max:=360.0 \
  -p range_max:=12.0 \
  -p ticks_per_unit:=100.0 \
  -p axis:=1 \
  -p output_dir:=/tmp/scans \
  -p lidar_topic:=/scan \
  -p position_topic:=/current_position
```

### Simulation mode (no encoder hardware)

```bash
ros2 run medical_scanner_pkg scanner_3d_node --ros-args -p simulate_encoder:=true
```

---

## 4. Control scan lifecycle

Start scan:

```bash
ros2 service call /start_scan std_srvs/srv/Trigger
```

Stop and save scan:

```bash
ros2 service call /stop_scan std_srvs/srv/Trigger
```

Clear RViz visualization only (does not delete saved files):

```bash
ros2 service call /clear_visualization std_srvs/srv/Trigger
```

Saved files are written to `output_dir` as:

```text
scan_YYYYMMDD_HHMMSS.ply
```

---

## 5. Firmware control + GUI

Start GUI:

```bash
ros2 run medical_scanner_pkg scanner_gui
```

Set initial camera topic from CLI (optional):

```bash
ros2 run medical_scanner_pkg scanner_gui --ros-args -p image_topic:=/camera/image_raw
```

GUI capabilities:
- Start/stop scanning via ROS services
- Configure scan traversal `Start Pos` and `End Pos` in the scan panel
- On **Start Scan**, auto-move to the endpoint farthest from current position (one-way per press)
- Send firmware motion commands on `/position`, `/speed`, `/acceleration` (`std_msgs/msg/Float32`)
- Keep legacy publish on `/scanner/cmd` (`geometry_msgs/msg/Vector3`) for compatibility
- Sync and set scanner parameters from/to `scanner_3d_node`
- Launch/close RViz using packaged config
- Show live camera feed in a dedicated **Camera Feed** panel directly on the **right side of Scan Control**

Optional stitcher integration (best-effort service trigger from GUI):

```bash
ros2 run medical_scanner_pkg scanner_gui --ros-args \
  -p enable_stitcher_integration:=true \
  -p stitcher_start_service:=/stitcher/start_session \
  -p stitcher_stop_service:=/stitcher/stop_session
```

When enabled:
- successful **Start Scan** triggers `/stitcher/start_session`
- successful **Stop Scan** triggers `/stitcher/stop_session`

Start micro-ROS agent (for ESP32 firmware bridge):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Firmware command topics used by GUI:
- `/position` (target mm)
- `/speed` (mm/s)
- `/acceleration` (mm/s²)

---

## 6. Interfaces reference

### `scanner_3d_node`

| Interface | Name | Type |
|---|---|---|
| Subscribe | `/scan` (or `lidar_topic`) | `sensor_msgs/msg/LaserScan` |
| Subscribe | `/current_position` (or `position_topic`) | `std_msgs/msg/Float32` |
| Publish | `/scanner/pointcloud` | `sensor_msgs/msg/PointCloud2` |
| Service | `/start_scan` | `std_srvs/srv/Trigger` |
| Service | `/stop_scan` | `std_srvs/srv/Trigger` |
| Service | `/clear_visualization` | `std_srvs/srv/Trigger` |

Parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `angle_min` | `0.0` | Min accepted scan angle (deg) |
| `angle_max` | `90.0` | Max accepted scan angle (deg) |
| `range_max` | `12.0` | Max accepted distance (m) |
| `ticks_per_unit` | `100.0` | Simulation increment scale (used when `simulate_encoder=true`) |
| `axis` | `1` | `0`: rotational Y, `1`: linear Z |
| `simulate_encoder` | `false` | Use synthetic encoder increment |
| `output_dir` | `/tmp` | Output directory for saved scans |
| `lidar_topic` | `/scan` | LaserScan input topic |
| `position_topic` | `/current_position` | Position feedback input topic (`Float32`) |

### `scanner_gui`

| Interface | Name | Type |
|---|---|---|
| Subscribe | `/scanner/pointcloud` | `sensor_msgs/msg/PointCloud2` |
| Subscribe | `/current_position` | `std_msgs/msg/Float32` |
| Subscribe | `/motor_speed` | `std_msgs/msg/Float32` |
| Subscribe | Configurable image topic (default `/camera/image_raw`) | `sensor_msgs/msg/Image` |
| Publish | `/position` | `std_msgs/msg/Float32` |
| Publish | `/speed` | `std_msgs/msg/Float32` |
| Publish | `/acceleration` | `std_msgs/msg/Float32` |
| Publish | `/scanner/cmd` | `geometry_msgs/msg/Vector3` |
| Client | `/start_scan`, `/stop_scan`, `/clear_visualization` | `std_srvs/srv/Trigger` |

---

## 7. Camera feed options

### Raw USB camera (v4l2)

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -r image_raw:=/camera/image_raw
```

In GUI, keep `Image Topic` as `/camera/image_raw`.

### Processed image pipeline

Set GUI `Image Topic` to whichever processed image topic you expose from your pipeline  
(for example, from a future `massage_perception` image publisher).  
The camera feed is topic-agnostic and can switch at runtime from the GUI.

---

## 8. Start/End auto traversal behavior

`Start Scan` executes this flow:
1. Calls `/start_scan`
2. Reads current position (`/current_position`)
3. Compares distance to configured **Start Pos** and **End Pos**
4. Sends move command to the **farther** endpoint once

---

## 9. RViz visualization

You can launch RViz from the GUI button, or manually:

```bash
rviz2 -d $(ros2 pkg prefix medical_scanner_pkg)/share/medical_scanner_pkg/config/scanner_rviz.rviz
```

Point cloud topic:
- `/scanner/pointcloud`

Expected fixed frame:
- `scanner_frame`

---

## 10. Quick workflow

1. Start LiDAR and `/current_position` publishers (or use simulation mode)
2. Run `scanner_3d_node`
3. Start micro-ROS agent for firmware-connected motion control
4. Start camera stream (`v4l2_camera`) or your processed image topic
5. Run `scanner_gui`
6. Call `/start_scan`
7. Move gantry from GUI (mm-based controls)
8. Call `/stop_scan` to save `.ply`
9. Inspect output in CloudCompare / MeshLab / RViz

---

## 11. Troubleshooting

### No points published
- Confirm `/scan` is active: `ros2 topic hz /scan`
- Confirm scan is started: `ros2 service call /start_scan std_srvs/srv/Trigger`
- Check angle/range filters are not too restrictive

### Stop scan says no points collected
- Verify LiDAR is producing valid ranges
- Ensure `/current_position` updates are present (or enable simulation mode)

### GUI does not start
- Install PyQt5: `sudo apt install python3-pyqt5`
- Ensure X/desktop session is available

### Camera panel shows no image
- Verify topic exists: `ros2 topic list | grep image`
- Confirm GUI `Image Topic` matches the published topic exactly
- For v4l2 input, ensure `v4l2_camera_node` is running and remapped to `/camera/image_raw`

### Firmware move commands have no effect
- Ensure micro-ROS agent is running and connected to ESP32
- Check command topics are active: `ros2 topic list | grep -E "position|speed|acceleration"`
- Verify firmware node (`/gantry_controller`) is present

### RViz not found
- Install: `sudo apt install ros-$ROS_DISTRO-rviz2`

### Package executable not found

```bash
cd ~/ros2_ws
colcon build --packages-select medical_scanner_pkg
source install/setup.bash
ros2 pkg executables medical_scanner_pkg
```

Expected executables:
- `medical_scanner_pkg scanner_3d_node`
- `medical_scanner_pkg scanner_gui`
