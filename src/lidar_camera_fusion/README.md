# lidar_camera_fusion

A ROS 2 Jazzy C++ package that fuses an RPLiDAR A1 and a USB camera into a
custom RGB-D sensor. It provides two nodes and a complete single-command
system bringup.

| Node | Role |
|---|---|
| `scan_assembler_node` | Accumulates 2D laser rings into a growing 3D point cloud using `laser_geometry` + TF2. Built-in gantry bridge converts `/current_position` (mm) → `/joint_states` (m) so `robot_state_publisher` keeps the TF tree live. |
| `cloud_colorizer_node` | Colorizes each per-scan cloud against its time-synchronized camera frame and accumulates colored points. |

---

## System overview

```
ESP32 firmware
  └── /current_position (Float32, mm)
        │
        └── scan_assembler_node  ← built-in bridge: mm÷1000 → /joint_states
                                          │
                              robot_state_publisher
                                          │
                                    TF2 tree:
                              base_link → gantry_link → lidar_link
                              gantry_link → camera_link → camera_optical_frame

RPLiDAR A1
  └── /scan  (LaserScan, frame_id="lidar_link")
        │
        └── scan_assembler_node  ──►  /scanner/assembled_cloud  (PointCloud2, base_link)
                        │
                        └──────────►  /scanner/scan_cloud       (PointCloud2, base_link)
                                                │
v4l2_camera                                     │
  ├── /image_raw  (Image)  ──────────────────►  cloud_colorizer_node
  └── /camera_info (CameraInfo) ─────────────►
                                                │
                                                └──►  /scanner/colored_cloud  (PointCloud2, base_link)
```

### TF frame tree

```
base_link  ──[prismatic gantry_joint, X-axis]──►  gantry_link
                                                        │
                                          ┌─────────────┴─────────────┐
                                  [fixed, −90° pitch]          [fixed offset]
                                          │                           │
                                     lidar_link                  camera_link
                                     (frame_id of /scan)              │
                                                            [fixed optical rotation]
                                                                       │
                                                           camera_optical_frame
```

The `base_link → gantry_link` edge is driven by `/joint_states`, which
`scan_assembler_node` publishes whenever `/current_position` arrives from the
ESP32 firmware. As the gantry moves, each laser ring is stamped with the
correct 3D position in `base_link` space — this is what builds the 3D shape.

> **No separate `gantry_bridge_node` is needed.** The mm → m conversion is
> a built-in 7-line callback inside `scan_assembler_node`.

---

## Nodes

### `scan_assembler_node`

Converts incoming `LaserScan` messages into a growing 3D `PointCloud2`.

| | |
|---|---|
| **Subscribes** | `/scan` — `sensor_msgs/LaserScan` (SensorDataQoS) |
| **Subscribes** | `/current_position` — `std_msgs/Float32` (mm) |
| **Publishes** | `/scanner/assembled_cloud` — `sensor_msgs/PointCloud2` (SensorDataQoS, timer-driven) |
| **Publishes** | `/scanner/scan_cloud` — `sensor_msgs/PointCloud2` (SensorDataQoS, per scan) |
| **Publishes** | `/joint_states` — `sensor_msgs/JointState` (on every `/current_position` message) |

#### How it works

1. **Guard** — `is_scanning_` (`std::atomic<bool>`) must be `true`. If not,
   every incoming scan is dropped immediately (lock-free check).
2. **Project** — `laser_geometry::projectLaser()` converts the 2D polar ring
   to Cartesian XYZ in the scanner's own frame. No TF needed here.
3. **Transform** — `tf_buffer_->lookupTransform()` fetches the exact-timestamp
   transform from `lidar_link` → `base_link` with a 100 ms wait to absorb
   `robot_state_publisher` latency. `tf2::doTransform()` moves the ring into
   `base_link`.
4. **Accumulate** — Points are written into a pre-allocated circular ring
   buffer of `max_points` entries. When full, the oldest points are silently
   overwritten — no heap allocation after startup.
5. **Publish** — A wall timer serialises the buffer into a `PointCloud2` and
   publishes at `publish_rate` Hz (decoupled from the ~10 Hz scan rate).
6. **Gantry bridge** — Every `/current_position` message triggers
   `position_callback`, which divides by 1000 and publishes to `/joint_states`
   so `robot_state_publisher` animates the `gantry_joint` TF edge in real time.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_frame` | string | `base_link` | TF frame to accumulate the cloud in |
| `scan_topic` | string | `/scan` | LaserScan input topic |
| `joint_name` | string | `gantry_joint` | Joint name in the URDF driven by `/joint_states` |
| `output_dir` | string | `.` | Directory where `.pcd` files are saved on `/scanner/stop` |
| `max_points` | int | `500000` | Circular buffer capacity (≈ 18 MB for XYZ float32) |
| `publish_rate` | double | `2.0` | Assembled-cloud publish rate in Hz |

#### Services

| Service | Type | Effect |
|---|---|---|
| `/scanner/start` | `std_srvs/Trigger` | **Clears the buffer** then opens the accumulation gate (`is_scanning_ = true`). Every new session starts with an empty cloud. |
| `/scanner/stop` | `std_srvs/Trigger` | Closes the gate (`is_scanning_ = false`) then **saves the accumulated cloud** to `<output_dir>/scan_YYYYMMDD_HHMMSS.pcd` as a binary PCD file. |
| `/scanner/clear_cloud` | `std_srvs/Trigger` | Resets the buffer without changing the scanning gate state. |

---

### `cloud_colorizer_node`

Assigns RGB colours to the 3D cloud using a pinhole camera model.

| | |
|---|---|
| **Subscribes (synced)** | `/scanner/scan_cloud` — `sensor_msgs/PointCloud2` |
| | `/image_raw` — `sensor_msgs/Image` |
| | `/camera_info` — `sensor_msgs/CameraInfo` |
| **Publishes** | `/scanner/colored_cloud` — `sensor_msgs/PointCloud2` (XYZRGB, `base_link`) |

#### How it works

1. **`ApproximateTime` sync** — waits for one message from each of the three
   topics whose timestamps are within `approx_time_slop` seconds of each other.
2. **TF lookup** — `lookupTransform(camera_optical_frame, cloud.frame_id,
   image.stamp)` — transforms the per-scan cloud into camera space in one batch
   call.
3. **Pinhole projection** — for each point in the optical frame:
   `u = fx·X/Z + cx`, `v = fy·Y/Z + cy`. Points with `Z ≤ 0` or outside the
   image bounds are discarded.
4. **Colour sampling** — samples BGR from `cv::Mat` at `(v, u)`, converts to
   RGB and stores in `pcl::PointXYZRGB`.
5. **Output** — XYZ coordinates are the **original `base_link` values**, not
   the optical-frame projection. The coloured cloud overlays exactly with
   `/scanner/assembled_cloud` in RViz.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `optical_frame` | string | `camera_optical_frame` | Camera optical TF frame for projection |
| `queue_size` | int | `10` | `ApproximateTime` synchroniser queue depth |
| `approx_time_slop` | double | `0.1` | Max timestamp gap (s) to consider messages synchronised |

---

## Topic and service summary

| Topic / Service | Direction | Type | Notes |
|---|---|---|---|
| `/scan` | → node | `LaserScan` | rplidar_ros, `frame_id=lidar_link` |
| `/current_position` | → node | `Float32` | ESP32 firmware, mm |
| `/joint_states` | node → | `JointState` | Feeds `robot_state_publisher` |
| `/scanner/assembled_cloud` | node → | `PointCloud2` | XYZ, `base_link`, 2 Hz |
| `/scanner/scan_cloud` | node → | `PointCloud2` | XYZ, `base_link`, per scan |
| `/scanner/colored_cloud` | node → | `PointCloud2` | XYZRGB, `base_link` |
| `/image_raw` | → node | `Image` | v4l2_camera |
| `/camera_info` | → node | `CameraInfo` | v4l2_camera |
| `/scanner/start` | service | `Trigger` | Clear buffer + start scan |
| `/scanner/stop` | service | `Trigger` | Stop scan + save `.pcd` |
| `/scanner/clear_cloud` | service | `Trigger` | Clear buffer only |

---

## Dependencies

### ROS 2 packages

| Package | Purpose |
|---|---|
| `rclcpp` | C++ client library |
| `sensor_msgs`, `std_msgs`, `geometry_msgs`, `std_srvs` | Message and service types |
| `laser_geometry` | `LaserScan` → `PointCloud2` projection |
| `tf2`, `tf2_ros`, `tf2_sensor_msgs`, `tf2_geometry_msgs`, `tf2_eigen` | Transform lookups and batch cloud transforms |
| `pcl_conversions` | PCL ↔ ROS 2 bridge |
| `cv_bridge` | ROS 2 `Image` ↔ OpenCV `cv::Mat` |
| `message_filters` | `ApproximateTime` synchroniser |

### System libraries

| Library | Purpose |
|---|---|
| PCL (`libpcl-dev`, components `common` + `io`) | `pcl::PointXYZRGB`, `pcl::io::savePCDFileBinary` |
| OpenCV | Image decode and pixel sampling |

### Runtime prerequisites

| Process | Why required |
|---|---|
| `micro_ros_agent` (serial, `/dev/ttyUSB0`, 115200) | Bridges ESP32 firmware to ROS 2; provides `/current_position` |
| `rplidar_ros rplidar_a1_launch.py` | Publishes `/scan` |
| `robot_state_publisher` (with `urdf/scanner_bridge.urdf`) | Broadcasts TF tree from URDF |
| `v4l2_camera_node` | Publishes `/image_raw` and `/camera_info` (required only by `cloud_colorizer_node`) |

---

## Installation

```bash
sudo apt install -y \
  ros-jazzy-laser-geometry \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-eigen \
  ros-jazzy-pcl-conversions \
  ros-jazzy-cv-bridge \
  ros-jazzy-message-filters \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rplidar-ros \
  ros-jazzy-v4l2-camera \
  libpcl-dev
```

---

## Build

```bash
cd ~/Documents/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidar_camera_fusion --symlink-install
source install/setup.bash
```

---

## Usage

### One-command bringup (recommended)

```bash
ros2 launch lidar_camera_fusion full_system.launch.py
```

All hardware defaults match the physical setup out of the box. Override
anything on the command line:

```bash
ros2 launch lidar_camera_fusion full_system.launch.py \
    mcu_port:=/dev/ttyUSB0             \
    lidar_port:=/dev/ttyUSB1           \
    camera_device:=/dev/video0         \
    output_dir:=/home/azzam/scans      \
    max_points:=1000000                \
    scan_mode:=Boost
```

#### `full_system.launch.py` arguments

| Argument | Default | Description |
|---|---|---|
| `mcu_port` | `/dev/ttyUSB0` | Serial port for micro-ROS / ESP32 |
| `lidar_port` | `/dev/ttyUSB1` | Serial port for RPLiDAR A1 |
| `camera_device` | `/dev/video0` | V4L2 device node |
| `lidar_frame_id` | `lidar_link` | `frame_id` stamped on `/scan` |
| `scan_mode` | `Standard` | RPLiDAR scan mode (`Standard` / `Express` / `Boost`) |
| `camera_info_url` | `file:///home/azzam/Documents/webcam_calibration.yaml` | Camera calibration file |
| `camera_frame_id` | `camera_optical_frame` | TF frame stamped on camera images |
| `output_dir` | `~/ros2_scans` | Directory for saved `.pcd` files |
| `max_points` | `500000` | Circular buffer capacity |
| `publish_rate` | `2.0` | Cloud publish rate in Hz |
| `scan_source_frame` | `lidar_link` | Passed to `fusion.launch.py` |

> **Camera V4L2 controls** (`focus_absolute`, `exposure_time_absolute`) are
> **not** set at node startup — applying them from a params file before the
> camera begins streaming corrupts `VIDIOC_REQBUFS` on UVC cameras like the
> C922. Set them after the node is running:
> ```bash
> ros2 param set /v4l2_camera_node focus_absolute 0
> ros2 param set /v4l2_camera_node exposure_time_absolute 512
> ```

---

### Manual bringup (individual terminals)

If you need to start components separately — e.g. for debugging or when
`full_system.launch.py` is too coarse-grained:

**Terminal 1 — micro-ROS agent**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Terminal 2 — RPLiDAR A1**
```bash
source /opt/ros/jazzy/setup.bash && source ~/Documents/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/ttyUSB1 \
    frame_id:=lidar_link      \
    scan_mode:=Standard
```

**Terminal 3 — TF tree (URDF lives in this package)**
```bash
ros2 launch lidar_camera_fusion scanner_bridge.launch.py
```

**Terminal 4 — USB camera**
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p camera_info_url:="file:///home/azzam/Documents/webcam_calibration.yaml" \
    -p camera_frame_id:="camera_optical_frame"
```

**Terminal 5 — Fusion nodes**
```bash
ros2 launch lidar_camera_fusion fusion.launch.py
```

---

### Scan workflow

```bash
# 1. Start a new scan session (clears buffer, opens accumulation gate)
ros2 service call /scanner/start std_srvs/srv/Trigger

# 2. Move the gantry to sweep the scan volume
ros2 topic pub /position std_msgs/msg/Float32 "{data: 100.0}" -1

# 3. Stop — closes gate and saves  ~/ros2_scans/scan_YYYYMMDD_HHMMSS.pcd
ros2 service call /scanner/stop std_srvs/srv/Trigger

# Optional: clear without stopping the gate
ros2 service call /scanner/clear_cloud std_srvs/srv/Trigger
```

---

### Visualise in RViz2

```bash
rviz2
```

| Display | Topic | Setting |
|---|---|---|
| PointCloud2 | `/scanner/assembled_cloud` | Color by Z or Intensity |
| PointCloud2 | `/scanner/colored_cloud` | Color Transformer → **RGB8** |
| RobotModel | — | — |
| TF | — | — |

Set **Fixed Frame** to `base_link`.

---

### Useful CLI checks

```bash
# Verify the full TF chain is live
ros2 run tf2_tools view_frames
# Expected: base_link → gantry_link → lidar_link
#           gantry_link → camera_link → camera_optical_frame

# Spot-check a transform
ros2 run tf2_ros tf2_echo base_link lidar_link

# Confirm /joint_states is being published by scan_assembler_node
ros2 topic hz /joint_states             # mirrors /current_position rate

# Check assembled cloud rate
ros2 topic hz /scanner/assembled_cloud  # ~2 Hz

# Check colored cloud (requires camera to be running)
ros2 topic hz /scanner/colored_cloud
```

---

## Troubleshooting

### `"base_link" passed to lookupTransform does not exist`

**Cause:** `robot_state_publisher` is not running.

**Fix:** Run `scanner_bridge.launch.py` (now in **this** package, not
`medical_scanner_pkg`):
```bash
ros2 launch lidar_camera_fusion scanner_bridge.launch.py
```
Or use `full_system.launch.py` which starts it automatically.

---

### `TF transform failed: "lidar_link" does not exist`

**Cause:** `robot_state_publisher` started but has not yet received a
`/joint_states` message, so the dynamic `gantry_joint` edge is not yet in the
buffer. This is transient at startup.

**Fix:** `scan_assembler_node` waits up to 100 ms per scan for TF data and
logs a throttled warning. The warning disappears within 1–2 seconds once
`/current_position` messages begin arriving from the ESP32.

---

### RPLiDAR exits immediately with `scan mode 'Sensitivity' is not supported`

**Cause:** `rplidar_a1_launch.py` defaults to `scan_mode:=Sensitivity`, which
only exists on the S-series. The A1 supports **Standard**, **Express**, and
**Boost** only.

**Fix:** Always pass `scan_mode:=Standard` (or `Boost`) explicitly.
`full_system.launch.py` already sets `Standard` as its default.

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/ttyUSB1 \
    frame_id:=lidar_link      \
    scan_mode:=Standard
```

---

### Camera: `Failed mapping device memory`

**Cause:** Applying V4L2 hardware controls (`focus_absolute`,
`exposure_time_absolute`) via a ROS 2 params file **before** the camera begins
streaming corrupts the `VIDIOC_REQBUFS` (MMAP) call. This affects UVC cameras
like the Logitech C922, whose focus and exposure controls are UVC extension
controls — v4l2_camera cannot enumerate them at startup ("Available controls:"
shows empty), so it never declares those ROS 2 parameters; when the params
file tries to apply them, the camera initialisation fails.

**Fix:** Do **not** pass these controls at node startup. Set them after the
node is running:
```bash
ros2 param set /v4l2_camera_node focus_absolute 0
ros2 param set /v4l2_camera_node exposure_time_absolute 512
```
`full_system.launch.py` already omits these from startup params.

---

### `cloud_colorizer_node` callback never fires

**Cause:** `ApproximateTime` cannot find matching messages across all three
topics within `approx_time_slop`.

**Checklist:**
```bash
ros2 topic hz /scanner/assembled_cloud   # ~2 Hz
ros2 topic hz /image_raw                 # ~30 Hz
ros2 topic hz /camera_info               # ~30 Hz
```
If all are publishing, widen the sync window:
```bash
ros2 launch lidar_camera_fusion fusion.launch.py approx_time_slop:=0.5
```

---

### `Lookup would require extrapolation into the future`

**Cause:** The `LaserScan` stamp is ahead of the latest TF data.
`robot_state_publisher` has a 7–80 ms latency between an encoder change and
the `/tf` update appearing on the network.

**Status: handled.** The node uses `projectLaser()` (no TF, always instant)
followed by `lookupTransform(..., 100 ms timeout)` and `tf2::doTransform()`.
The `TransformListener` runs on a dedicated thread (`spin_thread=true`) so it
keeps receiving `/tf` while `scan_callback` blocks in the lookup — eliminating
the executor-deadlock root cause.

If the warning persists, your system TF latency exceeds 100 ms. Increase the
timeout at `rclcpp::Duration(0, 100'000'000)` in `scan_assembler_node.cpp`
(value is in nanoseconds).

---

### Coloured cloud has very few points

**Cause:** Most scan points project outside the camera's FOV, or the
`camera_to_optical` rotation in `scanner_bridge.urdf` is wrong.

**Check:** In RViz, display both `/scanner/assembled_cloud` and the camera
image. If the cloud visually falls inside the frustum but points are still
discarded, verify the `rpy` of the `camera_to_optical` joint in
`urdf/scanner_bridge.urdf`.

---

## Package layout

```
lidar_camera_fusion/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── scanner_bridge.urdf          Robot URDF — defines the full TF tree
├── include/lidar_camera_fusion/
│   ├── scan_assembler_node.hpp
│   └── cloud_colorizer_node.hpp
├── src/
│   ├── scan_assembler_node.cpp      LaserScan → accumulated PointCloud2
│   │                                 + gantry bridge (/current_position → /joint_states)
│   │                                 + /scanner/start|stop|clear_cloud services
│   │                                 + binary PCD save on /scanner/stop
│   └── cloud_colorizer_node.cpp     assembled cloud + image → XYZRGB cloud
└── launch/
    ├── full_system.launch.py        Single-command bringup of all hardware + nodes
    ├── scanner_bridge.launch.py     robot_state_publisher only (TF tree from URDF)
    └── fusion.launch.py             scan_assembler + cloud_colorizer + static TF
```
