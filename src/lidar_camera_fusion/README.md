# lidar_camera_fusion

A ROS 2 Jazzy C++ package that combines an RPLiDAR A1 and a USB camera into a custom RGB-D sensor. It provides two nodes:

1. **`scan_assembler_node`** — projects 2D laser sweeps into an accumulating 3D point cloud using the official `laser_geometry` + TF2 pipeline.
2. **`cloud_colorizer_node`** — colorizes the 3D cloud by pinhole-projecting it onto a synchronized camera frame.

---

## System Overview

```
ESP32 firmware
  └── /current_position (Float32, mm)
        └── gantry_bridge_node  ──►  /joint_states
                                          │
                              robot_state_publisher
                                          │
                                    TF2 tree published:
                              base_link → gantry_link → lidar_link
                              gantry_link → camera_link → camera_optical_frame

RPLiDAR A1
  └── /scan (LaserScan, frame_id="laser")
        │
        │   (static TF: lidar_link → laser, published by fusion.launch.py)
        │
        └── scan_assembler_node  ──►  /scanner/assembled_cloud (PointCloud2, base_link)
                                                │
v4l2_camera                                     │
  ├── /image_raw  (Image)  ──────────────────►  │
  └── /camera_info (CameraInfo) ─────────────►  cloud_colorizer_node
                                                │
                                                └──► /scanner/colored_cloud (PointCloud2, base_link)
```

### TF frame tree

```
base_link  ──[prismatic gantry_joint, X-axis]──►  gantry_link
                                                        │
                                           ┌────────────┴────────────┐
                              [fixed -90° pitch]               [fixed offset]
                                           │                         │
                                      lidar_link               camera_link
                                           │                         │
                              [static identity TF]         [fixed optical rotation]
                                           │                         │
                                         laser               camera_optical_frame
```

The `base_link → gantry_link` edge is dynamic (driven by `/joint_states` from `gantry_bridge_node`), so as the gantry moves, every laser scan is automatically projected into the correct 3D position.

---

## Nodes

### `scan_assembler_node`

Converts incoming 2D `LaserScan` messages into a growing 3D `PointCloud2` by using `laser_geometry` and the live TF2 tree.

| | |
|---|---|
| **Subscribes** | `/scan` — `sensor_msgs/msg/LaserScan` (SensorDataQoS) |
| **Publishes** | `/scanner/assembled_cloud` — `sensor_msgs/msg/PointCloud2` (SensorDataQoS, timer-driven) |
| **Service** | `/scanner/clear_cloud` — `std_srvs/srv/Trigger` |

**How it works:**

For each incoming scan, `laser_geometry::LaserProjection::transformLaserScanToPointCloud()` is called with the TF2 buffer. It looks up the transform from the scan's `frame_id` (`laser`) to `target_frame` (`base_link`) at the scan's exact timestamp, accounting for the gantry's position at that moment. The resulting 3D points are written into a **pre-allocated circular buffer** of `max_points` entries. When the buffer is full, the oldest points are overwritten automatically — no dynamic memory allocation occurs after startup.

A wall timer publishes the full accumulated cloud at `publish_rate` Hz.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_frame` | string | `base_link` | TF frame to accumulate the cloud in |
| `scan_topic` | string | `/scan` | LaserScan input topic |
| `max_points` | int | `500000` | Circular buffer capacity (≈18 MB) |
| `publish_rate` | double | `2.0` | Publish rate in Hz |

#### Services

| Service | Type | Effect |
|---|---|---|
| `/scanner/clear_cloud` | `std_srvs/srv/Trigger` | Resets the accumulation buffer |

---

### `cloud_colorizer_node`

Projects the 3D point cloud onto a synchronized camera image to assign RGB colors to each point.

| | |
|---|---|
| **Subscribes (synced)** | `/scanner/assembled_cloud` — `PointCloud2` |
| | `/image_raw` — `sensor_msgs/msg/Image` |
| | `/camera_info` — `sensor_msgs/msg/CameraInfo` |
| **Publishes** | `/scanner/colored_cloud` — `sensor_msgs/msg/PointCloud2` (SensorDataQoS) |

**How it works:**

1. **Time synchronization** — `message_filters::ApproximateTime` aligns the three streams within `approx_time_slop` seconds.
2. **TF lookup** — transforms the entire cloud from `base_link` to `camera_optical_frame` at the image timestamp via `tf2::doTransform` (Eigen batch transform, no per-point overhead).
3. **Pinhole projection** — for each point in the optical frame:
   - Computes pixel `(u, v)` using the intrinsics from `CameraInfo.k`:
     `u = fx·X/Z + cx`,  `v = fy·Y/Z + cy`
   - Discards points with `Z ≤ 0` (behind the camera) or out-of-image-bounds.
4. **Color sampling** — samples BGR from the `cv::Mat` at `(v, u)` and stores it in a `pcl::PointXYZRGB`.
5. **Output frame** — the XYZ coordinates in the colored output are in the **original `base_link` frame** (not the optical frame), so `/scanner/colored_cloud` overlays exactly with `/scanner/assembled_cloud` in RViz.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `optical_frame` | string | `camera_optical_frame` | Camera optical TF frame for projection |
| `queue_size` | int | `10` | ApproximateTime synchronizer queue depth |
| `approx_time_slop` | double | `0.1` | Maximum time gap in seconds to consider messages synchronized |

---

## Published Topics Summary

| Topic | Type | Frame | Publisher | Consumers |
|---|---|---|---|---|
| `/scanner/assembled_cloud` | `PointCloud2` | `base_link` | `scan_assembler_node` | `cloud_colorizer_node`, RViz |
| `/scanner/colored_cloud` | `PointCloud2` | `base_link` | `cloud_colorizer_node` | RViz |

---

## Dependencies

### ROS 2 packages
| Package | Purpose |
|---|---|
| `rclcpp` | C++ client library |
| `sensor_msgs`, `std_msgs`, `geometry_msgs`, `std_srvs` | Message/service types |
| `laser_geometry` | `LaserScan` → `PointCloud2` projection |
| `tf2`, `tf2_ros`, `tf2_sensor_msgs`, `tf2_geometry_msgs`, `tf2_eigen` | Transform tree lookups and batch cloud transforms |
| `pcl_conversions` | PCL ↔ ROS 2 message bridge |
| `cv_bridge` | ROS 2 `Image` ↔ OpenCV `cv::Mat` |
| `message_filters` | `ApproximateTime` synchronizer |

### System libraries
| Library | Purpose |
|---|---|
| PCL 1.14 (`libpcl-all-dev`) | `pcl::PointXYZRGB` and `pcl::toROSMsg` |
| OpenCV | Image decoding and pixel sampling |

### Runtime prerequisites (must already be running)
| Node | Launched by | Why required |
|---|---|---|
| `robot_state_publisher` | `scanner_bridge.launch.py` | Broadcasts the TF tree from the URDF (`base_link`, `gantry_link`, `lidar_link`, `camera_optical_frame`) |
| `gantry_bridge_node` | `medical_scanner_pkg` | Publishes `/joint_states` so the gantry_joint TF edge tracks the encoder position |
| `rplidar_node` | `rplidar_ros` | Publishes `/scan` |
| `v4l2_camera_node` | `v4l2_camera` | Publishes `/image_raw` and `/camera_info` |

---

## Installation

```bash
# Install ROS 2 package dependencies
sudo apt install -y \
  ros-jazzy-laser-geometry \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-eigen \
  ros-jazzy-pcl-conversions \
  ros-jazzy-cv-bridge \
  ros-jazzy-message-filters \
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

### Full system bringup (recommended order)

**Terminal 1 — URDF + TF tree (required first)**
```bash
source /opt/ros/jazzy/setup.bash && source ~/Documents/ros2_ws/install/setup.bash
ros2 launch medical_scanner_pkg scanner_bridge.launch.py
```

**Terminal 2 — Encoder → joint states bridge**
```bash
ros2 run medical_scanner_pkg gantry_bridge_node
```

**Terminal 3 — RPLiDAR A1**
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

**Terminal 4 — USB camera**
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r image_raw:=/image_raw \
  -r camera_info:=/camera_info
```

**Terminal 5 — Fusion nodes**
```bash
ros2 launch lidar_camera_fusion fusion.launch.py
```

### Launch arguments

```bash
# Change accumulation frame (unusual)
ros2 launch lidar_camera_fusion fusion.launch.py target_frame:=base_link

# Increase buffer for longer scans
ros2 launch lidar_camera_fusion fusion.launch.py max_points:=1000000

# Slower publish rate to reduce bandwidth
ros2 launch lidar_camera_fusion fusion.launch.py publish_rate:=1.0

# If rplidar is configured to publish in lidar_link frame directly
ros2 launch lidar_camera_fusion fusion.launch.py scan_source_frame:=lidar_link

# Wider time sync window (for slow cameras)
ros2 launch lidar_camera_fusion fusion.launch.py approx_time_slop:=0.2
```

### Visualize in RViz2

```bash
rviz2
```

Recommended RViz displays:

| Display | Topic | Setting |
|---|---|---|
| PointCloud2 | `/scanner/assembled_cloud` | Color by Z or intensity |
| PointCloud2 | `/scanner/colored_cloud` | Color Transformer → **RGB8** |
| RobotModel | — | Fixed frame: `base_link` |
| TF | — | — |

Set **Fixed Frame** to `base_link`.

### Useful CLI checks

```bash
# Verify TF chain is complete
ros2 run tf2_tools view_frames
# Expected edges: base_link→gantry_link→lidar_link→laser, gantry_link→camera_link→camera_optical_frame

# Spot-check a specific transform
ros2 run tf2_ros tf2_echo base_link laser

# Check assembled cloud is publishing
ros2 topic hz /scanner/assembled_cloud     # should be ~2 Hz
ros2 topic info /scanner/assembled_cloud

# Check colored cloud
ros2 topic hz /scanner/colored_cloud

# Reset the accumulated cloud
ros2 service call /scanner/clear_cloud std_srvs/srv/Trigger
```

---

## Troubleshooting

### `"base_link" passed to lookupTransform argument target_frame does not exist`

**Cause:** `robot_state_publisher` is not running, so the TF tree (`base_link`, `gantry_link`, `lidar_link`, etc.) has never been broadcasted.

**Fix:** Launch `scanner_bridge.launch.py` from `medical_scanner_pkg` **before** or alongside `fusion.launch.py`. It starts `robot_state_publisher` with the URDF that defines all the robot frames.

```bash
# Terminal 1 (run first)
ros2 launch medical_scanner_pkg scanner_bridge.launch.py

# Terminal 2 (then fusion)
ros2 launch lidar_camera_fusion fusion.launch.py
```

---

### `TF transform failed: "laser" passed to lookupTransform argument source_frame does not exist`

**Cause:** The static TF `lidar_link → laser` has not been published yet, or `fusion.launch.py` is not running.

**Fix:** Ensure `fusion.launch.py` is running (it publishes the static TF). If you changed the rplidar `frame_id` parameter away from `laser`, pass the matching value:
```bash
ros2 launch lidar_camera_fusion fusion.launch.py scan_source_frame:=<your_frame_id>
```

---

### `cloud_colorizer_node` callback never fires

**Cause:** `ApproximateTime` cannot find three matching messages within `approx_time_slop`.

**Checklist:**
```bash
ros2 topic hz /scanner/assembled_cloud   # should publish
ros2 topic hz /image_raw                 # should publish
ros2 topic hz /camera_info               # should publish
```
If all three are publishing but the callback still never fires, try increasing the slop:
```bash
ros2 launch lidar_camera_fusion fusion.launch.py approx_time_slop:=0.5
```

---

### Colored cloud has very few points (most discarded)

**Cause:** The camera FOV does not cover much of the laser scan, or the `camera_to_optical` TF rotation in the URDF is incorrect.

**Check:** Visualize the assembled cloud and the camera image simultaneously in RViz. If the cloud extends far outside the camera's view frustum, this is expected. If the cloud appears to be *inside* the FOV but points are still discarded, verify the `camera_to_optical` joint's `rpy` in `scanner_bridge.urdf`.

---

### `Lookup would require extrapolation into the future`

**Full message:**
```
TF transform failed: Lookup would require extrapolation into the future.
Requested time T but the latest data is at time T-Δ,
when looking up transform from frame [lidar_link] to frame [base_link]
```

**Cause:** The LaserScan's `header.stamp` is ahead of the latest TF data in the buffer. `robot_state_publisher` publishes joint-state-driven transforms with a latency of roughly 7–80 ms, so a transform at exactly `T_scan` may not exist yet.

**Status: Fixed in the current code.** Two changes were required:

1. **`projectLaser()` + explicit `lookupTransform`** — replaced `laser_geometry::transformLaserScanToPointCloud` (which has no timeout and fails immediately on any latency) with a two-step approach: `projectLaser()` converts the 2D scan to 3D in the laser's own frame (no TF, always instant), then `lookupTransform` fetches the exact-timestamp transform with a 100 ms wait.

2. **`spin_thread=true` on `TransformListener`** — this is the root fix for the persistent version of this error. With a single-threaded ROS2 executor, if `lookupTransform` blocks waiting for TF data, it occupies the same thread that the `TransformListener` uses to receive `/tf` messages — a deadlock where the lookup waits for data that can never arrive. Passing `spin_thread=true` gives the `TransformListener` its own dedicated thread, so TF messages are processed independently and the 100 ms timeout works correctly.

If you still see this warning it means the TF latency on your system exceeds 100 ms. You can increase the timeout in [scan_assembler_node.cpp](src/lidar_camera_fusion/src/scan_assembler_node.cpp) at the `rclcpp::Duration(0, 100'000'000)` line (value is in nanoseconds).

---

### `ApproximateTime` warning about dropped messages

If `queue_size` is too small relative to the frequency mismatch between the cloud and the camera, messages are dropped. Increase `queue_size`:
```bash
ros2 launch lidar_camera_fusion fusion.launch.py queue_size:=20
```

---

## Package File Reference

```
lidar_camera_fusion/
├── CMakeLists.txt                          Build configuration
├── package.xml                             Package metadata and dependencies
├── README.md                               This file
├── include/
│   └── lidar_camera_fusion/
│       ├── scan_assembler_node.hpp         Node class declaration
│       └── cloud_colorizer_node.hpp        Node class declaration
├── src/
│   ├── scan_assembler_node.cpp             LaserScan → accumulated PointCloud2
│   └── cloud_colorizer_node.cpp           PointCloud2 + Image → colored PointCloud2
└── launch/
    └── fusion.launch.py                    Launches both nodes + static TF bridge
```
