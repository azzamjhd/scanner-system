# lidar_camera_fusion

A ROS 2 Jazzy package that turns an RPLiDAR A1 + USB camera mounted on a
linear gantry into a colorized 3D body scanner with manual region
segmentation. It spans three C++ nodes and four Python nodes, plus a
single-command bringup and a PyQt5 control GUI.

## Pipeline at a glance

```
ESP32 (micro-ROS)                     RPLiDAR A1            USB camera (v4l2)
  /current_position (mm)                /scan                 /image_raw
        |                                 |                    /camera_info
        v                                 v                         |
 scan_assembler_node  --/joint_states-->  |                         |
   |  (mm/1000 -> robot_state_publisher -> TF tree)                 |
   |                                       |                         |
   |--/scanner/scan_cloud (per ring)-------+                  rectify_node
   |--/scanner/assembled_cloud (2 Hz)                          (image_proc)
   |--/scanner/is_scanning (latched)                                |
                       |                                  /image_rect_color
                       v                                            |
                cloud_colorizer_node  <----------------------------+
                       |  (projects with CameraInfo.P, rectified)
                       v
            /scanner/colored_cloud (XYZRGB, base_link)
                       |
                       +--> scan_image_recorder_node --> stitched_*.png
                       |
                       v
                body_preprocess_node  (RANSAC bed removal + clustering, on stop)
                       |
                       v
            /scanner/body_cloud (latched)
                       |
                       v
            manual_segmentation_node (PyQt polygon GUI)
                       |
                       v
            /scanner/segmented_regions (XYZRGB + label)

scan_cycle_node : orchestrates position -> start -> sweep -> stop
fusion_gui      : PyQt5 control panel (status, sweep, params, logs)
```

### TF frame tree (from `urdf/scanner_bridge.urdf`)

```
base_link --[prismatic gantry_joint, +X]--> gantry_link
                                                |
                          +---------------------+---------------------+
                   [fixed, -90deg pitch]                       [fixed offset]
                          |                                           |
                     lidar_link                                  camera_link
                  (frame_id of /scan)                                 |
                                                            [fixed optical offset]
                                                                      |
                                                          camera_optical_frame
```

`base_link -> gantry_link` is the only dynamic edge. `scan_assembler_node`
divides `/current_position` (mm) by 1000 and publishes `/joint_states`, which
`robot_state_publisher` consumes to animate `gantry_joint`. As the gantry
sweeps, every laser ring is stamped at its correct 3D position in `base_link`.

> No separate `gantry_bridge_node` exists. The mm -> m conversion is a 7-line
> callback inside `scan_assembler_node` (`position_callback`).

---

## Nodes

### `scan_assembler_node` (C++)

LaserScan -> accumulated 3D PointCloud2, plus the gantry bridge.

| | |
|---|---|
| Subscribes | `/scan` LaserScan (SensorDataQoS) |
| Subscribes | `/current_position` Float32 (mm) |
| Publishes | `/scanner/assembled_cloud` PointCloud2 (timer, `publish_rate` Hz) |
| Publishes | `/scanner/scan_cloud` PointCloud2 (per scan) |
| Publishes | `/joint_states` JointState (per `/current_position`) |
| Publishes | `/scanner/is_scanning` Bool (latched, transient_local) |
| Services | `/scanner/start`, `/scanner/stop`, `/scanner/clear_cloud` (Trigger) |

Flow: `is_scanning_` gate (atomic) -> `projectLaser()` (no TF) ->
`lookupTransform(lidar_link -> base_link, scan stamp, 100 ms wait)` ->
`doTransform` -> pre-allocated circular ring buffer (`max_points`) ->
timer serialises to PointCloud2. The `TransformListener` runs on its own
thread (`spin_thread=true`) so the lookup never deadlocks the executor.

`start` clears the buffer, opens the gate, publishes `is_scanning=true`.
`stop` closes the gate, saves `<output_dir>/scan_YYYYMMDD_HHMMSS.pcd`, then
publishes `is_scanning=false`.

Params: `target_frame` (base_link), `scan_topic` (/scan), `joint_name`
(gantry_joint), `output_dir` (.), `max_points` (500000), `publish_rate` (2.0).

---

### `cloud_colorizer_node` (C++)

Colorizes the per-scan cloud against the time-synced **rectified** camera frame
and accumulates the colored result.

| | |
|---|---|
| Subscribes (synced) | `/scanner/scan_cloud` PointCloud2 |
| | `image_topic` Image (default `/image_rect_color`) |
| | `camera_info_topic` CameraInfo (default `/camera_info`) |
| Subscribes | `/scanner/is_scanning` Bool (clear on start, save on stop) |
| Publishes | `/scanner/colored_cloud` PointCloud2 (XYZRGB, base_link) |

**Option A (rectified) projection:** reads `rectify_node`'s
`/image_rect_color` and projects with the rectified projection matrix
`CameraInfo.P` (`fx=P[0], fy=P[5], cx=P[2], cy=P[6]`) — not raw `K`/`D`.
Distortion is already removed from the image, so no `D` is applied here. This
fixes the edge misalignment the old raw-image pinhole path produced.

`ApproximateTime` sync -> TF lookup `cloud.frame_id -> optical_frame` at
`Time(0)` (latest available; gantry is slow so error is sub-mm) ->
`doTransform` -> per-point pinhole projection, discarding `Z<=0` and
out-of-bounds points. Output XYZ stays in `base_link` so it overlays
`/scanner/assembled_cloud` exactly. On stop it saves
`<output_dir>/colored_YYYYMMDD_HHMMSS.pcd`.

Params: `optical_frame` (camera_optical_frame), `image_topic`
(/image_rect_color), `camera_info_topic` (/camera_info), `queue_size` (10),
`approx_time_slop` (0.1), `output_dir` (.).

---

### `body_preprocess_node` (C++)

RANSAC bed-plane removal + largest-cluster extraction. Runs once per scan,
triggered on `/scanner/is_scanning=false`.

| | |
|---|---|
| Subscribes | `input_topic` PointCloud2 (default `/scanner/colored_cloud`) |
| Subscribes | `/scanner/is_scanning` Bool (transient_local) |
| Publishes | `output_topic` PointCloud2 (default `/scanner/body_cloud`, latched RELIABLE + TRANSIENT_LOCAL) |

While scanning it caches the latest colored cloud (which already holds every
accumulated point). On stop: Z passthrough crop `[z_min, z_max]` ->
`SACMODEL_PERPENDICULAR_PLANE` RANSAC constrained to +Z within
`plane_eps_angle_deg` -> remove inliers (the bed) -> optional Euclidean
clustering keeping the largest cluster. Publishes the body cloud latched and
saves `<output_dir>/body_YYYYMMDD_HHMMSS.pcd`.

Params: `z_min` (-0.50), `z_max` (0.80), `plane_distance_thresh` (0.005),
`plane_max_iter` (200), `plane_eps_angle_deg` (15.0), `enable_cluster` (true),
`cluster_tolerance` (0.02), `cluster_min_size` (500), `cluster_max_size`
(2000000), `output_dir` (.).

---

### `scan_image_recorder_node` (Python)

Captures rectified camera frames in step with the colorizer and composites a
single geometry-correct panorama when the scan stops — no feature stitching,
alignment is driven entirely by the TF tree.

Per `/scanner/colored_cloud` while scanning: grab the latest `/image_raw`
frame, `lookup_transform(base_link, camera_frame, Time(0))`, take the camera
X (gantry travel axis), store `(image, x_mm)` — gated by
`min_capture_spacing_mm` of travel so capture density is decoupled from gantry
speed. On stop a background thread sorts frames by X, places each at
`(x_mm - x_min) * pixels_per_mm`, and saves `stitched_YYYYMMDD_HHMMSS.png`.

`pixels_per_mm` source priority: manual `pixels_per_mm` (non-zero) > LiDAR live
distance (`use_lidar_distance`, median of closest `lidar_distance_quantile`
fraction of ranges) > `working_distance_mm` > 2.0 px/mm fallback. Each frame is
optionally rectified with `cv2.remap` (`undistort_images`, default true) to
kill C922 tangential-distortion jitter at strip boundaries.

---

### `body_preprocess_node` see above (C++). `manual_segmentation_node` (Python)

Interactive 2D polygon segmentation over a top-down (Z-projected) view of
`/scanner/body_cloud`, rendered with pyqtgraph + PyQt5.

| | |
|---|---|
| Subscribes | `input_topic` (default `/scanner/body_cloud`, latched) |
| Subscribes | `/scanner/is_scanning` Bool |
| Publishes | `output_topic` (default `/scanner/segmented_regions`, PointCloud2 + `label` uint8) |
| Service | `/scanner/open_segmenter` (Trigger) — open the GUI on demand |

The user draws N labeled polygons; each body point inside a polygon (in X-Y)
gets that label, last-drawn-wins on overlap. Default vocabulary: head,
shoulders, upper_back, lower_back, left_arm, right_arm, left_leg, right_leg,
other (label 0 = unlabeled). On publish it saves
`<output_dir>/regions_YYYYMMDD_HHMMSS.pcd`. With `auto_open_on_stop=true` the
GUI pops automatically when `/scanner/is_scanning` flips false. ROS spins on a
background thread; the Qt loop owns the main thread.

---

### `scan_cycle_node` (Python)

Headless sweep orchestrator for one-button scans (designed for Foxglove Call
Service panels).

| | |
|---|---|
| Subscribes | `/current_position` Float32 (mm) |
| Publishes | `/position` Float32 (mm), `/speed` Float32 (mm/s) |
| Services | `/scanner/start_cycle`, `/scanner/interrupt` (Trigger) |
| Clients | `/scanner/start`, `/scanner/stop` |

`start_cycle` runs: move to `start_mm` -> wait arrival -> `/scanner/start` ->
move to `end_mm` -> wait arrival -> `/scanner/stop`. Params: `start_mm` (0.0),
`end_mm` (300.0), `speed_mm_s` (20.0), `tolerance_mm` (0.5), `timeout_s`
(60.0); read at construction, change with `ros2 param set` between cycles.

---

### `fusion_gui` (Python / PyQt5)

Single-window control panel: node status (2 Hz heartbeat), Start/Stop sweep
control with a state machine, auto-discovered parameters for
`scan_assembler_node` and `cloud_colorizer_node`, and a `/rosout` log viewer.
ROS lives in `ros_worker.py` (ROSBridge + WorkerThread); the GUI talks to it
across threads.

```bash
ros2 run lidar_camera_fusion fusion_gui
```

---

## Topic and service summary

| Topic / Service | Dir | Type | Notes |
|---|---|---|---|
| `/scan` | in | LaserScan | rplidar_ros, `frame_id=lidar_link` |
| `/current_position` | in | Float32 | ESP32, mm |
| `/image_raw` `/camera_info` | in | Image / CameraInfo | v4l2_camera |
| `/image_rect_color` | int | Image | rectify_node output |
| `/joint_states` | out | JointState | feeds robot_state_publisher |
| `/scanner/assembled_cloud` | out | PointCloud2 | XYZ, base_link, `publish_rate` Hz |
| `/scanner/scan_cloud` | out | PointCloud2 | XYZ, base_link, per scan |
| `/scanner/colored_cloud` | out | PointCloud2 | XYZRGB, base_link |
| `/scanner/body_cloud` | out | PointCloud2 | XYZRGB, latched, on stop |
| `/scanner/segmented_regions` | out | PointCloud2 | XYZRGB + label |
| `/scanner/is_scanning` | out | Bool | latched session state |
| `/position` `/speed` | out | Float32 | scan_cycle_node -> ESP32 |
| `/scanner/start` `/stop` `/clear_cloud` | srv | Trigger | scan_assembler_node |
| `/scanner/start_cycle` `/interrupt` | srv | Trigger | scan_cycle_node |
| `/scanner/open_segmenter` | srv | Trigger | manual_segmentation_node |

PCD outputs in `output_dir`: `scan_*.pcd` (raw XYZ), `colored_*.pcd` (XYZRGB),
`body_*.pcd` (segmented body), `regions_*.pcd` (labeled), `stitched_*.png`
(panorama).

---

## Dependencies

ROS 2 (Jazzy): `rclcpp`, `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`,
`std_srvs`, `laser_geometry`, `tf2`/`tf2_ros`/`tf2_sensor_msgs`/
`tf2_geometry_msgs`/`tf2_eigen`/`tf2_ros_py`, `pcl_conversions`, `cv_bridge`,
`message_filters`, `image_proc` (rectify_node), `robot_state_publisher`,
`rplidar_ros`, `v4l2_camera`, `rcl_interfaces`, `sensor_msgs_py`.

System: PCL with components `common io filters segmentation sample_consensus
search kdtree` (body_preprocess_node needs the segmentation/filter libs on top
of common+io), OpenCV.

Python: `numpy`, `python3-opencv`, `python3-pyqtgraph`, `python3-pyqt5`
(GUI/segmentation), `python3-matplotlib`, `python3-tk`.

Runtime: `micro_ros_agent` (ESP32 bridge -> `/current_position`), `rplidar_ros`
(`/scan`), `robot_state_publisher` (TF tree from URDF), `v4l2_camera`
(`/image_raw`, `/camera_info`).

```bash
sudo apt install -y \
  ros-jazzy-laser-geometry ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-eigen \
  ros-jazzy-pcl-conversions ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-image-proc ros-jazzy-robot-state-publisher \
  ros-jazzy-rplidar-ros ros-jazzy-v4l2-camera \
  python3-pyqtgraph python3-pyqt5 libpcl-dev
```

---

## Build

```bash
cd ~/Documents/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidar_camera_fusion --symlink-install
source install/setup.bash
```

The PCL `CMP0074`/`PCL_ROOT` cmake warnings during the build are cosmetic and
can be ignored.

---

## Usage

### One-command bringup

```bash
ros2 launch lidar_camera_fusion full_system.launch.py
```

This starts, in order: micro-ROS agent, RPLiDAR A1, robot_state_publisher,
v4l2_camera, a delayed `v4l2-ctl` control lock (focus/exposure/WB at +4 s),
scan_cycle_node (+3.5 s), the fusion pipeline (rectify_node + scan_assembler +
cloud_colorizer + scan_image_recorder, +2 s), and the segmentation pipeline
(body_preprocess + manual_segmentation, +3 s).

Key arguments (defaults in parentheses): `mcu_port` (/dev/ttyUSB0),
`lidar_port` (/dev/ttyUSB1), `camera_device` (/dev/video0), `scan_mode`
(Boost), `camera_info_url`
(file:///home/azzam/Documents/webcam_calibration.yaml), `output_dir`
(~/ros2_scans), `max_points` (500000), `publish_rate` (2.0). Sweep:
`scan_start_mm` (0.0), `scan_end_mm` (300.0), `scan_speed_mm_s` (20.0).
Recorder: `use_lidar_distance` (true), `undistort_images` (true),
`min_capture_spacing_mm` (10.0), `lidar_distance_quantile` (0.25).

> **Camera controls.** Focus/exposure/WB are NOT passed as node params —
> applying UVC extension controls before streaming corrupts `VIDIOC_REQBUFS`
> on the C922. `full_system.launch.py` sets them via `v4l2-ctl` 4 s after the
> camera node is up (auto-off lines first, then manual values).

### Scan workflow

```bash
# Option A: one-button sweep (moves gantry, starts, stops, saves)
ros2 service call /scanner/start_cycle std_srvs/srv/Trigger

# Option B: manual control
ros2 service call /scanner/start std_srvs/srv/Trigger
ros2 topic pub /position std_msgs/msg/Float32 "{data: 300.0}" -1
ros2 service call /scanner/stop std_srvs/srv/Trigger
```

On stop, each node saves its artifact to `output_dir` and (if
`auto_open_on_stop=true`) the segmentation GUI opens on the body cloud.

### RViz2

Set Fixed Frame to `base_link`. Add PointCloud2 displays for
`/scanner/assembled_cloud` (color by Z), `/scanner/colored_cloud` and
`/scanner/segmented_regions` (Color Transformer -> RGB8), plus RobotModel + TF.

---

## Troubleshooting

- **`"base_link" does not exist`** — `robot_state_publisher` isn't running. Use
  `full_system.launch.py` or `scanner_bridge.launch.py`.
- **`"lidar_link" does not exist` (transient at startup)** — TF tree has no
  `/joint_states` yet. Clears within 1-2 s once `/current_position` arrives.
- **RPLiDAR exits with `scan mode 'Sensitivity' is not supported`** — the A1
  only supports Standard/Express/Boost. Pass `scan_mode:=Boost`.
- **Camera `Failed mapping device memory`** — focus/exposure applied before
  streaming. Don't pass them as node params; set via `v4l2-ctl` after startup.
- **Colorizer callback never fires** — `ApproximateTime` can't match. Check
  `/scanner/scan_cloud`, `/image_rect_color`, `/camera_info` are all
  publishing; widen `approx_time_slop`.
- **Colored cloud has few points / misaligned edges** — confirm rectify_node is
  up and the colorizer reads `/image_rect_color` (not `/image_raw`); the
  colorizer must project with `CameraInfo.P`. Check the `camera_to_optical` rpy
  in the URDF.
- **Body cloud empty after stop** — Z crop `[z_min, z_max]` removed everything,
  or no cluster >= `cluster_min_size`. Loosen those or set
  `enable_cluster:=false`.
- **`Lookup would require extrapolation into the future`** — TF latency >100 ms.
  Handled by the 100 ms lookup timeout + `spin_thread=true`; raise the timeout
  in the node if your system is slower.

---

## Package layout

```
lidar_camera_fusion/
├── CMakeLists.txt  package.xml  README.md
├── urdf/scanner_bridge.urdf                 full TF tree
├── include/lidar_camera_fusion/             *.hpp for the 3 C++ nodes
├── src/
│   ├── scan_assembler_node.cpp              LaserScan -> cloud + gantry bridge
│   ├── cloud_colorizer_node.cpp             rectified colorize -> XYZRGB
│   └── body_preprocess_node.cpp             RANSAC bed removal + cluster
├── lidar_camera_fusion/                     Python package
│   ├── scan_cycle_node.py                   sweep orchestrator
│   ├── manual_segmentation_node.py          polygon segmentation
│   ├── segmentation_editor_qt.py            pyqtgraph editor widget
│   ├── fusion_gui.py  ros_worker.py         PyQt5 control panel
│   └── pyqtgraph_editor.py
├── scripts/                                 ros2 run entry points
│   ├── scan_image_recorder_node             TF-driven panorama stitcher
│   ├── scan_cycle_node  manual_segmentation_node  fusion_gui  restitch_scan.py
└── launch/
    ├── full_system.launch.py                everything, single command
    ├── fusion.launch.py                     rectify + assembler + colorizer + recorder
    ├── segmentation.launch.py               body_preprocess + manual_segmentation
    └── scanner_bridge.launch.py             robot_state_publisher only
```
