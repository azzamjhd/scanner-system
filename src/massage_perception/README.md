# massage_perception

ROS 2 Python package for real-time human pose detection and derived massage-point estimation from a camera image stream.

## What this package does

The `pose_node` executable:
- Subscribes to `camera/image_raw` (`sensor_msgs/msg/Image`)
- Runs MediaPipe Pose detection
- Publishes:
  - `pose_landmarks` (`geometry_msgs/msg/PoseArray`)
  - `massage_points` (`geometry_msgs/msg/PoseArray`)
- Optionally shows an OpenCV viewer window with skeleton, massage-point labels, and FPS
- Hot-reloads massage point config when the YAML file changes

---

## 1. Prerequisites

1. ROS 2 installed and sourced (tested workflow assumes a standard ROS 2 workspace with `colcon`)
2. Python 3 environment available
3. Camera image topic publishing BGR-compatible frames on `camera/image_raw`

Install common dependencies:

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-v4l2-camera python3-pip
```

> **⚠️ Important:** MediaPipe requires `numpy<2`. Ubuntu 24.04 ships with numpy 2.x.
> Use the bootstrap script or install into an isolated venv:
> ```bash
> python3 -m venv ~/.scanner_venv
> ~/.scanner_venv/bin/pip install "numpy<2" mediapipe opencv-python pyyaml
> ```
> Then activate the venv before running `pose_node`:
> ```bash
> source ~/.scanner_venv/bin/activate
> ```

> `ament_index_python` is typically available with ROS 2 Python installations.

---

## 2. Build in a ROS 2 workspace

From your workspace root (example: `~/ros2_ws`):

```bash
cd ~/ros2_ws
colcon build --packages-select massage_perception
source install/setup.bash
```

If you open a new terminal, source again:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## 3. Run the node

### Default run

```bash
ros2 run massage_perception pose_node
```

### Headless run (no GUI window)

```bash
ros2 run massage_perception pose_node --ros-args -p show_viewer:=false
```

### Run with custom massage-points config file

```bash
ros2 run massage_perception pose_node --ros-args -p config_path:=/absolute/path/to/massage_points.yaml
```

---

## 4. Parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `config_path` | string | Installed `config/massage_points.yaml` | YAML file used to define massage points from pose landmarks |
| `show_viewer` | bool | `true` | If true, opens OpenCV viewer window |

---

## 5. Topics and message semantics

| Topic | Direction | Type | Notes |
|---|---|---|---|
| `camera/image_raw` | Subscribed | `sensor_msgs/msg/Image` | Input image stream |
| `pose_landmarks` | Published | `geometry_msgs/msg/PoseArray` | 33 MediaPipe landmarks, normalized image coordinates in `position.x/y`; depth in `position.z` |
| `massage_points` | Published | `geometry_msgs/msg/PoseArray` | Computed points from your YAML config (`position.x/y` normalized, `z=0`) |

All published arrays use header `frame_id = "camera"`.

Quick inspection commands:

```bash
ros2 topic list
ros2 topic echo /pose_landmarks
ros2 topic echo /massage_points
```

---

## 6. Massage points config (`massage_points.yaml`)

Default file in this repo:
- `config/massage_points.yaml` (copied into installed share path on build)

Schema:

```yaml
massage_points:
  Point_Name:
    components:
      - landmark: <index>
        weight: <float>
      - landmark: <index>
        weight: <float>
    offset:
      x: <float>
      y: <float>
```

Computation:

```text
point = Σ(weight_i * landmark_i) + offset
```

Where each landmark index is a MediaPipe BlazePose landmark ID (0..32).

### Hot reload behavior

The node checks the config file periodically and reloads automatically when it changes.  
You can tune points at runtime without restarting the node.

---

## 7. Supplying camera input (example)

If you stream from `v4l2_camera`, start it like this:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -r image_raw:=/camera/image_raw
```

Then run `pose_node` in another terminal.

---

## 8. Troubleshooting

### `ModuleNotFoundError` for `mediapipe`, `cv2`, or `yaml`
MediaPipe requires `numpy<2`. If you see import errors, activate the venv:

```bash
source ~/.scanner_venv/bin/activate
```

If the venv doesn't exist yet, create it:
```bash
python3 -m venv ~/.scanner_venv
~/.scanner_venv/bin/pip install "numpy<2" mediapipe opencv-python pyyaml
```

### No output on `pose_landmarks` / `massage_points`
- Confirm camera topic exists and is active:
  ```bash
  ros2 topic hz /camera/image_raw
  ```
- Ensure topic name matches exactly `camera/image_raw` (without leading `/` remap if your system differs).
- Verify person is visible to the camera and sufficiently lit.

### Viewer window does not appear or crashes on headless machine
Run with:

```bash
ros2 run massage_perception pose_node --ros-args -p show_viewer:=false
```

### Config changes are not reflected
- Confirm you are editing the same file used by `config_path`
- Check node logs for config reload/load errors
- Ensure YAML is valid

### `Package 'massage_perception' not found`
Rebuild and source your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select massage_perception
source install/setup.bash
```

---

## 9. Package executable

Installed ROS 2 executable:

```bash
ros2 pkg executables massage_perception
```

Expected entry:
- `massage_perception pose_node`
