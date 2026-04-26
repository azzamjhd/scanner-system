# gantry_image_stitcher

ROS 2 package for distance-triggered image capture and stitching on a moving gantry.

## Features

- 1D strip capture based on gantry travel distance.
- Capture every `N` mm from scan start (`capture_spacing_mm`).
- Live incremental preview published to `/stitcher/preview`.
- Final high-quality stitch generated at stop and saved to disk.
- Listener-only design:
  - listens to camera images + gantry position
  - does not command movement.

## Interfaces

### Subscribed topics

| Topic | Type | Purpose |
|---|---|---|
| `image_topic` (default `/camera/image_raw`) | `sensor_msgs/msg/Image` | Camera frames |
| `position_topic` (default `/current_position`) | `std_msgs/msg/Float32` | Gantry position in mm |

### Published topics

| Topic | Type | Purpose |
|---|---|---|
| `/stitcher/preview` | `sensor_msgs/msg/Image` | Live preview mosaic (`bgr8`) |
| `/stitcher/status` | `std_msgs/msg/String` | Stitcher state (`idle`, `capturing`, `finalizing`, `error`) |

### Services

| Service | Type | Purpose |
|---|---|---|
| `/stitcher/start_session` | `std_srvs/srv/Trigger` | Start capture session |
| `/stitcher/stop_session` | `std_srvs/srv/Trigger` | Stop and generate final stitch |
| `/stitcher/save_snapshot` | `std_srvs/srv/Trigger` | Save latest raw frame snapshot |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera/image_raw` | Input image topic |
| `position_topic` | `/current_position` | Input position topic in mm |
| `capture_spacing_mm` | `5.0` | Capture interval in mm from session start |
| `min_motion_for_frame_mm` | `0.5` | Debounce threshold to avoid duplicate captures |
| `max_frame_buffer` | `300` | Max captured frames per session |
| `preview_scale` | `0.35` | Preview downscale factor |
| `preview_publish_period_ms` | `250` | Preview publish period |
| `preview_max_frames` | `10` | Max recent frames used for preview stitch |
| `output_dir` | `/tmp/gantry_stitcher` | Session output directory |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select gantry_image_stitcher
source install/setup.bash
```

## Run

### Node

```bash
ros2 run gantry_image_stitcher stitcher_node
```

### Launch

```bash
ros2 launch gantry_image_stitcher stitcher.launch.py
```

With topic overrides:

```bash
ros2 launch gantry_image_stitcher stitcher.launch.py \
  image_topic:=/camera/image_raw \
  position_topic:=/current_position
```

## Typical workflow

1. Start camera topic and `/current_position` publisher.
2. Start stitcher node.
3. Start session:
   ```bash
   ros2 service call /stitcher/start_session std_srvs/srv/Trigger
   ```
4. Move gantry (from existing scanner GUI/firmware workflow).
5. Stop session and finalize:
   ```bash
   ros2 service call /stitcher/stop_session std_srvs/srv/Trigger
   ```
6. Output saved under:
   - `/tmp/gantry_stitcher/<session_id>/stitched_final.png`
   - `/tmp/gantry_stitcher/<session_id>/manifest.json`

## Notes

- Supported image encodings: `bgr8`, `rgb8`, `mono8`.
- If OpenCV stitcher fails, the node falls back to overlap-based strip concatenation.
