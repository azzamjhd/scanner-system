# gantry_image_stitcher

ROS 2 package for distance-triggered image capture and stitching on a moving gantry.

## Features

- 1D strip capture based on gantry travel distance.
- Capture every `N` mm from scan start (`capture_spacing_mm`), changeable at runtime.
- Live incremental preview published to `/stitcher/preview`.
- High-quality stitching via the [OpenStitching](https://github.com/OpenStitching/stitching) library: SIFT features, affine warper (correct for flat-surface linear scans), multiband blending, gain-blocks exposure compensation, and `range_width=2` sequential matching — sharp results with O(N) feature matching.
- Finalization runs in a background thread; `/stitcher/stop_session` returns immediately.
- Real-time finalization progress on `/stitcher/status`.
- Listener-only design: subscribes to camera images and gantry position, does not command movement.

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
| `/stitcher/status` | `std_msgs/msg/String` | Stitcher state (see below) |

#### `/stitcher/status` message format

| State | Format | Example |
|---|---|---|
| Idle | plain string | `idle` |
| Capturing | plain string | `capturing` |
| Finalizing (progress) | JSON string | `{"state":"finalizing","progress":45,"total":94}` |
| Error | plain string | `error` |

### Services

| Service | Type | Purpose |
|---|---|---|
| `/stitcher/start_session` | `std_srvs/srv/Trigger` | Start capture session |
| `/stitcher/stop_session` | `std_srvs/srv/Trigger` | Stop capturing and start background finalization |
| `/stitcher/save_snapshot` | `std_srvs/srv/Trigger` | Save latest raw frame to disk |

`/stitcher/stop_session` returns immediately. Watch `/stitcher/status` for `finalizing` progress JSON and the final `idle` when the image is written.

## Parameters

All parameters except `image_topic`, `position_topic`, `preview_publish_period_ms`, and `output_dir` can be changed at runtime via `ros2 param set`.

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera/image_raw` | Input image topic |
| `position_topic` | `/current_position` | Input position topic in mm |
| `capture_spacing_mm` | `5.0` | Capture interval in mm from session start. Can be changed mid-session; bucketing resets to current position immediately. |
| `min_motion_for_frame_mm` | `0.5` | Debounce threshold to avoid duplicate captures |
| `max_frame_buffer` | `300` | Max captured frames per session |
| `preview_scale` | `0.35` | Preview downscale factor (0.1–1.0) |
| `preview_publish_period_ms` | `250` | Minimum interval between preview publishes |
| `preview_max_frames` | `10` | Max recent frames used for preview stitch |
| `output_dir` | `/tmp/gantry_stitcher` | Session output directory |
| `pixels_per_mm` | `0.0` | Calibration factor for position-based translation fallback (phase-corr pipeline only). `0` disables it. |
| `phase_corr_min_response` | `0.1` | Minimum phase-correlation response to trust the result (0.0–1.0). Used in the fallback pipeline only. |
| `stitcher_detector` | `sift` | Feature detector for OpenStitching: `sift`, `orb`, `brisk`, `akaze`. `sift` gives the best quality; `orb` is faster. |
| `stitcher_nfeatures` | `1000` | Number of features to detect per frame. Higher = better matching, slower. |
| `stitcher_confidence` | `0.3` | Minimum match confidence for OpenStitching to accept a frame pair (0.0–1.0). Lower this if the "not all images included" warning appears. |

### Changing parameters at runtime

```bash
# Change capture spacing (takes effect immediately, even mid-session)
ros2 param set /stitcher_node capture_spacing_mm 10.0

# Tune phase-correlation confidence threshold
ros2 param set /stitcher_node phase_corr_min_response 0.15

# Set calibration for position-based fallback (pixels per mm of travel)
ros2 param set /stitcher_node pixels_per_mm 12.5
```

## Build

```bash
cd ~/Documents/ros2_ws
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
5. Stop session — returns immediately, finalization runs in background:
   ```bash
   ros2 service call /stitcher/stop_session std_srvs/srv/Trigger
   ```
6. Monitor progress:
   ```bash
   ros2 topic echo /stitcher/status
   # {"state": "finalizing", "progress": 45, "total": 94}
   # ...
   # idle
   ```
7. Output saved under:
   - `/tmp/gantry_stitcher/<session_id>/stitched_final.png`
   - `/tmp/gantry_stitcher/<session_id>/manifest.json`

## Stitching algorithm

### Primary: OpenStitching

Uses the [`stitching`](https://pypi.org/project/stitching/) library with settings tuned for flat-surface linear gantry scans:

| Setting | Value | Why |
|---|---|---|
| `warper_type` | `affine` | Flat surface + linear motion = affine transform (no spherical/cylindrical distortion) |
| `detector` | `sift` (default) | Sharp, accurate keypoints; `orb` available for faster but lower-quality |
| `blender_type` | `multiband` | Frequency-band pyramid blending — crisp seams, no blur |
| `compensator` | `gain_blocks` | Per-block gain correction removes exposure differences between frames |
| `wave_correct_kind` | `no` | No wave correction needed for a straight linear scan |
| `range_width` | `2` | Only matches each frame against its immediate neighbors — O(N) matching instead of O(N²) |

### Fallback: incremental phase-correlation

If OpenStitching fails (insufficient features, low contrast frames), the node falls back to:

1. `cv2.phaseCorrelate` per consecutive frame pair for sub-pixel translation
2. ORB feature matching + RANSAC affine, if phase-correlation response is below `phase_corr_min_response`
3. Position-based translation (`pixels_per_mm` × Δmm), if `pixels_per_mm > 0`
4. Overlap-based horizontal concatenation (last resort)

## Notes

- Supported image encodings: `bgr8`, `rgb8`, `mono8`.
- Preview still uses the OpenCV stitcher on a small window of scaled-down frames; this is fast enough at preview scale.
- A second `/stitcher/stop_session` call while finalization is in progress returns `"No active session to stop"` — this is intentional.
