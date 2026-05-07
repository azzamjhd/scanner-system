# Smart Massage Machine — System Specification

**Project type:** Academic capstone  
**Operator model:** Single operator, fully offline, local network only  
**Your scope:** Scanning, perception, path generation, and the web dashboard. The AR3 arm execution is owned by a separate team member.

---

## 1. System Overview

A human lies prone (face-down) on a massage table under a linear gantry. The gantry carries an RPLiDAR A1 and an RGB camera. One traversal of the gantry produces:
- A **3D point cloud** of the body surface (stacked LiDAR cross-sections)
- A **stitched panoramic image** of the body (assembled camera frames)

The operator then uses a web dashboard to:
1. View the panorama with body-region overlays (MediaPipe-derived labels)
2. Draw freehand zones on the panorama and assign massage parameters (technique, pressure, stroke pattern)
3. Preview the generated 3D arm path overlaid on the panorama
4. Approve and order zones, then trigger execution

The scanning system publishes a custom ROS2 message with full waypoint metadata. The arm team consumes it.

---

## 2. Hardware

| Component | Device | ROS2 interface |
|---|---|---|
| Linear gantry | ESP32 + FastAccelStepper | micro-ROS serial `/dev/ttyUSB0 @115200` |
| LiDAR | RPLiDAR A1 | `/dev/ttyUSB1` → `/scan` (LaserScan) |
| Camera | USB RGB camera | `/dev/video0` → `/camera/image_raw` (Image) |
| Robot arm | AR3 / AR2 (6-DOF) | Owned by arm team — consumes published waypoints |
| F/T sensor | Mounted at arm end-effector | Arm team's responsibility; feedback published on `/arm/force` |

**Gantry constants (ESP32 firmware):**
- `STEPS_PER_MM = 80`
- Position clamp: ±2000 mm
- Speed clamp: 0.1–200 mm/s
- Acceleration clamp: 1–400 mm/s²
- Homing: switch on `GPIO 32`, backoff = 5 mm

**Scanning geometry:**
- LiDAR faces **downward**, mounted on gantry carriage
- Gantry moves along the **body's long axis** (head → toe = +Y direction in world frame)
- At each Y position, LiDAR sweeps a transverse X-Z cross-section of the body
- ROS2 mode: `axis = 1` (linear Z) in `scanner_3d_node`
- World frame origin: gantry home position (Y = 0)
- Camera is top-down, moves with gantry → produces orthographic-approximation panorama

---

## 3. What Is Already Built

| Component | ROS2 node | Status |
|---|---|---|
| Gantry motion control | ESP32 firmware + micro-ROS agent | **Working** |
| 3D point cloud generation | `scanner_3d_node` | **Working** — saves `.ply` to `/tmp` |
| Camera stitching | `stitcher_node` | **Working** — saves `stitched_final.png` |
| Scanner GUI (PyQt5) | `scanner_gui_node` | **Working** — gantry control + scan start/stop |
| Body-pose detection | `massage_perception/pose_node` | **Partially built** — MediaPipe landmarks not yet integrated into stitched image |
| Web dashboard | — | **Not built** |
| Path generation | — | **Not built** |
| Patient profile system | — | **Not built** |
| Arm interface message | — | **Not defined** |

---

## 4. ROS2 Topic / Service Graph

```
RPLiDAR ──────────────────► /scan (LaserScan)
                                    │
                                    ▼
                          scanner_3d_node
                          │    │    │
              /start_scan ◄    │    ► /scanner/pointcloud (PointCloud2)
              /stop_scan  ◄    │    ► saves scan_YYYYMMDD.ply
                               │
ESP32 firmware ──► /current_position (Float32) ──► scanner_3d_node
                                                 └► stitcher_node
                                                 └► scanner_gui_node

scanner_gui_node ──► /position, /speed, /acceleration (Float32) ──► ESP32

Camera ──────────────► /camera/image_raw (Image)
                                │
                     ┌──────────┴──────────┐
                     ▼                     ▼
              stitcher_node         pose_node (MediaPipe)
              │                     │
              ► /stitcher/preview   ► /pose_landmarks (PoseArray)
              ► saves stitched.png  ► /massage_points (PoseArray)

[NEW] path_generator_node
   ◄ PLY file + stitched PNG (file paths via parameter or service)
   ◄ zone config from web dashboard (ROS2 service or parameter)
   ► /massage_path (MassagePath msg — see §7)

[NEW] web_bridge_node  (rosbridge_suite or custom FastAPI)
   ↔ browser ↔ all topics/services above
```

---

## 5. System State Machine

```
IDLE
  │ operator clicks "Start Scan"
  ▼
SCANNING
  │ gantry traverses, LiDAR captures cross-sections, camera captures frames
  │ gantry reaches end position
  ▼
PROCESSING
  │ scanner_3d_node finalizes PLY
  │ stitcher_node finalizes stitched PNG
  │ pose_node projections applied as overlays (§8.1)
  │ (~30–60 s acceptable)
  ▼
ZONE_SELECTION
  │ operator views panorama in browser
  │ operator draws freehand polygons, assigns technique+force+pattern per zone
  │ operator drags to order zones
  │ operator clicks "Generate Path"
  ▼
PATH_PREVIEW
  │ system generates 3D waypoints (§8.2)
  │ path overlaid on panorama (projected back to 2D) and optionally in RViz
  │ operator reviews, clicks "Approve & Start Massage"
  ▼
EXECUTING
  │ system publishes /massage_path
  │ arm team consumes and executes zone-by-zone
  │ GUI shows: live camera, progress, F/T readout, pause/abort controls
  │ F/T threshold exceeded → PAUSED (auto)
  │ person movement detected → PAUSED (auto)
  │ operator abort → IDLE
  │ all zones complete → COMPLETE
  ▼
COMPLETE / IDLE
```

---

## 6. Scan Protocol

**Operator configures before each scan (in existing GUI):**
- Start position (mm) — default 0
- End position (mm) — default covers full body length, typically 1500–1800 mm
- Speed (mm/s) — suggested 30–50 mm/s for body scanning
- Person must hold breath during the traversal (approximately 20–30 s at 50 mm/s for 1 m)

**LiDAR parameters:**
- `angle_min`, `angle_max`: set to cover the table width (e.g., 0°–180°)
- `range_max`: 1.5 m (table height + body height, ~1.2 m typical clearance)
- `axis = 1` (linear Z mode)

**Camera capture spacing:** 5 mm default (via `capture_spacing_mm` in stitcher)

**Body positions supported:**
- **Prone (face-down):** Primary. Back, shoulders, glutes, legs accessible.
- **Supine (face-up):** Planned for future. Operator selects position before scan; no system change needed — same scan process.

---

## 7. Custom ROS2 Message: `MassagePath`

Define in `medical_scanner_pkg/msg/`:

```
# MassagePath.msg — published on /massage_path
std_msgs/Header header
MassageZone[] zones
```

```
# MassageZone.msg
string zone_id               # e.g. "zone_0", "zone_1"
string technique             # "effleurage" | "petrissage" | "tapotement" | "circular"
string stroke_pattern        # "linear" | "circular" | "cross_fiber"
uint32 pass_count            # number of times to repeat the path over this zone
MassageWaypoint[] waypoints
```

```
# MassageWaypoint.msg
uint32 waypoint_index
geometry_msgs/Pose pose      # position (XYZ in world frame) + orientation (surface normal as quaternion)
float32 force_n              # target normal force in Newtons (e.g. 5–30 N)
bool approach                # true = this is an approach waypoint (reduced force, arm coming down to surface)
```

**Orientation convention:** The pose quaternion encodes the surface normal such that the arm's Z-axis (tool axis) aligns with the inward surface normal. The arm team is responsible for IK from these Cartesian poses.

---

## 8. Path Generation Pipeline

### 8.1 Body-Region Overlay on Stitched Image

The stitcher captures frames at known gantry positions. Each camera frame has a corresponding Y position in world frame.

**Pixel → World mapping (orthographic approximation):**
```
world_Y = gantry_position_at_capture_mm
world_X = (pixel_u - image_width/2) * (table_width_mm / image_width)
world_Z = from nearest point cloud point at (world_X, world_Y)
```

MediaPipe runs on individual captured frames (not the final stitch). The landmark positions in each frame are transformed into stitched image coordinates:
```
stitch_u = (landmark.x * frame_width) + (frame_index * stride_px)
stitch_v = landmark.y * frame_height
```

These projected landmarks define clickable body-region polygons (shoulders, upper back, lower back, glutes, legs) drawn as overlays on the panorama. Labels are **visual guides only** — the operator still draws their own zone boundaries.

### 8.2 2D Zone → 3D Waypoints

**Input:** Freehand polygon drawn on the stitched image (list of pixel coordinates), technique, force, pattern.

**Step 1: Polygon → World XY footprint**
Convert each polygon vertex from pixel → world XY using the orthographic mapping above.

**Step 2: Grid sampling**
Generate a regular grid of sample points inside the polygon's XY footprint. Grid spacing = configurable (default 20 mm). This gives a list of `(world_X, world_Y)` sample locations.

**Step 3: Z + normal from point cloud**
For each grid point, query the nearest N points in the loaded point cloud (kdtree or voxel lookup). Extract:
- `world_Z` = mean Z of neighbors (surface height)
- Surface normal via PCA on the N-point neighborhood (N ≥ 10)
- Reject points where no neighbor found within 30 mm (outside body extent)

**Step 4: Normal → Quaternion**
Convert inward surface normal vector to quaternion so the arm's tool axis aligns with it.

**Step 5: Stroke pattern ordering**
- **Linear strokes:** Sort grid points into parallel rows along the short axis of the zone bounding box. Each row is a forward-then-return stroke.
- **Circular:** Spiral outward from zone centroid.
- **Cross-fiber:** Two passes: one at 0° and one at 90° to the long axis.

**Step 6: Approach waypoints**
Before each zone, prepend a waypoint 50 mm above the first contact point with `approach=true` and `force_n=0`. Between zones, same approach sequence.

**Processing time target:** Under 60 seconds for a full-body scan at 20 mm grid spacing (~2000–3000 waypoints per zone).

---

## 9. Web Dashboard

**Technology stack (recommended):**
- Backend: **FastAPI** Python server running on the ROS2 machine
- ROS2 bridge: **rosbridge_suite** (`ros2 run rosbridge_server rosbridge_websocket`) for live topic streaming to the browser
- Frontend: Single-page HTML/JS app served by FastAPI; uses **roslibjs** to subscribe to topics and call services
- No internet required — served at `http://localhost:8080`

### 9.1 Pages / Phases

#### Patients Page
- Create / select patient profile
- List of past sessions per patient with date, zones massaged, PLY + image download links

#### Scan Page
- Gantry controls (start pos, end pos, speed — mirrors existing scanner GUI)
- "Start Scan" button — calls `/start_scan` service
- Live LiDAR point count + connection indicator
- Live camera feed (`/camera/image_raw`)
- "Stop Scan" button — calls `/stop_scan` service, saves session

#### Zone Selection Page
- Displays the stitched panoramic image full-width
- MediaPipe body-region polygon overlays (translucent, named)
- **Freehand draw tool:** Click-drag to draw a polygon; polygon is highlighted on release
- Per-zone sidebar (appears when a zone is drawn):
  - Zone name (auto: "Zone 1", "Zone 2", ...)
  - Technique: dropdown (`effleurage` / `petrissage` / `tapotement`)
  - Stroke pattern: dropdown (`linear` / `circular` / `cross-fiber`)
  - Force level: slider 5–30 N with three labeled presets (Light 5–10 N, Medium 10–20 N, Deep 20–30 N)
  - Pass count: integer spinner (1–5)
  - Delete zone button
- Zone ordering panel: drag handles to reorder zones (determines arm execution sequence)
- "Generate Path" button — sends zone polygons + parameters to `path_generator_node`

#### Path Preview Page
- Stitched panorama with zone boundaries highlighted
- Sampled waypoints rendered as dots projected back from 3D → 2D (colour-coded by force)
- "Approve & Start Massage" button (disabled until all zones have valid waypoints)
- Warning badges on zones with unreachable waypoints (flagged by arm team's reachability check if available)

#### Massage Control Page (active during execution)
- Live camera feed (top section)
- Zone queue: list of zones with status (Pending / Active / Complete / Skipped)
- Current waypoint progress bar within active zone
- Force readout: live gauge from `/arm/force` topic
- Prominent **PAUSE** button and **ABORT** (E-stop) button
- Session log (timestamped events)

### 9.2 Status Bar (all pages)
- Connection indicator (ROS2 bridge connected / disconnected)
- Gantry position (mm) — from `/current_position`
- Current phase badge

---

## 10. Patient Profile Storage

**Storage location:** `~/.massage_machine/patients/`

**Directory structure:**
```
patients/
  <patient_id>/            # UUID generated at profile creation
    profile.json           # name, DOB (optional), notes
    sessions/
      <session_id>/        # timestamp-based ID e.g. 20260427_143022
        scan.ply           # 3D point cloud
        panorama.png       # stitched camera image
        zones.json         # zone polygons, technique, force, pattern, order
        path.json          # generated waypoints (for replay without re-generating)
        session_log.json   # timestamped events: scan started, zones massaged, force events
```

**`zones.json` schema:**
```json
{
  "zones": [
    {
      "zone_id": "zone_0",
      "name": "Upper Back",
      "polygon_px": [[x1,y1], [x2,y2], ...],
      "technique": "effleurage",
      "stroke_pattern": "linear",
      "force_n": 15.0,
      "pass_count": 2,
      "order": 0
    }
  ]
}
```

**Session reload:** Operator can load a previous session's `zones.json` onto a new scan's panorama, re-generate waypoints with the new geometry. This allows repeating a massage protocol on a new scan without reconfiguring zones from scratch.

---

## 11. Safety Requirements

### 11.1 Force/Torque Threshold
- Soft limit: 35 N → arm slows to 25% speed, logs warning
- Hard limit: 50 N → arm stops immediately, session PAUSED, operator must acknowledge before resuming
- Implementation: arm team monitors `/arm/force`; your system also subscribes and disables the "Resume" button until force drops below 20 N

### 11.2 Motion Detection During Massage
- `pose_node` (MediaPipe) runs continuously during execution
- If centroid of detected landmarks shifts > 50 mm from the position at session start → publish `/safety/motion_alert` (std_msgs/Bool)
- Web dashboard receives alert, pauses session, shows warning modal

### 11.3 Pre-Execution Checks
Before publishing `MassagePath`:
- Warn if any waypoint is outside the expected reachable envelope (configurable bounding box in world frame — arm team to define limits)
- Warn if total waypoint count > 5000 (session will be long)
- Error if no body detected in point cloud (point cloud below 500 points after removing table plane)

### 11.4 E-Stop
- Physical: existing gantry E-stop publishes `speed=0, accel=500` to halt gantry
- Software: "ABORT" button in dashboard publishes on `/safety/estop` (std_msgs/Bool true); arm team must subscribe and halt

---

## 12. New ROS2 Nodes to Build

| Node | Package | Responsibility |
|---|---|---|
| `path_generator_node` | `medical_scanner_pkg` | Load PLY, receive zone config, generate `MassagePath`, publish `/massage_path` |
| `web_bridge_node` | `medical_scanner_pkg` | Serve FastAPI app + proxy ROS2 topics/services to browser via WebSocket |
| `session_manager_node` | `medical_scanner_pkg` | Patient profile CRUD, session save/load, file management |
| `safety_monitor_node` | `medical_scanner_pkg` | Subscribe to `/arm/force`, compare pose landmarks, publish `/safety/motion_alert`, `/safety/estop` |

**`path_generator_node` services:**
- `/path_generator/load_scan` (Trigger) — loads latest PLY and panorama for active session
- `/path_generator/set_zones` (custom srv: accepts `zones.json` content) — sets zone config
- `/path_generator/generate` (Trigger) — runs path generation, publishes `/massage_path`

---

## 13. New tmuxinator Windows Needed

Add to `~/.tmuxinator.yml`:
```yaml
  - processing:
      layout: even-horizontal
      panes:
        - path_gen:
            - source /opt/ros/jazzy/setup.bash && source install/setup.bash
            - ros2 run medical_scanner_pkg path_generator_node
        - safety:
            - source /opt/ros/jazzy/setup.bash && source install/setup.bash
            - ros2 run medical_scanner_pkg safety_monitor_node
        - web:
            - source /opt/ros/jazzy/setup.bash && source install/setup.bash
            - ros2 run medical_scanner_pkg web_bridge_node
        - rosbridge:
            - source /opt/ros/jazzy/setup.bash && source install/setup.bash
            - ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## 14. Open Risks and Known Gaps

### HIGH — No patient registration on table
The person may lie down in a slightly different position each session. Without physical alignment guides or reference markers, all waypoints are in the scanner frame, which may not match the arm's frame after repositioning. **Mitigation:** Install a head-rest and shoulder-width guide on the table as physical constraints. Document the assumption that the person's shoulder line is always at Y = 100 mm from gantry home.

### HIGH — AR3 arm integration undefined
The AR3 arm uses Arduino Mega firmware communicating over USB serial. The `ar3_ros` community driver exists but is not officially supported. The arm team needs to:
1. Integrate `ar3_ros` (or write a micro-ROS firmware) to receive Cartesian pose waypoints
2. Run MoveIt2 for IK from your Cartesian `MassagePath` messages
3. Define and publish `/arm/force` for the safety monitor
**Your deliverable:** A valid `MassagePath` message on `/massage_path` in the world frame. The coordinate frame transform between world frame and AR3 base frame must be agreed with the arm team.

### MEDIUM — Camera-LiDAR offset not calibrated
The camera and LiDAR are physically offset on the gantry. At a given gantry Y position, the camera sees a slightly different Y slice than the LiDAR scanned. At 50 mm/s and 5 mm capture spacing (0.1 s between frames), this offset is small but non-zero. Measure the camera-LiDAR Y offset and add it as a parameter to the stitcher (`camera_lidar_offset_mm`).

### MEDIUM — Prone/supine requires separate scan origin
When the person flips to supine, the body height and surface shape change completely. The stored PLY for a prone session cannot be reused for supine. Store body position as metadata in `session_log.json`.

### LOW — MediaPipe accuracy on prone body
MediaPipe BlazePose is trained on upright or seated humans. Detection quality on a top-down view of a prone body may be poor. The landmark overlays are **visual guides only** — the operator's freehand selection is authoritative. Degrade gracefully: if no landmarks detected, show the panorama without overlays and let the operator select blind.

### LOW — Point cloud density vs. path resolution
At 50 mm/s gantry speed, the RPLiDAR A1 scans at 10 Hz → one cross-section every 5 mm. At 20 mm grid spacing for path generation, each grid point will have at least 4 nearby cross-sections for normal estimation. If gantry speed increases to > 100 mm/s, cross-section spacing exceeds 10 mm and normal quality degrades. Document gantry speed limit for good scan quality: **≤ 60 mm/s**.
