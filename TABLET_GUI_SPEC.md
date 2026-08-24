# Smart Massage Machine: Tablet GUI Implementation Specification

## Problem Statement

The operator of the Smart Massage Machine currently interacts with the system using either a local desktop PyQt5 GUI (`MissionControlGUI`) or a text curses TUI (`fusion_tui`) running directly on the ROS 2 Jazzy host machine. In practical clinical or home settings, operating a desktop workstation or executing raw terminal commands next to a massage bed is awkward and inefficient. The operator needs a lightweight, touch-optimized, full-screen tablet dashboard that runs on a handheld tablet (e.g., iPad or Android tablet) connected over the local network. This tablet interface must allow the operator to trigger scans, view live 3D visual progress, interactively pinpoint massage landmarks on the patient's body contour, configure force parameters, and command execution, all while maintaining absolute safety controls.

## Solution

Create a touch-optimized Progressive Web Application (PWA) frontend using **SvelteKit** that runs full-screen in a tablet web browser. The frontend communicates with the ROS 2 Jazzy backend (running natively on the Ubuntu host) via a **WebSocket** bridge provided by standard `rosbridge_suite` (`rosbridge_websocket_node`). 

Key architectural components include:
1. **Real-time 3D Canvas:** Built with `ros3djs` (Three.js-backed) to subscribe to TF `/tf` transforms and render the gantry and a downsampled point cloud (`/point_cloud/downsampled`).
2. **Scan Action Client:** Interacts with the `/scan_body` ROS 2 Action Server to sequence the physical gantry sweeps, reporting live feedback and progress.
3. **Interactive Target Selector:** Raycasts touch inputs into the 3D scene, rendering large interactive bounding spheres (touch targets ≥ 44x44px) to select massage points.
4. **Execution Action Client:** Replaces the static `/execute_massage_plan` Service with a ROS 2 Action Server that provides real-time force feedback, current waypoint status, and clean cancellation/halt functions.
5. **Dual-Layer E-Stop & Software Abort:** Anchors a prominent red Software Abort button inside the UI viewport CSS lockdown while preserving the dedicated physical hardware gantry E-stop.

## User Stories

1. As an operator, I want to load a patient profile from the tablet UI, so that I can access historical massage settings and scan data.
2. As an operator, I want to configure the start position, end position, and speed using touch sliders, so that I can set scan limits for different height patients.
3. As an operator, I want to tap a "Start Contour Scan" button, so that the dual-axis gantry enters sweeping mode.
4. As an operator, I want to see a circular progress bar and text status, so that I know whether the system is homing, scanning, or processing Mediapipe markers.
5. As an operator, I want the tablet screen's gestures (pinch, double-tap, sweep) blocked by default, so that accidental browser zoom doesn't disrupt my view.
6. As an operator, I want a downsampled 3D point cloud of the patient's body to stream onto the tablet browser, so that I do not suffer low frame rates or browser crashes.
7. As an operator, I want to see the detected anatomical landmarks (e.g. Jian Jing points) highlighted as large 3D spheres on the canvas, so that I can visually verify correct pose detection.
8. As an operator, I want to tap on individual landmark spheres to toggle them (active/inactive), so that I can customize the massage targets.
9. As an operator, I want each touchable sphere to have a minimum target size of 44x44 pixels, so that I do not miss-click or fail to toggle elements.
10. As an operator, I want to slide parameters for target force levels (Light, Medium, Deep) per zone, so that I can adjust the therapy intensity.
11. As an operator, I want to select a massage pattern (Direct Pressure, Circular, Cross-fiber) from a dropdown menu, so that I can tailor the technique.
12. As an operator, I want to drag and reorder the massage sequence blocks on my sidebar, so that I can control which body region is targeted first.
13. As an operator, I want to tap a "Start Massage" button, so that the arm receives coordinates and begins execution.
14. As an operator, I want to see live force/torque feedback (F/T readings) and waypoint index progress on the tablet, so that I can monitor physical contact safety.
15. As an operator, I want to trigger a persistent red "SOFTWARE ABORT" button on the UI, so that I can immediately tell the gantry and robot arm to freeze movements.
16. As an operator, I want to tap "Pause" and "Resume", so that I can temporarily halt the robot arm during execution.

## Implementation Decisions

### 1. Frontend Framework
Use SvelteKit configured with static single-page pre-rendering. This eliminates high runtime framework overhead, maximizing CPU allocation for rendering point clouds and handling High-frequency TF frames in WebGL.

### 2. Middleware & Communication
Use `rosbridge_suite` for WebSocket serialization. SvelteKit mounts `roslibjs` in the browser, communicating as a ROS 2 client. Custom types will be serialized to JSON, avoiding compiler-level Protobuf dependencies in browser JS.

### 3. Point Cloud Processing & URDF Hosting
- **Voxel Grid Downsampling:** A ROS 2 node (`body_preprocess_node` or a dedicated `voxel_filter_node`) will downsample the dense input cloud to ~2,000 vertices and publish to `/point_cloud/downsampled`.
- **Static Asset Serving:** Host the URDF mesh STL/DAE files via a local HTTP server (served alongside SvelteKit static build) so `ros3djs` can parse and display the gantry structure.

### 4. ROS 2 Interface Upgrade (Action Server)
- **Scan Action:** Refactor `scan_cycle_node.py` or wrap its trigger services under a ROS 2 Action Server `/scan_body` (`lidar_camera_fusion_interfaces/action/ScanBody`).
- **Execution Action:** Expose `/execute_massage_plan` as an Action Server (`/execute_massage_plan`). Feedback publishes the current target point index, trajectory percentage, and active force feedback.

### 5. Touch Interactive Raycaster
Instead of relying on selecting the exact tiny points of the point cloud, `ros3djs` will project spheres on the target coordinates. The `THREE.Raycaster` threshold config `raycaster.params.Points.threshold` will be set to `0.05` to capture low-accuracy touch coordinates.

### 6. Viewport Interdictions
Enforce CSS `touch-action: none; user-select: none; -webkit-touch-callout: none;` on the root HTML body to deactivate native browser double-tap-zoom and swipe-to-refresh gestures.

### 7. Dual-Channel E-Stop Boundary
The UI button is explicitly named "SOFTWARE ABORT (STOP GANTRY / ARM)". It streams velocity command overrides to zero and calls execution cancellation services. The physical gantry E-stop switch remains the direct hardware line cutoff of stepper driver power.

## Testing Decisions

- **Test Seam (Mock client/server via roslibjs):** Create a standalone mocking layer in SvelteKit that mocks `window.WebSocket` and emits simulated ROS 2 topic updates (mock TF, mock point cloud messages) and Action feedback. This allows full unit testing of UI state-machine transitions, canvas event listeners, and styling triggers without needing ROS 2 or physical hardware under the test runner.
- **Component under test:** SvelteKit screens, ROS 2 WebSocket controller logic, and the viewport scale locking.
- **Backend Tests:** Write python integration tests under `colcon test` for the new Action server endpoints, verifying response status on cancel and timeout requests.

## Out of Scope

- **MoveIt Collision Avoidance:** Automatic real-time target avoidance of the robotic arm is handled by the arm team's MoveIt configuration on ROS 2 Jazzy.
- **Physical Force/Torque Sensor Integration:** Direct manipulation of force/torque endpoints is processed on the arm client board (Teensy); tablet GUI only displays telemetry published to `/arm/force`.
- **Database Synchronization:** Synced cloud data storage of patient records; the tablet GUI operates purely local-network SQL queries or offline mock filesystem logs.

## Further Notes

Hardware limits documented in ESP32 firmware (80 steps/mm, clamps, backoff, and homing parameters) are to be hard-coded into the configuration parameters of the ROS 2 Action Server to prevent invalid user inputs from reaching the micro-ROS nodes.
