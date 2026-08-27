#!/usr/bin/env bash
# rosbag_replay.sh — Run fusion pipeline with rosbag as sensor input
#
# What it does:
#   1. Starts rosbridge (WebSocket gateway for the UI)
#   2. Runs the real C++ fusion nodes (scan_assembler + cloud_colorizer)
#   3. Plays the rosbag with ONLY raw sensor topics
#   4. The nodes process the raw data and publish /scanner/colored_cloud fresh
#
# The UI then subscribes to /scanner/colored_cloud via rosbridge — same as
# live hardware, but driven by recorded data.
set -euo pipefail

BAG="${1:-/home/gin/rosbags/scan_session_20260826_175758/scan_session_20260826_175758_0.mcap}"
RATE="${2:-1.0}"

# ── Clean environment for ROS 2 ──────────────────────────────────────────────
unset PYTHONPATH

# Source ROS in a way that tolerates unbound variables
set +u
source /opt/ros/jazzy/setup.bash
source ~/scanner-system/install/setup.bash
set -u

echo "══════════════════════════════════════════════════════════════════"
echo "  SMM Rosbag Replay — Full Fusion Pipeline"
echo "══════════════════════════════════════════════════════════════════"
echo "  Bag:     $BAG"
echo "  Rate:    ${RATE}x"
echo "  UI:      http://$(hostname -I | awk '{print $1}'):5173/"
echo "══════════════════════════════════════════════════════════════════"

cleanup() {
  echo ""
  echo "Shutting down..."
  kill $ROSBRIDGE_PID $ASSEMBLER_PID $COLORIZER_PID $BAG_PID 2>/dev/null || true
  wait $ROSBRIDGE_PID $ASSEMBLER_PID $COLORIZER_PID $BAG_PID 2>/dev/null || true
  echo "All stopped."
}
trap cleanup EXIT INT TERM

# ── 1. Rosbridge WebSocket (for the tablet UI) ───────────────────────────────
echo ""
echo "[1/3] Starting rosbridge..."
set +u
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages
set -u
/usr/bin/python3 /opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket &
ROSBRIDGE_PID=$!
sleep 2

# ── 2. Fusion processing nodes (NOT hardware drivers) ────────────────────────
echo "[2/3] Starting fusion nodes..."

# scan_assembler_node: fuses /scan + /current_position → /scanner/assembled_cloud
set +u
source /opt/ros/jazzy/setup.bash
source ~/scanner-system/install/setup.bash
set -u
/usr/bin/python3 /home/gin/scanner-system/install/lidar_camera_fusion/lib/lidar_camera_fusion/scan_assembler_node \
  --ros-args \
    -p scan_topic:=/scan \
    -p position_topic:=/current_position \
    -p joint_topic:=/joint_states \
    -p publish_rate:=2.0 \
    -p max_points:=500000 &
ASSEMBLER_PID=$!
sleep 1

# cloud_colorizer_node: projects camera onto assembled cloud → /scanner/colored_cloud
set +u
source /opt/ros/jazzy/setup.bash
source ~/scanner-system/install/setup.bash
set -u
/usr/bin/python3 /home/gin/scanner-system/install/lidar_camera_fusion/lib/lidar_camera_fusion/cloud_colorizer_node \
  --ros-args \
    -p assembled_topic:=/scanner/assembled_cloud \
    -p image_topic:=/image_rect_color \
    -p camera_info_topic:=/camera_info \
    -p output_topic:=/scanner/colored_cloud \
    -p max_queue:=8 &
COLORIZER_PID=$!
sleep 1

echo "  ✓ scan_assembler_node  (pid $ASSEMBLER_PID)"
echo "  ✓ cloud_colorizer_node (pid $COLORIZER_PID)"

# ── 3. Play rosbag with RAW topics only ──────────────────────────────────────
# Exclude processed outputs so the fusion nodes produce fresh data
echo ""
echo "[3/3] Playing rosbag (rate ${RATE}x, loop)..."
echo "  Raw topics: /scan, /current_position, /joint_states, /image_raw, /camera_info, /tf, /tf_static"
echo ""
ros2 bag play "$BAG" \
  --rate "$RATE" \
  --loop \
  --topics /scan /current_position /joint_states /image_raw /camera_info /tf /tf_static \
  &
BAG_PID=$!

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  ✅ All running! Open http://$(hostname -I | awk '{print $1}'):5173/"
echo ""
echo "  The fusion nodes are processing the bag data live."
echo "  The UI will show the REAL /scanner/colored_cloud output."
echo ""
echo "  Press Ctrl+C to stop everything."
echo "══════════════════════════════════════════════════════════════════"

wait