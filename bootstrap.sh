#!/usr/bin/env bash
# bootstrap.sh — one-command setup for the massage scanning system.
#
# REQUIRES: Ubuntu 24.04 (Noble), internet access.
#
# Quick start:
#   git clone https://github.com/YOUR_USERNAME/scanner_system.git ~/scanner_ws
#   cd ~/scanner_ws && ./bootstrap.sh
#
set -euo pipefail

# ---- colour helpers ----
RED=$(tput setaf 1 2>/dev/null || echo '')
GREEN=$(tput setaf 2 2>/dev/null || echo '')
BOLD=$(tput bold 2>/dev/null || echo '')
NC=$(tput sgr0 2>/dev/null || echo '')

log()  { echo "${GREEN}==>${NC} ${BOLD}$*${NC}"; }
err()  { echo "${RED}ERROR:${NC} $*" >&2; exit 1; }

# ---- checks ----
[[ "$(lsb_release -rs 2>/dev/null || echo '')" == "24.04" ]] \
  || err "This bootstrap requires Ubuntu 24.04."

command -v colcon &>/dev/null || command -v ros2 &>/dev/null \
  || log "ROS 2 Jazzy not found — installing…"

# ---- 0. Pull external dependencies (rplidar_ros) ----
log "Pulling external dependencies (rplidar_ros)…"
sudo apt-get install -y -qq python3-vcstool 2>/dev/null || true
vcs import src < scanner_system.repos 2>/dev/null || true

# ---- 1. Install ROS 2 Jazzy (if absent) ----
if ! command -v ros2 &>/dev/null; then
  log "Adding ROS 2 repository…"
  sudo apt-get update -qq
  sudo apt-get install -y -qq curl gnupg2 lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update -qq
  log "Installing ros-jazzy-desktop (this may take a while)…"
  sudo apt-get install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
fi

# ---- 2. rosdep ----
log "Initialising rosdep (if not done)…"
sudo rosdep init 2>/dev/null || true
rosdep update

# ---- 3. System dependencies (rosdep + explicit) ----
log "Installing system dependencies…"

# rosdep handles ROS-level deps
rosdep install --from-paths src --ignore-src -y -r \
  --skip-keys='libpcl-all-dev,libpcl-common1.14,libpcl-io1.14,libpcl-filters1.14,libpcl-segmentation1.14,libpcl-sample-consensus1.14,libpcl-search1.14,libpcl-kdtree1.14' \
  2>&1 | tail -5

# PCL — rosdep doesn't map versioned .so deps; we use the build-dev metapackage
sudo apt-get install -y -qq \
  libpcl-dev \
  python3-opencv python3-numpy python3-scipy python3-matplotlib \
  python3-tk \
  python3-pyqt5 python3-pyqtgraph \
  python3-pyqt6

# ---- 4. Pip dependencies (not in apt) ----
log "Installing pip dependencies…"

# mediapipe has a strict numpy<2 requirement on Noble (system numpy is 2.x).
# Install into a venv pinned to numpy<2 so we don't break system packages.
MEDIAPIPE_VENV="$HOME/.scanner_venv"
if [[ ! -d "$MEDIAPIPE_VENV" ]]; then
  python3 -m venv "$MEDIAPIPE_VENV"
fi
"$MEDIAPIPE_VENV/bin/pip" install -q "numpy<2" mediapipe

# Optional / experimental — only needed for bodypix_segmentation_node
# pip install tf-bodypix

log "MediaPipe installed into $MEDIAPIPE_VENV"
log "  → launch with: source $MEDIAPIPE_VENV/bin/activate before colcon build/run"

# ---- 5. Patch rplidar_ros (A1 angle_max fix) ----
RPLIDAR_CPP="src/rplidar_ros/src/rplidar_node.cpp"
if grep -q 'angle_max = DEG2RAD(359.0f)' "$RPLIDAR_CPP" 2>/dev/null; then
  log "Patching rplidar_ros (359° → 360°)…"
  sed -i 's/DEG2RAD(359.0f)/DEG2RAD(360.0f)/' "$RPLIDAR_CPP"
fi

# ---- 6. Build ----
log "Sourcing ROS 2 and building…"
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo ""
echo "${GREEN}============================================${NC}"
echo "${GREEN}  Bootstrap complete!${NC}"
echo ""
echo "  Source the overlay and run:"
echo "    source install/setup.bash"
echo "    ros2 launch lidar_camera_fusion full_system.launch.py"
echo ""
echo "  Without hardware (simulated encoder):"
echo "    ros2 launch lidar_camera_fusion full_system.launch.py \\"
echo "      mcu_port:='' lidar_port:=/dev/null scan_mode:=Standard \\"
echo "      simulate_encoder:=true"
echo ""
echo "  If using MediaPipe nodes, activate the venv first:"
echo "    source $MEDIAPIPE_VENV/bin/activate"
echo "${GREEN}============================================${NC}"
