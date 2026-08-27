#!/usr/bin/env bash
# bootstrap.sh — one-command setup for the massage scanning system.
#
# REQUIRES: Ubuntu 24.04 (Noble), internet access.
#
# Quick start:
#   git clone https://github.com/azzamjhd/scanner-system.git ~/scanner_ws
#   cd ~/scanner_ws && ./bootstrap.sh
#
set -euo pipefail

# ---- parse arguments ----
HEADLESS=false
for arg in "$@"; do
  if [[ "$arg" == "--headless" ]]; then
    HEADLESS=true
  fi
done

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$WORKSPACE_DIR"
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

# ---- Install / Source ROS 2 Jazzy ----
ROS_INSTALL_PATH="/opt/ros/jazzy"
if [[ ! -f "$ROS_INSTALL_PATH/setup.bash" ]]; then
  log "ROS 2 Jazzy is not installed. Installing..."
  
  sudo apt-get update -y -qq
  sudo apt-get install -y -qq software-properties-common curl gnupg lsb-release
  sudo add-apt-repository universe -y
  
  sudo curl -sSL https://raw.githubusercontent.com/ros2/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  sudo apt-get update -y -qq
  if [[ "$HEADLESS" == "true" ]]; then
    sudo apt-get install -y -qq ros-jazzy-ros-base
  else
    sudo apt-get install -y -qq ros-jazzy-desktop
  fi
  sudo apt-get install -y -qq ros-dev-tools python3-rosdep
fi

# Source ROS 2 for this script
source "$ROS_INSTALL_PATH/setup.bash"

# ---- Install colcon if missing ----
if ! command -v colcon &>/dev/null; then
  log "colcon build tool not found. Installing python3-colcon-common-extensions..."
  sudo apt-get update -y -qq && sudo apt-get install -y -qq python3-colcon-common-extensions
fi

# ---- 0. Pull external dependencies (rplidar_ros) ----
log "Pulling external dependencies (rplidar_ros)…"
sudo apt-get install -y -qq python3-vcstool 2>/dev/null || true
vcs import src < scanner_system.repos 2>/dev/null || true

# ---- 2. rosdep ----
log "Initialising rosdep (if not done)…"
sudo rosdep init 2>/dev/null || true
rosdep update

# ---- 3. System dependencies (rosdep + explicit) ----
log "Installing system dependencies…"

# rosdep handles ROS-level deps
if [[ "$HEADLESS" == "true" ]]; then
  # Skip GUI dependencies in rosdep for headless builds
  rosdep install --from-paths src --ignore-src -y -r \
    --skip-keys='libpcl-all-dev,libpcl-common1.14,libpcl-io1.14,libpcl-filters1.14,libpcl-segmentation1.14,libpcl-sample-consensus1.14,libpcl-search1.14,libpcl-kdtree1.14,python3-pyqt5,python3-pyqtgraph,python3-matplotlib,python3-tk' \
    2>&1 | tail -5
else
  rosdep install --from-paths src --ignore-src -y -r \
    --skip-keys='libpcl-all-dev,libpcl-common1.14,libpcl-io1.14,libpcl-filters1.14,libpcl-segmentation1.14,libpcl-sample-consensus1.14,libpcl-search1.14,libpcl-kdtree1.14' \
    2>&1 | tail -5
fi

# PCL and other core system libraries
if [[ "$HEADLESS" == "true" ]]; then
  sudo apt-get install -y -qq \
    libpcl-dev \
    python3-venv \
    python3-opencv python3-numpy python3-scipy
else
  sudo apt-get install -y -qq \
    libpcl-dev \
    python3-venv \
    python3-opencv python3-numpy python3-scipy python3-matplotlib \
    python3-tk \
    python3-pyqt5 python3-pyqtgraph \
    python3-pyqt6
fi

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

# ---- 5.5 Install micro-ROS Agent & PlatformIO ----
log "Installing micro-ROS Agent…"
ROS_DISTRO=${ROS_DISTRO:-jazzy}

if ! grep -q "source /opt/ros/.*setup.bash" ~/.bashrc; then
  echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source $WORKSPACE_DIR/install/local_setup.bash" ~/.bashrc; then
  echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc
fi

source /opt/ros/$ROS_DISTRO/setup.bash

mkdir -p ~/microros_ws
cd ~/microros_ws
if [[ ! -d "src/micro_ros_setup" ]]; then
  git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install -y python3-pip

colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

if ! grep -q "source ~/microros_ws/install/local_setup.bash" ~/.bashrc; then
  echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
fi

log "Installing PlatformIO…"
cd "$WORKSPACE_DIR"
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm -f get-platformio.py

export PATH="$PATH:$HOME/.platformio/penv/bin"

if ! grep -q "export PATH=\$PATH:\$HOME/.platformio/penv/bin" ~/.bashrc; then
  echo 'export PATH=$PATH:$HOME/.platformio/penv/bin' >> ~/.bashrc
fi

cd "$WORKSPACE_DIR/src/lidar_camera_fusion/gantry_2axis_firmware"
pio init && pio run

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
if [[ "$HEADLESS" == "true" ]]; then
  echo "    ros2 launch lidar_camera_fusion hardware_only.launch.py enable_camera:=false"
  echo ""
  echo "  To start the SSH curses control panel, run:"
  echo "    ros2 run lidar_camera_fusion fusion_tui"
else
  echo "    ros2 launch lidar_camera_fusion full_system.launch.py"
fi
echo ""
echo "  Without hardware (simulated encoder):"
if [[ "$HEADLESS" == "true" ]]; then
  echo "    ros2 launch lidar_camera_fusion hardware_only.launch.py \\"
  echo "      mcu_port:='' lidar_port:=/dev/null scan_mode:=Standard \\"
  echo "      enable_camera:=false enable_scan_cycle:=false"
else
  echo "    ros2 launch lidar_camera_fusion full_system.launch.py \\"
  echo "      mcu_port:='' lidar_port:=/dev/null scan_mode:=Standard \\"
  echo "      simulate_encoder:=true"
fi
echo ""
echo "  If using MediaPipe nodes, activate the venv first:"
echo "    source $MEDIAPIPE_VENV/bin/activate"
echo "${GREEN}============================================${NC}"
