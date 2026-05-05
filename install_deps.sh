#!/usr/bin/env bash
set -euo pipefail

# ===============================
# Global configuration
# ===============================

# Target catkin workspace (can be overridden by first argument)
WS_DIR="${1:-$HOME/catkin_ws}"
SRC_DIR="${WS_DIR}/src"

# Detect if sudo is needed
if [ "$(id -u)" -eq 0 ]; then
  SUDO=""
else
  SUDO="sudo"
fi

echo ">> Target workspace: ${WS_DIR}"
echo ">> Using SUDO='${SUDO}'"
echo

# ===============================
# Update APT package index
# ===============================
echo ">> Updating package index..."
$SUDO apt-get update

# ===============================
# Install ROS-related dependencies
# ===============================
echo ">> Installing ROS packages for Gazebo plugins, localization and control..."
$SUDO apt-get install -y \
  ros-noetic-hector-gazebo-plugins \
  ros-noetic-robot-localization \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-realsense2-description

# (Opcional, mas normalmente já vem com desktop-full; se quiser usar a câmera real)
# $SUDO apt-get install -y ros-noetic-realsense2-camera

# ===============================
# Install additional system libraries
# ===============================
echo ">> Installing additional system libraries..."
$SUDO apt-get install -y \
  libignition-math4-dev \
  git \
  python3-pip

# ===============================
# Python dependencies (pip)
# ===============================
echo ">> Installing Python packages with pip..."
$SUDO python3 -m pip install --upgrade pip
$SUDO python3 -m pip install bluerobotics-ping

# ===============================
# Optional: other sensor-related dependencies
# (add or uncomment as needed)
# ===============================
echo ">> (Optional) Installing additional sensor-related dependencies (edit this section as needed)..."
# Exemplo, se quiser garantir ferramentas gráficas:
# $SUDO apt-get install -y \
#   ros-noetic-joint-state-publisher \
#   ros-noetic-joint-state-publisher-gui \
#   ros-noetic-rviz

# ===============================
# Prepare workspace
# ===============================
echo ">> Preparing workspace at: ${WS_DIR}"
mkdir -p "${SRC_DIR}"
cd "${SRC_DIR}"

# ===============================
# Clone livox_laser_simulation
# ===============================
if [ ! -d "livox_laser_simulation" ]; then
  echo ">> Cloning livox_laser_simulation..."
  git clone https://github.com/Livox-SDK/livox_laser_simulation
else
  echo ">> Directory livox_laser_simulation already exists, skipping clone."
fi

echo ">> Patching livox_laser_simulation CMakeLists.txt to use C++17 instead of C++11..."
cd "${SRC_DIR}/livox_laser_simulation"
sed -i 's/-std=c++11/-std=c++17/gi' CMakeLists.txt

# Return to src directory for the next clones
cd "${SRC_DIR}"

# ===============================
# Clone ping360_gazebo (with submodules)
# ===============================
if [ ! -d "ping360_gazebo" ]; then
  echo ">> Cloning ping360_gazebo (with submodules)..."
  git clone --recurse-submodules https://github.com/ttrindader/ping360_gazebo.git
else
  echo ">> Directory ping360_gazebo already exists, updating submodules..."
fi

cd "${SRC_DIR}/ping360_gazebo"
git submodule update --init --recursive

# ===============================
# Make ping360_gazebo helper scripts executable
# ===============================
SCRIPTS_DIR="${SRC_DIR}/ping360_gazebo/ping360_gazebo_plugin/scripts"

if [ -d "${SCRIPTS_DIR}" ]; then
  echo ">> Setting executable permissions on ping360_gazebo helper scripts..."
  chmod +x "${SCRIPTS_DIR}/numpy_pc2.py" "${SCRIPTS_DIR}/pcl_gen.py" || true
else
  echo ">> WARNING: Scripts directory not found: ${SCRIPTS_DIR}"
fi

# Back to src in case more repos are added later
cd "${SRC_DIR}"

echo
echo ">> Third-party dependencies installed and sensor repositories cloned."
echo ">> Now you can run rosdep"
#echo "   rosdep update && rosdep install --from-paths src --ignore-src -r -y"

