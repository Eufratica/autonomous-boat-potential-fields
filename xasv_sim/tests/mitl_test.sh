#!/usr/bin/env bash
#
# mitl_test.sh – MITL workflow for migbot1
#
# 1) Detect and source the catkin workspace (devel/setup.bash)
# 2) Start xasv_sim world (madeira_river, migbot1)
# 3) Start control allocation node (migbot_allocation)
# 4) Start teleoperation node (teleop_wrench_keyboard) in a separate terminal
# 5) Record rosbag with relevant topics
# 6) On user request, stop recording and generate mission via rosgps2mission.py
# 7) Stop all nodes started by this script
#

set -e

# Directory where this script lives (xasv_sim/tests)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== MITL test for migbot1 ==="
echo "[MITL] Script directory: ${SCRIPT_DIR}"

# xasv_sim package root and mission output directory
XASV_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MISSION_DIR="${XASV_SIM_DIR}/data/mission"

# Ensure mission output directory exists
mkdir -p "${MISSION_DIR}"

# ----------------------------------------------------------------------
# 1) Detect catkin workspace (walk upwards until devel/setup.bash is found)
# ----------------------------------------------------------------------
WS_DIR="${SCRIPT_DIR}"
FOUND_WS=""

while [[ "${WS_DIR}" != "/" ]]; do
  if [[ -f "${WS_DIR}/devel/setup.bash" ]]; then
    FOUND_WS="${WS_DIR}"
    break
  fi
  WS_DIR="$(dirname "${WS_DIR}")"
done

if [[ -z "${FOUND_WS}" ]]; then
  echo "[ERROR] Could not find a catkin workspace (no devel/setup.bash found above ${SCRIPT_DIR})"
  echo "[ERROR] Expected something like <ws>/devel/setup.bash."
  exit 1
fi

SETUP_FILE="${FOUND_WS}/devel/setup.bash"

echo "[MITL] Using catkin workspace: ${FOUND_WS}"
echo "[MITL] Sourcing: ${SETUP_FILE}"
# shellcheck source=/dev/null
source "${SETUP_FILE}"

# Now ROS environment is configured for this shell
cd "${SCRIPT_DIR}"

# Timestamp for unique filenames (bags, missions, logs)
DATE_STAMP="$(date +%Y%m%d_%H%M%S)"

BAG_BASENAME="mitl_migbot1"
BAG_NAME="${BAG_BASENAME}_${DATE_STAMP}.bag"
MISSION_OUT="${MISSION_DIR}/${BAG_BASENAME}_${DATE_STAMP}.plan"
ERR_LOG="rosbag_record_${DATE_STAMP}.log"

# Parameters for mission generation
MISSION_POINTS=20
MISSION_FORMAT="plan"                  # {waypoint,plan}
#MISSION_TOPIC="/mavros/global_position/global"
MISSION_TOPIC="/fix"
MISSION_ALT=58.0

# ----------------------------------------------------------------------
# 2) Start world + robot (xasv_sim)
# ----------------------------------------------------------------------
echo
echo "[1/4] Starting xasv_sim (world=madeira_river, robot=migbot1, ardupilot=false)..."
roslaunch xasv_sim xasv_sim.launch world_name:=madeira_river robot_name:=migbot1 ardupilot:=false &
WORLD_PID=$!

sleep 8

# ----------------------------------------------------------------------
# 3) Start control allocation node
# ----------------------------------------------------------------------
echo "[2/4] Starting control allocation node (migbot_allocation) in /migbot1..."
rosrun migbot_allocation migbot_allocation_node __ns:=/migbot1 &
ALLOC_PID=$!

sleep 3

# ----------------------------------------------------------------------
# 4) Start teleoperation node (keyboard) in a separate terminal
# ----------------------------------------------------------------------
echo "[3/4] Starting teleop node (teleop_wrench_keyboard) in /migbot1 (separate terminal)..."

TELEOP_TERM_PID=""

if command -v gnome-terminal >/dev/null 2>&1; then
  gnome-terminal -- bash -c "
    source '${SETUP_FILE}' 2>/dev/null || true;
    echo '[TELEOP] Starting teleop_wrench_keyboard in namespace /migbot1...';
    rosrun teleop_wrench_keyboard teleop_wrench_keyboard.py __ns:=/migbot1;
    echo;
    echo '[TELEOP] teleop node has exited. Press ENTER to close this window.';
    read" &
  TELEOP_TERM_PID=$!
  echo "[MITL] Teleop started in a new gnome-terminal (pid=${TELEOP_TERM_PID})."
else
  echo "[WARN] gnome-terminal not found. Please start teleop manually in another terminal:"
  echo "       source ${SETUP_FILE}"
  echo "       rosrun teleop_wrench_keyboard teleop_wrench_keyboard.py __ns:=/migbot1"
fi

sleep 2

# ----------------------------------------------------------------------
# 5) Start rosbag recording in background (no console spam)
# ----------------------------------------------------------------------
echo "[4/4] Starting rosbag recording..."
echo "[MITL] Bag file : ${BAG_NAME}"
echo "[MITL] Log file : ${ERR_LOG}"

rosbag record -O "${BAG_NAME}" \
  /fix \
  /fix_velocity \
  /migbot1/Engine_helice_1_effort_controller/command \
  /migbot1/Engine_helice_2_effort_controller/command \
  /migbot1/Engine_helice_3_effort_controller/command \
  /migbot1/Engine_helice_4_effort_controller/command \
  /migbot1/Engine_helice_5_effort_controller/command \
  /migbot1/Engine_helice_6_effort_controller/command \
  /migbot1/Wrench \
  /migbot1/imu_data \
  /livox/pcl \
  >"${ERR_LOG}" 2>&1 &
BAG_PID=$!

echo
echo "[MITL] rosbag is now recording in the background (pid=${BAG_PID})."
echo "[MITL] Use the teleop window to drive migbot1 inside the simulated river environment."
echo "[MITL] When you finish the test, come back to THIS terminal and press ENTER."
echo
read -p '>>> Press ENTER here to stop recording and generate the mission... ' _

# ----------------------------------------------------------------------
# 6) Stop rosbag and generate mission
# ----------------------------------------------------------------------
echo
echo "[MITL] Stopping rosbag (pid=${BAG_PID})..."
kill -SIGINT "${BAG_PID}" 2>/dev/null || true
wait "${BAG_PID}" 2>/dev/null || true
echo "[MITL] rosbag finished. File: ${BAG_NAME}"
echo "[MITL] rosbag stdout/stderr was written to ${ERR_LOG}"

echo
echo "[MITL] Generating mission file from rosbag using rosgps2mission.py..."
rosrun xasv_sim rosgps2mission.py \
  --bag "${BAG_NAME}" \
  --topic "${MISSION_TOPIC}" \
  --points "${MISSION_POINTS}" \
  --format "${MISSION_FORMAT}" \
  --output "${MISSION_OUT}" \
  --alt "${MISSION_ALT}"

echo
echo "[MITL] Mission file generated: ${MISSION_OUT}"

# ----------------------------------------------------------------------
# 7) Stop all nodes started by this script
# ----------------------------------------------------------------------
echo
echo "[MITL] Stopping launched nodes..."

echo "[MITL] Killing xasv_sim launch (pid=${WORLD_PID})..."
kill "${WORLD_PID}" 2>/dev/null || true
wait "${WORLD_PID}" 2>/dev/null || true

echo "[MITL] Killing migbot_allocation node (pid=${ALLOC_PID})..."
kill "${ALLOC_PID}" 2>/dev/null || true
wait "${ALLOC_PID}" 2>/dev/null || true

if [[ -n "${TELEOP_TERM_PID}" ]]; then
  echo "[MITL] Killing teleop terminal (pid=${TELEOP_TERM_PID})..."
  kill "${TELEOP_TERM_PID}" 2>/dev/null || true
fi

echo
echo "=== MITL test finished ==="
echo "[MITL] All nodes launched by this script have been requested to stop."
echo "[MITL] Mission stored under: ${MISSION_DIR}"
echo "[MITL] You can now close any remaining GUI windows (e.g., Gazebo, rviz, QGC) manually if needed."

