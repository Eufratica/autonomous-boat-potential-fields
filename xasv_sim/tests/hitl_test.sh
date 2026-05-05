#!/usr/bin/env bash
#
# hitl_test.sh – xasv-sim + ArduPilot HITL (Pixhawk) + MAVProxy + ardupilot_hitl
#
# Flow:
#   1) Detect and source the ROS workspace (devel/setup.bash)
#   2) Launch xasv_sim (madeira_river, migbot1)
#   3) Wait until Gazebo is fully up (check /gazebo/link_states)
#   4) Ask the user to connect the Pixhawk board via USB and wait for boot
#   5) Start MAVProxy (master=/dev/ttyACM0 by default) in a separate terminal
#   6) Launch ardupilot_hitl nodes (apm.launch and link.launch)
#   7) Tell the user to open QGroundControl and perform mission/arming steps
#   8) When the user presses ENTER in this terminal or hits CTRL+C, stop everything
#

set -euo pipefail

WORLD_PID=""
APM_PID=""
LINK_PID=""
MAVPROXY_TERM_PID=""

cleanup() {
  # Avoid running cleanup more than once
  trap - INT TERM EXIT

  echo
  echo "[HITL_TEST] Cleaning up..."

  if [[ -n "${LINK_PID}" ]] && kill -0 "${LINK_PID}" 2>/dev/null; then
    echo "[HITL_TEST] Stopping ardupilot_hitl link.launch (pid=${LINK_PID})..."
    kill "${LINK_PID}" 2>/dev/null || true
    wait "${LINK_PID}" 2>/dev/null || true
  fi

  if [[ -n "${APM_PID}" ]] && kill -0 "${APM_PID}" 2>/dev/null; then
    echo "[HITL_TEST] Stopping ardupilot_hitl apm.launch (pid=${APM_PID})..."
    kill "${APM_PID}" 2>/dev/null || true
    wait "${APM_PID}" 2>/dev/null || true
  fi

  if [[ -n "${WORLD_PID}" ]] && kill -0 "${WORLD_PID}" 2>/dev/null; then
    echo "[HITL_TEST] Stopping xasv_sim (pid=${WORLD_PID})..."
    kill "${WORLD_PID}" 2>/dev/null || true
    wait "${WORLD_PID}" 2>/dev/null || true
  fi

  if [[ -n "${MAVPROXY_TERM_PID}" ]] && kill -0 "${MAVPROXY_TERM_PID}" 2>/dev/null; then
    echo "[HITL_TEST] Closing MAVProxy terminal (pid=${MAVPROXY_TERM_PID})..."
    kill "${MAVPROXY_TERM_PID}" 2>/dev/null || true
    wait "${MAVPROXY_TERM_PID}" 2>/dev/null || true
  fi

  # Extra safety: kill any remaining MAVProxy processes started by this user
  echo "[HITL_TEST] Ensuring no MAVProxy process is left running..."
  pkill -f "mavproxy.py" 2>/dev/null || true

  echo "[HITL_TEST] Done."
  exit 0
}

trap cleanup INT TERM EXIT

# ----------------------------------------------------------------------
# 1) Detect catkin workspace and source ROS environment
# ----------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XASV_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== HITL test for migbot1 ==="
echo "[HITL_TEST] Script directory : ${SCRIPT_DIR}"
echo "[HITL_TEST] xasv_sim directory: ${XASV_SIM_DIR}"

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
  echo "[ERROR] Could not find a catkin workspace (no devel/setup.bash above ${SCRIPT_DIR})"
  echo "[ERROR] Expected something like <ws>/devel/setup.bash."
  exit 1
fi

SETUP_FILE="${FOUND_WS}/devel/setup.bash"

echo "[HITL_TEST] Using catkin workspace: ${FOUND_WS}"
echo "[HITL_TEST] Sourcing: ${SETUP_FILE}"
# shellcheck source=/dev/null
source "${SETUP_FILE}"

cd "${SCRIPT_DIR}"

# ----------------------------------------------------------------------
# 2) Start world + robot (xasv_sim)
# ----------------------------------------------------------------------
echo
echo "[1/5] Starting xasv_sim (world=madeira_river, robot=migbot1)..."
roslaunch xasv_sim xasv_sim.launch \
  world_name:=madeira_river \
  robot_name:=migbot1 &
WORLD_PID=$!

echo "[HITL_TEST] xasv_sim launched (pid=${WORLD_PID})."
echo "[HITL_TEST] Waiting a few seconds for Gazebo to start..."
sleep 10

echo
echo "[HITL_TEST] Waiting for Gazebo topics to appear (/gazebo/link_states)..."
# Wait until Gazebo publishes /gazebo/link_states (max ~90s)
if ! timeout 90 bash -c 'until rostopic list 2>/dev/null | grep -q "/gazebo/link_states"; do sleep 1; done'; then
  echo "[ERROR] Timed out waiting for Gazebo (/gazebo/link_states)."
  cleanup
fi

echo "[HITL_TEST] Gazebo appears to be up. You should now see the 'madeira_river' world and 'migbot1' in the GUI."

# ----------------------------------------------------------------------
# 3) Ask for serial device and wait for the board to boot
# ----------------------------------------------------------------------
echo
read -rp "[HITL] Enter autopilot serial device [/dev/ttyACM0]: " SERIAL_DEV
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"

echo "[HITL] Please connect your Pixhawk-class board to USB on ${SERIAL_DEV}."
echo "[HITL] Waiting for ${SERIAL_DEV} to appear..."
while [[ ! -e "${SERIAL_DEV}" ]]; do
  sleep 1
done
echo "[HITL] ${SERIAL_DEV} detected."

echo
echo "[HITL] Wait until the board finishes booting (LEDs stable, no bootloader mode)."
read -rp "[HITL] When the board is fully initialized, press ENTER to continue..." _

# ----------------------------------------------------------------------
# 4) Start MAVProxy (master on the Pixhawk serial port)
# ----------------------------------------------------------------------
echo
echo "[2/5] Starting MAVProxy console on ${SERIAL_DEV}..."

if command -v gnome-terminal >/dev/null 2>&1; then
  gnome-terminal -- bash -c "
    source '${SETUP_FILE}' 2>/dev/null || true
    echo '[MAVPROXY] Starting MAVProxy on ${SERIAL_DEV}...';
    mavproxy.py --mav20 --console \
      --out=127.0.0.1:14550 \
      --out=127.0.0.1:14552 \
      --master=${SERIAL_DEV} \
      --baudrate=115200;
    echo;
    echo '[MAVPROXY] MAVProxy has exited. Press ENTER to close this window.';
    read" &
  MAVPROXY_TERM_PID=$!
  echo "[HITL_TEST] MAVProxy started in a new gnome-terminal (pid=${MAVPROXY_TERM_PID})."
else
  echo "[WARN] gnome-terminal not found."
  echo "[WARN] Please start MAVProxy manually in another terminal with:"
  echo "       source ${SETUP_FILE}"
  echo "       mavproxy.py --mav20 --console \\"
  echo "         --out=127.0.0.1:14550 \\"
  echo "         --out=127.0.0.1:14552 \\"
  echo "         --master=${SERIAL_DEV} \\"
  echo "         --baudrate=115200"
fi

sleep 3

# ----------------------------------------------------------------------
# 5) Start ardupilot_hitl nodes (apm.launch and link.launch)
#    (redirect output to log files to keep this terminal clean)
# ----------------------------------------------------------------------
APM_LOG="${SCRIPT_DIR}/hitl_apm.log"
LINK_LOG="${SCRIPT_DIR}/hitl_link.log"

echo
echo "[3/5] Starting ardupilot_hitl apm.launch (logging to ${APM_LOG})..."
roslaunch ardupilot_hitl apm.launch >"${APM_LOG}" 2>&1 &
APM_PID=$!
echo "[HITL_TEST] ardupilot_hitl apm.launch started (pid=${APM_PID})."
sleep 3

echo
echo "[4/5] Starting ardupilot_hitl link.launch (logging to ${LINK_LOG})..."
roslaunch ardupilot_hitl link.launch >"${LINK_LOG}" 2>&1 &
LINK_PID=$!
echo "[HITL_TEST] ardupilot_hitl link.launch started (pid=${LINK_PID})."
sleep 3

# ----------------------------------------------------------------------
# Final instructions for the user
# ----------------------------------------------------------------------
echo
echo "---------------------------------------------"
echo "[HITL_TEST] HITL environment is up and running."
echo
echo "You now have:"
echo "  - xasv_sim + Gazebo running the 'madeira_river' world with 'migbot1';"
echo "  - Your Pixhawk-class board connected on ${SERIAL_DEV};"
echo "  - MAVProxy bridging the serial link to 127.0.0.1:14550 / 14552;"
echo "  - ardupilot_hitl nodes (apm.launch and link.launch) sending Gazebo sensor data"
echo "    to the autopilot via custom MAVLink messages."
echo
echo "Logs from ardupilot_hitl nodes are stored in:"
echo "  - ${APM_LOG}"
echo "  - ${LINK_LOG}"
echo
echo "Next steps (manual, by the user):"
echo "  - Start QGroundControl (or another GCS)."
echo "  - Connect to the vehicle (typically via UDP 127.0.0.1:14550 or through MAVProxy)."
echo "  - Check that the board is recognized and that pre-arm checks are satisfied."
echo "  - Load the desired mission, select the appropriate mode (e.g., AUTO) and arm"
echo "    the rover only when you are ready to run the HITL experiment."
echo
echo "[HITL_TEST] When you want to stop the HITL test, press ENTER in this terminal."
echo "---------------------------------------------"
read -r _

cleanup

