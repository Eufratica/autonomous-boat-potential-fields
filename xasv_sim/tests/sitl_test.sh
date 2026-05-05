#!/usr/bin/env bash
#
# sitl_test.sh – xasv-sim + ArduPilot Rover SITL + MAVProxy console
#                  with AHRS/EKF health check (all driven from bash)
#
# Flow:
#   1) Detect and source the ROS workspace (devel/setup.bash)
#   2) Launch xasv_sim (madeira_river, migbot1, ardupilot=true)
#   3) Start ArduPilot Rover SITL directly (ardurover, model gazebo-rover)
#   4) Wait until sensors + AHRS/EKF are healthy (pymavlink, TCP 127.0.0.1:5760)
#   5) Start MAVProxy console (no auto mission, no AUTO, no arming)
#   6) When you quit MAVProxy or press CTRL+C, stop SITL and Gazebo
#

set -euo pipefail

WORLD_PID=""
SITL_PID=""

cleanup() {
  echo
  echo "[SITL_TEST] Cleaning up..."

  if [[ -n "${SITL_PID}" ]] && kill -0 "${SITL_PID}" 2>/dev/null; then
    echo "[SITL_TEST] Stopping ArduPilot SITL (pid=${SITL_PID})..."
    kill "${SITL_PID}" 2>/dev/null || true
    wait "${SITL_PID}" 2>/dev/null || true
  fi

  if [[ -n "${WORLD_PID}" ]] && kill -0 "${WORLD_PID}" 2>/dev/null; then
    echo "[SITL_TEST] Stopping Gazebo world (pid=${WORLD_PID})..."
    kill "${WORLD_PID}" 2>/dev/null || true
    wait "${WORLD_PID}" 2>/dev/null || true
  fi

  echo "[SITL_TEST] Done."
  exit 0
}

trap cleanup INT TERM

# ----------------------------------------------------------------------
# 0) Rodar os comandos pedidos (sem falhar se não existir)
# ----------------------------------------------------------------------
# shellcheck source=/dev/null
source "${HOME}/.bashrc" 2>/dev/null || true

if [[ -f "devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "devel/setup.bash"
fi

# shellcheck source=/dev/null
. "${HOME}/.profile" 2>/dev/null || true

# ----------------------------------------------------------------------
# 1) Detect catkin workspace and source ROS environment
# ----------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XASV_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== SITL test for migbot1 ==="
echo "[SITL_TEST] Script directory : ${SCRIPT_DIR}"
echo "[SITL_TEST] xasv_sim directory: ${XASV_SIM_DIR}"

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

echo "[SITL_TEST] Using catkin workspace: ${FOUND_WS}"
echo "[SITL_TEST] Sourcing: ${SETUP_FILE}"
# shellcheck source=/dev/null
source "${SETUP_FILE}"

cd "${SCRIPT_DIR}"

# ----------------------------------------------------------------------
# 2) Start world + robot (xasv_sim with ArduPilot bridges enabled)
# ----------------------------------------------------------------------
echo
echo "[1/4] Starting xasv_sim (world=madeira_river, robot=migbot1, ardupilot=true)..."
roslaunch xasv_sim xasv_sim.launch \
  world_name:=madeira_river \
  robot_name:=migbot1 \
  ardupilot:=true &
WORLD_PID=$!

echo "[SITL_TEST] Gazebo world launched (pid=${WORLD_PID})."
echo "[SITL_TEST] Waiting a few seconds for Gazebo to stabilize..."
sleep 8

# ----------------------------------------------------------------------
# 3) Ask for ArduPilot root (a partir da HOME) and start Rover SITL (ardurover)
# ----------------------------------------------------------------------
if [[ $# -ge 1 ]]; then
  ARDUPILOT_IN="$1"
else
  read -rp "[SITL] Enter ArduPilot path RELATIVE to \$HOME (e.g. ardupilot_new3/ardupilot): " ARDUPILOT_IN
fi

# Se vier absoluto (/...) ou com ~, respeita/expande; senão, assume relativo à HOME
if [[ "${ARDUPILOT_IN}" == /* ]]; then
  ARDUPILOT_DIR="${ARDUPILOT_IN}"
else
  # Expande "~" se usar
  ARDUPILOT_IN="${ARDUPILOT_IN/#\~/$HOME}"
  if [[ "${ARDUPILOT_IN}" == "$HOME"* ]]; then
    ARDUPILOT_DIR="${ARDUPILOT_IN}"
  else
    ARDUPILOT_DIR="${HOME}/${ARDUPILOT_IN}"
  fi
fi

if [[ ! -d "${ARDUPILOT_DIR}" ]]; then
  echo "[ERROR] ArduPilot directory not found: ${ARDUPILOT_DIR}"
  cleanup
fi

SITL_BIN="${ARDUPILOT_DIR}/build/sitl/bin/ardurover"
ROVER_DIR="${ARDUPILOT_DIR}/Rover"

if [[ ! -x "${SITL_BIN}" ]]; then
  echo "[ERROR] SITL binary not found or not executable:"
  echo "        ${SITL_BIN}"
  echo "        Build it with:"
  echo "          cd ${ARDUPILOT_DIR}"
  echo "          ./waf configure --board sitl"
  echo "          ./waf rover"
  cleanup
fi

if [[ ! -d "${ROVER_DIR}" ]]; then
  echo "[ERROR] Rover directory not found at: ${ROVER_DIR}"
  cleanup
fi

echo
echo "[2/4] Starting ArduPilot Rover SITL (gazebo-rover model)..."
(
  cd "${ROVER_DIR}"

  "${SITL_BIN}" \
    -S \
    --model gazebo-rover \
    --speedup 10 \
    --slave 0 \
    --defaults ../Tools/autotest/default_params/rover.parm,../Tools/autotest/default_params/rover-skid.parm,mig_sitl.params \
    --sim-address=127.0.0.1 \
    -I0 \
    --home -8.798638,-63.952087,58.0,0.0
) &
SITL_PID=$!

echo "[SITL_TEST] Rover SITL started (pid=${SITL_PID})."
echo "[SITL_TEST] Waiting a bit for SITL to start listening on TCP 5760..."
sleep 5

# ----------------------------------------------------------------------
# 4) Wait until sensors and AHRS/EKF are healthy (pymavlink inline)
# ----------------------------------------------------------------------
echo
echo "[3/4] Checking sensors and AHRS/EKF health via pymavlink..."
echo "[SITL_TEST] Using connection: tcp:127.0.0.1:5760"

set +e
python3 - << 'EOF'
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("[PY][ERROR] pymavlink is not installed. Please run:", file=sys.stderr)
    print("        python3 -m pip install pymavlink", file=sys.stderr)
    sys.exit(1)

conn_str = "tcp:127.0.0.1:5760"
timeout  = 180.0

print(f"[PY] Connecting to {conn_str} ...")
master = mavutil.mavlink_connection(conn_str)

print("[PY] Waiting for HEARTBEAT from ArduPilot SITL...")
hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
if hb is None:
    print("[PY][ERROR] Timeout waiting for HEARTBEAT. Is SITL running?", file=sys.stderr)
    sys.exit(2)

print(f"[PY] HEARTBEAT from system {master.target_system}, component {master.target_component}.")

# MAV_SYS_STATUS_SENSOR bits (from MAVLink common.xml)
MAV_SYS_STATUS_SENSOR_3D_GYRO           = 1
MAV_SYS_STATUS_SENSOR_3D_ACCEL          = 2
MAV_SYS_STATUS_SENSOR_3D_MAG            = 4
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8
MAV_SYS_STATUS_SENSOR_GPS               = 32

SENSOR_MASK = (
    MAV_SYS_STATUS_SENSOR_3D_GYRO
    | MAV_SYS_STATUS_SENSOR_3D_ACCEL
    | MAV_SYS_STATUS_SENSOR_3D_MAG
    | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    | MAV_SYS_STATUS_SENSOR_GPS
)

ESTIMATOR_ATTITUDE = 1  # EKF_STATUS_FLAGS: attitude estimate is good

have_sys = False
have_ekf = False

print("[PY] Waiting for healthy sensors and EKF/AHRS...")

start = time.time()
while time.time() - start < timeout:
    msg = master.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue

    mtype = msg.get_type()

    if mtype == "SYS_STATUS":
        present = msg.onboard_control_sensors_present
        enabled = msg.onboard_control_sensors_enabled
        health  = msg.onboard_control_sensors_health

        if (
            (present & SENSOR_MASK) == SENSOR_MASK
            and (enabled & SENSOR_MASK) == SENSOR_MASK
            and (health  & SENSOR_MASK) == SENSOR_MASK
        ):
            if not have_sys:
                print("[PY] SYS_STATUS: key sensors are present/enabled/healthy.")
            have_sys = True

    elif mtype in ("EKF_STATUS_REPORT", "ESTIMATOR_STATUS"):
        flags = msg.flags
        if flags & ESTIMATOR_ATTITUDE:
            if not have_ekf:
                print("[PY] EKF/AHRS: attitude estimate is GOOD.")
            have_ekf = True

    if have_sys and have_ekf:
        print("[PY] All checks passed: sensors and AHRS/EKF appear healthy.")
        sys.exit(0)

print("[PY][ERROR] Timeout waiting for healthy sensors / EKF/AHRS.", file=sys.stderr)
sys.exit(3)
EOF
PY_STATUS=$?
set -e

if [[ ${PY_STATUS} -ne 0 ]]; then
  echo "[ERROR] Sensors / AHRS did not become healthy (python exit code=${PY_STATUS})."
  echo "        Check SITL logs and pre-arm checks."
  cleanup
fi

echo "[SITL_TEST] AHRS/EKF and sensors reported as healthy."

# ----------------------------------------------------------------------
# 5) Start MAVProxy in THIS terminal (foreground), user handles mission/arming
# ----------------------------------------------------------------------
echo
echo "[4/4] Starting MAVProxy console (no auto mission, no AUTO, no arming)..."
echo
echo "[SITL_TEST] MAVProxy command:"
echo "  mavproxy.py --mav20 --console --out=127.0.0.1:14550 --out=127.0.0.1:14552 --master=tcp:127.0.0.1:5760"
echo
echo "[SITL_TEST] Next steps (manual, by the user):"
echo "  - Start QGroundControl (or another GCS) and connect to:"
echo "      * TCP 127.0.0.1:5760  (same as MAVProxy); or"
echo "      * UDP 127.0.0.1:14550 (if configured)."
echo "  - Wait until all pre-arm checks are green in the GCS."
echo "  - Load the desired mission (e.g., a .plan file) using QGroundControl."
echo "  - When ready, set mode to AUTO (or another mode) and ARM the rover from the GCS."
echo
echo "[SITL_TEST] When you quit MAVProxy (or press CTRL+C), SITL and Gazebo will be stopped."

set +e
mavproxy.py --mav20 --console \
  --out=127.0.0.1:14550 \
  --out=127.0.0.1:14552 \
  --master=tcp:127.0.0.1:5760
MP_STATUS=$?
set -e

echo
echo "[SITL_TEST] MAVProxy exited with status ${MP_STATUS}."
cleanup

