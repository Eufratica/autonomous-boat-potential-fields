#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-/opt/ardupilot}"
XASV_REPO="${XASV_REPO:-/work/xasv-simu}"
OVERLAY_DIR="${XASV_REPO}/ardupilot_sitl"

if [[ -d "${OVERLAY_DIR}" ]]; then
  echo "[sitl] Applying xasv-simu SITL overlay from ${OVERLAY_DIR}"
  rsync -a --exclude 'ardupilot_sitl_config.sh' "${OVERLAY_DIR}/" "${ARDUPILOT_DIR}/Rover/"
fi

LOC_FILE="${ARDUPILOT_DIR}/Tools/autotest/locations.txt"
TAG="SAE_XASV_SIM"
LOCATION_LINE="${TAG}=-8.798638,-63.952087,58,0"

mkdir -p "$(dirname "${LOC_FILE}")"
touch "${LOC_FILE}"
grep -q "^${TAG}=" "${LOC_FILE}" || echo "${LOCATION_LINE}" >> "${LOC_FILE}"

if [[ "${REBUILD_SITL:-0}" == "1" ]]; then
  echo "[sitl] Rebuilding ArduPilot Rover SITL"
  cd "${ARDUPILOT_DIR}"
  ./waf rover
fi

exec "$@"
