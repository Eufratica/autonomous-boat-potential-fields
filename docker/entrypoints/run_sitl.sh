#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-/opt/ardupilot}"
SITL_BIN="${ARDUPILOT_DIR}/build/sitl/bin/ardurover"
DEFAULTS="../Tools/autotest/default_params/rover.parm,../Tools/autotest/default_params/rover-skid.parm,mig_sitl.params"

if [[ ! -x "${SITL_BIN}" ]]; then
  echo "[sitl] Missing SITL binary: ${SITL_BIN}" >&2
  exit 1
fi

cd "${ARDUPILOT_DIR}/Rover"

exec "${SITL_BIN}" \
  -S \
  --model gazebo-rover \
  --speedup "${SITL_SPEEDUP:-10}" \
  --slave 0 \
  --defaults "${SITL_DEFAULTS:-$DEFAULTS}" \
  --sim-address="${SITL_SIM_ADDRESS:-127.0.0.1}" \
  -I"${SITL_INSTANCE:-0}" \
  --home "${SITL_HOME:--8.798638,-63.952087,58.0,0.0}"
