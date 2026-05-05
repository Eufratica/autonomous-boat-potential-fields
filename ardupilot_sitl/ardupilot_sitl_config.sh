#!/usr/bin/env bash
#
# ardupilot_sitl_config – helper script for ArduPilot SITL configuration
#
#  - Copies the contents of the current directory into the ArduPilot Rover/
#    directory (excluding this script itself).
#  - Ensures the custom SAE_XASV_SIM location is present in Tools/autotest/locations.txt.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "$0")"

echo "=== ArduPilot SITL config helper ==="
echo "[INFO] This script will:"
echo "  - copy the contents of the current directory to the Rover/ folder"
echo "  - register the SAE_XASV_SIM location in Tools/autotest/locations.txt"
echo

# Ask user for ArduPilot path relative to $HOME
read -rp "Enter the path to your ArduPilot repository relative to your home (e.g., ardupilot): " ARDUPILOT_REL

if [[ -z "${ARDUPILOT_REL}" ]]; then
  echo "[ERROR] Empty path. Aborting."
  exit 1
fi

ARDUPILOT_DIR="${HOME}/${ARDUPILOT_REL}"

if [[ ! -d "${ARDUPILOT_DIR}" ]]; then
  echo "[ERROR] Directory '${ARDUPILOT_DIR}' does not exist."
  echo "        Please check the path and try again."
  exit 1
fi

ROVER_DIR="${ARDUPILOT_DIR}/Rover"

if [[ ! -d "${ROVER_DIR}" ]]; then
  echo "[ERROR] Rover directory '${ROVER_DIR}' does not exist."
  echo "        Make sure you provided the root of the ArduPilot repo (e.g., ~/ardupilot)."
  exit 1
fi

echo
echo "[INFO] ArduPilot repo : ${ARDUPILOT_DIR}"
echo "[INFO] Rover folder   : ${ROVER_DIR}"
echo "[INFO] Current folder : ${SCRIPT_DIR}"
echo

# ----------------------------------------------------------------------
# Copy current directory contents to Rover/, excluding this script itself
# ----------------------------------------------------------------------
echo "[STEP 1] Copying current directory contents to Rover/ (excluding this script)..."

cd "${SCRIPT_DIR}"

# Use rsync to copy everything except this script
rsync -av \
  --exclude "${SCRIPT_NAME}" \
  ./ \
  "${ROVER_DIR}/"

echo "[OK] Files copied to ${ROVER_DIR}"
echo

# ----------------------------------------------------------------------
# Ensure SAE_XASV_SIM location is present in Tools/autotest/locations.txt
# ----------------------------------------------------------------------
LOC_FILE="${ARDUPILOT_DIR}/Tools/autotest/locations.txt"
TAG="SAE_XASV_SIM"
LOCATION_LINE="${TAG}=-8.798638,-63.952087, 58, 0"

echo "[STEP 2] Updating locations.txt with ${TAG} location..."

if [[ ! -f "${LOC_FILE}" ]]; then
  echo "[WARN] locations.txt not found at: ${LOC_FILE}"
  echo "[WARN] Creating a new locations.txt file."
  mkdir -p "$(dirname "${LOC_FILE}")"
  echo "# Custom locations for SITL" > "${LOC_FILE}"
fi

# (Opcional) Remover a entrada antiga SAE=, se você quiser "migrar" de vez.
# Descomente as 3 linhas abaixo para ativar.
# if grep -q "^SAE=" "${LOC_FILE}"; then
#   sed -i '/^SAE=/d' "${LOC_FILE}"
# fi

if grep -q "^${TAG}=" "${LOC_FILE}"; then
  echo "[INFO] A line starting with '${TAG}=' already exists in locations.txt."
  echo "[INFO] No new ${TAG} entry was added."
else
  echo "${LOCATION_LINE}" >> "${LOC_FILE}"
  echo "[OK] Added ${TAG} location to locations.txt:"
  echo "     ${LOCATION_LINE}"
fi

echo
echo "=== Done ==="
echo "[INFO] Rover sources updated from: ${SCRIPT_DIR}"
echo "[INFO] locations.txt updated at : ${LOC_FILE}"
