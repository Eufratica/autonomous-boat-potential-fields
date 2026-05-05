#!/usr/bin/env python3
"""
sitl_wait_ahrs.py - Wait until ArduPilot SITL sensors and AHRS/EKF are healthy.

This script connects directly to the SITL MAVLink endpoint (default: udp:127.0.0.1:14550)
and blocks until:
  - Key sensors (gyro, accel, mag, baro, GPS) are present, enabled and healthy
    according to SYS_STATUS.onboard_control_sensors_* bitmasks.
  - EKF/AHRS reports a good attitude estimate (EKF_STATUS_REPORT.flags, ESTIMATOR_ATTITUDE bit).

No MAVROS is used here; everything is done via pymavlink.
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink is not installed. Please install it, e.g.:", file=sys.stderr)
    print("        python3 -m pip install pymavlink", file=sys.stderr)
    sys.exit(1)

# MAV_SYS_STATUS_SENSOR bit values (see MAVLink common.xml)
MAV_SYS_STATUS_SENSOR_3D_GYRO              = 1      # 0x01
MAV_SYS_STATUS_SENSOR_3D_ACCEL             = 2      # 0x02
MAV_SYS_STATUS_SENSOR_3D_MAG               = 4      # 0x04
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE    = 8      # 0x08
MAV_SYS_STATUS_SENSOR_GPS                  = 32     # 0x20

# We consider these as "key" sensors for AHRS/EKF
SENSOR_MASK = (
    MAV_SYS_STATUS_SENSOR_3D_GYRO
    | MAV_SYS_STATUS_SENSOR_3D_ACCEL
    | MAV_SYS_STATUS_SENSOR_3D_MAG
    | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    | MAV_SYS_STATUS_SENSOR_GPS
)

# EKF_STATUS_FLAGS (from MAVLink docs): we care about attitude flag
ESTIMATOR_ATTITUDE = 1  # True if the attitude estimate is good


def wait_for_ahrs(conn_str: str, timeout: float) -> bool:
    """
    Connect to SITL and wait until sensors and AHRS/EKF are healthy.

    :param conn_str: MAVLink connection string, e.g. "udp:127.0.0.1:14550"
    :param timeout:  timeout in seconds
    :return: True if everything is healthy, False on timeout
    """
    print(f"[SITL] Connecting to {conn_str} ...")
    master = mavutil.mavlink_connection(conn_str)

    # Wait for first HEARTBEAT
    print("[SITL] Waiting for HEARTBEAT from ArduPilot SITL...")
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        print("[ERROR] Timeout waiting for HEARTBEAT. Is gzboat.sh running?", file=sys.stderr)
        return False

    print(
        f"[SITL] HEARTBEAT received from system {master.target_system}, "
        f"component {master.target_component}."
    )

    start = time.time()
    have_good_sys = False
    have_good_ekf = False

    print("[SITL] Waiting for healthy sensors and EKF/AHRS...")

    while time.time() - start < timeout:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == "SYS_STATUS":
            present = msg.onboard_control_sensors_present
            enabled = msg.onboard_control_sensors_enabled
            health  = msg.onboard_control_sensors_health

            # Check if key sensors are present, enabled and healthy
            if (
                (present & SENSOR_MASK) == SENSOR_MASK
                and (enabled & SENSOR_MASK) == SENSOR_MASK
                and (health  & SENSOR_MASK) == SENSOR_MASK
            ):
                if not have_good_sys:
                    print("[SITL] SYS_STATUS: key sensors are present/enabled/healthy.")
                have_good_sys = True

        elif mtype in ("EKF_STATUS_REPORT", "ESTIMATOR_STATUS"):
            flags = msg.flags
            if flags & ESTIMATOR_ATTITUDE:
                if not have_good_ekf:
                    print("[SITL] EKF/AHRS: attitude estimate is reported as GOOD.")
                have_good_ekf = True

        if have_good_sys and have_good_ekf:
            print("[SITL] All checks passed: sensors and AHRS/EKF appear healthy.")
            print("[SITL] You can now safely load the test mission and switch to AUTO.")
            return True

    print(
        "[ERROR] Timeout while waiting for healthy sensors / AHRS. "
        "Check SITL status and pre-arm checks.",
        file=sys.stderr,
    )
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Wait for ArduPilot SITL sensors and AHRS/EKF to become healthy."
    )
    parser.add_argument(
        "--conn",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string (default: udp:127.0.0.1:14550)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="Timeout in seconds (default: 180)",
    )
    args = parser.parse_args()

    ok = wait_for_ahrs(args.conn, args.timeout)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

