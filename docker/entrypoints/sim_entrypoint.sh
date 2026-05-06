#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash

CATKIN_WS="${CATKIN_WS:-/ws}"
XASV_SRC="${XASV_SRC:-${CATKIN_WS}/src/xasv-simu}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export GAZEBO_MODEL_PATH="${XASV_SRC}/xasv_sim/models:${XASV_SRC}/robots/migbot_gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="${CATKIN_WS}/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_PACKAGE_PATH="${CATKIN_WS}/src:${ROS_PACKAGE_PATH:-}"

[[ -f "${CATKIN_WS}/devel/setup.bash" ]] && source "${CATKIN_WS}/devel/setup.bash" || true

exec "$@"
