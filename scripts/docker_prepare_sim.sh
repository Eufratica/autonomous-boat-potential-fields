#!/usr/bin/env bash
set -euo pipefail

mkdir -p .docker/ws/src .docker/sitl/ccache
sudo chown -R "$USER":"$USER" .docker || true

sudo docker compose run --rm --entrypoint bash sim -lc '
source /opt/ros/noetic/setup.bash
mkdir -p /ws/src
cd /ws

catkin init || true
catkin config --workspace /ws \
  --extend /opt/ros/noetic \
  --skiplist ardupilot_hitl \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

catkin build --workspace /ws
'
