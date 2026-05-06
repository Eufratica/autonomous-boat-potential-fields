#!/usr/bin/env bash
set -euo pipefail

xhost +local:
sudo -E docker compose up sim
