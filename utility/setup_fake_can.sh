#!/usr/bin/env bash
set -euo pipefail

# Userspace CAN simulation for environments without SocketCAN/vcan (e.g. some WSL kernels).
#
# This relies on python-can's "virtual" backend plus a small fake responder inside robot_controller
# (enabled via SCARA_CAN_FAKE=1).

CHANNEL="${1:-scara_virtual}"

cat <<EOF
# Run these in your terminal:

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Enable userspace CAN (no kernel vcan needed)
export SCARA_CAN_FORCE=1
export SCARA_CAN_INTERFACE=virtual
export SCARA_CAN_CHANNEL=${CHANNEL}
export SCARA_CAN_FAKE=1

ros2 run robot_controller controller
EOF
