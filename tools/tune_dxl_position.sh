#!/usr/bin/env bash
# Resumable Dynamixel tuning wrapper.  See --help for the four stages.

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"

if [[ -n "$ROS_SETUP" ]]; then
    source "$ROS_SETUP"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    source /opt/ros/noetic/setup.bash
fi
source "$WORKSPACE_DIR/devel/setup.bash"
cd "$WORKSPACE_DIR"

if [[ "${1-}" == "--" ]]; then
    shift
fi
rosrun climbing_hw_bridge tune_dxl_position.py "$@"
