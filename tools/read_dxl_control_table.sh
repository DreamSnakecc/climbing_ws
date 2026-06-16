#!/usr/bin/env bash
#
# Read selected Dynamixel X-series control table parameters for IDs 2/4/6/8.
#
# Stop jetson_bringup / multi_dxl_node before running, because this script opens
# /dev/ttyUSB_l and /dev/ttyUSB_r directly with Dynamixel SDK.

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"

usage() {
    cat <<'EOF'
Usage:
    ./tools/read_dxl_control_table.sh [-- <read_dxl_control_table.py args...>]

Examples:
    ./tools/read_dxl_control_table.sh
    ./tools/read_dxl_control_table.sh -- --output-dir test_logs
    ./tools/read_dxl_control_table.sh -- --left-ids 2,8 --right-ids 4,6

Default reads:
    left_board  /dev/ttyUSB_l IDs 2,8
    right_board /dev/ttyUSB_r IDs 4,6

Important:
    Stop multi_dxl_node before running, otherwise the serial ports may be busy.
EOF
}

if [[ "${1-}" == "-h" || "${1-}" == "--help" ]]; then
    usage
    exit 0
fi

PY_ARGS=()
SAW_DASHDASH=0
for arg in "$@"; do
    if [[ "$SAW_DASHDASH" -eq 1 ]]; then
        PY_ARGS+=("$arg")
    elif [[ "$arg" == "--" ]]; then
        SAW_DASHDASH=1
    else
        echo "Unrecognized argument: $arg (use '--' to forward args)" >&2
        usage
        exit 1
    fi
done

source_environment() {
    if [[ -n "$ROS_SETUP" ]]; then
        if [[ ! -f "$ROS_SETUP" ]]; then
            echo "ROS_SETUP does not exist: $ROS_SETUP" >&2
            exit 1
        fi
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
    elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "/opt/ros/noetic/setup.bash"
    fi

    if [[ -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "$WORKSPACE_DIR/devel/setup.bash"
    fi
}

source_environment

if command -v rosrun >/dev/null 2>&1 && rosrun climbing_hw_bridge read_dxl_control_table.py --help >/dev/null 2>&1; then
    rosrun climbing_hw_bridge read_dxl_control_table.py "${PY_ARGS[@]}"
else
    python3 "$WORKSPACE_DIR/src/climbing_hw_bridge/scripts/read_dxl_control_table.py" "${PY_ARGS[@]}"
fi
