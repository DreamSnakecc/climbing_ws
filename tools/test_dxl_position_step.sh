#!/usr/bin/env bash
#
# Direct Dynamixel position-step diagnostic wrapper.
#
# Recommended bringup:
#   roslaunch climbing_bringup jetson_bringup.launch \
#     enable_auto_position_commands:=false \
#     enable_auto_mode_switching:=false \
#     enable_auto_current_control:=false
#
# Then run:
#   ./tools/test_dxl_position_step.sh -- --step-ticks 50 --ramp-steps 1
#   ./tools/test_dxl_position_step.sh -- --no-confirm

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
NODE_WAIT_TIMEOUT="${NODE_WAIT_TIMEOUT:-30}"

usage() {
    cat <<'EOF'
Usage:
    ./tools/test_dxl_position_step.sh [-- <test_dxl_position_step.py args...>]

Examples:
    ./tools/test_dxl_position_step.sh -- --dry-run
    ./tools/test_dxl_position_step.sh -- --step-ticks 50 --ramp-steps 1
    ./tools/test_dxl_position_step.sh -- --no-confirm

Default Python args:
    --motor-ids 2,4,6,8
    --step-ticks 500
    --ramp-steps 5
    --hold-s 1.0
    --settle-s 0.5
    --pass-error-ticks 10
    --output-dir test_logs

Environment:
    WORKSPACE_DIR        default $HOME/climbing_ws
    ROS_SETUP            optional explicit ROS setup.bash path
    NODE_WAIT_TIMEOUT    wait timeout in seconds, default 30

Safety:
    Start bringup with auto position/mode/current control disabled before
    running this direct /set_position test.
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
    else
        echo "Cannot find ROS setup.bash; set ROS_SETUP." >&2
        exit 1
    fi

    if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
        echo "Workspace devel/setup.bash missing under $WORKSPACE_DIR" >&2
        echo "Run catkin_make first." >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    source "$WORKSPACE_DIR/devel/setup.bash"
}

wait_for_topic() {
    local topic="$1"
    local timeout="$2"
    local start
    start=$(date +%s)
    while true; do
        if rostopic list 2>/dev/null | grep -qx "$topic"; then
            return 0
        fi
        if (( $(date +%s) - start >= timeout )); then
            echo "Timed out waiting for topic $topic (>${timeout}s)." >&2
            return 1
        fi
        sleep 0.5
    done
}

source_environment

echo "=== test_dxl_position_step: waiting for board topics ==="
wait_for_topic "/jetson/left_board/joint_state" "$NODE_WAIT_TIMEOUT"
wait_for_topic "/jetson/right_board/joint_state" "$NODE_WAIT_TIMEOUT"

echo "=== Safety reminder ==="
echo "Use bringup args: enable_auto_position_commands:=false enable_auto_mode_switching:=false enable_auto_current_control:=false"

echo "=== test_dxl_position_step: starting ==="
rosrun climbing_hw_bridge test_dxl_position_step.py "${PY_ARGS[@]}"
