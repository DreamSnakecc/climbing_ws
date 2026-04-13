#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
ROBOT_CONFIG_PATH="${ROBOT_CONFIG_PATH:-$WORKSPACE_DIR/src/climbing_description/config/robot.yaml}"
BACKUP_DIR="${BACKUP_DIR:-$WORKSPACE_DIR/.calibration_backups}"
SKIP_CONFIRM=0

usage() {
    cat <<'EOF'
Usage:
    calibrate_motor_home_ticks.sh [--yes]

Options:
    --yes   Skip the interactive confirmation prompt.

Behavior:
    1. Back up robot.yaml.
    2. Launch a minimal Jetson-side calibration stack.
    3. Read the current servo positions.
    4. Write them back into gait_controller.motor_home_ticks.
    5. Shut the calibration stack down.

Before running this script, manually place the robot in the desired nominal pose.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --yes)
            SKIP_CONFIRM=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
done

source_environment() {
    if [[ -n "$ROS_SETUP" ]]; then
        if [[ ! -f "$ROS_SETUP" ]]; then
            echo "ROS_SETUP does not exist: $ROS_SETUP" >&2
            exit 1
        fi
    elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [[ -f "/opt/ros/melody/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/melody/setup.bash"
    elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/noetic/setup.bash"
    else
        echo "Unable to find a usable ROS setup.bash. Set ROS_SETUP explicitly." >&2
        exit 1
    fi

    source "$ROS_SETUP"
    source "$WORKSPACE_DIR/devel/setup.bash"
}

wait_for_node() {
    local node_name="$1"
    local timeout_sec="${2:-20}"
    local start_sec
    start_sec=$(date +%s)
    while true; do
        if rosnode list 2>/dev/null | grep -qx "$node_name"; then
            return 0
        fi
        if (( $(date +%s) - start_sec >= timeout_sec )); then
            echo "Timed out waiting for node $node_name" >&2
            return 1
        fi
        sleep 0.5
    done
}

cleanup() {
    local exit_code=$?
    if [[ -n "${LAUNCH_PID:-}" ]]; then
        kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    fi
    wait >/dev/null 2>&1 || true
    exit "$exit_code"
}

source_environment
trap cleanup EXIT INT TERM

if [[ ! -f "$ROBOT_CONFIG_PATH" ]]; then
    echo "robot.yaml not found: $ROBOT_CONFIG_PATH" >&2
    exit 1
fi

mkdir -p "$BACKUP_DIR"
BACKUP_PATH="$BACKUP_DIR/robot.yaml.$(date +%Y%m%d_%H%M%S).bak"
cp "$ROBOT_CONFIG_PATH" "$BACKUP_PATH"

echo "Backup created: $BACKUP_PATH"
echo "Target config: $ROBOT_CONFIG_PATH"
echo
echo "Place the robot manually in the desired nominal pose before continuing."
echo "The script will capture the current servo positions and overwrite gait_controller.motor_home_ticks."

if [[ "$SKIP_CONFIRM" -eq 0 ]]; then
    read -r -p "Press Enter to start calibration, or Ctrl+C to abort..." _unused
fi

echo "Launching calibration stack..."
roslaunch climbing_bringup jetson_calibrate_home.launch >/tmp/calibrate_motor_home_ticks.log 2>&1 &
LAUNCH_PID=$!

wait_for_node "/jetson/left_board/multi_dxl_node_left"
wait_for_node "/jetson/right_board/multi_dxl_node_right"
wait_for_node "/jetson/leg_ik_executor"

sleep 2

echo
echo "Calibration capture completed. Updated motor_home_ticks:"
python3 - <<PY
from pathlib import Path

path = Path(r"$ROBOT_CONFIG_PATH")
lines = path.read_text().splitlines()
inside = False
for line in lines:
    if line.strip() == "motor_home_ticks:":
        inside = True
        print(line)
        continue
    if inside:
        if line.startswith("  ") and not line.startswith("    "):
            break
        print(line)
PY

echo
echo "Calibration stack log: /tmp/calibrate_motor_home_ticks.log"
echo "Original config backup: $BACKUP_PATH"
echo "Stopping calibration stack..."
