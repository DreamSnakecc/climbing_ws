#!/usr/bin/env bash

# Convenience wrapper around
#   climbing_control_core/scripts/test_crawl_gait.py
#
# The script assumes that the user either:
#   * already started the robot's control stack with the desired UJC z
#     (e.g. `roslaunch climbing_bringup test_crawl_gait.launch ujc_z_mm:=-80.0`
#      on the PC side, plus jetson_bringup.launch on the Jetson), OR
#   * is running everything on a single host and wants this script to bring
#     up pc/jetson via the --launch-* options.
#
# Typical usage:
#   ./tools/test_crawl_gait.sh --ujc-z-mm -80 --test-duration-s 30 --verbose
#
# The script will:
#   1. Source the workspace setup.
#   2. Optionally launch jetson_bringup.launch / test_crawl_gait.launch
#      (single-host convenience).
#   3. Wait for the required nodes.
#   4. Run scripts/test_crawl_gait.py with the resolved arguments.
#   5. Leave pc_bringup / jetson_bringup untouched on exit unless it started
#      them in step 2 (in which case it kills its own children).

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
UJC_Z_MM="${UJC_Z_MM:--80.0}"
TEST_DURATION_S="${TEST_DURATION_S:-30.0}"
WARMUP_S="${WARMUP_S:-1.5}"
COOLDOWN_S="${COOLDOWN_S:-1.0}"
CONTROL_RATE_HZ="${CONTROL_RATE_HZ:-50.0}"
LOG_RATE_HZ="${LOG_RATE_HZ:-50.0}"
LINEAR_VELOCITY=${LINEAR_VELOCITY:-"0.0 0.0 0.0"}
ANGULAR_VELOCITY=${ANGULAR_VELOCITY:-"0.0 0.0 0.0"}
OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE_DIR/test_logs}"
VERBOSE_STATUS=0
RELEASE_FANS_ON_EXIT=0
LAUNCH_JETSON=0
LAUNCH_PC=0
JETSON_PID=""
PC_PID=""

usage() {
    cat <<'EOF'
Usage:
    test_crawl_gait.sh [options]

Options:
    --ujc-z-mm MM              override nominal_universal_joint_center_z (default: -80.0)
    --test-duration-s SEC      recording window after mission_start (default: 30)
    --warmup-s SEC             delay before mission_start (default: 1.5)
    --cooldown-s SEC           delay after mission_pause before saving CSV (default: 1.0)
    --control-rate-hz VAL      monitoring loop rate (default: 50)
    --log-rate-hz VAL          CSV sample rate (default: 50)
    --linear-velocity "X Y Z"  body linear velocity (m/s) pushed to body_planner
                               (default: "0 0 0"; requires planner restart)
    --angular-velocity "X Y Z" body angular velocity (rad/s) for body_planner
                               (default: "0 0 0"; requires planner restart)
    --output-dir PATH          CSV log directory (default: $WORKSPACE_DIR/test_logs)
    --verbose                  live per-leg status line while running
    --release-fans-on-exit     explicitly set all fans to RELEASE/rpm=0 at cleanup
    --launch-jetson            start jetson_bringup.launch locally
    --launch-pc                start test_crawl_gait.launch locally
                               (uses --ujc-z-mm to set the UJC z override at launch)
    -h | --help                show this message
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ujc-z-mm) UJC_Z_MM="$2"; shift 2 ;;
        --test-duration-s) TEST_DURATION_S="$2"; shift 2 ;;
        --warmup-s) WARMUP_S="$2"; shift 2 ;;
        --cooldown-s) COOLDOWN_S="$2"; shift 2 ;;
        --control-rate-hz) CONTROL_RATE_HZ="$2"; shift 2 ;;
        --log-rate-hz) LOG_RATE_HZ="$2"; shift 2 ;;
        --linear-velocity) LINEAR_VELOCITY="$2"; shift 2 ;;
        --angular-velocity) ANGULAR_VELOCITY="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --verbose) VERBOSE_STATUS=1; shift ;;
        --release-fans-on-exit) RELEASE_FANS_ON_EXIT=1; shift ;;
        --launch-jetson) LAUNCH_JETSON=1; shift ;;
        --launch-pc) LAUNCH_PC=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
    esac
done

source_environment() {
    if [[ -n "$ROS_SETUP" ]]; then
        if [[ ! -f "$ROS_SETUP" ]]; then
            echo "ROS_SETUP does not exist: $ROS_SETUP" >&2
            exit 1
        fi
        source "$ROS_SETUP"
    elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
        source "/opt/ros/noetic/setup.bash"
    else
        echo "Unable to find ROS setup.bash. Set ROS_SETUP." >&2
        exit 1
    fi

    if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
        echo "Workspace devel/setup.bash missing. Run catkin_make first." >&2
        exit 1
    fi
    source "$WORKSPACE_DIR/devel/setup.bash"
}

wait_for_node() {
    local node="$1"
    local timeout="${2:-25}"
    local start
    start=$(date +%s)
    while true; do
        if rosnode list 2>/dev/null | grep -qx "$node"; then
            return 0
        fi
        if (( $(date +%s) - start >= timeout )); then
            echo "Timed out waiting for node $node" >&2
            return 1
        fi
        sleep 0.5
    done
}

cleanup() {
    local exit_code=$?
    if [[ -n "$PC_PID" ]]; then
        kill "$PC_PID" >/dev/null 2>&1 || true
    fi
    if [[ -n "$JETSON_PID" ]]; then
        kill "$JETSON_PID" >/dev/null 2>&1 || true
    fi
    wait >/dev/null 2>&1 || true
    exit "$exit_code"
}

source_environment
trap cleanup EXIT INT TERM

if [[ "$LAUNCH_JETSON" -eq 1 ]]; then
    echo "Starting jetson_bringup.launch..."
    roslaunch climbing_bringup jetson_bringup.launch \
        enable_auto_position_commands:=true \
        enable_auto_mode_switching:=true \
        enable_auto_current_control:=true \
        >/tmp/test_crawl_gait_jetson.log 2>&1 &
    JETSON_PID=$!
fi

if [[ "$LAUNCH_PC" -eq 1 ]]; then
    echo "Starting test_crawl_gait.launch (ujc_z_mm=$UJC_Z_MM)..."
    roslaunch climbing_bringup test_crawl_gait.launch \
        ujc_z_mm:="$UJC_Z_MM" \
        mission_auto_start:=false \
        enable_auto_adhesion_commands:=true \
        >/tmp/test_crawl_gait_pc.log 2>&1 &
    PC_PID=$!
fi

echo "Waiting for required nodes..."
wait_for_node "/jetson/leg_ik_executor"
wait_for_node "/jetson/dynamixel_bridge"
wait_for_node "/jetson/fan_serial_bridge"
wait_for_node "/state_estimator"
wait_for_node "/swing_leg_controller"
wait_for_node "/body_planner"
wait_for_node "/mission_supervisor"

echo
echo "Whole-body crawl-gait test"
echo "  ujc_z_mm=$UJC_Z_MM  test_duration_s=$TEST_DURATION_S"
echo "  warmup_s=$WARMUP_S  cooldown_s=$COOLDOWN_S"
echo "  control_rate_hz=$CONTROL_RATE_HZ  log_rate_hz=$LOG_RATE_HZ"
echo "  linear_velocity=[$LINEAR_VELOCITY]  angular_velocity=[$ANGULAR_VELOCITY]"
echo "  output_dir=$OUTPUT_DIR"
echo

PYTHON_ARGS=(
    --ujc-z-mm "$UJC_Z_MM"
    --test-duration-s "$TEST_DURATION_S"
    --warmup-s "$WARMUP_S"
    --cooldown-s "$COOLDOWN_S"
    --control-rate-hz "$CONTROL_RATE_HZ"
    --log-rate-hz "$LOG_RATE_HZ"
    --output-dir "$OUTPUT_DIR"
)

# Expand space-separated velocity triplets into positional triplets expected
# by argparse.
read -r -a LINEAR_VELOCITY_ARR <<<"$LINEAR_VELOCITY"
read -r -a ANGULAR_VELOCITY_ARR <<<"$ANGULAR_VELOCITY"
if [[ "${#LINEAR_VELOCITY_ARR[@]}" -ne 3 ]]; then
    echo "--linear-velocity expects 3 values (got: '$LINEAR_VELOCITY')" >&2
    exit 1
fi
if [[ "${#ANGULAR_VELOCITY_ARR[@]}" -ne 3 ]]; then
    echo "--angular-velocity expects 3 values (got: '$ANGULAR_VELOCITY')" >&2
    exit 1
fi
PYTHON_ARGS+=(--linear-velocity-mps "${LINEAR_VELOCITY_ARR[@]}")
PYTHON_ARGS+=(--angular-velocity-rps "${ANGULAR_VELOCITY_ARR[@]}")

if [[ "$VERBOSE_STATUS" -eq 1 ]]; then
    PYTHON_ARGS+=(--verbose-status)
fi
if [[ "$RELEASE_FANS_ON_EXIT" -eq 1 ]]; then
    PYTHON_ARGS+=(--release-fans-on-exit)
fi

rosrun climbing_control_core test_crawl_gait.py "${PYTHON_ARGS[@]}"
