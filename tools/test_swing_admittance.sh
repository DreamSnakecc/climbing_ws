#!/usr/bin/env bash

# Convenience wrapper around climbing_control_core/scripts/test_swing_admittance.py.
#
# The script assumes the user has either already started the required ROS
# stack, or is running everything on a single host. It:
#   1. Sources the workspace setup.
#   2. Optionally launches jetson_bringup.launch (single-host-only option).
#   3. Optionally launches pc_static_bringup.launch (single-host-only option).
#   4. Waits for the required nodes.
#   5. Runs scripts/test_swing_admittance.py with the provided arguments.
#
# Typical usage on the robot Jetson with the PC side already running:
#   ./tools/test_swing_admittance.sh --leg lf --lift 0.04 --press 0.045

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
LEG="${LEG:-lf}"
LIFT_M="${LIFT_M:-0.04}"
PRESS_M="${PRESS_M:-0.045}"
FAN_RPM="${FAN_RPM:-30000}"
FAN_MODE="${FAN_MODE:-2}"
FAN_ON_PHASE_ID="${FAN_ON_PHASE_ID:-2}"
HOLD_ADHESION_S="${HOLD_ADHESION_S:-2.0}"
TEST_TIMEOUT_S="${TEST_TIMEOUT_S:-15.0}"
CONTROL_RATE_HZ="${CONTROL_RATE_HZ:-100.0}"
LOG_RATE_HZ="${LOG_RATE_HZ:-50.0}"
OUTPUT_DIR="${OUTPUT_DIR:-$HOME/.ros/climbing_logs/swing_admittance}"
VERBOSE_STATUS=0
LAUNCH_JETSON=0
LAUNCH_PC=0
JETSON_PID=""
PC_PID=""

usage() {
    cat <<'EOF'
Usage:
    test_swing_admittance.sh [options]

Options:
    --leg NAME                 lf | rf | rr | lr (default: lf)
    --lift METERS              lift magnitude along wall normal (default: 0.04)
    --press METERS             press magnitude along wall normal; sent as -press into the wall (default: 0.045)
    --fan-rpm VALUE            fan RPM during press/settle (default: 30000)
    --fan-mode {0,1,2}         AdhesionCommand mode while boosting (default: 2)
    --fan-on-phase-id ID       phase id after which fan boost kicks in (default: 2 = TEST_PRESS_CONTACT)
    --hold-adhesion-s SEC      hold after attachment/adhesion latches (default: 2.0)
    --test-timeout-s SEC       overall safety timeout (default: 15.0)
    --control-rate-hz VAL      BodyReference publish rate (default: 100)
    --log-rate-hz VAL          log sample rate (default: 50)
    --output-dir PATH          CSV log directory (default: ~/.ros/climbing_logs/swing_admittance)
    --verbose                  print a throttled live status during the run
    --launch-jetson            start jetson_bringup.launch locally before the test
    --launch-pc                start pc_static_bringup.launch locally before the test
    -h | --help                show this message

Environment variables override the defaults before parsing arguments
(LEG, LIFT_M, PRESS_M, FAN_RPM, HOLD_ADHESION_S, ...).

The script assumes mission_supervisor is not auto-adhesion-enabled and body_planner
is configured with mission_active=False (pc_static_bringup.launch does both).
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --leg) LEG="$2"; shift 2 ;;
        --lift) LIFT_M="$2"; shift 2 ;;
        --press) PRESS_M="$2"; shift 2 ;;
        --fan-rpm) FAN_RPM="$2"; shift 2 ;;
        --fan-mode) FAN_MODE="$2"; shift 2 ;;
        --fan-on-phase-id) FAN_ON_PHASE_ID="$2"; shift 2 ;;
        --hold-adhesion-s) HOLD_ADHESION_S="$2"; shift 2 ;;
        --test-timeout-s) TEST_TIMEOUT_S="$2"; shift 2 ;;
        --control-rate-hz) CONTROL_RATE_HZ="$2"; shift 2 ;;
        --log-rate-hz) LOG_RATE_HZ="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --verbose) VERBOSE_STATUS=1; shift ;;
        --launch-jetson) LAUNCH_JETSON=1; shift ;;
        --launch-pc) LAUNCH_PC=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
    esac
done

case "$LEG" in
    lf|rf|rr|lr) ;;
    *) echo "Invalid --leg $LEG (expected lf|rf|rr|lr)" >&2; exit 1 ;;
esac

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
        >/tmp/test_swing_admittance_jetson.log 2>&1 &
    JETSON_PID=$!
fi

if [[ "$LAUNCH_PC" -eq 1 ]]; then
    echo "Starting pc_static_bringup.launch..."
    roslaunch climbing_bringup pc_static_bringup.launch \
        >/tmp/test_swing_admittance_pc.log 2>&1 &
    PC_PID=$!
fi

echo "Waiting for required nodes..."
wait_for_node "/jetson/leg_ik_executor"
wait_for_node "/jetson/dynamixel_bridge"
wait_for_node "/jetson/fan_serial_bridge"
wait_for_node "/state_estimator"
wait_for_node "/swing_leg_controller"

echo
echo "Swing-leg admittance test"
echo "  leg=$LEG lift=$LIFT_M press=$PRESS_M fan_rpm=$FAN_RPM"
echo "  fan_mode=$FAN_MODE fan_on_phase_id=$FAN_ON_PHASE_ID"
echo "  hold_adhesion_s=$HOLD_ADHESION_S test_timeout_s=$TEST_TIMEOUT_S"
echo "  output_dir=$OUTPUT_DIR"
echo

PYTHON_ARGS=(
    --leg "$LEG"
    --lift "$LIFT_M"
    --press "$PRESS_M"
    --fan-rpm "$FAN_RPM"
    --fan-mode "$FAN_MODE"
    --fan-on-phase-id "$FAN_ON_PHASE_ID"
    --hold-adhesion-s "$HOLD_ADHESION_S"
    --test-timeout-s "$TEST_TIMEOUT_S"
    --control-rate-hz "$CONTROL_RATE_HZ"
    --log-rate-hz "$LOG_RATE_HZ"
    --output-dir "$OUTPUT_DIR"
)
if [[ "$VERBOSE_STATUS" -eq 1 ]]; then
    PYTHON_ARGS+=(--verbose-status)
fi

rosrun climbing_control_core test_swing_admittance.py "${PYTHON_ARGS[@]}"
