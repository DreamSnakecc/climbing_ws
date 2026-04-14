#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
WITH_PC=0
SKIP_LAUNCH=0
ENABLE_OBSERVER=0
STEP_M="${STEP_M:-0.008}"
NORMAL_FORCE_LIMIT="${NORMAL_FORCE_LIMIT:-10.0}"
SKIRT_TARGET="${SKIRT_TARGET:-0.0}"
LEG_NAME="${LEG_NAME:-rr}"
OBSERVER_PIDS=()

normalize_leg_name() {
    case "$1" in
        lf|rf|rr|lr)
            printf '%s\n' "$1"
            ;;
        *)
            echo "Unsupported leg: $1. Use one of lf, rf, rr, lr." >&2
            exit 1
            ;;
    esac
}

usage() {
    cat <<'EOF'
Usage:
    test_single_leg.sh [--leg rr] [--with-pc] [--skip-launch] [--observer] [--step-m 0.008]

Options:
    --leg          Leg to test: lf, rf, rr, lr. Default: rr.
    --with-pc      Also launch pc_static_bringup for state-estimation observation.
    --skip-launch  Do not launch ROS nodes; assume the required stack is already running.
    --observer     Start observer windows or background observers automatically.
    --step-m       Per-step translation magnitude in meters. Default: 0.008.

This script tests a single leg by publishing a sequence of manual
/control/swing_leg_target commands in delta-from-nominal mode.
All x/y/z values are body-frame offsets relative to the nominal pose.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --leg)
            LEG_NAME=$(normalize_leg_name "$2")
            shift 2
            ;;
        --with-pc)
            WITH_PC=1
            shift
            ;;
        --skip-launch)
            SKIP_LAUNCH=1
            shift
            ;;
        --observer)
            ENABLE_OBSERVER=1
            shift
            ;;
        --step-m)
            STEP_M="$2"
            shift 2
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
        source "$ROS_SETUP"
        return
    fi

    if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/noetic/setup.bash"
    elif [[ -f "/opt/ros/melody/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/melody/setup.bash"
    else
        echo "Unable to find a usable ROS setup.bash. Set ROS_SETUP explicitly." >&2
        exit 1
    fi

    source "$ROS_SETUP"
    source "$WORKSPACE_DIR/devel/setup.bash"
}

cleanup() {
    local exit_code=$?
    for pid in "${OBSERVER_PIDS[@]:-}"; do
        kill "$pid" >/dev/null 2>&1 || true
    done
    if [[ -n "${PC_PID:-}" ]]; then
        kill "$PC_PID" >/dev/null 2>&1 || true
    fi
    if [[ -n "${JETSON_PID:-}" ]]; then
        kill "$JETSON_PID" >/dev/null 2>&1 || true
    fi
    wait >/dev/null 2>&1 || true
    exit "$exit_code"
}

wait_for_node() {
    local node_name="$1"
    local timeout_sec="${2:-25}"
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

open_observer() {
    local title="$1"
    local command="$2"
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -lc "$command; exec bash" >/dev/null 2>&1 &
        return 0
    fi
    if command -v x-terminal-emulator >/dev/null 2>&1; then
        x-terminal-emulator -T "$title" -e bash -lc "$command; exec bash" >/dev/null 2>&1 &
        return 0
    fi
    if command -v xterm >/dev/null 2>&1; then
        xterm -T "$title" -e bash -lc "$command; exec bash" >/dev/null 2>&1 &
        return 0
    fi

    bash -lc "$command" >/tmp/$(echo "$title" | tr ' ' '_' | tr '[:upper:]' '[:lower:]').log 2>&1 &
    OBSERVER_PIDS+=("$!")
    return 0
}

start_observers() {
    open_observer "single-leg-joint-state" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /jetson/dynamixel_bridge/joint_state"
    open_observer "single-leg-joint-ticks" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /jetson/joint_position_ticks_cmd"
    open_observer "single-leg-swing-target" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /control/swing_leg_target"
    if [[ "$WITH_PC" -eq 1 ]]; then
        open_observer "single-leg-estimated-state" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /state/estimated | grep -E 'wall_touch_mask|preload_ready_mask|attachment_ready_mask|adhesion_mask'"
    fi
}

publish_leg_target() {
    local x="$1"
    local y="$2"
    local z="$3"
    local label="$4"
    echo
    echo "==> $label"
    rostopic pub -1 /control/swing_leg_target climbing_msgs/LegCenterCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
leg_name: '${LEG_NAME}'
center: {x: ${x}, y: ${y}, z: ${z}}
center_velocity: {x: 0.0, y: 0.0, z: 0.0}
skirt_compression_target: ${SKIRT_TARGET}
support_leg: false
desired_normal_force_limit: ${NORMAL_FORCE_LIMIT}"
}

pause_step() {
    local prompt="$1"
    read -r -p "$prompt" _unused
}

source_environment
trap cleanup EXIT INT TERM

if [[ "$WITH_PC" -eq 1 && "${ROS_DISTRO:-}" != "noetic" ]]; then
    echo "--with-pc expects the local machine to run the PC-side Noetic stack." >&2
    echo "Current ROS_DISTRO=${ROS_DISTRO:-unknown}. Start pc_static_bringup on the PC separately, or rerun without --with-pc." >&2
    exit 1
fi

if [[ "$SKIP_LAUNCH" -eq 0 ]]; then
    echo "Starting Jetson-side single-leg test stack..."
    roslaunch climbing_bringup jetson_bringup.launch \
        enable_auto_position_commands:=true \
        enable_auto_mode_switching:=false \
        enable_auto_current_control:=false \
        leg_ik_input_mode:=delta_from_nominal_m \
        >/tmp/test_single_leg_jetson.log 2>&1 &
    JETSON_PID=$!

    if [[ "$WITH_PC" -eq 1 ]]; then
        echo "Starting PC-side observation stack..."
        roslaunch climbing_bringup pc_static_bringup.launch \
            >/tmp/test_single_leg_pc.log 2>&1 &
        PC_PID=$!
    fi
fi

echo "Waiting for key nodes..."
wait_for_node "/jetson/leg_ik_executor"
wait_for_node "/jetson/dynamixel_bridge"

if [[ "$WITH_PC" -eq 1 ]]; then
    wait_for_node "/state_estimator"
    wait_for_node "/swing_leg_controller"
fi

STEP_VALUE=$(python3 - <<PY
print(float("$STEP_M"))
PY
)

echo
echo "Single-leg test ready for ${LEG_NAME}."
echo "Leg IK input mode: delta_from_nominal_m"
echo "Step magnitude: ${STEP_VALUE} m"
if [[ "$ENABLE_OBSERVER" -eq 1 ]]; then
    echo "Starting observer mode..."
    start_observers
fi
echo
echo "Suggested observers:"
echo "  rostopic echo /jetson/dynamixel_bridge/joint_state"
echo "  rostopic echo /jetson/joint_position_ticks_cmd"
echo "  rostopic echo /control/swing_leg_target"
if [[ "$WITH_PC" -eq 1 ]]; then
    echo "  rostopic echo /state/estimated | grep -E \"wall_touch_mask|preload_ready_mask|attachment_ready_mask|adhesion_mask\""
fi

pause_step "Press Enter to publish the nominal ${LEG_NAME} hold target..."
publish_leg_target "0.0" "0.0" "0.0" "${LEG_NAME^^} nominal hold"

pause_step "Press Enter to move ${LEG_NAME} in body-frame +x..."
publish_leg_target "${STEP_VALUE}" "0.0" "0.0" "${LEG_NAME^^} body-frame +x"

pause_step "Press Enter to move ${LEG_NAME} in body-frame -x..."
publish_leg_target "-${STEP_VALUE}" "0.0" "0.0" "${LEG_NAME^^} body-frame -x"

pause_step "Press Enter to move ${LEG_NAME} in body-frame +y..."
publish_leg_target "0.0" "${STEP_VALUE}" "0.0" "${LEG_NAME^^} body-frame +y"

pause_step "Press Enter to move ${LEG_NAME} in body-frame -y..."
publish_leg_target "0.0" "-${STEP_VALUE}" "0.0" "${LEG_NAME^^} body-frame -y"

pause_step "Press Enter to move ${LEG_NAME} in body-frame z+..."
publish_leg_target "0.0" "0.0" "${STEP_VALUE}" "${LEG_NAME^^} body-frame z+"

pause_step "Press Enter to move ${LEG_NAME} in body-frame z-..."
publish_leg_target "0.0" "0.0" "-${STEP_VALUE}" "${LEG_NAME^^} body-frame z-"

pause_step "Press Enter to send ${LEG_NAME} back to nominal hold and finish..."
publish_leg_target "0.0" "0.0" "0.0" "${LEG_NAME^^} return to nominal"

echo
echo "Test sequence finished. Press Ctrl+C to stop the launched nodes, or keep them running for further observation."
while true; do
    sleep 1
done