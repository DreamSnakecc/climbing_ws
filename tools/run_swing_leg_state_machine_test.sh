#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
ROLE="pc"
LEG_NAME="${LEG_NAME:-rr}"
DURATION_S="${DURATION_S:-20}"
RATE_HZ="${RATE_HZ:-10}"
PRINT_PERIOD_S="${PRINT_PERIOD_S:-0.5}"
OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE_DIR/test_logs}"
TRIGGER_SWING=0
SWING_DURATION_S="${SWING_DURATION_S:-2.0}"
LAUNCH_LOCAL_STACK=0
ISOLATE_FROM_AUTO_CONTROL=0
FORCE_LIMIT_TOLERANCE_N="${FORCE_LIMIT_TOLERANCE_N:-0.5}"
STARTUP_GRACE_S="${STARTUP_GRACE_S:-3.0}"

normalize_role() {
    case "$1" in
        pc|jetson)
            printf '%s\n' "$1"
            ;;
        *)
            echo "Unsupported role: $1. Use pc or jetson." >&2
            exit 1
            ;;
    esac
}

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
    run_swing_leg_state_machine_test.sh --role pc|jetson [options]

Roles:
    pc       Run the Python observer/logger script. Optional --launch-local-stack starts pc_static_bringup.
    jetson   Launch the Jetson-side bringup. Optional --launch-local-stack is implied for this role.

Options:
    --role pc|jetson              Local machine role. Default: pc.
    --leg rr                      Target leg: lf, rf, rr, lr. Default: rr.
    --duration-s 20               Observer duration for PC role. Use <=0 to run until Ctrl+C.
    --rate-hz 10                  Observer/logging rate for PC role.
    --print-period-s 0.5          Minimum console print interval for PC role.
    --output-dir DIR              Log output directory for PC role.
    --trigger-swing               For PC role only: publish /control/body_reference to force one swing cycle.
    --swing-duration-s 2.0        Triggered swing duration when --trigger-swing is used.
    --force-limit-tolerance-n 0.5 Phase inference tolerance passed to the Python observer.
    --startup-grace-s 3.0        Delay before the observer warns that /control/swing_leg_target has no messages.
    --launch-local-stack          Also launch the local bringup before running the test command.
    --isolate-from-auto-control   Use pc_static_bringup, stop body_planner and mission_supervisor, and send zero-RPM fan commands.
    -h, --help                    Show this help.

Examples:
    ./tools/run_swing_leg_state_machine_test.sh --role jetson --launch-local-stack
    ./tools/run_swing_leg_state_machine_test.sh --role pc --leg rr --duration-s 30
    ./tools/run_swing_leg_state_machine_test.sh --role pc --leg rr --trigger-swing --duration-s 15
    ./tools/run_swing_leg_state_machine_test.sh --role pc --launch-local-stack --isolate-from-auto-control --leg rr --duration-s 15
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --role)
            ROLE=$(normalize_role "$2")
            shift 2
            ;;
        --leg)
            LEG_NAME=$(normalize_leg_name "$2")
            shift 2
            ;;
        --duration-s)
            DURATION_S="$2"
            shift 2
            ;;
        --rate-hz)
            RATE_HZ="$2"
            shift 2
            ;;
        --print-period-s)
            PRINT_PERIOD_S="$2"
            shift 2
            ;;
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --trigger-swing)
            TRIGGER_SWING=1
            shift
            ;;
        --swing-duration-s)
            SWING_DURATION_S="$2"
            shift 2
            ;;
        --force-limit-tolerance-n)
            FORCE_LIMIT_TOLERANCE_N="$2"
            shift 2
            ;;
        --startup-grace-s)
            STARTUP_GRACE_S="$2"
            shift 2
            ;;
        --launch-local-stack)
            LAUNCH_LOCAL_STACK=1
            shift
            ;;
        --isolate-from-auto-control)
            ISOLATE_FROM_AUTO_CONTROL=1
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
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
    else
        if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
            ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
        elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
            ROS_SETUP="/opt/ros/noetic/setup.bash"
        elif [[ -f "/opt/ros/melody/setup.bash" ]]; then
            ROS_SETUP="/opt/ros/melody/setup.bash"
        else
            echo "Unable to locate ROS setup.bash. Set ROS_SETUP explicitly." >&2
            exit 1
        fi
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
    fi

    if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
        echo "Workspace devel/setup.bash not found under $WORKSPACE_DIR. Build the workspace first." >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    source "$WORKSPACE_DIR/devel/setup.bash"
}

kill_node_if_present() {
    local node_name="$1"
    if rosnode list 2>/dev/null | grep -qx "$node_name"; then
        echo "Stopping interfering node $node_name ..."
        rosnode kill "$node_name" >/dev/null 2>&1 || true
    fi
}

stop_all_fans() {
    local leg_index
    for leg_index in 0 1 2 3; do
        rostopic pub -1 /jetson/fan_serial_bridge/adhesion_command climbing_msgs/AdhesionCommand \
            "{header: {stamp: now, frame_id: ''}, leg_index: ${leg_index}, mode: 0, target_rpm: 0.0, normal_force_limit: 0.0, required_adhesion_force: 0.0}" \
            >/dev/null 2>&1 || true
    done
}

apply_isolation_steps() {
    echo "Applying isolation steps: stopping body_planner and mission_supervisor, then sending zero-RPM fan commands ..."
    kill_node_if_present "/body_planner"
    kill_node_if_present "/mission_supervisor"
    stop_all_fans
}

run_pc_role() {
    if [[ "$ISOLATE_FROM_AUTO_CONTROL" -eq 1 && "$TRIGGER_SWING" -eq 0 ]]; then
        echo "Isolation mode requires a manual swing trigger; enabling --trigger-swing automatically."
        TRIGGER_SWING=1
    fi

    if [[ "$LAUNCH_LOCAL_STACK" -eq 1 ]]; then
        local bringup_launch="pc_static_bringup.launch"
        if [[ "$ISOLATE_FROM_AUTO_CONTROL" -eq 0 ]]; then
            bringup_launch="pc_static_bringup.launch"
        fi
        echo "Starting local PC bringup (${bringup_launch}) in the background..."
        roslaunch climbing_bringup "${bringup_launch}" >/tmp/swing_state_machine_pc_bringup.log 2>&1 &
        LOCAL_BRINGUP_PID=$!
        trap 'kill ${LOCAL_BRINGUP_PID:-0} >/dev/null 2>&1 || true' EXIT INT TERM
        sleep 3
        if ! kill -0 "$LOCAL_BRINGUP_PID" >/dev/null 2>&1; then
            echo "${bringup_launch} exited early. Recent log:" >&2
            tail -n 40 /tmp/swing_state_machine_pc_bringup.log >&2 || true
            exit 1
        fi
    else
        echo "Expecting an already running ROS graph that publishes /state/estimated and /control/swing_leg_target."
    fi

    if [[ "$ISOLATE_FROM_AUTO_CONTROL" -eq 1 ]]; then
        apply_isolation_steps
    fi

    local cmd=(
        python3 "$WORKSPACE_DIR/tools/test_swing_leg_state_machine.py"
        --leg "$LEG_NAME"
        --duration-s "$DURATION_S"
        --rate-hz "$RATE_HZ"
        --print-period-s "$PRINT_PERIOD_S"
        --output-dir "$OUTPUT_DIR"
        --swing-duration-s "$SWING_DURATION_S"
        --force-limit-tolerance-n "$FORCE_LIMIT_TOLERANCE_N"
        --startup-grace-s "$STARTUP_GRACE_S"
    )
    if [[ "$TRIGGER_SWING" -eq 1 ]]; then
        cmd+=(--trigger-swing)
    fi

    echo "Running PC-side observer: ${cmd[*]}"
    "${cmd[@]}"
}

run_jetson_role() {
    echo "Starting Jetson-side bringup..."
    exec roslaunch climbing_bringup jetson_bringup.launch
}

source_environment

if [[ "$ROLE" == "pc" ]]; then
    run_pc_role
else
    run_jetson_role
fi