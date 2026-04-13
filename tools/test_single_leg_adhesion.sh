#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
SKIP_LAUNCH=0
ENABLE_OBSERVER=0
PRELOAD_OFFSET_M="${PRELOAD_OFFSET_M:-0.005}"
PRELOAD_SKIRT_TARGET="${PRELOAD_SKIRT_TARGET:-0.85}"
NORMAL_FORCE_LIMIT="${NORMAL_FORCE_LIMIT:-25.0}"
REQUIRED_ADHESION_FORCE="${REQUIRED_ADHESION_FORCE:-100.0}"
RPM_LOW="${RPM_LOW:-30000.0}"
RPM_HIGH="${RPM_HIGH:-40000.0}"
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

leg_index_for_name() {
    case "$1" in
        lf) printf '0\n' ;;
        rf) printf '1\n' ;;
        rr) printf '2\n' ;;
        lr) printf '3\n' ;;
    esac
}

usage() {
    cat <<'EOF'
Usage:
    test_single_leg_adhesion.sh [--leg rr] [--skip-launch] [--observer]

Options:
    --leg          Leg to test: lf, rf, rr, lr. Default: rr.
    --skip-launch  Do not launch ROS nodes; assume the required stack is already running.
    --observer     Start observer windows or background observers automatically.

Environment overrides:
  PRELOAD_OFFSET_M         Default: 0.005
  PRELOAD_SKIRT_TARGET     Default: 0.85
  NORMAL_FORCE_LIMIT       Default: 25.0
  REQUIRED_ADHESION_FORCE  Default: 100.0
  RPM_LOW                  Default: 30000.0
  RPM_HIGH                 Default: 40000.0

This script tests a single leg adhesion flow:
  nominal hold -> preload -> low-rpm attach -> high-rpm attach -> release -> return.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --leg)
            LEG_NAME=$(normalize_leg_name "$2")
            shift 2
            ;;
        --skip-launch)
            SKIP_LAUNCH=1
            shift
            ;;
        --observer)
            ENABLE_OBSERVER=1
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
    local timeout_sec="${2:-30}"
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
    open_observer "single-leg-adhesion-fan-currents" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /jetson/fan_serial_bridge/fan_currents"
    open_observer "single-leg-adhesion-command" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /jetson/fan_serial_bridge/adhesion_command"
    open_observer "single-leg-adhesion-state" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /state/estimated | grep -E 'wall_touch_mask|preload_ready_mask|attachment_ready_mask|adhesion_mask|skirt_compression_estimate|seal_confidence'"
    open_observer "single-leg-adhesion-target" "source '$ROS_SETUP'; source '$WORKSPACE_DIR/devel/setup.bash'; rostopic echo /control/swing_leg_target"
}

pause_step() {
    local prompt="$1"
    read -r -p "$prompt" _unused
}

publish_leg_target() {
    local x="$1"
    local y="$2"
    local z="$3"
    local skirt_target="$4"
    local force_limit="$5"
    local label="$6"
    echo
    echo "==> $label"
    rostopic pub -1 /control/swing_leg_target climbing_msgs/LegCenterCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
leg_name: '${LEG_NAME}'
center: {x: ${x}, y: ${y}, z: ${z}}
center_velocity: {x: 0.0, y: 0.0, z: 0.0}
skirt_compression_target: ${skirt_target}
support_leg: false
desired_normal_force_limit: ${force_limit}"
}

publish_leg_adhesion() {
    local rpm="$1"
    local mode="$2"
    local label="$3"
    echo
    echo "==> $label"
    rostopic pub -1 /jetson/fan_serial_bridge/adhesion_command climbing_msgs/AdhesionCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
leg_index: ${LEG_INDEX}
mode: ${mode}
target_rpm: ${rpm}
normal_force_limit: ${NORMAL_FORCE_LIMIT}
required_adhesion_force: ${REQUIRED_ADHESION_FORCE}"
}

source_environment
trap cleanup EXIT INT TERM
LEG_INDEX=$(leg_index_for_name "$LEG_NAME")

if [[ "$SKIP_LAUNCH" -eq 0 && "${ROS_DISTRO:-}" != "noetic" ]]; then
    echo "This script launches pc_static_bringup locally, which should run on the Noetic PC side." >&2
    echo "Current ROS_DISTRO=${ROS_DISTRO:-unknown}. On Jetson/Melody, start the PC stack on the PC first and rerun with --skip-launch." >&2
    exit 1
fi

if [[ "$SKIP_LAUNCH" -eq 0 ]]; then
    echo "Starting Jetson-side adhesion test stack..."
    roslaunch climbing_bringup jetson_bringup.launch \
        enable_auto_position_commands:=true \
        enable_auto_mode_switching:=false \
        enable_auto_current_control:=false \
        >/tmp/test_single_leg_adhesion_jetson.log 2>&1 &
    JETSON_PID=$!

    echo "Starting PC-side observation stack with manual mission/adhesion control..."
    roslaunch climbing_bringup pc_static_bringup.launch \
        >/tmp/test_single_leg_adhesion_pc.log 2>&1 &
    PC_PID=$!
fi

echo "Waiting for key nodes..."
wait_for_node "/jetson/leg_ik_executor"
wait_for_node "/jetson/fan_serial_bridge"
wait_for_node "/jetson/dynamixel_bridge"
wait_for_node "/state_estimator"

NOMINAL_Z_M=$(python3 - <<'PY'
import subprocess

try:
    output = subprocess.check_output(
        ["rosparam", "get", "/gait_controller/nominal_universal_joint_center_z"],
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()
    value_mm = float(output)
except Exception:
    value_mm = -179.0
print(value_mm / 1000.0)
PY
)

PRELOAD_Z_M=$(python3 - <<PY
print(float("${NOMINAL_Z_M}") - float("${PRELOAD_OFFSET_M}"))
PY
)

echo
echo "Single-leg adhesion test ready for ${LEG_NAME}."
echo "Nominal ${LEG_NAME} center z: ${NOMINAL_Z_M} m"
echo "Preload ${LEG_NAME} center z: ${PRELOAD_Z_M} m"
echo "Low/High target rpm: ${RPM_LOW} / ${RPM_HIGH}"
if [[ "$ENABLE_OBSERVER" -eq 1 ]]; then
    echo "Starting observer mode..."
    start_observers
fi
echo
echo "Suggested observers:"
echo "  rostopic echo /jetson/fan_serial_bridge/fan_currents"
echo "  rostopic echo /jetson/fan_serial_bridge/adhesion_command"
echo "  rostopic echo /state/estimated | grep -E \"wall_touch_mask|preload_ready_mask|attachment_ready_mask|adhesion_mask|skirt_compression_estimate|seal_confidence\""
echo "  rostopic echo /control/swing_leg_target"

pause_step "Press Enter to send ${LEG_NAME} to nominal hold..."
publish_leg_target "0.0" "0.0" "${NOMINAL_Z_M}" "0.0" "10.0" "${LEG_NAME^^} nominal hold"

pause_step "Press Enter to move ${LEG_NAME} into preload pose..."
publish_leg_target "0.0" "0.0" "${PRELOAD_Z_M}" "${PRELOAD_SKIRT_TARGET}" "${NORMAL_FORCE_LIMIT}" "${LEG_NAME^^} preload pose"

pause_step "Press Enter to command low-rpm adhesion for ${LEG_NAME}..."
publish_leg_adhesion "${RPM_LOW}" "1" "${LEG_NAME^^} adhesion command at low rpm"

pause_step "Observe current and masks, then press Enter to command high-rpm adhesion..."
publish_leg_adhesion "${RPM_HIGH}" "2" "${LEG_NAME^^} adhesion command at high rpm"

pause_step "Observe current and masks, then press Enter to release the fan..."
publish_leg_adhesion "0.0" "0" "${LEG_NAME^^} adhesion release"

pause_step "Press Enter to return ${LEG_NAME} to nominal hold..."
publish_leg_target "0.0" "0.0" "${NOMINAL_Z_M}" "0.0" "10.0" "${LEG_NAME^^} return to nominal"

echo
echo "Adhesion test sequence finished. Press Ctrl+C to stop the launched nodes, or keep them running for further observation."
while true; do
    sleep 1
done