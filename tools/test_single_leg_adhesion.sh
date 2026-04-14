#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
SKIP_LAUNCH=0
ENABLE_OBSERVER=0
MANUAL_MODE=0
PRELOAD_OFFSET_M="${PRELOAD_OFFSET_M:-0.012}"
PRELOAD_SKIRT_TARGET="${PRELOAD_SKIRT_TARGET:-0.85}"
NORMAL_FORCE_LIMIT="${NORMAL_FORCE_LIMIT:-25.0}"
REQUIRED_ADHESION_FORCE="${REQUIRED_ADHESION_FORCE:-100.0}"
RPM_LOW="${RPM_LOW:-30000.0}"
RPM_HIGH="${RPM_HIGH:-40000.0}"
LIFT_OFFSET_M="${LIFT_OFFSET_M:-0.025}"
FORWARD_OFFSET_M="${FORWARD_OFFSET_M:-0.020}"
NOMINAL_HOLD_S="${NOMINAL_HOLD_S:-1.0}"
LIFT_SETTLE_S="${LIFT_SETTLE_S:-1.0}"
FORWARD_SETTLE_S="${FORWARD_SETTLE_S:-1.0}"
PRELOAD_SETTLE_S="${PRELOAD_SETTLE_S:-1.5}"
LOW_ATTACH_WAIT_S="${LOW_ATTACH_WAIT_S:-2.0}"
HIGH_ATTACH_WAIT_S="${HIGH_ATTACH_WAIT_S:-3.0}"
RELEASE_WAIT_S="${RELEASE_WAIT_S:-1.0}"
JUDGE_TIMEOUT_S="${JUDGE_TIMEOUT_S:-3.0}"
LOG_ROOT_DIR="${LOG_ROOT_DIR:-$WORKSPACE_DIR/test_logs}"
LEG_NAME="${LEG_NAME:-rr}"
OBSERVER_PIDS=()
SESSION_DIR=""
SESSION_BASENAME=""
STATE_LOGGER_PID=""

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
    test_single_leg_adhesion.sh [--leg rr] [--skip-launch] [--observer] [--manual]

Options:
    --leg          Leg to test: lf, rf, rr, lr. Default: rr.
    --skip-launch  Do not launch ROS nodes; assume the required stack is already running.
    --observer     Start observer windows or background observers automatically.
        --manual       Keep the old step-by-step interactive prompts instead of auto-running.

Environment overrides:
        LIFT_OFFSET_M            Default: 0.025
        FORWARD_OFFSET_M         Default: 0.020
        PRELOAD_OFFSET_M         Default: 0.012
        PRELOAD_SKIRT_TARGET     Default: 0.85
        NORMAL_FORCE_LIMIT       Default: 25.0
        REQUIRED_ADHESION_FORCE  Default: 100.0
        RPM_LOW                  Default: 30000.0
        RPM_HIGH                 Default: 40000.0
    NOMINAL_HOLD_S           Default: 1.0
    LIFT_SETTLE_S            Default: 1.0
    FORWARD_SETTLE_S         Default: 1.0
    PRELOAD_SETTLE_S         Default: 1.5
    LOW_ATTACH_WAIT_S        Default: 2.0
    HIGH_ATTACH_WAIT_S       Default: 3.0
    RELEASE_WAIT_S           Default: 1.0
    JUDGE_TIMEOUT_S          Default: 3.0
    LOG_ROOT_DIR             Default: $WORKSPACE_DIR/test_logs

This script tests a single leg adhesion flow:
    lift -> forward -> preload -> attach -> judge -> release -> return.

Commands are published in delta-from-nominal mode, so x/y/z are body-frame offsets
relative to the nominal universal-joint-center pose.
It also records a timestamped session with state_logger snapshots plus phase markers
and per-phase estimated-state summaries for tuning code and parameters.
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
        --manual)
            MANUAL_MODE=1
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
    if [[ -n "${STATE_LOGGER_PID:-}" ]]; then
        kill "$STATE_LOGGER_PID" >/dev/null 2>&1 || true
    fi
    wait >/dev/null 2>&1 || true
    exit "$exit_code"
}

json_escape() {
    python3 - "$1" <<'PY'
import json
import sys
print(json.dumps(sys.argv[1], ensure_ascii=False))
PY
}

init_session_dir() {
    local timestamp
    timestamp=$(date +%Y%m%d_%H%M%S)
    SESSION_BASENAME="single_leg_adhesion_${LEG_NAME}_${timestamp}"
    SESSION_DIR="$LOG_ROOT_DIR/$SESSION_BASENAME"
    mkdir -p "$SESSION_DIR"
}

log_phase_record() {
    local phase_name="$1"
    local detail="${2:-}"
    local phase_json detail_json now_iso
    phase_json=$(json_escape "$phase_name")
    detail_json=$(json_escape "$detail")
    now_iso=$(date --iso-8601=seconds)
    printf '{"wall_time":"%s","phase":%s,"detail":%s}\n' "$now_iso" "$phase_json" "$detail_json" >> "$SESSION_DIR/phase_markers.jsonl"
}

start_state_logger() {
    local logger_output_dir
    logger_output_dir="$SESSION_DIR/state_logger"
    mkdir -p "$logger_output_dir"
    rosrun climbing_control_core state_logger.py \
        __name:=single_leg_test_state_logger \
        _output_dir:="$logger_output_dir" \
        _session_name:="$SESSION_BASENAME" \
        _snapshot_rate_hz:=20.0 \
        _flush_every_n_records:=5 \
        >/tmp/${SESSION_BASENAME}_state_logger.log 2>&1 &
    STATE_LOGGER_PID=$!
}

save_session_metadata() {
    cat > "$SESSION_DIR/session_meta.env" <<EOF
LEG_NAME=${LEG_NAME}
LEG_INDEX=${LEG_INDEX}
LIFT_OFFSET_M=${LIFT_OFFSET_M}
FORWARD_OFFSET_M=${FORWARD_OFFSET_M}
PRELOAD_OFFSET_M=${PRELOAD_OFFSET_M}
PRELOAD_SKIRT_TARGET=${PRELOAD_SKIRT_TARGET}
NORMAL_FORCE_LIMIT=${NORMAL_FORCE_LIMIT}
REQUIRED_ADHESION_FORCE=${REQUIRED_ADHESION_FORCE}
RPM_LOW=${RPM_LOW}
RPM_HIGH=${RPM_HIGH}
NOMINAL_HOLD_S=${NOMINAL_HOLD_S}
LIFT_SETTLE_S=${LIFT_SETTLE_S}
FORWARD_SETTLE_S=${FORWARD_SETTLE_S}
PRELOAD_SETTLE_S=${PRELOAD_SETTLE_S}
LOW_ATTACH_WAIT_S=${LOW_ATTACH_WAIT_S}
HIGH_ATTACH_WAIT_S=${HIGH_ATTACH_WAIT_S}
RELEASE_WAIT_S=${RELEASE_WAIT_S}
JUDGE_TIMEOUT_S=${JUDGE_TIMEOUT_S}
ROS_SETUP=${ROS_SETUP}
WORKSPACE_DIR=${WORKSPACE_DIR}
EOF
}

capture_topic_once() {
    local topic="$1"
    local output_path="$2"
    timeout 6s rostopic echo -n 1 "$topic" > "$output_path" 2>/dev/null || true
}

capture_phase_snapshot() {
    local label="$1"
    local raw_state_path="$SESSION_DIR/${label}_state_estimated.txt"
    capture_topic_once "/state/estimated" "$raw_state_path"
    if [[ -s "$raw_state_path" ]]; then
        python3 - "$raw_state_path" "$SESSION_DIR/${label}_state_summary.json" "$LEG_INDEX" <<'PY'
import ast
import json
import re
import sys

raw_path, out_path, leg_index = sys.argv[1], sys.argv[2], int(sys.argv[3])
text = open(raw_path, 'r', encoding='utf-8').read()
fields = [
    'wall_touch_mask',
    'preload_ready_mask',
    'attachment_ready_mask',
    'adhesion_mask',
    'seal_confidence',
    'skirt_compression_estimate',
    'fan_current',
    'contact_confidence',
    'support_mask',
    'measured_contact_mask',
    'compression_ready_mask',
    'leg_torque_sum',
    'leg_torque_contact_confidence',
]
summary = {}
for field in fields:
    match = re.search(r'(^|\n)%s:\s*(\[[^\n]*\])' % re.escape(field), text)
    if not match:
        continue
    try:
        values = ast.literal_eval(match.group(2))
    except Exception:
        continue
    summary[field] = values
    if isinstance(values, list) and leg_index < len(values):
        summary[field + '_leg'] = values[leg_index]
open(out_path, 'w', encoding='utf-8').write(json.dumps(summary, indent=2, ensure_ascii=False))
PY
    fi
    capture_topic_once "/jetson/fan_serial_bridge/fan_currents" "$SESSION_DIR/${label}_fan_currents.txt"
    capture_topic_once "/jetson/fan_serial_bridge/leg_rpm" "$SESSION_DIR/${label}_leg_rpm.txt"
    capture_topic_once "/control/swing_leg_target" "$SESSION_DIR/${label}_swing_target.txt"
    capture_topic_once "/jetson/fan_serial_bridge/adhesion_command" "$SESSION_DIR/${label}_adhesion_command.txt"
}

print_phase_summary() {
    local label="$1"
    local summary_path="$SESSION_DIR/${label}_state_summary.json"
    if [[ ! -s "$summary_path" ]]; then
        echo "  state summary unavailable for phase $label"
        return
    fi
    python3 - "$summary_path" <<'PY'
import json
import sys
summary = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
keys = [
    'wall_touch_mask_leg',
    'preload_ready_mask_leg',
    'attachment_ready_mask_leg',
    'adhesion_mask_leg',
    'seal_confidence_leg',
    'skirt_compression_estimate_leg',
    'fan_current_leg',
    'contact_confidence_leg',
    'leg_torque_sum_leg',
    'leg_torque_contact_confidence_leg',
]
parts = []
for key in keys:
    if key in summary:
        parts.append('%s=%s' % (key, summary[key]))
print('  ' + ', '.join(parts))
PY
}

capture_and_report_phase() {
    local label="$1"
    capture_phase_snapshot "$label"
    print_phase_summary "$label"
}

wait_for_leg_success() {
    local label="$1"
    local timeout_s="$2"
    local start_sec now_sec
    start_sec=$(date +%s)
    while true; do
        capture_phase_snapshot "$label"
        local summary_path="$SESSION_DIR/${label}_state_summary.json"
        if [[ -s "$summary_path" ]]; then
            if python3 - "$summary_path" <<'PY'
import json
import sys
summary = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
success = bool(summary.get('attachment_ready_mask_leg', False) or summary.get('adhesion_mask_leg', False))
sys.exit(0 if success else 1)
PY
            then
                print_phase_summary "$label"
                return 0
            fi
        fi
        now_sec=$(date +%s)
        if (( now_sec - start_sec >= ${timeout_s%.*} )); then
            print_phase_summary "$label"
            return 1
        fi
        sleep 0.5
    done
}

sleep_phase() {
    local seconds="$1"
    python3 - "$seconds" <<'PY'
import sys, time
time.sleep(float(sys.argv[1]))
PY
}

run_phase() {
    local phase_key="$1"
    local description="$2"
    local x="$3"
    local y="$4"
    local z="$5"
    local skirt_target="$6"
    local force_limit="$7"
    local settle_s="$8"
    log_phase_record "$phase_key" "$description"
    publish_leg_target "$x" "$y" "$z" "$skirt_target" "$force_limit" "$description"
    sleep_phase "$settle_s"
    capture_and_report_phase "$phase_key"
}

run_adhesion_phase() {
    local phase_key="$1"
    local description="$2"
    local rpm="$3"
    local mode="$4"
    local settle_s="$5"
    log_phase_record "$phase_key" "$description"
    publish_leg_adhesion "$rpm" "$mode" "$description"
    sleep_phase "$settle_s"
    capture_and_report_phase "$phase_key"
}

run_manual_sequence() {
    pause_step "Press Enter to send ${LEG_NAME} to nominal hold..."
    publish_leg_target "0.0" "0.0" "0.0" "0.0" "10.0" "${LEG_NAME^^} nominal hold"

    pause_step "Press Enter to move ${LEG_NAME} into preload pose..."
    publish_leg_target "0.0" "0.0" "-${PRELOAD_OFFSET_M}" "${PRELOAD_SKIRT_TARGET}" "${NORMAL_FORCE_LIMIT}" "${LEG_NAME^^} preload pose"

    pause_step "Press Enter to command low-rpm adhesion for ${LEG_NAME}..."
    publish_leg_adhesion "${RPM_LOW}" "1" "${LEG_NAME^^} adhesion command at low rpm"

    pause_step "Observe current and masks, then press Enter to command high-rpm adhesion..."
    publish_leg_adhesion "${RPM_HIGH}" "2" "${LEG_NAME^^} adhesion command at high rpm"

    pause_step "Observe current and masks, then press Enter to release the fan..."
    publish_leg_adhesion "0.0" "0" "${LEG_NAME^^} adhesion release"

    pause_step "Press Enter to return ${LEG_NAME} to nominal hold..."
    publish_leg_target "0.0" "0.0" "0.0" "0.0" "10.0" "${LEG_NAME^^} return to nominal"
}

run_auto_sequence() {
    log_phase_record "session_start" "Automated lift-forward-preload-attach-judge test started"
    run_phase "nominal_hold" "${LEG_NAME^^} nominal hold" "0.0" "0.0" "0.0" "0.0" "10.0" "$NOMINAL_HOLD_S"
    run_phase "lift" "${LEG_NAME^^} lift phase" "0.0" "0.0" "$LIFT_OFFSET_M" "0.0" "10.0" "$LIFT_SETTLE_S"
    run_phase "forward" "${LEG_NAME^^} forward phase" "$FORWARD_OFFSET_M" "0.0" "$LIFT_OFFSET_M" "0.0" "10.0" "$FORWARD_SETTLE_S"
    run_phase "preload" "${LEG_NAME^^} preload phase" "$FORWARD_OFFSET_M" "0.0" "-${PRELOAD_OFFSET_M}" "$PRELOAD_SKIRT_TARGET" "$NORMAL_FORCE_LIMIT" "$PRELOAD_SETTLE_S"
    run_adhesion_phase "attach_low" "${LEG_NAME^^} low-rpm adhesion command" "$RPM_LOW" "1" "$LOW_ATTACH_WAIT_S"
    run_adhesion_phase "attach_high" "${LEG_NAME^^} high-rpm adhesion command" "$RPM_HIGH" "2" "$HIGH_ATTACH_WAIT_S"

    echo
    echo "==> ${LEG_NAME^^} judging attachment outcome"
    if wait_for_leg_success "judge" "$JUDGE_TIMEOUT_S"; then
        echo "  attachment judgement: SUCCESS"
        log_phase_record "judge" "SUCCESS: attachment_ready_mask or adhesion_mask became true"
    else
        echo "  attachment judgement: FAILED"
        log_phase_record "judge" "FAILED: attachment_ready_mask and adhesion_mask stayed false within timeout"
    fi

    run_adhesion_phase "release" "${LEG_NAME^^} adhesion release" "0.0" "0" "$RELEASE_WAIT_S"
    run_phase "return_nominal" "${LEG_NAME^^} return to nominal" "0.0" "0.0" "0.0" "0.0" "10.0" "$NOMINAL_HOLD_S"
    log_phase_record "session_end" "Automated sequence complete"
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
init_session_dir
save_session_metadata

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
        leg_ik_input_mode:=delta_from_nominal_m \
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

echo
echo "Single-leg adhesion test ready for ${LEG_NAME}."
echo "Leg IK input mode: delta_from_nominal_m"
echo "Session directory: ${SESSION_DIR}"
echo "Manual mode: ${MANUAL_MODE}"
echo "Lift offset: ${LIFT_OFFSET_M} m"
echo "Forward offset: ${FORWARD_OFFSET_M} m"
echo "Preload normal offset: ${PRELOAD_OFFSET_M} m"
echo "Low/High target rpm: ${RPM_LOW} / ${RPM_HIGH}"
start_state_logger
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
echo "  session logs under ${SESSION_DIR}"

if [[ "$MANUAL_MODE" -eq 1 ]]; then
    run_manual_sequence
else
    run_auto_sequence
fi

echo
echo "Adhesion test sequence finished. Session data saved to ${SESSION_DIR}. Press Ctrl+C to stop the launched nodes, or keep them running for further observation."
while true; do
    sleep 1
done