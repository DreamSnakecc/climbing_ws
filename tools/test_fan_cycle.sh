#!/usr/bin/env bash
#
# 风机启停测试便捷脚本.
#
# 测试流程（每个 cycle）:
#   1. 全关 (2s baseline)
#   2. 全开 (--all-on-duration)
#   3. 按 [lf → rf → rr → lr] 顺序依次关闭每腿风机
#      (每腿关闭后等待 --leg-dwell)
#   4. 全关 (2s settle)
#   重复 --cycles 次
#
# 前置条件:
#   Jetson:  roslaunch climbing_bringup jetson_bringup.launch
#   PC:      roslaunch climbing_bringup test_crawl_gait.launch \
#              enable_auto_adhesion_commands:=false ujc_z_mm:=-195.5
#
# 注意: 必须设置 enable_auto_adhesion_commands:=false
#       否则 mission_supervisor 会与测试脚本争夺风机控制权.

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
NODE_WAIT_TIMEOUT="${NODE_WAIT_TIMEOUT:-30}"

usage() {
    cat <<'EOF'
Usage:
    ./tools/test_fan_cycle.sh [-- <test_fan_cycle.py args...>]

任何放在 `--` 之后的参数都会原样转发给 test_fan_cycle.py, 例如:

    ./tools/test_fan_cycle.sh -- --target-rpm 35000 --all-on-duration 5 \\
        --leg-dwell 3 --cycles 2

    ./tools/test_fan_cycle.sh -- --output-dir /tmp/fan_tests

参数列表:
    --target-rpm RPM       风机开启时的目标转速 (默认 35000)
    --all-on-duration S    全开保持时间秒 (默认 5)
    --leg-dwell S          每腿关闭后等待秒 (默认 4)
    --cycles N             cycle 次数 (默认 3)
    --output-dir DIR       CSV 输出目录 (默认 test_logs)

环境变量:
    WORKSPACE_DIR        默认 $HOME/climbing_ws
    ROS_SETUP            指定 ROS setup.bash 路径
    NODE_WAIT_TIMEOUT    等节点超时 (秒, 默认 30)
EOF
}

if [[ "${1-}" == "-h" || "${1-}" == "--help" ]]; then
    usage
    exit 0
fi

# 把 `--` 之后的参数收集起来作为 python 脚本的入参
PY_ARGS=()
SAW_DASHDASH=0
for arg in "$@"; do
    if [[ "$SAW_DASHDASH" -eq 1 ]]; then
        PY_ARGS+=("$arg")
    elif [[ "$arg" == "--" ]]; then
        SAW_DASHDASH=1
    else
        echo "Unrecognized argument: $arg (use '--' to forward args to test_fan_cycle.py)" >&2
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

wait_for_node() {
    local node="$1"
    local timeout="$2"
    local start
    start=$(date +%s)
    while true; do
        if rosnode list 2>/dev/null | grep -qx "$node"; then
            return 0
        fi
        if (( $(date +%s) - start >= timeout )); then
            echo "Timed out waiting for node $node (>${timeout}s)." >&2
            return 1
        fi
        sleep 0.5
    done
}

source_environment

REQUIRED_NODES=(
    /jetson/fan_serial_bridge
)

echo "Waiting for required nodes (timeout=${NODE_WAIT_TIMEOUT}s each)..."
for node in "${REQUIRED_NODES[@]}"; do
    wait_for_node "$node" "$NODE_WAIT_TIMEOUT" || {
        echo "Required node missing: $node" >&2
        echo "请确认已启动:" >&2
        echo "  Jetson:  roslaunch climbing_bringup jetson_bringup.launch" >&2
        echo "  PC:      roslaunch climbing_bringup test_crawl_gait.launch" >&2
        echo "           enable_auto_adhesion_commands:=false" >&2
        exit 1
    }
    echo "  ok: $node"
done

echo
echo "NOTE: mission_supervisor must have enable_auto_adhesion_commands=false"
echo "      to avoid conflicting fan commands."
echo
echo "All nodes ready, starting test_fan_cycle.py ..."
echo

exec rosrun climbing_control_core test_fan_cycle.py "${PY_ARGS[@]}"
