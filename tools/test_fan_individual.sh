#!/usr/bin/env bash
#
# 单腿风机隔离测试脚本.
#
# 逐腿测试：将目标风机开启到目标 RPM，其他风机关闭，
# 观察 RPM 响应和关闭行为。用于诊断哪条腿的风机通道异常。
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
    ./tools/test_fan_individual.sh [-- <test_fan_individual.py args...>]

`--` 之后的参数会转发给 test_fan_individual.py:

    ./tools/test_fan_individual.sh -- --target-rpm 30000 --dwell-on 4 --dwell-off 3
    ./tools/test_fan_individual.sh -- --reverse   (额外运行反向映射测试)
    ./tools/test_fan_individual.sh -- --target-rpm 20000 --dwell-on 6 --reverse

参数:
    --target-rpm RPM   风机目标转速 (默认 30000)
    --dwell-on S       每个腿开启观察时间秒 (默认 4)
    --dwell-off S      关闭后观察时间秒 (默认 3)
    --reverse          额外运行反向映射测试 (+30s)

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
echo "All nodes ready, starting test_fan_individual.py ..."
echo

exec rosrun climbing_control_core test_fan_individual.py "${PY_ARGS[@]}"
