#!/usr/bin/env bash
#
# 整机 crawl 步态测试便捷脚本.
#
# 该脚本只做最小化的一件事:
#   1. source ROS / workspace 环境
#   2. 等关键节点存在 (Jetson + PC 两端的核心节点)
#   3. rosrun climbing_control_core test_crawl_gait.py 转发用户参数
#
# 它不再帮你启动 jetson_bringup / pc_bringup. 启动顺序由用户掌控:
#
#   # Jetson:
#   roslaunch climbing_bringup jetson_bringup.launch
#   # 等四腿稳定停在 operating UJC (270.21, 0, -100) mm
#
#   # PC (另一台机或另一个终端):
#   roslaunch climbing_bringup test_crawl_gait.launch ujc_z_mm:=-195.5
#   # mission_state 进入 INIT, 四腿在位置模式下持位
#
#   # 然后跑测试:
#   ./tools/test_crawl_gait.sh --duration 60
#
# 测试脚本流程见 test_crawl_gait.py 文件头注释.

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
NODE_WAIT_TIMEOUT="${NODE_WAIT_TIMEOUT:-30}"

usage() {
    cat <<'EOF'
Usage:
    ./tools/test_crawl_gait.sh [-- <test_crawl_gait.py args...>]

任何放在 `--` 之后的参数都会原样转发给 test_crawl_gait.py, 例如:

    ./tools/test_crawl_gait.sh -- --duration 90 --hold-check-s 5
    ./tools/test_crawl_gait.sh -- --no-confirm --duration 30
    ./tools/test_crawl_gait.sh -- --output-dir /tmp/crawl_runs

环境变量:
    WORKSPACE_DIR        默认 $HOME/climbing_ws
    ROS_SETUP            指定 ROS setup.bash 路径; 未设置时自动按 ROS_DISTRO
                         或 /opt/ros/noetic/setup.bash 兜底
    NODE_WAIT_TIMEOUT    等节点超时 (秒, 默认 30)

前置条件 (脚本不会代为启动):
    Jetson:  roslaunch climbing_bringup jetson_bringup.launch
    PC:      roslaunch climbing_bringup test_crawl_gait.launch ujc_z_mm:=-195.5
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
        echo "Unrecognized argument: $arg (use '--' to forward args to test_crawl_gait.py)" >&2
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
    /jetson/leg_ik_executor
    /jetson/dynamixel_bridge
    /jetson/fan_serial_bridge
    /state_estimator
    /body_planner
    /swing_leg_controller
    /mission_supervisor
)

echo "Waiting for required nodes (timeout=${NODE_WAIT_TIMEOUT}s each)..."
for node in "${REQUIRED_NODES[@]}"; do
    wait_for_node "$node" "$NODE_WAIT_TIMEOUT" || {
        echo "Required node missing: $node" >&2
        echo "请确认已分别启动:" >&2
        echo "  Jetson:  roslaunch climbing_bringup jetson_bringup.launch" >&2
        echo "  PC:      roslaunch climbing_bringup test_crawl_gait.launch ujc_z_mm:=-195.5" >&2
        exit 1
    }
    echo "  ok: $node"
done

echo
echo "All nodes ready, starting test_crawl_gait.py ..."
echo

exec rosrun climbing_control_core test_crawl_gait.py "${PY_ARGS[@]}"
