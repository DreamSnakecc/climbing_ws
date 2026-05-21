#!/usr/bin/env bash
#
# 吸附爬行联合测试便捷脚本.
#
# 该脚本只做最小化的一件事:
#   1. source ROS / workspace 环境
#   2. 等关键节点存在 (Jetson + PC 两端的核心节点)
#   3. rosrun climbing_control_core test_crawl_gait_with_fan.py 转发用户参数
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
#   # 然后跑测试 (监控模式, mission_supervisor 自动控制风机):
#   ./tools/test_crawl_gait_with_fan.sh -- --duration 60
#
# 测试脚本流程见 test_crawl_gait_with_fan.py 文件头注释.

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/climbing_ws}"
ROS_SETUP="${ROS_SETUP:-}"
NODE_WAIT_TIMEOUT="${NODE_WAIT_TIMEOUT:-30}"

usage() {
    cat <<'EOF'
Usage:
    ./tools/test_crawl_gait_with_fan.sh [-- <test_crawl_gait_with_fan.py args...>]

任何放在 `--` 之后的参数都会原样转发给 test_crawl_gait_with_fan.py, 例如:

    ./tools/test_crawl_gait_with_fan.sh -- --duration 90 --hold-check-s 5
    ./tools/test_crawl_gait_with_fan.sh -- --no-confirm --duration 30
    ./tools/test_crawl_gait_with_fan.sh -- --output-dir /tmp/crawl_runs

参数列表 (转发至 test_crawl_gait_with_fan.py):
    --duration S          CLIMB 记录时长秒 (默认 60)
    --log-rate-hz N       CSV 采样率 (默认 50)
    --hold-check-s S      INIT 持位检查时长 (默认 3)
    --hold-tol-mm N       UJC z 偏差容差 (默认 20)
    --allow-init-mismatch 即使 INIT 检查失败也继续
    --stick-timeout-s S   STICK 等待超时 (默认 15)
    --climb-timeout-s S   CLIMB 等待超时 (默认 30)
    --stream-timeout-s S  主题流等待超时 (默认 10)
    --output-dir DIR      CSV 输出目录 (默认 ~/climbing_ws/test_logs)
    --no-confirm          跳过人工确认 (无人批处理模式)

环境变量:
    WORKSPACE_DIR        默认 $HOME/climbing_ws
    ROS_SETUP            指定 ROS setup.bash 路径; 未设置时自动按 ROS_DISTRO
                         或 /opt/ros/noetic/setup.bash 兜底
    NODE_WAIT_TIMEOUT    等节点超时 (秒, 默认 30)

前置条件 (脚本不会代为启动):
    Jetson:  roslaunch climbing_bringup jetson_bringup.launch
    PC:      roslaunch climbing_bringup test_crawl_gait.launch ujc_z_mm:=-195.5
    (mission_supervisor enable_auto_adhesion_commands=true, 默认开启)
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
        echo "Unrecognized argument: $arg (use '--' to forward args to test_crawl_gait_with_fan.py)" >&2
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

# ---------------------------------------------------------------------------
# wait for ros nodes
# ---------------------------------------------------------------------------
source_environment

NODE_LIST=$(mktemp)
trap 'rm -f "$NODE_LIST"' EXIT

wait_for_node() {
    local node_pattern=$1
    local label=$2
    local deadline
    deadline=$(($(date +%s) + NODE_WAIT_TIMEOUT))
    while [[ $(date +%s) -lt $deadline ]]; do
        rosnode list > "$NODE_LIST" 2>/dev/null || true
        if grep -qE "$node_pattern" "$NODE_LIST"; then
            echo "[OK] $label"
            return 0
        fi
        sleep 1
    done
    echo "[ERROR] $label not found within ${NODE_WAIT_TIMEOUT}s" >&2
    echo "  nodes seen:"
    sed 's/^/    /' "$NODE_LIST"
    return 1
}

echo "=== test_crawl_gait_with_fan: waiting for nodes ==="
wait_for_node "fan_serial_bridge"     "fan_serial_bridge (Jetson)"
wait_for_node "mission_supervisor"    "mission_supervisor"
wait_for_node "swing_leg_controller"  "swing_leg_controller"
echo ""

# ---------------------------------------------------------------------------
# run test
# ---------------------------------------------------------------------------
echo "=== test_crawl_gait_with_fan: starting ==="
rosrun climbing_control_core test_crawl_gait_with_fan.py "${PY_ARGS[@]}"
EXIT_CODE=$?

if [[ $EXIT_CODE -eq 0 ]]; then
    echo "=== test_crawl_gait_with_fan: PASS (exit=0) ==="
elif [[ $EXIT_CODE -eq 2 ]]; then
    echo "=== test_crawl_gait_with_fan: FAIL (stream timeout) ===" >&2
elif [[ $EXIT_CODE -eq 3 ]]; then
    echo "=== test_crawl_gait_with_fan: FAIL (INIT hold check) ===" >&2
elif [[ $EXIT_CODE -eq 4 ]]; then
    echo "=== test_crawl_gait_with_fan: FAIL (STICK timeout) ===" >&2
elif [[ $EXIT_CODE -eq 5 ]]; then
    echo "=== test_crawl_gait_with_fan: FAIL (CLIMB timeout) ===" >&2
elif [[ $EXIT_CODE -eq 6 ]]; then
    echo "=== test_crawl_gait_with_fan: FAIL (FAULT during CLIMB) ===" >&2
elif [[ $EXIT_CODE -eq 130 ]]; then
    echo "=== test_crawl_gait_with_fan: CANCELLED (Ctrl+C) ==="
else
    echo "=== test_crawl_gait_with_fan: EXIT CODE $EXIT_CODE ==="
fi

exit $EXIT_CODE
