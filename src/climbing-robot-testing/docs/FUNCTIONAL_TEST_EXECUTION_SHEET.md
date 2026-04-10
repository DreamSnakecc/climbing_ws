# climbing-robot-testing 实验执行版测试文档

## 1. 用途

本文档作为攀爬机器人测试的综合指南，详细描述了测试的目的、实验信息、通用准备命令、记录格式、环境与构建检查、节点启动检查、消息接口验证以及单节点功能验证。

## 2. 实验基础信息

- 实验日期：
- 实验人员：
- 机器人编号：
- 场地/壁面条件：
- 软件版本：
- `robot.yaml` 版本或哈希：
- 是否启用 logger：
- 是否录制 rosbag：
- 备注：

## 3. 通用准备命令

每个终端建议先执行：

```bash
cd ~/climbing-robot-testing
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

如果尚未构建：

```bash
cd ~/climbing-robot-testing
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

建议常驻的辅助观察命令：

```bash
rosnode list
rostopic list
rostopic hz /state/estimated
rostopic echo -n 1 /state/estimated
rosparam get /state_logger
```

建议的通用录包命令：

```bash
rosbag record -O test_session.bag /state/estimated /control/body_reference /control/swing_leg_target /control/mission_state /jetson/fan_serial_bridge/adhesion_command /jetson/fan_serial_bridge/fan_currents /jetson/dynamixel_bridge/joint_currents
```

## 4. 记录格式约定

每个测试项都使用以下结果标记：

- `[ ] 通过`
- `[ ] 失败`
- `[ ] 阻塞`

每个测试项至少填写：

- 实际现象：
- 关键观测数据：
- 问题原因或怀疑点：
- 后续动作：

---

## 5. 环境与构建检查

### T01 工作区构建

- 目标：确认工作区可编译，消息生成完整
- 验证方式：离线
- 前置条件：ROS1 环境已加载

执行命令：

```bash
cd ~/climbing-robot-testing
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
rosmsg show climbing_msgs/EstimatedState
```

观测点：

- `catkin build` 是否报错
- `EstimatedState` 是否包含 `wall_touch_mask`、`preload_ready_mask`、`attachment_ready_mask`

通过标准：

- 构建无错误
- 新增消息字段存在

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T02 参数文件加载

- 目标：确认关键参数能被 launch 正常加载
- 验证方式：离线
- 前置条件：工作区已 source

执行命令：

终端 1：

```bash
roslaunch climbing_bringup pc_bringup.launch
```

终端 2：

```bash
rosparam get /state_logger
rosparam get /swing_leg_controller
rosparam get /mission_supervisor
rosparam get /state_estimator
```

观测点：

- 参数树是否存在
- `state_logger` 和新 swing 参数是否完整

通过标准：

- 关键参数均能读取
- 无参数缺失导致的启动错误

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T03 启动文件基本正确性

- 目标：确认三类 launch 入口可正常拉起
- 验证方式：离线
- 前置条件：工作区已 source

执行命令：

```bash
roslaunch climbing_bringup pc_bringup.launch
roslaunch climbing_bringup pc_bringup.launch enable_state_logger:=true
roslaunch climbing_bringup jetson_bringup.launch
roslaunch climbing_bringup replay_bringup.launch
```

观测点：

- launch 是否报 XML/参数错误
- 启用 logger 时是否启动 `state_logger`

通过标准：

- 四条路径均可进入运行状态

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 6. 节点启动检查

### T04 PC 侧节点启动检查

- 目标：确认 PC 侧控制链全部存活
- 验证方式：离线
- 前置条件：PC 侧 launch 已启动

执行命令：

```bash
rosnode list
rosnode list | grep mocap_bridge
rosnode list | grep state_estimator
rosnode list | grep body_planner
rosnode list | grep stance_force_optimizer
rosnode list | grep swing_leg_controller
rosnode list | grep mission_supervisor
```

观测点：

- 节点是否存在
- 是否频繁崩溃重启

通过标准：

- 节点全部存活且稳定

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T05 Jetson 侧节点启动检查

- 目标：确认 Jetson 硬件桥接链路全部存活
- 验证方式：上机
- 前置条件：Jetson 侧已连接硬件

执行命令：

```bash
roslaunch climbing_bringup jetson_bringup.launch
rosnode list | grep jetson
rosnode list | grep multi_dxl_node_left
rosnode list | grep multi_dxl_node_right
rosnode list | grep dynamixel_bridge
rosnode list | grep imu_serial_bridge
rosnode list | grep fan_serial_bridge
rosnode list | grep leg_ik_executor
rosnode list | grep local_safety_supervisor
```

观测点：

- 节点是否启动
- 串口和电机通讯是否持续报错

通过标准：

- 所有节点均稳定运行

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T06 Logger 节点启动检查

- 目标：确认 logger 节点和日志目录正常工作
- 验证方式：离线
- 前置条件：PC 侧可启动 logger

执行命令：

```bash
roslaunch climbing_bringup pc_bringup.launch enable_state_logger:=true
rosnode list | grep state_logger
rosparam get /state_logger
ls -lah ~/.ros/climbing_logs
find ~/.ros/climbing_logs -maxdepth 2 -type f
```

观测点：

- `state_logger` 节点是否存在
- 是否生成 `params.json`、`events.jsonl`、`snapshots.jsonl`

通过标准：

- logger 正常启动并持续写文件

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 7. 消息接口验证

### T07 估计状态消息完整性

- 目标：确认 `/state/estimated` 完整且包含新增字段
- 验证方式：离线
- 前置条件：PC 侧节点已启动

执行命令：

```bash
rostopic echo -n 1 /state/estimated
rostopic echo -n 1 /state/estimated | grep -E "wall_touch_mask|preload_ready_mask|attachment_ready_mask|seal_confidence|leg_torque_sum|leg_torque_contact_confidence"
rostopic hz /state/estimated
```

观测点：

- 字段是否存在
- 数组长度是否正确
- 发布频率是否正常

通过标准：

- 所有关键字段存在且语义正常

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T08 BodyReference 发布正确性

- 目标：确认 body 参考轨迹持续发布
- 验证方式：离线
- 前置条件：`body_planner` 正在运行

执行命令：

```bash
rostopic echo -n 3 /control/body_reference
rostopic hz /control/body_reference
```

观测点：

- `pose`、`twist`、`support_mask`、`gait_mode`

通过标准：

- 消息持续发布且支撑节拍合理

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T09 SwingLegTarget 发布正确性

- 目标：确认摆动腿目标能随 gait 更新
- 验证方式：离线
- 前置条件：`swing_leg_controller` 正在运行

执行命令：

```bash
rostopic echo /control/swing_leg_target
rostopic echo /control/swing_leg_target | grep -E "leg_name|support_leg|skirt_compression_target|desired_normal_force_limit"
```

观测点：

- `leg_name`
- `center`
- `support_leg`
- `skirt_compression_target`

通过标准：

- 摆动腿和支撑腿在输出上有明确区分

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T10 支撑力消息完整性

- 目标：确认四条腿的支撑力话题内容完整
- 验证方式：离线
- 前置条件：`stance_force_optimizer` 正在运行

执行命令：

```bash
rostopic echo -n 1 /control/stance_wrench/lf
rostopic echo -n 1 /control/stance_wrench/rf
rostopic echo -n 1 /control/stance_wrench/lr
rostopic echo -n 1 /control/stance_wrench/rr
rostopic hz /control/stance_wrench/lf
```

观测点：

- `planned_support`
- `actual_contact`
- `active`
- `required_adhesion_force`

通过标准：

- 只有有效支撑腿输出激活支撑力

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T11 Jetson 桥接消息链路

- 目标：确认 Jetson 侧反馈链路持续更新
- 验证方式：上机
- 前置条件：Jetson 侧节点已启动并接入硬件

执行命令：

```bash
rostopic echo -n 1 /jetson/dynamixel_bridge/joint_state
rostopic echo -n 1 /jetson/dynamixel_bridge/joint_currents
rostopic echo -n 1 /jetson/fan_serial_bridge/fan_currents
rostopic echo -n 1 /jetson/imu_serial_bridge/imu
rostopic hz /jetson/dynamixel_bridge/joint_state
rostopic hz /jetson/fan_serial_bridge/fan_currents
rostopic hz /jetson/imu_serial_bridge/imu
```

观测点：

- 时间戳连续性
- 数值非全零
- 频率无明显中断

通过标准：

- 反馈链路连续、稳定、可用

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 8. 单节点功能验证

### T12 State Estimator 接触状态判定

- 目标：确认接触、预压、吸附状态的估计顺序正确
- 验证方式：离线 + 上机
- 前置条件：有扭矩、电流、风机反馈

执行命令：

```bash
rostopic echo /state/estimated
rostopic echo /state/estimated | grep -E "measured_contact_mask|wall_touch_mask|compression_ready_mask|preload_ready_mask|adhesion_mask|attachment_ready_mask"
```

观测点：

- 从自由空间到轻触、预压、吸附的状态演化

通过标准：

- 不会出现未接触直接 `attachment_ready_mask=true`

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T13 Swing Leg 五阶段行为检查

- 目标：确认摆动腿遵循五阶段引导吸附逻辑
- 验证方式：上机
- 前置条件：可安全执行单腿摆动

执行命令：

终端 1：

```bash
rostopic echo /control/swing_leg_target
```

终端 2：

```bash
rostopic echo /state/estimated | grep -E "wall_touch_mask|preload_ready_mask|attachment_ready_mask|skirt_compression_estimate|seal_confidence"
```

终端 3：

```bash
rostopic echo /jetson/fan_serial_bridge/adhesion_command
```

如果启用 logger：

```bash
tail -f ~/.ros/climbing_logs/<session_dir>/events.jsonl
```

观测点：

- 先轻触滑移
- 再切向对准
- 再法向预压
- 再风机吸附
- 再柔顺稳定

通过标准：

- 不跳相、不乱序、不提前强吸附

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T14 Mission Supervisor 风机时序检查

- 目标：确认风机命令受状态驱动而非纯时间驱动
- 验证方式：离线 + 上机
- 前置条件：`mission_supervisor` 正在运行

执行命令：

```bash
rostopic echo /control/mission_state
rostopic echo /jetson/fan_serial_bridge/adhesion_command
rostopic echo /jetson/fan_serial_bridge/adhesion_command | grep -E "target_rpm|normal_force_limit|required_adhesion_force|mode|leg_index"
```

观测点：

- `target_rpm`
- `normal_force_limit`
- `required_adhesion_force`
- mission 状态切换

通过标准：

- 风机增强时机和壁面接触/预压/吸附状态一致

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T15 Stance Force Optimizer 支撑腿激活逻辑

- 目标：确认 QP 只对 `attachment_ready_mask=true` 的腿激活
- 验证方式：离线
- 前置条件：估计器和优化器同步运行

执行命令：

```bash
rostopic echo /state/estimated | grep -E "attachment_ready_mask|support_mask|adhesion_mask"
rostopic echo /control/stance_wrench/lf | grep -E "active|planned_support|actual_contact|required_adhesion_force|normal_force_limit"
rostopic echo /control/stance_wrench/rf | grep -E "active|planned_support|actual_contact|required_adhesion_force|normal_force_limit"
rostopic echo /control/stance_wrench/lr | grep -E "active|planned_support|actual_contact|required_adhesion_force|normal_force_limit"
rostopic echo /control/stance_wrench/rr | grep -E "active|planned_support|actual_contact|required_adhesion_force|normal_force_limit"
```

观测点：

- `attachment_ready_mask` 与各腿 `active` 的一致性

通过标准：

- 未准备好的腿不会被错误纳入 QP 支撑集合

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T16 Leg IK Executor 映射检查

- 目标：确认 `LegCenterCommand` 到关节执行的方向和幅值正确
- 验证方式：上机
- 前置条件：硬件可响应位置指令

执行命令：

```bash
rosmsg show climbing_msgs/LegCenterCommand
rostopic pub -1 /control/swing_leg_target climbing_msgs/LegCenterCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
leg_name: 'lf'
center: {x: 0.01, y: 0.01, z: 0.0}
center_velocity: {x: 0.0, y: 0.0, z: 0.0}
skirt_compression_target: 0.0
support_leg: false
desired_normal_force_limit: 10.0"
rostopic echo /jetson/dynamixel_bridge/joint_state
```

观测点：

- 关节响应方向
- 是否越界
- 是否存在轴向颠倒

通过标准：

- 末端位移方向与坐标定义一致

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---