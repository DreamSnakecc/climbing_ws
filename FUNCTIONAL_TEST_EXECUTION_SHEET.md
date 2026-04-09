# climbing_ws 实验执行版测试文档

## 1. 用途

本文档把 [FUNCTIONAL_TEST_CHECKLIST.md](d:/Doctor/2026-3/A1-QP-MPC-Controller-main/climbing_ws/FUNCTIONAL_TEST_CHECKLIST.md) 和 [FUNCTIONAL_TEST_COMMANDS.md](d:/Doctor/2026-3/A1-QP-MPC-Controller-main/climbing_ws/FUNCTIONAL_TEST_COMMANDS.md) 合并为一份现场执行文档。目标是让测试人员在单一文档内同时看到：

- 测试目的
- 前置条件
- 执行命令
- 观测点
- 通过标准
- 记录栏

建议执行时直接在本文档中补充结果，形成每轮实验的原始记录。

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
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

如果尚未构建：

```bash
cd ~/climbing_ws
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

- 实际现象
- 关键观测数据
- 问题原因或怀疑点
- 后续动作

---

## 5. 第一层：环境与构建

### T01 工作区构建

- 目标：确认工作区可编译，消息生成完整
- 验证方式：离线
- 前置条件：ROS1 环境已加载

执行命令：

```bash
cd ~/climbing_ws
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

## 6. 第二层：节点启动与基础健康度

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

## 7. 第三层：消息接口验证

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

## 8. 第四层：单节点功能验证

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

## 9. 第五层：子系统联调

### T17 PC 控制链闭环联调

- 目标：确认 PC 内部规划、估计、摆动、QP 链路闭环一致
- 验证方式：离线
- 前置条件：PC 侧节点全部启动

执行命令：

```bash
rostopic echo /control/body_reference
rostopic echo /state/estimated
rostopic echo /control/swing_leg_target
rostopic echo /control/stance_wrench/lf
rosbag record -O pc_chain_debug.bag /state/estimated /control/body_reference /control/swing_leg_target /control/stance_wrench/lf /control/stance_wrench/rf /control/stance_wrench/lr /control/stance_wrench/rr
```

观测点：

- `support_mask` 在规划、估计、力输出之间是否一致传播

通过标准：

- 不存在明显的支撑集合错位

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T18 PC-Jetson 跨机链路联调

- 目标：确认 PC 与 Jetson 之间控制与反馈双向连通
- 验证方式：上机
- 前置条件：PC 与 Jetson ROS 网络配置正确

执行命令：

PC 侧：

```bash
rostopic list | grep jetson
rostopic hz /jetson/dynamixel_bridge/joint_state
rostopic hz /jetson/fan_serial_bridge/fan_currents
```

Jetson 侧：

```bash
rostopic hz /control/swing_leg_target
rostopic hz /jetson/fan_serial_bridge/adhesion_command
```

观测点：

- 反馈频率
- 控制命令频率
- 是否出现中断

通过标准：

- 双向消息流连续稳定

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T19 Logger 联调验证

- 目标：确认 logger 足以支持调参和复盘
- 验证方式：离线 + 上机
- 前置条件：logger 已启用

执行命令：

```bash
roslaunch climbing_bringup pc_bringup.launch enable_state_logger:=true
ls -1t ~/.ros/climbing_logs | head
cat ~/.ros/climbing_logs/<session_dir>/params.json
grep -n "attachment_ready_mask" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
grep -n "seal_confidence" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
grep -n "target_rpm" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
grep -n "required_adhesion_force" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
```

观测点：

- 是否能看到状态、命令和反馈的时间对应关系

通过标准：

- 日志足以定位吸附失败、切换异常和参数不合理问题

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 10. 第六层：全链路行为

### T20 静态全支撑保持

- 目标：确认全支撑静态状态稳定
- 验证方式：上机
- 前置条件：机器人已稳定吸附

执行命令：

```bash
rostopic echo /state/estimated | grep -E "support_mask|adhesion_mask|slip_risk|fan_current"
rostopic echo /jetson/dynamixel_bridge/joint_currents
```

观测点：

- `slip_risk` 是否持续上升
- 电流是否振荡异常

通过标准：

- 静态保持阶段无脱附、无大振荡

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T21 单步摆动换腿测试

- 目标：确认单次换腿链路完整有效
- 验证方式：上机
- 前置条件：静态吸附稳定

执行命令：

```bash
rostopic echo /control/swing_leg_target
rostopic echo /state/estimated | grep -E "support_mask|attachment_ready_mask|preload_ready_mask|adhesion_mask"
rostopic echo /control/mission_state
```

观测点：

- 旧支撑腿退出
- 新腿预压吸附
- 新腿成功进入支撑

通过标准：

- 单步换腿期间机体保持稳定

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T22 连续 crawl 步态测试

- 目标：确认系统能连续完成多个步态周期
- 验证方式：上机
- 前置条件：单步换腿已通过

执行命令：

```bash
rosbag record -O crawl_cycle_test.bag /state/estimated /control/body_reference /control/swing_leg_target /control/mission_state /jetson/fan_serial_bridge/adhesion_command /jetson/fan_serial_bridge/fan_currents /jetson/dynamixel_bridge/joint_currents
rostopic echo /state/estimated | grep -E "attachment_ready_mask|support_mask|slip_risk"
```

观测点：

- 步态节拍稳定性
- 机体姿态漂移
- 是否频繁早接触或吸附失败

通过标准：

- 至少连续完成多个步态周期

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T23 摆动腿对壁引导吸附专项测试

- 目标：确认摆动腿导向贴壁、预压、吸附、柔顺收敛符合设计预期
- 验证方式：上机
- 前置条件：可安全观察单腿末端行为

执行命令：

```bash
rostopic echo /control/swing_leg_target
rostopic echo /state/estimated | grep -E "wall_touch_mask|preload_ready_mask|attachment_ready_mask|seal_confidence|skirt_compression_estimate|leg_torque_sum"
rostopic echo /jetson/fan_serial_bridge/adhesion_command
rostopic echo /jetson/fan_serial_bridge/fan_currents
```

观测点：

- 轻触滑移
- 切向对准
- 法向预压
- 风机启动
- 柔顺稳定

通过标准：

- 能容忍轻微误差和早接触，不会直接失效

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 11. 第七层：异常与恢复

### T24 安全模式触发测试

- 目标：确认安全模式触发后系统进入保护流程
- 验证方式：上机
- 前置条件：`local_safety_supervisor` 正常运行

执行命令：

```bash
rostopic echo /jetson/local_safety_supervisor/safe_mode
rostopic echo /control/mission_state
```

观测点：

- mission 是否暂停或进入故障态

通过标准：

- 系统停止激进换步并进入预期降级

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T25 吸附失败恢复测试

- 目标：确认吸附失败腿不会被错误当作有效支撑
- 验证方式：上机
- 前置条件：可人为制造吸附不足场景

执行命令：

```bash
rostopic echo /state/estimated | grep -E "attachment_ready_mask|adhesion_mask|support_mask"
rostopic echo /control/stance_wrench/lf | grep -E "active|actual_contact|planned_support"
rostopic echo /control/mission_state
```

观测点：

- `attachment_ready_mask`
- QP 激活状态
- mission 响应

通过标准：

- 吸附失败腿不会进入有效支撑集合

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T26 传感器中断测试

- 目标：确认传感器中断后系统表现可解释
- 验证方式：上机
- 前置条件：系统处于稳定运行

执行命令：

```bash
rostopic hz /state/estimated
rostopic echo /control/mission_state
rosnode list
rosnode info /state_estimator
```

观测点：

- 状态估计是否超时
- mission 是否暂停或降级

通过标准：

- 系统不会在关键感知中断后无保护继续运行

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### T27 通讯抖动与消息延迟测试

- 目标：确认轻度通讯抖动不会直接打崩控制链
- 验证方式：上机
- 前置条件：跨机网络已建立

执行命令：

```bash
rostopic hz /state/estimated
rostopic hz /control/swing_leg_target
rostopic hz /jetson/dynamixel_bridge/joint_state
rostopic hz /jetson/fan_serial_bridge/fan_currents
tail -n 50 ~/.ros/climbing_logs/<session_dir>/events.jsonl
tail -n 20 ~/.ros/climbing_logs/<session_dir>/snapshots.jsonl
```

观测点：

- 频率波动
- 日志时间连续性
- 状态是否异常跳变

通过标准：

- 轻度抖动下系统仍能保持基本稳定

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 12. 新改动专项验收

### A01 `attachment_ready_mask` 真实性

- 目标：确认不是把计划支撑或接触直接误当成“已可支撑吸附”

执行命令：

```bash
rostopic echo /state/estimated | grep -E "wall_touch_mask|preload_ready_mask|attachment_ready_mask|seal_confidence|leg_torque_contact_confidence"
```

通过标准：

- 必须在接触、预压、吸附相关特征满足后才置真

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### A02 摆动腿五阶段顺序正确

- 目标：确认行为顺序不跳段、不乱序

执行命令：

```bash
rostopic echo /control/swing_leg_target
rostopic echo /state/estimated | grep -E "wall_touch_mask|preload_ready_mask|attachment_ready_mask|skirt_compression_estimate"
rostopic echo /jetson/fan_serial_bridge/adhesion_command
```

通过标准：

- 行为顺序与设计一致

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### A03 QP 支撑腿选择正确

- 目标：确认 QP 激活条件与 `attachment_ready_mask` 一致

执行命令：

```bash
rostopic echo /state/estimated | grep -E "attachment_ready_mask|support_mask"
rostopic echo /control/stance_wrench/lf | grep -E "active|planned_support|actual_contact"
```

通过标准：

- 未准备好的腿不会输出有效支撑力

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### A04 Logger 可用于调参与复盘

- 目标：确认日志足以支撑 `Q/R`、柔顺参数、风机时序和法向力限制调试

执行命令：

```bash

### A05 Jetson 侧风机通信单测（不启动舵机）

- 目标：只验证 Jetson 与 STM32 风机板之间的串口收发、协议解析和 ROS 发布，不启动任何舵机控制节点

执行步骤：

终端 1：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

终端 2：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosparam load src/climbing_description/config/robot.yaml
rosrun climbing_hw_bridge fan_serial_bridge.py
```

终端 3：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice list | grep set_fan_speed_once
rosservice type /set_fan_speed_once
rosservice call /set_fan_speed_once "leg: 'lf'
grep -n "slip_risk" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
```

终端 4：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo /fan_currents
rostopic echo /leg_rpm
rostopic hz /fan_currents
rostopic hz /leg_rpm
```

必要时的原始串口抽查：

```bash
pkill -f fan_serial_bridge.py
sudo stty -F /dev/ttyUSB_fan 115200 raw -echo
timeout 2s cat /dev/ttyUSB_fan | hexdump -C | head
```

观测点：

- `fan_serial_bridge.py` 启动日志应显示已打开 `/dev/ttyUSB_fan`
- 服务 `/set_fan_speed_once` 存在且调用返回 `accepted: True`
- `/leg_rpm` 应接近下发目标值，例如 `10000 rpm` 时误差通常只在几十 rpm 内
- `/fan_currents` 应持续有非零电流反馈
- 原始串口反馈帧应能看到 `ff dd 20` 开头

通过标准：

- 不启动 `jetson_bringup.launch` 也能独立完成风机命令发送与反馈接收
- `/leg_rpm`、`/fan_currents` 持续更新且数值与实际工况一致
- 单独改某一条腿目标转速时，仅对应 ROS 数组位置发生变化

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

### A06 Jetson 侧 IMU 接收与解码单测（不启动舵机）

- 目标：只验证 Jetson 能否从 IMU 串口收到 FDILink 数据、完成解码并发布 `sensor_msgs/Imu`，不启动任何舵机节点

已知配置：

- IMU 参数位于 `robot.yaml` 的 `/imu` 命名空间
- 当前协议为 `fdilink`
- 当前串口为 `/dev/ttyUSB_imu`
- 当前波特率为 `921600`
- 当前代码发布的话题是 `/imu`
- 如需确认实际话题名，以 `rostopic list | grep imu` 为准

执行步骤：

终端 1：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

终端 2：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosparam load src/climbing_description/config/robot.yaml
rosrun climbing_hw_bridge imu_serial_bridge.py
```

终端 3：

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosnode list | grep imu_serial_bridge
rostopic list | grep imu
rostopic hz /imu
rostopic echo -n 1 /imu
```

如果需要先看原始串口字节，先停掉桥接节点，再执行：

```bash
pkill -f imu_serial_bridge.py
sudo stty -F /dev/ttyUSB_imu 921600 raw -echo
timeout 2s cat /dev/ttyUSB_imu | hexdump -C | head -n 20
```

静态解码检查步骤：

```bash
rosrun tf tf_echo /world /imu
rostopic echo /imu
```

1. 让机器人或 IMU 模块静止放置 10 秒。
2. 观察 `angular_velocity` 三轴是否接近 0。
3. 观察 `linear_acceleration` 的模是否接近 `9.8 m/s^2`。
4. 观察 `orientation` 四元数是否稳定，不应持续跳变或归一化失效。

动态解码检查步骤：

1. 手动绕一个轴缓慢转动 IMU。
2. 观察 `angular_velocity` 是否只在对应轴显著变化。
3. 观察 `orientation` 是否随姿态连续变化，不应出现突跳到全零或 `NaN`。
4. 将 IMU 放回初始姿态后，姿态应基本回到初始值附近。

推荐的快速定量检查：

```bash
rostopic echo /imu | grep -E "orientation:|angular_velocity:|linear_acceleration:" -A 12
```

观测点：

- `imu_serial_bridge.py` 日志应显示已打开 `/dev/ttyUSB_imu`
- `/imu` 频率应接近配置值 `100 Hz`
- 静止时角速度接近 0，线加速度模接近 `9.8`
- 缓慢单轴转动时，姿态和角速度响应方向一致且连续
- 无连续 CRC/串口重连告警，无明显丢帧卡顿

通过标准：

- Jetson 在不启动舵机链路的情况下可稳定收到 IMU 数据并持续发布 `Imu` 消息
- 发布频率、姿态连续性和静态重力幅值均合理
- 原始串口有稳定数据流，ROS 话题数值与人工转动方向一致

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：
grep -n "seal_confidence" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
grep -n "required_adhesion_force" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
grep -n "joint_currents" ~/.ros/climbing_logs/<session_dir>/events.jsonl | head
```

通过标准：

- 关键状态、命令和反馈能在时间上对齐分析

记录：

- 实际现象：
- 关键观测数据：
- 结果： [ ] 通过 [ ] 失败 [ ] 阻塞
- 问题与备注：

---

## 13. 最小执行闭环建议

如果只做一轮最小有效验证，建议按下列顺序打勾：

1. T01
2. T02
3. T04
4. T06
5. T07
6. T09
7. T12
8. T15
9. T19
10. T21
11. T23
12. T25

## 14. 结论记录

- 本轮总体结论：
- 当前主要风险：
- 下一轮优先修复项：
- 建议调整参数：
- 关联日志/rosbag 路径：