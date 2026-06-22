# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ROS2 (humble) 室外机器人清扫系统，核心功能：RTK 多航点循迹导航、CAN 总线差速底盘控制、MQTT 云端桥接、485 传感器采集、激光辅助进仓对位。

## 构建、测试与运行

```bash
# 构建
colcon build --symlink-install
colcon build --symlink-install --packages-select motor_control

# 启动
source install/setup.bash && ros2 launch rtk_nav run.launch.py
ros2 launch rtk_nav run.launch.py robot_ID:=my_robot_id

# 测试
colcon test --packages-select motor_control
colcon test --packages-select rtk_nav
colcon test --packages-select mqtt_ros2

# CAN 初始化（启动前执行一次）
sudo bash can_ch340_init.sh

# 自定义消息包需先构建
colcon build --symlink-install --packages-select custom_msgs
```

## 包结构与职责

```
src/
├── custom_msgs/         # 自定义 ROS2 消息和服务 (CMake)
│   ├── msg/WTRTK.msg    # RTK 数据（惯导航向、定位状态、差分距离等）
│   ├── msg/LatLonPoint.msg
│   └── srv/ChargeControl.srv
├── motor_control/       # 底盘控制与传感器采集 (Python)
│   ├── motor_control.py # ★ 主控制节点：状态机/键盘/MQTT/遥控，进仓出仓
│   ├── motor_driver.py  # CAN 电机驱动（左/右/滚刷，socketcan）
│   ├── remote_control.py # SBUS 遥控器解析
│   ├── sensors_485.py   # RS485 传感器（IO 边界触发 + 电池）
│   ├── charging.py      # 485 无线充电模块
│   ├── laser_distance.py # 激光测距
│   └── battery_test.py
├── rtk_nav/             # RTK 导航 (Python)
│   ├── rtk_nav.py       # ★ RTK 导航节点（Stanley 控制器、状态机、边界矫正）
│   ├── wtrtk_serial_driver.py  # RTK 接收机串口驱动
│   ├── wtrtk_parse_txt.py      # RTK 录包回放（离线测试）
│   ├── full_path_planner_dense.py  # 多区域路径拼接
│   ├── cleaning_path/   # 航点路径文件 (*.txt，每行 lon,lat,heading)
│   ├── config/          # YAML 路径配置
│   └── launch/run.launch.py # ★ 唯一启动文件
└── mqtt_ros2/           # MQTT-ROS2 桥接
    └── mqtt_ros2_bridge.py  # 双向桥接，含远程 shell（高风险）
```

## 核心架构

### 控制模式

三种模式（`ControlMode`）：`NORMAL`（键盘）、`REMOTE`（遥控器）、`AUTO_CLEANING`（RTK 自动导航）。通过 `/keyboard/control` 话题切换。

**主状态机**（`motor_control.py`, 10Hz）：根据当前模式和输入下发速度，处理 START/LOADING 流程。

**导航状态机**（`rtk_nav.py`, 10Hz）：
`IDLE → INITIAL_MOVE → WAYPOINT_MOVE → WAYPOINT_CALIB → COMPLETED`
- 航点到达后原地旋转校准（误差 <1° 后切换）
- 最后一个航点后自动切换下一路径文件（按数字前缀排序）
- RTK 非固定解 → PAUSE 暂停，恢复后继续

### Stanley 控制器

`total_steering = heading_error + atan(K * lateral_error / velocity)`，clamp ±45°，差速分配。

当前参数：`STANLEY_K_BASE=0.5`, `STANLEY_MIN_SPEED=0.15`, `MAX_LATERAL_ERROR=1.0`, `SPEED_CMD_TO_MPS=0.0345`, `STRAIGHT_MAX_CORRECTION=1.5`，自适应 K：近距(<1.3m)=0.42，远距(≥1.3m)=0.45。航向优先抑制：`abs(hdg_err)>4°` 且横向项同向时 st_corr 减半。

`path_direction` 使用固定路径段方向，投影检测越过终点时切换为指向目标。

### 进仓/出仓

**出仓**（START）：`u` 键 → 直行 → 旋转到目标角度 → 完成。

**进仓**（LOADING）：`l` 键 → 激光对位前进 → 后退 → 旋转。激光/电机异常时走兜底：直接前进 26s → 完成。

### 边界矫正

6 路 IO 触发后：`TURNING(偏转1s) → BACKING(后退4s) → RETURNING(退回1s) → IDLE`。

### 关键话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `/fix` | wtrtk → rtk_nav | GPS (NavSatFix) |
| `/wtrtk_data` | wtrtk → rtk_nav | RTK 完整数据 (WTRTK) |
| `/motor_speed` | rtk_nav → motor_control | 目标左右轮速度 (Vector3) |
| `/keyboard/control` | 外部 → motor_control | 控制指令 (String) |
| `/robot_cmd` | mqtt → motor_control | MQTT 下发指令 (String) |
| `/robot_state` | motor_control → mqtt | 状态上报 (String) |
| `/laser_distance` | laser → * | 激光测距 |

## 已知问题

1. **MQTT 远程 shell**：`mqtt_ros2_bridge.py` 中 `topic_command` 直接 `Popen(shell=True)`，P0 风险。
2. **硬编码**：串口、文件路径、MQTT 凭据硬编码在 `run.launch.py` 和代码中。
3. **CAN 依赖**：电机通过 socketcan (can0) 通信，需 `can_ch340_init.sh` + `ch341.ko`。
4. **systemd 自启**：`can_ch340_init.service` + `motor_start.service`，后者等待 MQTT broker 后启动。
5. **路径文件格式**：每行 `lon,lat,heading`，跨文件自动按数字前缀切换。
