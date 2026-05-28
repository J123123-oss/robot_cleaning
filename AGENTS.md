# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目概述

ROS2 (humble) 室外机器人清扫系统，核心功能：RTK 多航点循迹导航、CAN 总线差速底盘控制、MQTT 云端桥接、485 传感器采集（电池/充电/边界 IO）、激光辅助进仓对位。

## 构建、测试与运行

```bash
# 构建（ROS2 colcon）
colcon build --symlink-install

# 构建单个包
colcon build --symlink-install --packages-select motor_control

# 启动全部节点
source install/setup.bash && ros2 launch rtk_nav run.launch.py

# 启动时指定 robot_ID
ros2 launch rtk_nav run.launch.py robot_ID:=my_robot_id

# 运行测试（每个包都有 pytest）
colcon test --packages-select motor_control
colcon test --packages-select rtk_nav
colcon test --packages-select mqtt_ros2

# CAN 接口初始化（启动前执行一次）
sudo bash can_ch340_init.sh

# 自定义消息包需先构建
colcon build --symlink-install --packages-select custom_msgs
```

## 包结构与职责

```
src/
├── custom_msgs/         # 自定义 ROS2 消息和服务定义 (CMake)
│   ├── msg/WTRTK.msg    # RTK 接收机完整数据（差分距离、角度、定位状态、惯导数据等）
│   ├── msg/LatLonPoint.msg
│   └── srv/ChargeControl.srv  # 充电控制（mode + addr 列表）
├── motor_control/       # 底盘控制与传感器采集 (Python)
│   ├── motor_control.py # ★ 主控制节点：状态机核心，键盘/MQTT/遥控三模切换，进仓出仓逻辑
│   ├── motor_driver.py  # CAN 总线电机驱动（左/右/滚刷 3 电机，socketcan）
│   ├── remote_control.py # SBUS 遥控器解析
│   ├── sensors_485.py   # RS485 传感器轮询（6 路 IO 边界触发 + 电池电压/温度）
│   ├── charging.py      # 485 无线充电模块（启停、电压电流查询、故障码）
│   ├── laser_distance.py # 激光测距传感器
│   ├── battery_test.py  # 电池测试工具
│   └── custom_rc_parser.py
├── rtk_nav/             # RTK 导航与路径规划 (Python)
│   ├── rtk_nav.py       # ★ RTK 导航控制节点（Stanley 控制器、航点状态机、边界矫正）
│   ├── wtrtk_serial_driver.py  # RTK 接收机串口驱动（发布 /fix 和 /wtrtk_data）
│   ├── wtrtk_parse_txt.py      # RTK 录包回放（离线测试用）
│   ├── cleaning_path_planner.py    # 区域清扫路径生成（矩形参数 → UTM 航点 + 可视化）
│   ├── three_point_planner.py      # 三点标定多区域路径规划
│   ├── full_path_planner.py / full_path_planner_dense.py  # 多区域路径拼接（稀疏/稠密）
│   ├── calucate_bearing.py  # 航向角计算
│   ├── cla_distance.py      # 距离计算工具
│   ├── gnrmc_parse.py       # NMEA RMC 帧解析
│   ├── latlon_test_point.py # 经纬度测试点生成
│   ├── navsat_key_publisher.py
│   ├── config/              # 各区域 YAML 路径配置（000~006，北/南区域）
│   └── launch/run.launch.py # ★ 唯一启动文件，编排所有节点及参数
└── mqtt_ros2/           # MQTT-ROS2 桥接 (Python)
    └── mqtt_ros2_bridge.py  # MQTT 双向桥接：/robot_cmd → MQTT，MQTT → /robot_state
                              # 注意：包含远程 shell 执行能力（topic_command），安全风险高
```

## 核心架构

### 控制模式与状态机

三种控制模式（`ControlMode`）：`NORMAL`（键盘）、`REMOTE`（遥控器）、`AUTO_CLEANING`（RTK 自动导航）。模式切换通过 `/keyboard/control` 话题发布 `r`/`n`/`m` 字符实现。

**主状态机**驱动逻辑在 `motor_control.py` 的 `MotorControlNode`：10Hz 定时器根据当前模式和键盘/MQTT 输入下发速度。电机使能/失能通过 `x` 键（START）和 `z` 键（DISABLE）。

**导航状态机**在 `rtk_nav.py` 的 `RTKNavControlNode`（10Hz）：
`IDLE → INITIAL_MOVE → WAYPOINT_MOVE → WAYPOINT_CALIB → ... → COMPLETED`
- 每个航点到达后原地旋转校准航向（`WAYPOINT_CALIB`），角度误差 <1° 后切换到下一个
- 最后一个航点到达后自动检测是否有下一个路径文件（按数字前缀排序），实现跨文件衔接
- RTK 非固定解时进入 `PAUSE` 暂停，恢复固定解后继续

### Stanley 控制器

横向控制使用 Stanley 算法：`total_steering = heading_error + atan2(K * lateral_error, velocity)`，输出 clamped 到 ±45°，再通过差速分配计算左右轮速度。关键参数：`STANLEY_K_BASE=0.4`, `MAX_LATERAL_ERROR=0.15m`。

### 边界矫正状态机

6 路 IO 传感器触发后：`TURNING(偏转1s) → BACKING(后退4s) → RETURNING(反向退回1s) → IDLE`。

### 关键话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `/fix` | wtrtk → rtk_nav | GPS 经纬度 (NavSatFix) |
| `/wtrtk_data` | wtrtk → rtk_nav | RTK 完整数据 (WTRTK) |
| `/motor_speed` | rtk_nav → motor_control | 目标左右轮速度 (Vector3) |
| `/keyboard/control` | 外部 → motor_control | 控制指令字符 (String) |
| `/robot_cmd` | mqtt → motor_control | MQTT 下发的控制指令 (String) |
| `/robot_state` | motor_control → mqtt | 机器人状态上报 (String) |
| `/charging_fault_code` | charging → * | 充电故障码 (Int16) |
| `/laser_distance` | laser → * | 激光测距数据 |

### 关键服务

| 服务 | 说明 |
|------|------|
| `/start_charging` | 开始充电 (ChargeControl) |
| `/stop_charging` | 停止充电 (ChargeControl) |
| `/query_volt_curr` | 查询电压电流 (Trigger) |

### 进仓/出仓流程

**出仓**（START）：键盘 `u` 触发 → 直行（`unloading_forword_threshold` 秒）→ 旋转到目标角度 → 完成。

**进仓**（LOADING）：键盘 `l` 触发 → 激光距离 < 阈值时一直前进 → 后退固定时长 → 旋转到目标角度。

## 已知问题与注意事项

1. **MQTT 远程 shell 执行**：`mqtt_ros2_bridge.py` 中 `topic_command` 会直接 `subprocess.Popen(shell=True)` 执行任意命令。这是 P0 安全风险，生产环境应移除或加白名单。

2. **硬编码路径和参数大量存在**：`run.launch.py` 和 `motor_control.py` 中串口设备名（`/dev/ttyS2`, `/dev/laser`, `/dev/WTRTK`）、文件路径、MQTT 凭据均硬编码。换机器需逐一修改。

3. **CAN 接口依赖**：电机驱动通过 `socketcan`（can0）通信，启动前需 `can_ch340_init.sh` 初始化。还依赖 `ch341.ko` 内核模块。

4. **systemd 自启**：`can_ch340_init.service` 和 `motor_start.service` 用于开机自启，后者会等待 MQTT broker 可达后启动 ROS2 launch。

5. **路径文件格式**：航点文件为每行 `lon,lat,heading` 的纯文本，跨文件导航时按文件名数字前缀（如 `001_xxx.txt` → `002_xxx.txt`）自动切换。

6. **REVIEW.md** 记录了详细的需求确认清单和排障顺序，接手项目时应先阅读。
