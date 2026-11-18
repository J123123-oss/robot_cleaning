# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目概述

ROS2 (jazzy) 室外机器人清扫系统，核心功能：RTK 多航点循迹导航、CAN 总线差速底盘控制、MQTT 云端桥接、485 传感器采集（电池/充电/边界 IO）、激光辅助进仓对位。

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
| `/rtk/motor_speed` | rtk_nav → motor_control | AIMotor 配置使用的左右轮及滚刷转速 (Vector3) |
| `/rtk/motion_command` | rtk_nav → motor_control | 混合协议使用的带时间戳轮速及滚刷三态指令 (RtkMotionCommand) |
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

## MQTT 状态上报约定

- `publish_state()` 的 `state_msg` 中，除 `current_lon/current_lat` 保留原始精度外，所有层级的浮点数统一四舍五入至两位小数；保持数字类型，整数、布尔值、字符串和空值不转换，不修改内部采样数据。

## 清扫测试与导航收尾约定

- 仅测试清扫、不自动进仓时使用 `auto_cleaning_finish_to_loading:=false`（当前 launch 默认值）；设为 true 时清扫完成后进入 LOADING。独立运行 motor_control 节点时该参数默认 true，测试时须显式关闭。
- 导航 COMPLETED 仅在底盘仍处于 AUTO_CLEANING 控制模式且不在进出仓流程时触发收尾；重复或切出自动模式后到达的完成消息不得再次切换状态。
- 非 AUTO_CLEANING 的模式消息可重复发布，零速停车保留；无航点的上下文重置和对应日志仅在模式发生变化时执行，避免周期消息反复清理、刷屏。

## 导航航段缓存约定

- Stanley 路径起点和方向属于同一航段，失效时必须同时清除；切点、恢复进度、重建航段后不得使用上一航段方向。
- 暂停恢复和 AUTO 航向门控对齐共用当前航段方向选择：有效缓存优先；无有效缓存时，使用当前目标及其前置航点重算，首点使用跨文件前置点或当前位置。
- 航段缓存回归测试放在 `src/rtk_nav/test/test_segment_direction_recovery.py`，覆盖切点后立即暂停恢复和航向门控对齐。

## 导航定位与到点确认约定

- `navigation_position_safety.py` 负责定位跳变检查和到点确认；规则说明放在 `docs/navigation_position_safety.md`，测试放在 `src/rtk_nav/test/test_navigation_position_safety.py`。
- 自动导航中，车体中心定位位移不得超过 `0.35m/s × 样本间隔 + 0.05m`，间隔最多按 1 秒计算。跳变后停车、保留可信坐标，不能随等待时间增长而放宽已触发的跳变界限。
- 跳变恢复须连续 3 个新定位样本落在原可信位置允许范围内；持续偏移不得仅因新坐标稳定就自动重设基准，须切出自动模式核实位置。
- 有效且新鲜的 IMU/里程计备用定位可提供恢复位置参考，沿用 0.30m 残差上限；积分及切回平滑期间不累计到点确认。
- 仅定位 Fixed、坐标有效且时间戳递增的样本可更新导航位置；到点计数另要求定位与定向均 Fixed。重复、乱序和无效帧不得累计确认。
- `WTRTK.position_stamp_ns` 保存实际 GGA 定位帧的原始 ROS 时间戳（纳秒），不能用 WTRTK 帧的 `header.stamp` 替代；串口和回放驱动都须携带该字段，缓存 GGA 不得重复计数。
- 普通航点进入 0.10m 范围先停车，连续 3 个新可信样本确认后才进入最终航向校准；首点保留连续 5 个新样本确认。越界、质量异常、暂停恢复、模式或目标变化均清除旧确认。

## 滚刷混合协议实现约定

- `motor_driver_kinco.py` 实现步科滚刷协议适配，复用 AIMotor 驱动的 CAN 总线、线程和反馈接口；不得为滚刷创建第二个 CAN 接收线程。
- `brush_use_kinco=false` 保持现有 AIMotor 行为；为 true 时仅 ID 3、4 使用步科协议，滚刷数量仍由 `brush_motor_count` 决定。
- 上层速度统一使用输出轴 r/min。步科编码器分辨率、机械减速比、rps/s 加减速度独立配置，不复用 AIMotor 的脉冲速度或加减速单位。
- 步科 ID 3、4 的方向由 `kinco_motor_direction_3/4` 独立配置（1 正常，-1 反向）；`motor_direction_1..4` 保留给 AIMotor，混合协议只覆盖已配置的步科滚刷方向。
- 驱动构造参数 `motor_directions` 中显式指定的 ID 优先于进程级 `motor_direction_*` 覆盖值；未显式指定的 ID 保留 ROS 参数配置能力，避免主节点选出的步科方向在基类构造时被 AIMotor 参数二次覆盖。
- 步科方向通过 `607E:00=0/1` 设置，不在 `60FF` 速度上重复乘方向；修改前零速并发送 `6040=0006`、实时回读 `6041` 确认退出使能，修改后必须回读 `607E` 一致，否则初始化失败并禁止使能。
- 协议依据与现场配置说明放在 `docs/kinco_brush_driver.md`，离线协议测试放在 `src/motor_control/test/test_motor_driver_kinco.py`。
- `motor_auto_enable=true` 时初始状态直接设为 ENABLE。启动、手动 ENABLE 和 START 仅使能 AIMotor，步科完成零速配置并确认失能；初始化失败转 DISABLE。ENABLE 下磁导航连续达到进仓完成条件后复用完成流程转 DISABLE。
- 滚刷 `RUN` 首次按需使能，所有已配置步科滚刷通过实时 `6041` 确认后才允许非零转速。`HOLD_ZERO` 保持当前使能状态，供可自动恢复的导航暂停使用。`DISABLE` 用于区间结束、返仓及任务退出，正常流程先零速并确认停稳，再失能。失效指令、故障和显式停机取消待执行的运行请求。
- 步科加减速度默认均为电机侧 `800.0 rps/s`，继续通过 `kinco_acceleration_rps2` 和 `kinco_deceleration_rps2` 配置。
- 启动 ENABLE 与手动进入 ENABLE 共用状态配套设置：清除完成标记和补发计数、取消方向定时器、状态发布周期设为 0.5 秒。启动初始化成功后立即发布状态，保持轮子和滚刷零速。
- 服务正常退出时 AIMotor 发送零速并保持使能，启动时实时回读 `6041`，已使能节点不得重发 `06→07→0F`；显式 DISABLE、异常退出、初始化或零速发送失败仍失能。步科退出时仍零速失能并停止节点保护。
- ROS 上下文关闭后的 `RCLError/InvalidHandle` 按正常退出处理，不得触发 AIMotor 失能或计为 CAN 接收故障；上下文有效时的同类异常及其他运行异常仍按故障处理。重启验证须包含不注入查询或保活帧的被动 CAN 抓包，避免测试报文影响断通信行为。
- 步科断通信保护采用节点保护，启用前回读确认 `1017=0`、`100C`、`100D`、`6007=1`；RTR 必须复用现有 CAN 发送线程，停止进程后不得由独立服务继续发送。
- 初始化在零速下实时读取 `6041`，仅对故障位有效的节点执行故障复位并确认退出故障；步科先恢复节点保护，再处理遗留故障及方向配置。AIMotor/Kinco 发送使能后必须回读确认 `(6041 & 0x006F)==0x0027`，否则初始化失败转 DISABLE。
- 故障话题中的非零码须回读 `6041` 确认真正有效，已恢复的旧消息不得复位正常节点；真实故障或状态回读失败先转 DISABLE，仅向确认故障的节点发送清故障指令，运行中不得自动恢复非零速度或使能。
- 步科故障描述使用手册 9.3.3.6 表 9-24 的 `KINCO_FAULT_TABLE`，不得套用 AIMotor 故障含义；EMCY 按小端解析 Byte0-1 的 `603F`、Byte2 的 `1001`、Byte4-5 的 `2601` 和 Byte6-7 的 `2602`，Byte3 不作推测。反馈保留原始值并追加中文描述，未知码不得丢弃。
