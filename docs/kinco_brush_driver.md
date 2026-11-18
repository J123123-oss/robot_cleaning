# 步科滚刷协议与配置

`AIMotor` 分支通过 `brush_use_kinco` 切换滚刷协议。默认 `false` 使用原有
`motor_driver(AIMotor).py`；`true` 使用 `motor_driver_kinco.py`，仅将已配置的
ID 3、4 切换为步科 iSMK 协议。ID 1、2 仍调用 AIMotor 原有实现。两类电机共用
同一个 500 kbit/s CAN 接口及收发线程，不修改硬件 ID，不写入永久存储参数。

## 启动配置

`motor_auto_enable` 默认 `true`：主控制节点初始状态直接设为 `ENABLE`，
对所有已配置的 AIMotor 和 Kinco 节点执行零速初始化和使能；任一选定节点初始化失败，
撤销这些节点的使能并转为 `DISABLE`。可通过 `motor_auto_enable:=false`
恢复启动时全部失能。步科 ID 3/4 仍由 `brush_motor_count` 决定是否配置，
使能前必须确认节点保护参数。`motor_auto_enable` 同时控制两种协议电机的启动自动使能。
这里是软件自动使能，不是写入电机的上电自使能参数。

开启磁导航时，`ENABLE` 下连续 3 个磁导航 mask 每次至少 6 点触发，
按现有进仓末端判定确认已在仓内，调用原进仓完成流程停车、置完成标记、
切换 `DISABLE` 并请求充电。无信号或不足 6 点会清零确认计数；
出仓 `START/UNLOADING` 等其他状态不会触发此启动在仓判定。

```bash
ros2 launch rtk_nav run.launch.py brush_use_kinco:=true brush_motor_count:=2
```

`brush_motor_count` 原有默认值仍为 `0`，因此仅打开协议开关不会自动增加电机。
数量为 `1` 时只控制 ID 3，为 `2` 时控制 ID 3、4。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `brush_use_kinco` | `false` | true 时使用步科滚刷协议 |
| `kinco_motor_direction_3` | `1` | 步科 3 号速度指令方向，1 正常，-1 反向 |
| `kinco_motor_direction_4` | `-1` | 步科 4 号速度指令方向，1 正常，-1 反向 |
| `kinco_encoder_resolution` | `65536` | 每个电机轴转动一圈的编码器计数 |
| `kinco_reduction_ratio` | `10.0` | 电机轴转速/滚刷输出轴转速 |
| `kinco_acceleration_rps2` | `100.0` | 电机侧加速度，rps/s |
| `kinco_deceleration_rps2` | `100.0` | 电机侧减速度，rps/s |
| `kinco_brush_speed_rpm` | `150.0` | 滚刷输出轴转速上限，r/min |
| `kinco_guard_time_ms` | `100` | 节点保护 RTR 请求周期，ms，范围 50～65535 |
| `kinco_guard_life_factor` | `5` | 节点保护超时倍数，范围 2～255 |

## AIMotor 服务重启与使能衔接

依据《AIMotor 系列 CanOpen 使用手册 20250718》印刷页 14～15，
已使能状态下发送 `6040=0006/0007/0000` 会退出使能。因此正常关闭服务时，
驱动停止后台轮询、清零速度缓存并发送 `60FF=0`，保留 AIMotor 的使能状态；
随后结束线程并关闭主机 CAN socket。再次启动时先发送零速，
在配置速度模式及加减速参数后，实时读取 `6041:00`：
`(status_word & 0x006F) == 0x0027` 表示仍已使能，不再发送 `06→07→0F`；
未使能的节点仍执行原使能序列。禁止用进程内缓存推断重启前的使能状态，
状态回读失败则初始化失败并失能。重启不会恢复上一次的非零目标速度。

显式 `DISABLE`、进仓完成、节点致命异常和初始化失败仍执行原失能流程。
退出时某节点零速发送失败，也会尝试失能该节点。步科不采用此保持策略，
仍发送零速、失能并停止 RTR，保留原断通信保护。

手册印刷页 20～21 说明 AIMotor 也支持节点/寿命保护及消费者心跳。
当前程序未为 AIMotor 启用这些保护，但不能据此认定现场断线必定保持使能：
若驱动器已有 `100C/100D` 或 `1016` 监视配置，或出现 CAN 故障报警，
仍可能退出使能。本次不修改驱动器持久参数或关闭现场已有保护。
单纯停止主控发送与物理断开 CAN 线是两项独立的硬件验证。

现场验证应先零速停车，保持电机供电，记录停止服务前、停止后超过原通信超时、
以及重新启动后的 `6041`。ID 1/2 可分别发送以下只读请求，
在 `0x581/0x582` 响应中检查状态字低字节在前的两个数据字节：

```bash
cansend can0 601#4041600000000000
cansend can0 602#4041600000000000
```

保持使能的判据为上述状态掩码持续成立，不能只看 ROS 中的 `ENABLE`。
重启报文中不应出现对 AIMotor 的 `6040=0000/0006/0007`；
若状态回读失败或触发实际保护，需另行检查相应故障。离线测试仅验证控制报文，
不能代替现场断线、重启及抱闸动作测试。

## 步科断通信停机保护

依据 iSMK 手册印刷页 109～111、113、115，在每次步科使能前写入并回读：
`1017:00=0`（关闭生产者心跳，避免与节点保护混用）、
`100C:00=kinco_guard_time_ms`（UINT16）、
`100D:00=kinco_guard_life_factor`（UINT8）及 `6007:00=1`（UINT16）。
任何写入或回读失败都禁止本次使能；这些参数每次重新配置，不写永久存储。

参数确认后先发送一次 `0x700+ID` 的零数据 RTR 远程帧，启动从站监视，
随后由现有单一 CAN 发送线程周期发送。ID 3/4 对应 `0x703/0x704`，
不是携带 `05` 的心跳数据帧，也不是普通 8 字节数据帧。
无需新增 CAN 接收或保活线程，未配置的滚刷不会收到请求。

默认连续约 `100 × 5 = 500 ms` 未收到节点保护请求后，由驱动器报警停机；
这是通信故障判定时间，不包含机械停止时间。停机保护依赖驱动器正常供电、
配置成功且已经收到首条 RTR。主控进程退出或 CAN 线断开后请求停止，
驱动器独立执行故障停机；ROS 回调卡住但 CAN 发送线程仍工作的情况不在此保护范围。
正常退出时步科仍先发送零速和失能指令。通信恢复不会直接触发重新使能。

手册印刷页 38～39、64、88、117 说明：通讯中断报错 `0x8100`
（`2602:00` bit12）触发后，驱动器先进入“故障停止”，按 `605E:00`
执行报错停机，再进入“故障”状态并禁用驱动功能，不是正常使能下的零速锁轴。
`605E` 决定停机过程，0=立即停止、1=当前减速度停止、2=快速停止减速度停止、
18=电机自身电阻制动；当前代码不写此对象，现场动作取决于驱动器已有配置。
500 ms 仅为通信故障判定时间，停机过程中可能仍有制动电流。
恢复通信不等于恢复运行：需先排除故障、复位，再执行使能流程。
现有主控制节点 `motor_fault_callback()` 收到故障时会对全部电机请求清故障，
步科使用 `6040:00=0x0086`。因此恢复通信后故障可能被自动清除，回到可重新使能的状态；
该回调不执行 `06→07→0F`，不会自动重新使能。机器人软件状态不会因该回调自动变成
`DISABLE`，必须区分软件状态与驱动器真实状态。重启程序时，
`motor_auto_enable=true` 会重新尝试零速初始化和使能，但初始化本身不包含故障复位。

现场需在受控低速、无负载条件下验证：观察 RTR 周期，终止控制进程或断开 CAN，
确认故障与停机，再手动恢复；本地离线测试不能替代驱动器硬件验证。

## 速度与方向配置

编码器分辨率 `65536` 来自手册第 105 页的示例，不是对现场型号的确认。
当前现场配置的机械减速比为 `10.0`；必须按实际安装传动比设置，不要沿用
AIMotor 滚刷的默认减速比 40。
启用步科协议后，RTK 导航清扫时的滚刷输出轴目标转速为 `150 r/min`；
AIMotor 协议仍使用 `70 r/min`。`kinco_brush_speed_rpm` 是步科滚刷的输出轴
转速上限，外部速度指令超过该值时保留正负方向并截断绝对值；设为 `0.0` 时不限制。
步科驱动器的波特率须配置为 500 kbit/s，ID 须预先设置为 3、4。

`brush_use_kinco=true` 时，3/4 号分别读取 `kinco_motor_direction_3/4`：
`1`（正常）和 `-1`（反向）由步科驱动映射到 `0x607E:00=0/1`。
launch 默认 3 号为 `1`、4 号为 `-1`，保留原先的默认方向；
1/2 号仍读取 `motor_direction_1/2`。
`brush_use_kinco=false` 时，所有 AIMotor 仍读取 `motor_direction_1..4`。
参数只应用于 `brush_motor_count` 选中的滚刷。`brush_direction_mode` 的原有速度符号处理
继续生效，设为 `opposite` 时还会对 4 号速度取反，与电机方向参数叠加。
方向只能在失能状态配置，初始化先发送 `0x6040=0x0006`。

例如将两台步科滚刷的速度指令方向均设为正常：

```bash
ros2 launch rtk_nav run.launch.py brush_use_kinco:=true brush_motor_count:=2 \
  kinco_motor_direction_3:=1 kinco_motor_direction_4:=1
```

## 协议依据

参考用户提供的《步科电机常用报文说明.docx》和《Kinco iSMK一体机使用手册
20250523.pdf》。下列页码为手册印刷页码，PDF 页码比印刷页码大 1。
文档中的操作说明只作为协议资料，不作为自动修改硬件 ID、保存参数或复位设备的指令。

| 功能 | 地址或格式 | 手册位置 |
|------|------------|----------|
| 速度模式 | `0x6060:00=3`（线值 `0x03`），1 字节 | 42、78 页 |
| 使能/失能/故障复位 | `0x6040:00=0x0006→0x0007→0x000F` / `0x0006` / `0x0086`，2 字节 | 78 页 |
| 方向 | `0x607E:00=0/1`，1 字节 | 42、79 页 |
| 目标/实际速度 | `0x60FF:00` / `0x606C:00`，有符号 4 字节 | 79–80 页 |
| 实际位置 | `0x6063:00`，有符号 4 字节 | 79 页 |
| 加/减速度 | `0x6083:00` / `0x6084:00`，无符号 4 字节 | 79–80 页 |
| 电机温度 | `0x6410:19`，有符号 2 字节，摄氏度 | 88 页 |
| 错误状态 | `0x2601:00` / `0x2602:00`，无符号 2 字节 | 86–87 页 |
| SDO 请求/响应 | 标准帧 `0x600+ID` / `0x580+ID`，小端 8 字节 | 101–104 页 |
| EMCY | `0x80+ID`，错误码、错误寄存器、两个错误状态字 | 112 页 |

上层传入速度为输出轴 r/min：

```text
motor_rpm = output_rpm * kinco_reduction_ratio
velocity_DEC = round(motor_rpm * 512 * encoder_resolution / 1875)
output_rpm = velocity_DEC * 1875 / (512 * encoder_resolution * reduction_ratio)
ramp_DEC = round(rps_per_second * 65536 * encoder_resolution / 4000000)
```

例如分辨率 65536、直驱、目标 -100 rpm 时，ID 3 发送：

```text
603  23 FF 60 00 7E B1 E4 FF
```

100 rps/s 的加速度对应 `23 83 60 00 6E A3 01 00`。
初始化先失能滚刷，再设置步科速度模式 `6060=3`。控制字按
`0006→0007→000F` 完成状态机使能，并保持 `6040h=000F`；发送 `0006` 仅用于失能；
减速度通过 SDO 回读确认。
加减速度换算为零或超出 UINT32 时，在启动线程前拒绝配置；加速度发送失败时拦截
该滚刷的后续使能，重新成功配置后才解除拦截。
使能前清零旧速度目标；停止和退出时沿用现有零速与失能流程。

## 反馈与当前限制

步科通过 SDO 读取实际位置、速度、状态、实际电流、温度、应急错误码和两个错误状态字。
不套用 AIMotor 默认 PDO 映射或厂家故障历史对象。收到写入确认、中止响应或长度
不足的帧时，不将其当作速度反馈；等待中的 SDO 回读仍能收到正常响应及中止通知。
`fault_code` 保留应急错误码，`kinco_error_status` 和 `kinco_error_status_2`
分别保留两个错误状态字。

驱动通过 SDO 查询 `0x6078:00`，请求帧为 `40 78 60 00 00 00 00 00`，
响应命令为 `0x4B`。驱动按有符号 `Integer16` 解析为 DEC，并依据手册公式
`Arms = DEC * Ipeak / (1.414 * 2048)` 转换后写入
`motor["actual_current"]`，同时更新 `actual_current_updated_monotonic`。
`kinco_peak_current_a` 默认取现场电机参数中的 `Ipeak=47.9 A`，可在 launch
中按实际电机参数调整；因此现有滚刷电流保护使用的是安培值，而不是原始 DEC。
实际电流查询失败或样本过期时，保护逻辑会忽略该样本；原有电池总电流保护继续使用。

## 实时调试话题

开启步科协议后，驱动在现有 `poll_feedback()` 发布周期同时发布
`/motor/kinco_feedback`，类型为 `std_msgs/msg/String`，内容是 JSON。
主控制节点正常运行时约 10 Hz 发布；消息是异步 SDO 缓存快照，各字段并非同一时刻采集。
仅包含实际配置的步科电机：单滚刷 ID 3，双滚刷 ID 3、4。
继续保留 `/motor_feedback`、`/motor_velocity_feedback` 等现有通用话题。
新增话题不创建 CAN 接收线程；原有 SDO 查询增加 `6061:00` 有效工作模式回读。

```bash
ros2 topic echo /motor/kinco_feedback
ros2 topic hz /motor/kinco_feedback
```

JSON 顶层包含 `timestamp_unix_sec`（发布时间）和 `motors` 列表，每台电机包含：

| 字段 | 含义与单位 |
|------|------------|
| `id` | CAN 节点 ID |
| `target_output_speed_rpm` | 软件最近缓存的输出轴目标转速，r/min；不是驱动器目标回读 |
| `output_speed_rpm` / `motor_speed_rpm` | 实际输出轴/电机轴转速，r/min |
| `output_current_arms` | 电机侧有符号等效 RMS 电流，A，沿用现有保护换算 |
| `output_current_ap` | 电机侧峰值标度电流，A，`DEC * Ipeak / 2048`，便于与上位机 Ap 对照 |
| `motor_position_rad` | 电机轴实际位置，rad，未除以减速比 |
| `temperature_c` | 电机温度，℃ |
| `status_word` | `6041:00` 状态字，无符号整数 |
| `mode_display` | `6061:00` 有效工作模式，有符号整数 |
| `fault_code` | `603F:00` 或 EMCY 错误码 |
| `error_status` / `error_status_2` | `2601:00` / `2602:00` 错误状态字 |
| `velocity_raw` / `position_raw` / `current_raw` | 对应 SDO 原始有符号 DEC 值 |
| `sample_age_sec` | 各反馈字段距最后有效响应的秒数 |

未收到有效反馈时，对应字段及其年龄为 `null`；通信中断后保留最后一次值，
其年龄持续增大，订阅端应按年龄判断是否过期，不能仅凭零速或零故障码判断在线。
电流字段是电机侧电流，不是 48 V 电源输入电流；`Arms` 和 `Ap` 两种标度不会
按电源电压换算成输入电流。

## 独立设定目标电流限制

`src/motor_control/motor_control/kinco_current_limit.py` 是独立的 CANopen
命令行工具，写入 `0x6073:00` 后自动写入 `0x2FF0:01=1` 保存控制参数，
不会启动 ROS 节点。
构建工作空间后可以使用安装的命令：

```bash
kinco_current_limit read --motor-id 3
kinco_current_limit read --motor-id 3 --ipeak 48
kinco_current_limit set --motor-id 3 --raw-dec 603
kinco_current_limit set --motor-id 3 --arms 10 --ipeak 48
```

默认使用 `can0` 和 `500000` bit/s，可通过 `--channel`、`--bitrate`、
`--timeout` 覆盖。`--raw-dec` 是手册对象的原始无符号 16 位值；
`--arms` 通过 `Ipeak` 按手册公式换算。工具收到从站 `0x60` 写入确认后才报告
成功，收到 `0x80` 或超时会报错。保存动作只保存控制参数，不执行
`0x2FF0:03` 电机参数保存。

离线测试对照手册报文验证速度、方向、使能/失能、复位、加减速、反馈及混合协议分派。
尚需在 ROS2/Linux 实机验证 ID、编码器分辨率、减速比、旋转方向及实际停机行为。

## 本次验证记录

2026-11-10 在 Windows 上运行离线测试。直接运行完整 `pytest -q` 时，
收集阶段有 14 个错误，尚未进入测试执行：

- `src/motor_control/motor_control/boundary_sensor_field_test.py`：缺少 `rclpy`。
- 三个包 `motor_control`、`mqtt_ros2`、`rtk_nav` 下的
  `test/test_copyright.py`、`test/test_flake8.py`、`test/test_pep257.py`：
  缺少相应 `ament_copyright`、`ament_flake8`、`ament_pep257` 依赖，共 9 项。
- `src/motor_control/test/test_low_battery_cleaning.py`：ROS 消息桩缺少 `String`。
- `src/rtk_nav/test/test_openmv_fill_light_contract.py`：缺少 `openmv_fill_light_debug.py`。
- `src/rtk_nav/test/test_openmv_serial_protocol.py`：缺少 `openmv_serial_protocol` 模块。
- `src/rtk_nav/test/test_openmv_serial_publisher_contract.py`：缺少 `openmv_serial_protocol.py`。

排除电机包的三个 ament 测试及 `test_low_battery_cleaning.py`，并指定独立的临时目录后，
电机包离线测试结果为 81 通过、5 失败。失败项均涉及本次未修改的实现与测试：

- `test_magnetic_navigation.py::test_decode_response_returns_register_and_rejects_bad_crc`
- `test_motor_driver_canopen.py::test_legacy_frame_format_and_speed_commands`
- `test_motor_driver_canopen.py::test_legacy_parameter_commands_and_fault_reset`
- `test_motor_driver_canopen.py::test_stop_all_motors_sends_zero_speed_and_disable_frames`
- `test_motor_driver_canopen.py::test_legacy_feedback_and_fault_parsing_uses_low_byte_motor_id`

后四项仍以 RS02 协议测试已经是 CANopen 实现的 `motor_driver.py`。
本次不修改这些无关失败。完整 ROS2 构建、ament 检查和硬件测试仍需在目标 Linux 环境执行。
