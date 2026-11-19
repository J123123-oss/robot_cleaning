# 步科滚刷协议与配置

`AIMotor` 分支通过 `brush_use_kinco` 切换滚刷协议。`false` 使用原有
`motor_driver(AIMotor).py`。`true` 使用 `motor_driver_kinco.py`，仅将已配置的
ID 3、4 切换为步科 iSMK 协议。ID 1、2 仍调用 AIMotor 原有实现。两类电机共用
同一个 500 kbit/s CAN 接口及收发线程，不修改硬件 ID，不写入永久存储参数。

## 启动配置

`motor_auto_enable` 默认 `true`：主控制节点初始状态直接设为 `ENABLE`，
对所有已配置电机进行零速初始化，只使能 AIMotor。手动 `ENABLE` 和 `START` 采用同一策略。任一选定节点初始化失败，
撤销这些节点的使能并转为 `DISABLE`。可通过 `motor_auto_enable:=false`
恢复启动时全部失能。步科 ID 3/4 仍由 `brush_motor_count` 决定是否配置，
使能前必须确认节点保护参数。步科在首次收到有效 `RUN` 时按需使能。
这里是软件自动使能，不是写入电机的上电自使能参数。

初始化先读取每个节点的实时 `6041`。故障位 bit3 有效时，只对该节点发送
失能及协议对应的故障复位，并在最多 0.5 秒内回读确认故障位清零；
未处于故障的 AIMotor 不发送复位或失能控制字，以保留服务重启时的使能衔接。
步科先重新配置、确认节点保护并恢复 RTR，再处理上次通信中断留下的故障，
避免尚未恢复保护就反复清故障。之后设置模式、方向、加减速，步科保持失能。
两种协议发送使能序列后，都必须在最多 0.5 秒内回读确认
`(6041 & 0x006F) == 0x0027`；发送成功但状态未转换不能算初始化成功。

启动零速初始化成功后，与手动进入 `ENABLE` 共用状态配套设置：
清除 `complete_state` 及完成消息补发计数，取消 MQTT 方向自动停止定时器，
将状态发布周期统一设为 `0.5 s`（2 Hz），并立即发布一次启动状态。
此设置不下发非零轮速或滚刷速度，不重复执行电机初始化；
初始化失败仍进入 `DISABLE`，采用原失能状态的发布策略。

开启磁导航时，`ENABLE` 下连续 3 个磁导航 mask 每次至少 6 点触发，
按现有进仓末端判定确认已在仓内，调用原进仓完成流程停车、置完成标记、
切换 `DISABLE` 并请求充电。无信号或不足 6 点会清零确认计数；
出仓 `START/UNLOADING` 等其他状态不会触发此启动在仓判定。

```bash
ros2 launch rtk_nav run.launch.py brush_use_kinco:=true brush_motor_count:=2
```

`brush_motor_count` 原有默认值仍为 `0`，因此仅打开协议开关不会自动增加电机。
数量为 `1` 时只控制 ID 3，为 `2` 时控制 ID 3、4。

室内仅测试滚刷时，固定车体并隔离滚刷后，可通过 launch 显式开启 `enable_brush_test`：

```bash
ros2 launch rtk_nav run.launch.py \
  brush_use_kinco:=true brush_motor_count:=2 \
  enable_brush_test:=true kinco_brush_speed_rpm:=10.0
```

也可单独启动底盘节点：

```bash
ros2 run motor_control motor_control --ros-args \
  -p brush_use_kinco:=true -p brush_motor_count:=2 \
  -p enable_brush_test:=true -p kinco_brush_speed_rpm:=10.0
```

开关默认关闭。开启后，节点进入 `NORMAL/ENABLE` 即给左右轮发零速并启动滚刷。
步科测试目标为 100 r/min，受 `kinco_brush_speed_rpm` 限制，示例限为 10 r/min。
使用 AIMotor 协议时测试转速为 70 r/min。手动行驶和遥控器接管时，滚刷继续使用该测试转速。
MQTT 手动 `HOLD` 停止车轮，滚刷继续运行。可用键盘 `h` 停止测试，或用 `z` 整机失能。
进入 RTK 自动模式后由 `rtk_nav` 接管滚刷。进出仓及保护停机优先于测试。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `brush_use_kinco` | launch 为 `true`，独立节点为 `false` | true 时使用步科滚刷协议 |
| `kinco_motor_direction_3` | `-1` | 步科 3 号速度指令方向，1 正常，-1 反向 |
| `kinco_motor_direction_4` | `-1` | 步科 4 号速度指令方向，1 正常，-1 反向 |
| `kinco_encoder_resolution` | `65536` | 每个电机轴转动一圈的编码器计数 |
| `kinco_reduction_ratio` | `10.0` | 电机轴转速/滚刷输出轴转速 |
| `kinco_acceleration_rps2` | `10.0` | 电机侧加速度，rps/s |
| `kinco_deceleration_rps2` | `10.0` | 电机侧减速度，rps/s |
| `kinco_brush_speed_rpm` | `110.0` | 滚刷输出轴转速上限，r/min |
| `kinco_guard_time_ms` | `100` | 节点保护 RTR 请求周期，ms，范围 50～65535 |
| `kinco_guard_life_factor` | `5` | 节点保护超时倍数，范围 2～255 |

## 清扫阶段与滚刷控制

混合协议通过 `/rtk/motion_command` 同帧接收轮速和滚刷动作。`RUN` 在首次清扫时确认各滚刷使能，再下发转速。`HOLD_ZERO` 用于可自动恢复的暂停，写零速并保留已有使能。清扫区间结束、返仓、普通 `z` 失能及节点正常退出时，步科先按配置减速，最多等待 3 秒确认滚刷输出轴转速降至 ±1 r/min 以内，再失能并确认 `6041`。混合协议 RTK 指令最大有效期为 0.5 秒。电机故障、反馈无法确认及节点异常退出仍可紧急失能。

更新时一并重建并重启 `custom_msgs`、`motor_control` 和 `rtk_nav` 对应组件。实车先观察 800 设置下的滚刷启动电流和停刷时间，再验收暂停、恢复与返仓时序。

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

正常重启还必须覆盖 ROS 上下文关闭与回调发布并发的情况。仅当异常为
`RCLError/InvalidHandle` 且 `rclpy.ok()` 已为 false 时，按正常退出处理，
由驱动发送零速并保留 AIMotor 使能；同类异常发生在有效上下文中、
以及其他运行异常，仍走致命故障失能路径。接收线程遇到上下文关闭后的
ROS 异常直接结束，不将其累计为 CAN 接收错误或触发总线重置。

2026-11-18 11:21:54 的纯被动抓包复现了重启时失能：
ROS 回调先抛出 `publisher's context is invalid`，旧异常分支调用
`stop_all_motors()`，随后在 11:21:54.374 和 11:21:54.385 分别发出
`601#2B40600000000000`、`602#2B40600000000000`，主动撤销两台轮子的使能。
新进程首读状态为 `0x1250`，只能重新执行 `06→07→0F`。
这是正常退出被误判为故障造成的软件失能，不能通过追加自动使能、
缩短启动延时或关闭通信保护解决。

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
另须执行一次仅运行 `candump`、停服间隙不注入查询或保活帧的重启测试，
检查退出日志无误判致命错误、ID 1/2 无失能控制字、新进程首读仍已使能。
持续发送 `6041` 查询会改变总线通信条件，不能单独用来验证断通信保持行为。

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
主控制节点 `motor_fault_callback()` 收到非零故障码后，会实时读取对应节点的 `6041`。
若故障位已经清零，该消息视为已恢复的旧采样，不发送复位、不破坏当前使能；
启动阶段 ROS 队列中残留的旧 `0x8100` 不能使正常轮子再次失能。
若确认仍有故障，先将整机转为 `DISABLE`、全部零速失能，
再仅向确认故障的节点发送清故障指令；步科使用 `6040:00=0x0086`。
若状态回读失败，也转 `DISABLE`，不盲目清故障。
故障回调不重新使能、不恢复速度。重启程序或显式 `x` 会按初始化流程
恢复保护、处理已锁存故障，并确认所有选定节点零速使能成功。

### 2026-11-18 启动日志诊断

现场日志在 `10:08:03.850` 输出零速使能完成，随后 `10:08:03.889` 至
`10:08:04.345` 多次出现滚刷 `0x8100` 和“发送清除故障指令”。
旧回调会因任意滚刷故障向全部节点发送复位：AIMotor 为 `6040=0080`，
步科为 `6040=0086`，均不等于正常使能 `000F`，软件状态却仍保留 ENABLE。
这解释了轮子也失能、再次发送 `x` 才恢复的控制流程缺陷。
日志只有 ROS 故障快照，没有当时逐节点的 `6041/2602` 或 RTR 总线记录，
因此不能仅凭它断言 `0x8100` 是本次实时超时还是上次停服留下的故障。
根据手册，`0x8100` 属总线通信错误；节点保护中断是当前配置下需要核查的原因，
也应检查 CAN 线路、终端电阻及 RTR 是否保持 100 ms 周期。

日志中的其他启动问题独立处理：

- `test.fence.json` 不存在：RTK 节点锁定自动清扫，不是 CAN 电机故障；
  应为现场路径提供匹配的围栏文件，本次不生成或修改现场边界。
- MQTT `ca_cert` 的参数声明警告：是 ROS 参数 API 的弃用提示，
  日志中没有它导致使能失败的证据。
- `rtk_status="0 / Ori 0"`、经纬度为空：该时段尚无有效定位/航向数据，
  与电机控制字或电流为零是不同问题。

现场需在受控低速、无负载条件下验证：观察 RTR 周期，终止控制进程或断开 CAN，
确认故障与停机，再手动恢复；本地离线测试不能替代驱动器硬件验证。

## 速度与方向配置

编码器分辨率 `65536` 来自手册第 105 页的示例，不是对现场型号的确认。
当前现场配置的机械减速比为 `10.0`；必须按实际安装传动比设置，不要沿用
AIMotor 滚刷的默认减速比 40。
启用步科协议后，程序中默认的滚刷运行转速及 RTK 导航清扫时的滚刷输出轴目标转速为 `100 r/min`；
AIMotor 协议仍使用 `70 r/min`。`kinco_brush_speed_rpm` 是步科滚刷的输出轴
转速上限，外部速度指令超过该值时保留正负方向并截断绝对值；设为 `0.0` 时不限制。
步科驱动器的波特率须配置为 500 kbit/s，ID 须预先设置为 3、4。

`brush_use_kinco=true` 时，3/4 号分别读取 `kinco_motor_direction_3/4`：
`1`（正常）和 `-1`（反向）由步科驱动映射到 `0x607E:00=0/1`。
launch 默认 3 号为 `-1`、4 号为 `1`；
1/2 号仍读取 `motor_direction_1/2`。
`brush_use_kinco=false` 时，所有 AIMotor 仍读取 `motor_direction_1..4`。
主节点选好方向后通过构造参数 `motor_directions` 传入驱动。对于字典中显式指定的 ID，
基类声明 `motor_direction_*` 时设置 `ignore_override=True`，防止同一进程的
AIMotor 参数覆盖步科方向。单独运行驱动或字典中未指定的 ID，仍可通过 ROS 参数
`motor_direction_*` 配置，未配置时使用默认方向 `1`。
参数只应用于 `brush_motor_count` 选中的滚刷。`brush_direction_mode` 的原有速度符号处理
继续生效，设为 `opposite` 时还会对 4 号速度取反，与电机方向参数叠加。
手册印刷页 42、79 规定方向只能在失能状态配置。设置方向时先发送零速和
`0x6040=0x0006`，最多等待 0.5 秒实时回读 `6041`，确认
`(status_word & 0x006F) == 0x0021` 后才写入 `607E`。
写入后读取 `607E:00`，只有回读值与配置一致才继续初始化、使能；
无响应、SDO 中止、仍处于使能或故障状态、方向回读不一致均使初始化失败。
日志 `Kinco node N direction confirmed: direction=..., 607E=...`
记录驱动器实际确认的方向，不能用“CAN 报文发送成功”代替设置成功。

`set_brush_speed()` 的正负号是逻辑速度方向，`607E` 决定其对应的物理转向；
配置 `-1` 不要求 `60FF` 指令或反馈速度显示为负值。
速度换算不再次乘方向，避免与驱动器极性叠加后抵消反转。
如果还启用 `brush_direction_mode=opposite`，4 号的逻辑速度也会取反，
应同时核对这两项设置。需要现场确认实际转向时，可只读查询：

```bash
cansend can0 603#407E600000000000
cansend can0 604#407E600000000000
```

在 `0x583/0x584` 响应 `4F 7E 60 00 xx 00 00 00` 中，
`xx=00` 对应正常、`xx=01` 对应反向。当前 launch 默认期望 3 号为 `01`、4 号为 `00`。
若有 `0x8100` 通信中断故障，需先恢复通信并按使能流程重新初始化，
仅写速度或改变逻辑速度正负号无法解决通信故障。

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

编码器分辨率为 65536 时，按当前驱动的换算，10 rps/s 的加速度写入 `23 83 60 00 F1 29 00 00`，减速度写入 `23 84 60 00 F1 29 00 00`。
初始化配置步科速度模式 `6060=3` 并保持滚刷失能。按需使能时，控制字按
`0006→0007→000F` 完成状态机使能，并保持 `6040h=000F`；发送 `0006` 仅用于失能；
加速度与减速度均通过 SDO 回读确认。
加减速度换算为零或超出 UINT32 时，在启动线程前拒绝配置；加速度写入或回读失败时拦截
该滚刷的后续使能，重新成功配置后才解除拦截。
使能前清零旧速度目标；停止和退出时沿用现有零速与失能流程。

## 反馈与当前限制

步科通过 SDO 读取实际位置、速度、状态、实际电流、温度、应急错误码和两个错误状态字。
不套用 AIMotor 默认 PDO 映射或厂家故障历史对象。收到写入确认、中止响应或长度
不足的帧时，不将其当作速度反馈；等待中的 SDO 回读仍能收到正常响应及中止通知。
`fault_code` 保留应急错误码，`kinco_error_status` 和 `kinco_error_status_2`
分别保留两个错误状态字。

### 应急故障解析

依据《Kinco iSMK 一体机使用手册 20250523》9.3.3.6
（印刷页 112，PDF 第 113 页），驱动的 `KINCO_FAULT_TABLE` 覆盖表 9-24
全部 30 个非零应急故障码，另以 `0000` 表示无故障。
`8100` 为“总线通讯错误”，`81FF` 为“总线通讯超时”，两者分别保留。
手册将 `7331` 同时用于“通讯式编码器没有连接”和“通讯式编码器没有回应”，
因此显示为“通讯式编码器没有连接或没有回应”，不推测具体是哪一种。
未知码显示“手册未定义故障码 0xXXXX”，仍保留原始数值。

EMCY 的 8 字节按表 9-23 解析：

| 字节 | 对象及含义 |
|------|------------|
| Byte 0–1 | `603F:00` 应急故障码，小端 UINT16 |
| Byte 2 | `1001:00` 错误寄存器，UINT8 |
| Byte 3 | 表中未定义用途，不用于故障推断 |
| Byte 4–5 | `2601:00` 错误状态，小端 UINT16 |
| Byte 6–7 | `2602:00` 错误状态，小端 UINT16 |

`KINCO_ERROR_REGISTER_TABLE` 按表 9-25 解析全部置位项：
bit0 一般错误、bit1 电流、bit2 电压、bit3 温度、bit4 通讯错误、
bit5 设备配置文件特定、bit6 编码器、bit7 保留。
错误寄存器可以同时有多个置位；它不是第二个应急故障码。
`2601/2602` 继续保留原始状态字，不套用 `1001` 的位定义。

收到非零 EMCY 时以 ERROR 记录故障中文说明、错误寄存器各置位说明及两个状态字，
`0000` 恢复报文以 INFO 记录。SDO `603F` 的有效故障码变化也使用同一张表记录说明。
这些解析不会额外发送 CAN 查询、清故障或使能指令，也不改变 AIMotor 的故障码含义。

例如 `00 81 10 00 00 00 00 10` 解析后的日志为：

```text
Kinco node 3 EMCY=0x8100 (总线通讯错误), 1001=0x10 (通讯错误), 2601=0x0000, 2602=0x1000
```

### 实际电流换算

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
| `fault_description` | `KINCO_FAULT_TABLE` 对应的中文故障说明 |
| `error_register` | 最近一次 EMCY Byte 2 的 `1001:00` 原始值 |
| `error_register_descriptions` | 错误寄存器所有置位项的中文说明列表 |
| `error_status` / `error_status_2` | `2601:00` / `2602:00` 错误状态字 |
| `velocity_raw` / `position_raw` / `current_raw` | 对应 SDO 原始有符号 DEC 值 |
| `sample_age_sec` | 各反馈字段距最后有效响应的秒数 |

未收到有效反馈时，对应字段及其年龄为 `null`；通信中断后保留最后一次值，
其年龄持续增大，订阅端应按年龄判断是否过期，不能仅凭零速或零故障码判断在线。
`fault_description` 从 `fault_code` 派生；`error_register_descriptions` 从
`error_register` 派生，各自使用原始字段的年龄判断时效。
错误寄存器只从 EMCY 取得，不新增 SDO 轮询；只收到 `603F` 时它仍为 `null`。
寄存器确认值为 `0` 时说明列表为 `[]`，未收到时为 `null`。
JSON 使用中文原文输出，保留既有数字字段，便于直接订阅调试。
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
