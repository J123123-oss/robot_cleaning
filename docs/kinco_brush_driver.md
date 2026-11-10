# 步科滚刷协议与配置

`AIMotor` 分支通过 `brush_use_kinco` 切换滚刷协议。默认 `false` 使用原有
`motor_driver(AIMotor).py`；`true` 使用 `motor_driver_kinco.py`，仅将已配置的
ID 3、4 切换为步科 iSMK 协议。ID 1、2 仍调用 AIMotor 原有实现。两类电机共用
同一个 500 kbit/s CAN 接口及收发线程，不修改硬件 ID，不写入永久存储参数。

## 启动配置

```bash
ros2 launch rtk_nav run.launch.py brush_use_kinco:=true brush_motor_count:=2
```

`brush_motor_count` 原有默认值仍为 `0`，因此仅打开协议开关不会自动增加电机。
数量为 `1` 时只控制 ID 3，为 `2` 时控制 ID 3、4。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `brush_use_kinco` | `false` | true 时使用步科滚刷协议 |
| `kinco_encoder_resolution` | `65536` | 每个电机轴转动一圈的编码器计数 |
| `kinco_reduction_ratio` | `1.0` | 电机轴转速/滚刷输出轴转速 |
| `kinco_acceleration_rps2` | `100.0` | 电机侧加速度，rps/s |
| `kinco_deceleration_rps2` | `100.0` | 电机侧减速度，rps/s |

编码器分辨率 `65536` 来自手册第 105 页的示例，不是对现场型号的确认。
减速比 `1.0` 表示直驱。安装减速机时必须按实际数值设置。例如实际减速比
为 5 时应加 `kinco_reduction_ratio:=5.0`；不要沿用 AIMotor 滚刷的默认减速比 40。
步科驱动器的波特率须配置为 500 kbit/s，ID 须预先设置为 3、4。

`motor_direction_3/4` 继续使用 `1`（正常）和 `-1`（反向），步科驱动分别映射到
`0x607E:00=0/1`。`brush_direction_mode` 的原有速度符号处理继续生效。
方向只能在失能状态配置，初始化先发送 `0x6040=0x0006`。

## 协议依据

参考用户提供的《步科电机常用报文说明.docx》和《Kinco iSMK一体机使用手册
20250523.pdf》。下列页码为手册印刷页码，PDF 页码比印刷页码大 1。
文档中的操作说明只作为协议资料，不作为自动修改硬件 ID、保存参数或复位设备的指令。

| 功能 | 地址或格式 | 手册位置 |
|------|------------|----------|
| 速度模式 | `0x6060:00=3`，1 字节 | 42、78 页 |
| 使能/失能/故障复位 | `0x6040:00=0x000F/0x0006/0x0086`，2 字节 | 78 页 |
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
初始化先失能滚刷，再复用现有流程设置模式、方向和加减速，减速度通过 SDO 回读确认。
加减速度换算为零或超出 UINT32 时，在启动线程前拒绝配置；加速度发送失败时拦截
该滚刷的后续使能，重新成功配置后才解除拦截。
使能前清零旧速度目标；停止和退出时沿用现有零速与失能流程。

## 反馈与当前限制

步科通过 SDO 读取实际位置、速度、状态、温度、应急错误码和两个错误状态字。
不套用 AIMotor 默认 PDO 映射或厂家故障历史对象。收到写入确认、中止响应或长度
不足的帧时，不将其当作速度反馈；等待中的 SDO 回读仍能收到正常响应及中止通知。
`fault_code` 保留应急错误码，`kinco_error_status` 和 `kinco_error_status_2`
分别保留两个错误状态字。

当前不读取或换算步科实际电流与转矩，避免沿用 AIMotor 的标度。
手册 80 页的电流换算说明列出 6071/6073，未明确说明实际电流 6078 的标度，
且现场驱动器型号及峰值电流尚未确认。因此，步科单滚刷的软件电流堵转保护暂时没有
有效采样输入；原有电池总电流保护继续使用。后续确认实际电流标度后再接入。

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
