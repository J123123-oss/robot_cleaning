# 导航定位保护与到点确认

本次修改独立于航段方向缓存修复，作为第二次提交交付。只修改导航位置的可信性检查和到点确认，不调整 Stanley 减速曲线、RTK 质量去抖时间或电机协议。

## 定位跳变

在天线位置按当前航向换算成车体中心后检查位移，再发布 `/car_center_gps` 和更新导航位置。普通 RTK 样本距上一可信车体中心位置的允许位移为 `实时速度（m/s）× min(样本时间差, 1.0 s) + 0.30 m`。样本时间差取本帧与上一可信定位帧的原始 GGA 时间戳之差。

实时速度取 `/odom` 中 `twist.twist.linear.x` 的绝对值，前进和倒车使用同一规则。里程计样本缺失、速度无效或接收超过 0.25 秒时，速度项按 0 m/s 计算，允许位移为 0.30 米。

使用 `WTRTK.position_stamp_ns` 携带的原始 GGA 定位时间戳识别新样本。WTRTK 航向帧可以复用缓存 GGA，因此其 `header.stamp` 不能证明定位更新。重复、乱序样本不更新导航坐标或确认计数。

自动导航中超限时立即停车并进入现有 AUTO 航向门控，保留上一可信坐标和定位基准。触发时的位移界限冻结，避免停留一段时间后将同一异常坐标误认为可接受。

连续 3 个新的双 Fixed 有效样本返回原可信位置允许范围后解除位置阻塞，再由已有航向门控决定恢复时机；中途质量异常重新计数。异常样本不能更新备用里程计定位的双 Fixed 锚点。持续偏移需要现场核实后切出自动模式；非自动模式下的新有效样本可重新建立位置基准。

重复、乱序或无效时间戳的 `/wtrtk_data` 在回调入口丢弃，不能刷新数据超时计时或航向稳定窗口。

启用现有 IMU/里程计备用定位时，若积分有效且里程计数据仍新鲜，恢复定位与积分预测位置比较，沿用现有 0.30 米恢复残差上限，避免把失锁期间真实行驶的路程误判为跳变。备用定位及切回平滑期间不累计到点确认。

备用定位遇到跳变时转入可恢复的停车等待，冻结行驶输出；位置恢复后仍须经过已有双 Fixed 稳定和位置融合流程，不得因保护自身产生的暂停转成人工锁定。

## 构建与部署

本次增加 `WTRTK.msg` 的 `uint64 position_stamp_ns` 字段，须同时更新消息包、串口驱动、回放驱动和导航节点。不能混用旧消息定义或旧驱动；未提供原始定位时间戳的样本不参与导航定位。

在 ROS2 Jazzy 环境运行：

```bash
colcon build --symlink-install --packages-select custom_msgs
source install/setup.bash
colcon build --symlink-install --packages-select rtk_nav
source install/setup.bash
```

## 到点确认

普通航点小于 0.10 米时立即输出零速，连续 3 个新的可信样本仍在范围内才开始最终航向校准。首点保持原有 5 样本要求。

普通航点在定位回调中逐帧累计，定时器重复读取同一帧不能追加计数；两次定时器之间出现的出界样本也会打断连续性。定位或定向非 Fixed、定位异常、目标变化以及暂停恢复均重新开始计数。

普通航点最终航向校准结束时，须确认当前定位可信且距原航点小于 0.50 米。超差时最多自动回对准两次，耗尽后停车等待人工处理。回对准使用最终转向前后的可信位置估计位移，向反方向设置临时接近点，补偿距离不超过 0.4 米；位移不可用或过小时接近原航点。临时点使用普通航点的三帧到点确认，接近前和接近期间检查至临时点的围栏线段。到达临时点后仍按原航点要求转向，并以转向后距原航点的距离决定是否切点。该预测依赖下一次转向产生相似位移，不判断转向期间的连续位移；不改变全局 RTK 跳变保护或电子围栏边界。首点最终校准及航向异常恢复不受此规则覆盖。

## 验证范围

离线测试使用真实位置处理方法和导航生成器，替代 ROS 消息发布接口。覆盖低速跳变、原地跳变、合法连续移动、重复和乱序时间戳、无效坐标、跳变恢复、首点确认以及切点后确认隔离。

现场仍需验证 5 厘米定位噪声容差适配实际接收机噪声；测试中记录跳变日志、原始 `/wtrtk_data`、`/car_center_gps` 和 `/motor_speed`，确认被拒绝的位置不参与导航或切点。

### 2026-11-18 离线验证记录

- 本次相关 75 项测试通过，包含真实串口/回放解析方法、GGA 1Hz 与航向 10Hz 的不同频率、两种到点生成器、备用定位跳变恢复和既有边界/进度回归。
- 8 个修改或新增的 Python 文件通过语法编译；本次差异通过 `git diff --check`。
- 当前 Windows 环境不能执行 ROS2 消息生成和 `colcon` 实机构建。全量 `pytest` 在收集阶段有 14 个既有错误：三个 ROS 包的 9 个 ament 检查、`boundary_sensor_field_test.py`、`test_low_battery_cleaning.py`，以及缺失 OpenMV 源文件导致的 `test_openmv_fill_light_contract.py`、`test_openmv_serial_protocol.py`、`test_openmv_serial_publisher_contract.py`。
- `test_waypoint_tracking_recovery.py` 中本次更新的两项到点/转向测试已包含在 75 项通过中。其余测试与 `7c7e8e7` 的源码基线对照，均为 1 项通过、19 项失败，未新增失败；旧测试仍依赖当前源码已无的视觉融合/航向恢复/进展计数接口或不完整的旧夹具。

上述 19 项既有失败对应以下用例及其参数组：

```text
test_centimeter_position_noise_does_not_clear_no_progress_count
test_disabled_rtk_correction_does_not_allow_a_recovery_turn (2组)
test_first_waypoint_uses_same_recovery_guard_after_initial_alignment (2组)
test_force_bearing_progress_prevents_false_fourth_alignment_stop
test_force_bearing_without_progress_retains_fourth_alignment_stop
test_heading_guard_accepts_only_the_commanded_recovery_corridor (7组)
test_initial_completion_does_not_compare_new_target_to_old_minimum
test_logged_stanley_recovery_is_not_interrupted_after_five_frames
test_shared_geometry_preserves_logged_wheel_commands (2组)
test_uncommanded_turn_still_stops_and_requests_recalibration
```
