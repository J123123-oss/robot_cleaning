# 电子围栏状态入口门控设计

## 目标

缺少当前路线同名 `.fence.json` 或电子围栏故障已锁存时，禁止底盘进入
`AUTO_CLEANING`，不执行首点导航。故障状态统一表现为：

- `status`: `HOLD`
- `nav_status`: `PAUSE`
- `error`: 包含 `ERROR_GEOFENCE_OUTSIDE = 1024`

## 方案

在 `motor_control.switch_state()` 的 `AUTO_CLEANING` 入口增加围栏预检，
检查当前 `rtk_path_file` 对应的 `.fence.json` 是否存在，以及围栏故障是否
已锁存。预检失败时保持或切换到 `HOLD`，发布 `PAUSE` 导航状态和故障码。

`rtk_nav` 继续保留运行时二次拦截，防止其他状态入口绕过底盘预检；缺少围栏
文件时在初始移动前停车，不允许驶向第一个航点。

电子围栏越界锁存后，`rtk_nav` 在底盘保持 `HOLD` 时持续检查当前位置；只有
确认当前位置已回到允许区域，才通过 `/rtk/geofence_recovery_ready` 发布恢复
就绪。`motor_control` 收到该信号后才允许下一次 `AUTO_CLEANING`，并清除本地
1024 锁存，避免故障永久阻塞，同时拒绝仍在围栏外的重复 AUTO。

导航状态和底盘状态分离：`/rtk/nav_state` 的人工介入暂停始终映射为
`nav_status=PAUSE`，底盘通过 `status=HOLD` 表示已停车。重复的
`AUTO_CLEANING` 指令在故障锁存期间被拒绝，不再产生
`status=AUTO_CLEANING, nav_status=HOLD` 的短暂异常组合。

## 验证

增加源代码契约测试覆盖入口预检、状态映射和缺围栏时的首点拦截；运行
RTK 与电机控制相关测试及 Python 语法检查。
