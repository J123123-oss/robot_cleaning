# Stanley 控制器修复记录

分支: `stanley` | 日期: 2026-05-24

## 提交历史

```
本次提交 fix: 航向异常超时保护+RTK恢复守卫+MQTT方向定时器HOLD清理
df82b2f fix: 航向角异常超时检测，IMU卡死时暂停导航防止反复校准死循环
bfa045d Stabilize Stanley line correction
82df935 fix: MAX_LATERAL_ERROR 0.2→5.0，消除横向纠偏饱和导致的抵消
620aef2 fix: path_direction 回退为固定路径方向，仅在越过终点时切换为指向目标
da228ab fix: 添加 Stanley 内部 debug 日志，排查长直线纠偏减小问题
9d203bf fix: path_direction 改为动态指向目标航点，防止越过后无法纠正
a1b4900 fix: STANLEY_MIN_SPEED 0.25→0.10，匹配实际低速段
a311ffb fix: 降低低速段横向纠偏强度，防止压过航向纠偏
fc2c98a fix: Stanley 控制器横向纠偏符号反转及航点切换路径方向未更新
```

## 修复清单

### 1. 转向符号反转（fc2c98a）
- **问题**: `speed_diff = steering_factor * STRAIGHT_MAX_CORRECTION` 导致纠偏方向与实际需求相反
- **修复**: `speed_diff = -steering_factor * STRAIGHT_MAX_CORRECTION`
- **影响**: 所有直线行驶阶段（INITIAL_MOVE, WAYPOINT_MOVE）

### 2. 航点切换路径方向未更新（fc2c98a）
- **问题**: `stanley_path_start` 和 `last_waypoint_cache` 在航点切换后未更新
- **修复**: 三处添加 `stanley_path_start = None`（初始移动完成、校准完成、切换检测），校准完成时同步更新 `last_waypoint_cache`
- **影响**: WAYPOINT_MOVE 航点间移动

### 3. 电机指令→真实速度转换（fc2c98a）
- **问题**: `atan(K * lateral / velocity)` 中 velocity=10（电机指令）而非 0.375 m/s（真实速度），压制横向纠偏 ~27 倍
- **修复**: 引入 `SPEED_CMD_TO_MPS = 0.0375`，`real_velocity = velocity * SPEED_CMD_TO_MPS`
- **参数调整**: STANLEY_K_BASE 0.4→0.5, STANLEY_MIN_SPEED 0.3→0.05, MAX_LATERAL_ERROR 0.15→0.3

### 4. MAX_LATERAL_ERROR 钳位致横向/航向纠偏抵消（82df935）
- **问题**: MAX_LATERAL_ERROR=0.2 把实际 2m 偏差钳在 0.2m，st_corr 饱和在 -16.2°，恰好与 hdg_err=+16° 抵消，total≈0
- **日志现象**: hdg_err 从 0° 增到 30°，lat_err 从 0.1m 增到 2m，但 st_corr 始终 -16.2°，left/right 几乎对称（无纠偏）
- **修复**: MAX_LATERAL_ERROR 0.2→5.0（实际等于不钳位），STANLEY_MIN_SPEED 已提供低速保护

### 5. path_direction 动态= bearing(robot,target) 导致 heading_error≈0（620aef2）
- **问题**: bearing(robot→target) 做 path_direction 导致 heading_error≈0，长直线只靠 lateral 项纠偏
- **修复**: 回退为固定 path_direction + 投影检测越过终点时翻转

### 6. path_direction 固定值导致越过航点后无法纠正（9d203bf）
- **问题**: `path_direction` 为固定值（路径起点→终点方位角），机器人越过航点后 `heading_error ≈ 0`（朝向与 path 一致），Stanley 认为不需要纠正，机器人持续远离
- **日志现象**: 距航点 0.14m → 0.55m 递增，同时 `speed_diff`（纠偏强度）递减至 0
- **修复**: `path_direction` 改为动态计算（`calculate_bearing(current_pos, target)`），始终指向目标；越过后 `heading_error ≈ 180°`，强制回转
- **影响**: `move_to_first_waypoint` 和 `WAYPOINT_MOVE` 两处

### 7. 低速段横向纠偏过激（a311ffb + a1b4900 + bfa045d）
- **问题**: 接近航点减速时 real_velocity=0.17，3cm 偏差→10° 纠偏，压过航向修正
- **修复**: STANLEY_MIN_SPEED 0.05→0.25，MAX_LATERAL_ERROR 0.3→0.2，近距K 1.0→0.6
- **二次修正** (a1b4900): STANLEY_MIN_SPEED 0.25→0.10，因低速段电机指令~2.0 (real_velocity≈0.075) 低于 0.25 地面，压制纠偏 3.3x
- **近期修正** (bfa045d): 到达阈值收紧为 0.10m，初始移动阈值收紧为 0.10m；近距K调整为距离<1.3m取0.26，其他取0.25；当前 `STANLEY_MIN_SPEED=0.15`、`MAX_LATERAL_ERROR=1.0`
- **起步横偏修正**: 最新日志中起步段 `lat_err≈0.08m` 但差速修正偏弱，同时接近航点时 `hdg_err≈5~6°` 与横向项有抵消；将常规K提升到0.45，近距K提升到0.42，`STRAIGHT_MAX_CORRECTION` 提升到1.5
- **航向优先抑制**: 起步横偏回收变快后，短段仍可能出现 `st_corr` 与 `hdg_err` 同向并把车体航向推到 >5°；当 `abs(hdg_err)>4°` 且横向项同向时，将横向修正减半，优先把车头拉回路径方向
- **效果**: 低速段横向项保留足够拉回能力，同时避免接近航点时横向纠偏完全压过航向项

### 8. 航向异常保护（b1d106c）
- **问题**: 行驶中如果车体航向与当前有效路径方向偏差持续过大，Stanley会继续输出直线纠偏，可能沿错误方向扩大偏差
- **修复**: 新增 `HEADING_ABNORMAL_THRESHOLD=15.0` 和 `ANGLE_ABNORMAL_COUNT=5` 组合保护；连续5帧超阈值后进入 `WAYPOINT_CALIB`，目标为当前有效 `path_direction`
- **恢复处理**: 从暂停恢复到 `WAYPOINT_MOVE` 时也检查航向偏差，若超过15°先重新校准
- **关键约束**: 异常重校准完成后不推进航点索引，继续前往当前航点；正常到达航点后的校准才推进到下一个航点

### 9. 初始校准完成后误回航点0（本次提交）
- **问题**: 初始移动到达第一个航点并按航点0航向完成最终校准后，残留的 `is_angle_recalib` 标志会让 StopIteration 分支继续前往航点0
- **日志现象**: `开始最终航向校准：目标89.74°` 后校准成功，但马上出现 `角度异常重置后继续前往航点0`，随后 Stanley 生成 `航点0路径：(点0) → (点0), 方向=0.0°`
- **结果**: 刚校准到约90°的车体被拿去对比0°零长度路径方向，触发 `hdg_err≈-90°` 并二次错误校准到0°
- **修复**: 初始移动完成后统一清除残留 `is_angle_recalib`，并强制 `current_waypoint_idx = 1`
- **影响**: 初始点→航点0完成后的第一段正式路径；按当前路径点1→点2理论方向约89.78°，与航点0最终航向89.74°一致

## 参数最终值

| 参数 | 原始值 | 最终值 |
|------|--------|--------|
| STANLEY_K_BASE | 0.4 | 0.5 |
| STANLEY_MIN_SPEED | 0.3 (电机指令) | 0.15 (Stanley计算最小速度) |
| MAX_LATERAL_ERROR | 0.15 m | 1.0 m |
| SPEED_CMD_TO_MPS | — | 0.0345 |
| 短距 K (<1.3m) | 1.0 | 0.42 |
| 常规 K (≥1.3m) | 0.4×v/5 | 0.45 |
| STRAIGHT_MAX_CORRECTION | 3.0 | 1.5 |
| SPEED_LIMIT | 1.5×BASE | 1.3×BASE |
| RTK_WAYPOINT_TOLERANCE | 0.2 m | 0.10 m |
| INITIAL_MOVE_TOLERANCE | 0.2 m | 0.10 m |
| HEADING_ABNORMAL_THRESHOLD | — | 15.0° 连续5帧 |
| 航向优先抑制 | — | `abs(hdg_err)>4°` 且横向项同向时横向修正减半 |

## 待验证项
- [ ] 初始移动完成后日志应进入 `航点1路径`，不应再出现 `航点0路径：(点0) → (点0), 方向=0.0°`
- [ ] 航点0最终校准到约90°后，点1→点2路段 `path_dir` 应接近89.78°
- [ ] 航向异常重校准完成后应继续当前航点，不能误推进或误回退索引
- [ ] `SPEED_CMD_TO_MPS` 系数是否匹配实际车速（当前 10→0.345）
- [ ] 横向偏差 > 0.1m 时，Stanley 是否有效拉回路径
- [ ] 起步段 `lat_err≈0.08m` 时应在 1~2m 内明显回收，不再需要 4~5m
- [ ] 横偏回收过程中 `hdg_err` 不应持续超过 5°

## 2026-05-22 RTK 数据超时保护

- **问题**: `wtrtk_serial_driver` 串口掉线并重连时，`/wtrtk_data` 可能完全停止输出；原导航逻辑只在收到新的非固定解消息时暂停，断流时会继续使用最后一次缓存的 `current_gps` 和 `imu_yaw` 执行 Stanley 控制。
- **现场证据**: `wtrtk_check.log` 中 `ch341` 反复出现 `USB disconnect` / 重新 attach，`lsof /dev/WTRTK` 显示仅 `wtrtk_serial_driver` 占用串口，基本排除多进程抢占，倾向于 USB/串口链路掉线。
- **修复**: 新增 `RTK_DATA_TIMEOUT=1.0` 和 `RTK_TIMEOUT_LOG_INTERVAL=2.0`；`heading_callback()` 每次收到 `/wtrtk_data` 记录时间；`rtk_timer_callback()` 在 AUTO_CLEANING 模式下优先检查超时。
- **保护动作**: 超过 1 秒未收到 `/wtrtk_data` 时立即 `publish_stop_speed()`，保存 `pre_pause_state` 和滚刷状态，将导航状态置为 `PAUSE`，并发布 `/rtk/nav_state`；断流期间每 2 秒重复停车并打印保持停车日志。
- **恢复动作**: 重新收到 `/wtrtk_data` 后清除超时标志；若恢复消息为 `RTK Fixed`，沿用原有固定解恢复逻辑自动回到暂停前状态。
- **验证**: 已使用 bundled Python 执行 `python -B -m py_compile src/rtk_nav/rtk_nav/rtk_nav.py`，语法检查通过。

## 2026-05-22 固定进仓 loading_gps

- **问题**: LOADING 的 RTK 返航点使用 UNLOADING 完成后发布的实时 `/unloading_gps`。实际出仓点可能存在小幅漂移，导致 RTK 清扫结束后的进仓导航不是固定点位。
- **修复**: `rtk_nav.py` 新增固定 `loading_gps` 参数和 `BUILTIN_LOADING_GPS` 默认值，节点启动时加载为 `loading_waypoint`；`/unloading_gps` 回调只记录日志，不再覆盖进仓航点。
- **配置**: `run.launch.py` 新增 `loading_gps = [0.0, 0.0, 0.0]` 并传入 `rtk_nav`，现场标定后填写 `[经度, 纬度, 航向角]`。
- **保留逻辑**: RTK 导航遍历完所有路径文件后，仍在 `get_target_waypoint()` 中追加到最后一个航点；继续使用 `return_to_loading_added` 保证每轮任务只追加一次。
- **验证**: 已使用 bundled Python 执行 `py_compile` 检查 `src/rtk_nav/rtk_nav/rtk_nav.py` 和 `src/rtk_nav/launch/run.launch.py`，语法通过。

## 2026-05-23 导航状态、区域和错误上报整理

- **清扫区域上报**: `load_waypoints_from_file()` 解析路径文件中的普通注释行作为区域名，按航点同步保存到 `waypoint_areas`；新增 `/rtk/cleaning_area` 发布器，`rtk_timer_callback()` 周期性发布当前航点所在区域，路径切换时清空并重新发布。
- **RTK error 上报**: 新增 `/rtk/error_status` (`Int16`) 和 `update_rtk_error_status()`；AUTO_CLEANING 下 RTK 非固定解发布 `ERROR_RTK_NOT_FIXED`，数据断流发布 `ERROR_RTK_TIMEOUT`，恢复固定解或退出 AUTO_CLEANING 后清零。
- **边界触发防抖框架**: 增加 `BOUNDARY_TRIGGER_CONFIRM_FRAMES` / `BOUNDARY_CLEAR_CONFIRM_FRAMES` 以及 `update_boundary_trigger_state()`，用于连续帧确认边界触发、首次触发立即停车、纠偏完成后清理锁定和计数。
- **当前边界触发状态**: `io_data_rtk_callback()` 中 `raw_boundary_trigger` 计算、边界变化日志和 `update_boundary_trigger_state(raw_boundary_trigger)` 调用目前仍处于注释状态；因此边界防抖框架已保留在代码中，但不会实际触发纠偏接管。
- **传感器状态整理**: IO 位解析改为显式 `int(...) << bit` 组合，保留 `sensors_status` 原始位图，避免原先布尔位运算可读性差。
- **导航完成收尾**: `COMPLETED` 收尾路径补齐停止 generator、停车、重置导航上下文并发布 `IDLE`；单航点路径在航点0完成后仍推进索引，让原有 `get_target_waypoint()` 追加固定进仓点逻辑继续生效。

## 2026-05-24 航向异常超时保护修复 + MQTT方向定时器清理

### 10. RTK 恢复覆盖航向异常超时 PAUSE 导致死循环

- **问题**: IMU 航向角卡死在 90° 后，`handle_rtk_data_timeout()` 在 15s 超时正确设置 `nav_state=PAUSE`、`heading_timed_out=True`、`nav_running=False`。但 `check_rtk_fix_status()` RTK 恢复分支（`heading_callback` 内）看到 `nav_state==PAUSE` 就**无条件恢复** `nav_running=True` 和 `nav_state=pre_pause_state`(WAYPOINT_CALIB)，IMU 仍卡死，再次超时→PAUSE→RTK恢复→CALIB，形成死循环。同时 `update_rtk_error_status(0)` 在 `heading_timed_out` 检查之前执行，导致错误码被立即清零（本应为8）。
- **现场证据**: `debug_log.txt` 中 IMU 从 0° 跳到 90° 后，`[航向异常超时] 已停车并暂停导航` → `[RTK状态] 恢复RTK固定解，自动恢复导航` → 再次超时，循环 100+ 秒，期间 RTK 状态在 Fixed(4) 和 Float(5) 之间反复抖动触发恢复。
- **修复**:
  - **RTK 恢复分支增加 `heading_timed_out` 守卫**: 当 PAUSE 由航向异常引起时直接 return，不清理错误码也不恢复导航。
  - **新增 `_is_heading_normal()`**: 通过当前 IMU 航向与路径方向偏差判断是否已恢复，无法判断时 fail-open（返回 True）。
  - **恢复检查机制实质化**: 原本每 3s 返回 False 让 nav loop 运行但 `nav_running=False` 导致 generator 不 tick，形同虚设。改为调用 `_is_heading_normal()` 判断，恢复后才清零标志位 + `nav_running=True`。
- **影响**: `src/rtk_nav/rtk_nav/rtk_nav.py` — `handle_rtk_data_timeout()` 恢复检查分支、新增 `_is_heading_normal()`、`heading_callback()` RTK Fixed 分支。

### 11. MQTT 方向指令 300s 定时器未在 HOLD/DISABLE/START 时取消

- **问题**: MQTT 点按方向指令（FORWARD/BACKWARD/LEFT/RIGHT）创建 300 秒 `direction_timer`，到期调用 `auto_stop` → `switch_state("h")`。HOLD 停止或进仓流程被 HOLD 中断时，定时器未被取消，残留的定时器可能在后续 AUTO_CLEANING 任务期间触发，强制 HOLD 导致导航中断。
- **修复**: 在 HOLD（含 `is_in_bin_process` 分支）、DISABLE、START 四个入口统一取消 `direction_timer` 并置 None。
- **影响**: `src/motor_control/motor_control/motor_control.py` — `switch_state()` 中 DISABLE/HOLD/START 分支。

## 2026-05-23 路径切换延后与MQTT同步

- **问题**: 当前路径文件完成后，`get_target_waypoint()` 会立即加载并执行下一个路径文件，导致同一次RTK清扫连续跨路径执行；改为任务完成后重置状态再切换路径时，自动切换又只发生在 `rtk_nav` 内部，MQTT后台看不到新的 `route_id`。
- **修复**: 当前路径结束时只记录 `pending_next_path_file`，本轮仍追加固定 `loading_gps` 进仓点；`COMPLETED` 后统一停车、重置导航上下文、发布 `IDLE`，再预加载下一条路径。
- **防止误启动**: 预加载下一条路径后设置 `waiting_for_next_unloading=True`，即使控制模式仍是 `AUTO_CLEANING`，`rtk_timer_callback()` 也只保持停车和 `IDLE`，直到下一次UNLOADING后重新切回 `AUTO_CLEANING` 才允许启动。
- **MQTT同步**: `rtk_nav` 新增 `/rtk/current_route_id` 发布器，自动预加载下一路径或手动 `/rtk/route_change` 成功加载后发布当前 `route_id`；`motor_control` 新增订阅该话题，更新本地 `route_id` 并调用 `publish_state()`，沿用 `/robot_state -> mqtt_ros2_bridge -> MQTT` 通路让后台看到路径变化。
- **验证**: 已使用 bundled Python 执行 `py_compile` 检查 `src/rtk_nav/rtk_nav/rtk_nav.py` 和 `src/motor_control/motor_control/motor_control.py`，语法通过。

## 2026-05-26 航向异常检测死锁与反复触发修复

### 12. `_is_heading_normal()` 恢复条件改为数据新鲜度

- **问题**: 航向异常超时停车后，`_is_heading_normal()` 检查 `abs(heading_err) <= 15°`。IMU 停车后持续漂移（~2-12°/s），航向误差始终 > 15°，恢复条件永远不满足。
- **现场证据**: `debug_log.txt` 中机器人停车后 IMU 从 82° 持续漂移到 -8°，航向误差始终 > 30°，147 秒后才碰巧漂回阈值范围。
- **修复**: `_is_heading_normal()` 不再检查航向误差绝对值，改为检查 `last_wtrtk_time` 是否在 `RTK_DATA_TIMEOUT`(1s) 内。数据新鲜说明 IMU 未死机/卡死，航向误差由 Stanley 控制器或重校准主动纠正；数据过期才表示 IMU 真正故障。
- **影响**: `handle_rtk_data_timeout()` 恢复检查分支每 3s 调用此函数。

### 13. INITIAL_MOVE 增加 5 帧连续航向异常重校准

- **问题**: INITIAL_MOVE 阶段航向异常只记录 `heading_abnormal_start_time`，等待 15s 全局超时。WAYPOINT_MOVE 已有 5 帧（0.5s）即时重校准，INITIAL_MOVE 缺少对称保护。
- **修复**: INITIAL_MOVE 中增加 `angle_abnormal_count` 计数，连续 5 帧 `abs(heading_err) > 15°` 后内联调用 `calibrate_heading_at_waypoint()`，以 `yield` 方式原地旋转校准，完成后 `continue` 回到主循环。
- **影响**: `move_to_first_waypoint()` 内 Stanley 控制循环。

### 14. 航向异常状态未在路径切换 / 模式切入时清零

- **问题**: `heading_abnormal_start_time` 和 `heading_timed_out` 在路径切换和 AUTO_CLEANING 切入时未重置。旧导航任务的航向异常计时污染新任务，导致一切入 AUTO_CLEANING 就立即触发超时。
- **现场证据**: `debug_log.txt` 中机器人从 DISABLE 切回 AUTO_CLEANING 时 `heading_abnormal_start_time` 已累积 30.4s，立即触发 `[航向异常超时] 已持续30.4s，已停车`。
- **修复**:
  - 路径切换回调: 加载新路径后清零 `heading_abnormal_start_time = None`, `heading_timed_out = False`
  - `mode_callback`: 进入 AUTO_CLEANING 时同上清零
- **影响**: `path_change_callback()` 和 `mode_callback()`。

### 15. t>1.0 方位角模式排除航向异常检测

- **问题**: t>1.0 时 `path_direction` 从固定段方向切换到 `bearing(current→target)`。当车体侧向接近航点时，方位角与 IMU 航向差可达 80-110°，这是几何现象而非 IMU 异常。上一版只屏蔽了 `angle_abnormal_count`（5 帧重校准）但未屏蔽 `heading_abnormal_start_time`（15s 超时），导致方位角模式下反复触发超时→恢复→超时循环。
- **现场证据**:
  - **振荡**: 车在航点附近 t≈1.0 徘徊，path_dir 在段方向和方位角之间交替跳变 → 5 帧重校准反复触发 → 车原地左右转动
  - **循环超时**: 方位角模式下 `heading_abnormal_start_time` 累积 15s → 停车 → 3s 后恢复 → 仍在方位角模式 → 再次超时，每 ~18s 一个周期
- **修复**: INITIAL_MOVE 和 WAYPOINT_MOVE 中，`t > 1.0` 时:
  - `angle_abnormal_count` 不累加，清空为 0
  - `heading_abnormal_start_time` 不启动，清空为 None
  - `heading_timed_out` 清空为 False
  - 航向误差 <= 15° 时的正常清零逻辑不变
- **安全网**: 方位角模式下的 IMU 彻底死机由 RTK 数据超时（1s）兜底。

### 16. 修复总结: 航向异常检测完整逻辑

```
段模式 (t <= 1.0):
  abs(hdg_err) > 15° × 5帧 → 即时重校准
  abs(hdg_err) > 15° × 15s → 全局超时停车

方位角模式 (t > 1.0):
  不触发任何航向异常保护
  → 航点到达后正常校准流程纠正航向
  → RTK 数据超时(1s) 作为最终安全网

恢复条件:
  数据新鲜 (last_wtrtk_time <= 1s) → 立即恢复
  数据过期 → 保持停车
```
