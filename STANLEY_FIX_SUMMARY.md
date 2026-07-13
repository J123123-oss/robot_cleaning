# Stanley 控制器修复记录

分支: `stanley` | 日期: 2026-06-22

## 提交历史

```
f235f75 refactor: 路径直角转弯改为两次转弯，优化边缘距离配置
a96dec8 fix: NAV_TO_GPS DRIVE阶段恢复航向差速纠偏，避免偏移目标
5fa0344 fix: NAV_TO_GPS DRIVE去掉差速转向，ALIGN对准后纯直线行驶
983f7eb fix: 航向校准卡滞重试和恢复时复用初始校准目标航向
09c62f2 fix: set/clear_rtk_error_bits只改内存不发布，避免非AUTO_CLEANING下错误码泄露
3c3b51f docs: STANLEY_FIX_SUMMARY 补充 #66 航向校准卡滞后退脱困
d5684d5 docs: 更新STANLEY_FIX_SUMMARY — 进仓打滑漂移修复(#64旋转/平移分离 + #65 NAV_TO_GPS后退行驶)
faafd3a fix: 撤销分段转向子状态机，改为旋转/平移分离的位置恢复
66dcb7d fix: LOADING_NAV_TO_GPS支持后退行驶，目标在后方时不调头
4f9a294 fix: 航向校准卡滞重试时增加后退脱困步骤
be90044 fix: 撤销heading_callback的AUTO_CLEANING守卫 + 进仓分段转向优化
8c4ddbf fix: publish_stop_speed不再清零brush_active，防止RTK震荡污染nav_context保存值
71adb2f fix: resume路径通用恢复滚刷状态，覆盖tilt_fault和calib_stuck
59d9224 fix: 倾斜恢复后还原滚刷状态，防止刷停后无法自动重启
1b4a1d7 fix: 航向稳定性检查防INS漂移 + 倾斜阈值10→15° + 短促颠簸跳过120s等待
04bee16 fix: 007-W23-W20航点坐标更新
61de8e6 fix: get_next_path_file最后文件循环回到第一个，支持往复清扫
0b06cb6 docs: 精简REVIEW.md + motor_start.sh补ROS2 source + 007配置更新区域坐标
72b9460 fix: 跌落后INS稳定等待120s，覆盖首次启动和清扫中恢复两条路径
4d82adf fix: PAUSE未停止滚刷(5处修复) + velocity改用电机实际反馈
adf5d4c fix: 航向校准超时修复 + 初始航向校验(90°±25°) + 卡滞提速
1c4ef6b fix: 滚刷索引改为队列，支持同一文件内多段刷区 + 过期索引清理
5062800 fix: #注释行start/stop分支中elif改为if，确保区域名同步更新
f84526c fix: 进仓导航超时从60s延长到120s，给GPS固定解恢复留足时间
43fc0ff fix: Stanley去对半砍 + t>1.0切force_bearing防追尾螺旋
a19a247 fix: state_callback/mode_callback竞态导致waiting标志残留，阻塞下次清扫启动
d40c5b5 feat: 倾斜跌落检测+angle姿态角MQTT上报+nav_context调试话题+日志精简
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
- [ ] DRIVE 直线行驶航向纠偏：航向差应在 2° 以内，不应出现旧版"来回小幅摆头"或新版"航向持续漂移"
- [ ] 进仓 DRIVE 阶段日志应出现 `航向差` 和 `纠偏` 字段，纠偏值应在 ±2.0 范围内平滑变化

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
- **修复**: 在 HOLD（含 `is_in_bin_process` 分支）、DISABLE、ENABLE 四个入口统一取消 `direction_timer` 并置 None。
- **影响**: `src/motor_control/motor_control/motor_control.py` — `switch_state()` 中 DISABLE/HOLD/ENABLE 分支。

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

### 17. get_next_path_file 正则修复

- **问题**: 路径文件切换正则 `^\d{3}_.*\.txt$` 只匹配下划线分隔的文件名（如 `001_E1-E8.txt`），新的短横线命名（如 `007-W23-W20.txt`）不匹配。
- **现象**: 路径 007 完成后切换到 005 而非 008，sorted 排序将 `007-W23-W20.txt` 排到 `005-*.txt` 之前（短横线 ASCII 45 vs 下划线 ASCII 95）。
- **修复**: `file_pattern = re.compile(r'^\d{3}[-_].*\.txt$')` 同时匹配两种分隔符。
- **影响**: `get_next_path_file()` — 路径文件自动切换。

### 18. SBUS 遥控器模块禁用

- **问题**: 现场未使用 SBUS 遥控器，串口初始化失败日志刷屏；且 REMOTE 模式逻辑在 `elif` 分支中，若遥控器未连接会 fallthrough 到 NORMAL 逻辑，行为不明确。
- **修复**: 注释掉 `SBUSRemoteController()` 初始化、`publish_rc_channels()` 调用、REMOTE 模式分支、shutdown 中的 `sbus_remote.stop()`；`elif self.current_control_mode == "NORMAL"` 改为 `if`。
- **影响**: `motor_control.py` — 仅保留 NORMAL 和 AUTO_CLEANING 两种控制模式。

### 19. RTK 串口驱动端口切换

- **问题**: `/dev/WTRTK` 为 udev 符号链接，USB 链路不稳定时可能掉线不恢复。
- **修复**: 改为直接使用物理串口 `/dev/ttyS2`。
- **影响**: `wtrtk_serial_driver.py` — port 默认值。

### 20. 充电模块地址修正

- **问题**: 充电模块 485 地址配置错误。
- **修复**: `THREE_BYTE_ADDR` 从 `[0x10, 0x04, 0x6B]` 改为 `[0x10, 0x04, 0x5D]`。
- **影响**: `charging.py` — 电池/充电数据读取。

### 21. 激光传感器告警日志降频

- **问题**: 激光传感器无回应时每 2s 打印一次 warn 日志，大量刷屏。
- **修复**: 日志间隔从 2.0s 改为 60.0s。
- **影响**: `laser_distance.py`。

### 22. 路径规划器输出参数化

- **问题**: 输出文件名固定使用序号+时间戳，输出目录硬编码 `/home/ubuntu/...`，不便批量生成和指定文件名。
- **修复**: 新增 `output_dir` 和 `output_name` 参数；`output_name` 非空时直接作为文件名前缀，跳过序号生成；输出目录从参数读取而非硬编码；`_plot_multi_area_path` 参数 `timestamp` → `file_prefix`。
- **影响**: `full_path_planner_dense.py` — `plan_multi_area_path()` 和 `_plot_multi_area_path()`。

### 23. 启动文件默认路径更新

- **问题**: 默认路径配置文件指向旧 `path-E1-E7.txt`。
- **修复**: 默认路径改为 `001-E1-E8.txt`；`loading_gps` 填入固定进仓坐标。
- **影响**: `run.launch.py`。

### 24. 路径配置文件重组

- **问题**: 旧配置文件（`003_south18-24.yaml`、`004_north24-18.yaml`、`all_areas.yaml`）与当前区域规划不匹配。
- **修复**: 删除旧配置，新增 001-011 共 11 个区域配置文件（E1-E8, E9-E11, E12-E14, E15-E18, E19-E21, E22-W24, W23-W20, W19-W16, W15-W13, W12-W9, W8-W1）。
- **影响**: `config/` 目录。

## 2026-05-27 打滑反复校准修复 + 真实速度 MQTT 上报

### 25. 打滑/位置偏移导致同航点反复校准死循环（force_bearing_mode）

- **问题**: 轮子打滑导致车体实际位置偏离路径，Stanley 输出大角度横向纠偏（st_corr≈-37.5°），车体航向快速漂移 >15°，触发航向异常重校准。校准后位置更偏，再次触发重校准，在同一航点形成死循环。Fix 15 的方位角模式排除（t>1.0）未生效，因为打滑发生时 t<1.0，仍处于段模式。
- **现场证据**: `debug2.log` 中航点 73 出现 5 次校准循环，每次校准后 ~2s 航向从 0° 漂到 -15°，横向误差从 -0.317m 累积到 -0.412m。
- **修复**: 新增 `waypoint_recalib_count` 和 `force_bearing_mode` 两个 nav_context 字段：
  - 同航点每次重校准累加计数，≥2 次后设置 `force_bearing_mode=True`
  - force_bearing_mode 下：path_direction 固定为 `bearing(current→target)`（方位角直行），跳过所有航向异常检测（`in_bearing_mode=True`），不再触发重校准
  - 航点切换时清零 `waypoint_recalib_count=0` 和 `force_bearing_mode=False`
  - 同时修复了 INITIAL_MOVE 中 `t` 变量在 force_bearing_mode 分支未定义的问题（提前计算 t）
- **影响**: INITIAL_MOVE 和 WAYPOINT_MOVE 两处 Stanley 控制循环 + 航点切换重置逻辑。离线验证：5 次循环 → 1 次校准 + force_bearing_mode 直行。

### 26. RTK 真实速度上报到 MQTT

- **需求**: 将 Stanley 控制器计算的真实速度（m/s）通过 MQTT 上报到云端，字段名为 `velocity`。
- **修复**:
  - `rtk_nav.py`: `stanley_steering_control()` 中存储 `self.real_velocity`；新增 `/rtk/velocity` (Float32) 发布器，`rtk_timer_callback()` 中随速度指令同步发布
  - `motor_control.py`: 新增订阅 `/rtk/velocity` → `rtk_velocity_callback` 存储到 `self.rtk_velocity`；`publish_state()` JSON 中新增 `"velocity"` 字段
- **影响**: rtk_nav.py + motor_control.py。转换系数 `SPEED_CMD_TO_MPS=0.0345`。

### 27. 路径配置微调

- **003-E12-E14.yaml**: 修正 south_14 calib_point_b 坐标；新增 back_14B-13A、back_13B-12A 两个回程区域；移除 south_12 多余的 edge_distance_lat
- **004-E15-E18.yaml**: 移除 south_15 多余的 edge_distance_lat
- **影响**: `config/003-E12-E14.yaml`, `config/004-E15-E18.yaml`

## 2026-05-28 方位角锁定迟滞 + 出仓航向校验 + 进仓修复 + RC_ENABLE 停车 + REMOTE 废弃

### 28. bearing_mode_locked 迟滞防止 t≈1.0 路径方向振荡

- **问题**: t≈1.0 时 `path_direction` 在固定段方向和 `bearing(current→target)` 之间交替跳变。当航段较短时，车速约 0.35m/s，t 在 0.95~1.05 反复穿越，path_dir 频繁切换导致航向误差突变，触发航向异常 5 帧重校准。
- **修复**: 新增 `bearing_mode_locked` nav_context 字段；一旦 `t > 1.0` 进入方位角模式就锁定，直到航点切换才解锁。避免短段末端反复振荡。
- **复位时机**: 航点切换时清零（与 `force_bearing_mode`、`waypoint_recalib_count` 同步）。
- **影响**: `rtk_nav.py` — INITIAL_MOVE 和 WAYPOINT_MOVE 两处 path_direction 计算 + 航向异常检测排除 + nav_context 初始化/重置。

### 29. 出仓完成后航向角校验（90°±25°）

- **问题**: 手动出仓后 IMU 航向约 -45°（应为 ~90°），直接切 AUTO_CLEANING 导致 INITIAL_MOVE 航向异常和大量重校准。
- **修复**: 新增 `UNLOADING_HEADING_TARGET=90.0` 和 `_is_unloading_heading_ready()`；START 完成切换 AUTO_CLEANING 前要求 RTK Fixed + IMU 航向在 65°~115° 范围内。
- **影响**: `motor_control.py` — 两处 START COMPLETE 分支增加航向校验条件。

### 30. 进仓超时日志刷屏 + 故障码上报

- **问题**: 进仓导航超时后每 100ms 打印一次 warn 日志并重复调用 `switch_state('z')`；超时后无故障码上报。
- **修复**:
  - 新增 `ERROR_LOADING_TIMEOUT=32` 错误码位
  - `build_error_code()` 中合并 `loading_timeout_error` 标志
  - 超时分支：设置 `loading_timeout_error=True`，日志级别改为 error，清理 loading 状态防止重入
  - 进仓入口清零 `loading_timeout_error=False`
- **影响**: `motor_control.py` — `handle_loading_step()` 超时分支 + `build_error_code()`。

### 31. complete_state 完成标志 + 进仓后补发 3 条消息

- **问题**: 进仓完成后切换 DISABLE，MQTT 消息中 `complete_state: false` 导致前台无法检测完成。
- **修复**:
  - `finish_loading_process()` 中 `switch_state('z')` 前设置 `self.complete_state = True`
  - DISABLE 状态处理：`complete_state=True` 时补发 3 条消息（0s/5s/10s）再切换为 3000s 低频定时器，防止单条丢失
  - 新增 `_publish_and_countdown()` 回调，发够 3 条后自动切回 3000s 定时器
  - 新增 `_disable_publish_countdown` 计数变量
- **影响**: `motor_control.py` — `finish_loading_process()` + `switch_state()` DISABLE 分支 + 新增 `_publish_and_countdown()`。

### 32. RC_ENABLE 遥控接管不停止

- **问题**: AUTO_CLEANING 导航中通过 MQTT 发送 RC_ENABLE 接管遥控，`rc_control=True` 后 timer 把模式切为 `"REMOTE"`（已废弃的死代码），NORMAL 和 AUTO_CLEANING 分支都不命中，电机保持导航最后速度继续运转。
- **现场证据**: RC_ENABLE 后 `acceleration: {x: -10.06, y: 9.94}` 持续 15+ 秒，imu_yaw 从 -0.8° 漂到 6.1°，机器人未停车。
- **修复**:
  - RC_ENABLE handler 中接管时立即 `set_motors_speed(0,0)` + `set_brush_speed(0)` 停车
  - `timer_callback`: `rc_control=True` → `"NORMAL"`（原 `"REMOTE"`），去掉 `"REMOTE"` 排除项
  - keyboard 'm' 键切到 `"NORMAL"`（原 `"REMOTE"` 已废弃）
  - `bin_process_origin_mode` 移除 `!= "REMOTE"` 条件
  - `status_callback` 移除两处 `REMOTE → NORMAL` 死代码
- **影响**: `motor_control.py` — RC_ENABLE handler + timer_callback 模式映射 + keyboard_callback + bin_process + status_callback。

### 参数更新

| 参数 | 值 | 说明 |
|------|-----|------|
| UNLOADING_HEADING_TARGET | 90.0° | 出仓后预期 IMU 航向 |
| UNLOADING_HEADING_TOLERANCE | 25.0° | 出仓航向允许偏差 |
| ERROR_LOADING_TIMEOUT | 32 (bit 5) | 进仓超时故障码 |
| _disable_publish_countdown | 3 | 进仓完成后补发消息数 |

## 2026-05-29 倾斜跌落检测 + angle_x/angle_y MQTT上报 + nav_context调试发布 + 日志精简

### 33. WTRTK angle_x/angle_y 通过 MQTT 上报

- **需求**: 将 RTK 接收机的姿态角 `angle_x`(横滚) 和 `angle_y`(俯仰) 通过 MQTT 上报到云端，便于后台监控机器人姿态。
- **修复**:
  - `motor_control.py`: 新增 `WTRTK` 消息导入 + `/wtrtk_data` 订阅 + `wtrtk_callback`，提取 `angle_x`/`angle_y` 存储到实例变量
  - `publish_state()` JSON 中新增 `"angle_x"` 和 `"angle_y"` 字段
- **影响**: `motor_control.py` — import + subscription + callback + publish_state。

### 34. 倾斜/跌落故障检测（ERROR_TILT_FAULT=64）

- **问题**: 清扫过程中机器人跌落会导致一侧倾斜严重（横滚>30°），继续导航可能损坏设备或产生危险。
- **修复**: `rtk_nav.py` `heading_callback` 中新增倾斜检测逻辑:
  - `abs(angle_x) > 30°` 或 `abs(angle_y) > 30°` → 倾斜帧计数
  - 连续 30 帧（~3s @10Hz）→ 锁定倾斜故障，停车 + PAUSE 导航 + 发布故障码 64
  - 姿态恢复后连续 5 帧正常 → 清除故障，自动恢复导航
- **故障码透传**: `motor_control.py` 中 `rtk_error_callback` 和 `build_error_code` 掩码均加上 `ERROR_TILT_FAULT`，确保故障码 64 不被过滤
- **参数**: `TILT_ANGLE_THRESHOLD=10.0°`, `TILT_CONFIRM_FRAMES=30`, `TILT_RECOVERY_FRAMES=5`
- **说明**: `angle_x`/`angle_y` 由传感器内部 IMU + 融合算法产出，不依赖 RTK 固定解状态，即使丢星仍有效
- **影响**: `rtk_nav.py` + `motor_control.py`。

### 35. nav_context 调试话题发布

- **需求**: 导航状态上下文（`nav_context`）信息量大且分散，出问题时难以快速定位。
- **修复**: 新增 `/rtk/nav_context` 话题（JSON/String），每 2s 发布一次完整状态快照:
  - 导航状态: `nav_state`, `current_waypoint_idx`, `total_waypoints`, `nav_running`
  - 故障状态: `rtk_error_code`, `tilt_fault`, `tilt_confirm_count`, `tilt_normal_count`
  - 控制状态: `pre_pause_state`, `force_bearing_mode`, `bearing_mode_locked`, `angle_abnormal_count`
  - 超时/异常: `heading_timed_out`, `rtk_data_timed_out`, `is_angle_recalib`
  - 运行信息: `control_mode`, `brush_active`, 时间戳
- **调试用法**: `ros2 topic echo /rtk/nav_context`
- **影响**: `rtk_nav.py` — 新增 `publish_nav_context()` + `/rtk/nav_context` publisher + rtk_timer_callback 中每 2s 调用。

### 36. 日志精简

- **问题**: 边界矫正状态每帧（10Hz）打印 `info` 日志刷屏；`boundary_triggered` 重复打印 `info` + `warn` 两条相同日志。
- **修复**:
  - 边界矫正每帧状态 `info` → `debug`（需 `--ros-args --log-level debug` 才可见）
  - `boundary_triggered` 去掉重复的 `info`，仅保留 `warn`
- **影响**: `rtk_nav.py`。

### 37. 同航点重校准计数在每次校准后被无条件清零（force_bearing_mode 不触发根因 #1）

- **问题**: 航点很近时（~0.56m），航向异常触发第一次重校准（count=1），但 WAYPOINT_CALIB handler 执行完校准后**无条件**把 `waypoint_recalib_count` 清零、`force_bearing_mode` 设为 False。导致第二次航向异常时 count 重新从 1 开始，永远到达不了 2，`force_bearing_mode` 永远不触发。
- **现场证据**: `run20260528.log` 中航点 314（距离 0.56m）反复校准，但 force_bearing_mode 从未激活，机器人绕圈寻找目标。
- **修复**: WAYPOINT_CALIB handler 中 `waypoint_recalib_count`、`force_bearing_mode`、`bearing_mode_locked` 的复位移入 `if not is_recalib:` 分支。`is_recalib=True`（航向异常校准）时计数保持，只在正常航点切换时清零。
- **影响**: `rtk_nav.py` — WAYPOINT_CALIB handler 中 `if not is_recalib:` 分支。

### 38. 重校准后 bearing_mode_locked 未清除压制第二次航向异常检测（force_bearing_mode 不触发根因 #2）

- **问题**: 修复 #37 后计数能保留了，但**更深层问题**仍然存在：第一次航向异常校准完成后，机器人已接近航段终点（t≈1.0 或 >1.0）。回到 WAYPOINT_MOVE 后，`bearing_mode_locked` 立即激活 → `in_bearing_mode=True` → 航向异常检测被完全压制（count 清零）。导致第二次航向异常**永远检测不到**，即使 count 能保留，也等不到 `>=2` 的那一刻。
- **流程**: 校准到 path_direction=0.2° → bearing_mode_locked=True → path_direction 跳变到 bearing(current→target)≈92° → hdg_err≈84° 但被 in_bearing_mode 忽略 → angle_abnormal_count 被清零 → 永远不会触发第二次重校准 → force_bearing_mode 不生效。
- **修复**: WAYPOINT_CALIB handler 的 `is_recalib=True` 分支中，清除 `bearing_mode_locked = False` 和 `stanley_path_start = None`。校准完成后下一帧 WAYPOINT_MOVE 会用当前位姿重新初始化 Stanley 路径段，t 从当前位置重算：
  - 若 t<1.0：段模式，航向异常检测正常运作 → 第二次异常触发 force_bearing_mode 直行
  - 若 t>1.0：进入 bearing mode 直接朝向目标，也比之前绕圈快
- **影响**: `rtk_nav.py` — WAYPOINT_CALIB handler 新增 `else` 分支（`is_recalib=True`），清除 `bearing_mode_locked` + `stanley_path_start`。

### 参数更新

| 参数 | 值 | 说明 |
|------|-----|------|
| ERROR_TILT_FAULT | 64 (bit 6) | 倾斜/跌落故障码 |
| TILT_ANGLE_THRESHOLD | 10.0° | 倾斜判定阈值 |
| TILT_CONFIRM_FRAMES | 30 | 连续倾斜确认帧数（防抖） |
| TILT_RECOVERY_FRAMES | 5 | 连续正常帧数恢复 |

### 39. 倾斜故障与 RTK 错误位互相覆盖

- **问题**: `update_rtk_error_status()` 采用整值覆盖。倾斜触发时写入 `64`，但后续 fixed 帧会直接写 `0`；倾斜恢复时也会把 `RTK_NOT_FIXED(4)` / `RTK_TIMEOUT(8)` 一并清掉，导致 `/rtk/error_status` 对外呈现不稳定。
- **修复**: 在 `rtk_nav.py` 中新增按 bit 增删的 helper，倾斜、RTK 非固定解、RTK/航向超时都改为按位置位/清位，不再互相抹掉。
- **影响**: `rtk_nav.py` — `heading_callback()`、`handle_rtk_data_timeout()`、`update_rtk_error_status()` 周边。

### 40. 倾斜恢复会误恢复其他原因造成的 PAUSE

- **问题**: 倾斜恢复分支此前只判断 `nav_state == PAUSE`，没有区分暂停是由倾斜、RTK 非固定解还是 `/wtrtk_data` 超时触发。结果是“先 RTK 故障暂停，后姿态恢复”也可能被错误拉回导航。
- **修复**: 为 `nav_context` 增加 `pause_reason`，在 `rtk_not_fixed`、`rtk_timeout`、`heading_timeout`、`tilt_fault` 进入 PAUSE 时记录来源；恢复时只处理自己触发的暂停。
- **影响**: `rtk_nav.py` — `nav_context`、`publish_nav_context()`、`heading_callback()`、`handle_rtk_data_timeout()`、`reset_nav_context()`。

## 2026-05-29 force_bearing_mode 修复（count 重置 bug + bearing_mode 抑制 + 追尾螺旋）

### 41. 同航点重校准计数在每次校准后被无条件清零（根因 #1）

- **问题**: 航点很近时（~0.56m），航向异常触发第一次重校准（count=1），但 WAYPOINT_CALIB handler 把 `waypoint_recalib_count` 无条件清零。导致第二次航向异常时 count 重新从 1 开始，`force_bearing_mode` 永远不触发。
- **修复**: `waypoint_recalib_count`、`force_bearing_mode`、`bearing_mode_locked` 的复位移入 `if not is_recalib:` 分支。航向异常校准（`is_recalib=True`）时计数保留，只在正常航点切换时清零。
- **影响**: `rtk_nav.py` — WAYPOINT_CALIB handler。

### 42. 重校准后 bearing_mode_locked 压制第二次航向异常检测（根因 #2）

- **问题**: 修复 #41 后计数保留了，但校准完成回到 WAYPOINT_MOVE 时 t>1.0，`bearing_mode_locked` 立即激活 → `in_bearing_mode=True` → 航向异常检测被压制（count 被清零）。第二次航向异常永远检测不到。
- **修复**: `is_recalib=True` 分支清除 `bearing_mode_locked=False` + `stanley_path_start=None`。校准完成后用当前位姿重新初始化 Stanley 路径段。
- **影响**: `rtk_nav.py` — WAYPOINT_CALIB handler 新增 `else` 分支。

### 43. force_bearing_mode 追尾螺旋 + 中途打滑无法纠正（根因 #3 — 最终修复）

- **问题**: 修复 #41+#42 后 `force_bearing_mode` 成功触发了，但 `path_direction = bearing(current→target)` 每帧重算，形成正反馈追尾螺旋：
  - 机器人转弯追赶 path_dir → path_dir 随位置变化 → 永远追不上
  - 日志证据：`run20260529.log` 航点 8，force_bearing_mode 激活后 path_dir 从 65.5° 螺旋到 173°→-177°→-167°→...，hdg_err 始终 80-116°，机器人绕圈
  - **进一步问题**：若用固定死缓存方位角，中途打滑航向偏了之后永远回不来
- **修复**（自适应循环对准）:
  1. **激活时先旋转校准再直行**（两处 WAYPOINT_MOVE + INITIAL_MOVE）：计算 `bearing(current→target)`，原地旋转对准目标方向
  2. **每帧检查航向偏差**：`bearing_err = abs(bearing(current→target) - imu_yaw)`
     - `bearing_err > 15°`：先原地旋转对准，再继续 — 处理打滑/漂移
     - `bearing_err ≤ 15°`：直接用实时方位角 — 偏差小不会螺旋，且随位置自适应更新，不会"死"
  3. **新增 `force_bearing_target` 字段**：nav_context 初始化、航点切换重置、debug 发布
- **流程**: 激活 → 原地旋转对准 → 直行（实时方位角，偏差小安全） → 偏差大 → 再原地对准 → 直行 → ...循环 → 到达目标
- **影响**: `rtk_nav.py` — 两处 force_bearing_mode 激活 + 两处 path_direction 计算（自适应循环）+ nav_context 初始化/重置/debug 发布。

### 参数更新

| 参数 | 值 | 说明 |
|------|-----|------|
| force_bearing_target | None / bearing° | force_bearing_mode 激活时缓存的固定目标方位角 |

## 2026-05-30 Stanley 去对半砍 + t>1.0 切 force_bearing 防追尾螺旋

### 44. steering_correction 对半砍导致横向偏差无法收敛

- **问题**: 航向误差 4-15° 区间时 `steering_correction *= 0.5`，横向纠偏被压制一半。长直线路段 `lat_err` 从 -0.272m 逐步漂移到 +0.848m 稳态值（纠正力不足，无法克服积累偏差）。
- **修复**: 移除 `steering_correction *= 0.5` 整段逻辑。>15° 已有航向重校准兜底，4-15° 区间不需要衰减。
- **K 参数**: 不需要立即重调——多个硬限制（clamp ±45°、STRAIGHT_MAX_CORRECTION=1.5、MAX_LATERAL_ERROR=1.0、atan 饱和）共同保护。若高速段出现振荡，可降低 K 20%。
- **影响**: `rtk_nav.py` — `stanley_steering_control()`。

### 45. t>1.0 切 bearing_mode_locked 导致追尾螺旋 + force_bearing_mode 死锁

- **问题**: t>1.0 时激活 `bearing_mode_locked`，每帧重算 `bearing(current→target)` 形成追尾螺旋（path_dir 随位置变化，永远追不上）。同时 `bearing_mode_locked` 屏蔽航向异常检测 → `waypoint_recalib_count` 死锁在 1 → `force_bearing_mode`（修复#43）永远无法触发。
- **现场证据**: `debug2.log` 航点 48 距离 1.18m，path_dir 在 ~60s 内从 180° 循环到 -83.8°，机器人绕圈。
- **修复**: t>1.0 改为直接激活 `force_bearing_mode`（原地旋转对准目标后直行），不再走 `bearing_mode_locked` 路径。`bearing_mode_locked` 保留为残留状态兼容路径，不再被新代码设置。
- **影响**: `rtk_nav.py` — INITIAL_MOVE 和 WAYPOINT_MOVE 两处 t>1.0 分支 + `in_bearing_mode` 简化。

## 2026-05-30 state_callback/mode_callback 竞态导致 waiting 标志残留

### 46. 预加载路径后无法启动导航

- **问题**: 清扫 COMPLETED → `load_pending_path_after_task()` 设置 `waiting_for_next_unloading=True`，等待下次 START→UNLOADING→AUTO_CLEANING 后清除。但 `state_callback` 和 `mode_callback` 是两个独立 ROS 订阅，以不确定顺序执行：
  - state_callback 先到 → `self.current_control_mode = AUTO_CLEANING`
  - mode_callback 后到 → `previous_mode = self.current_control_mode` 读到已被改过的 AUTO_CLEANING → 过渡检测为 False → 跳过 `waiting_for_next_unloading` 清除
  - `rtk_timer_callback` 被 `waiting_for_next_unloading == True` 拦住，不创建生成器，机器人不动
- **现场证据**: `debug3txt` 中 `启动/恢复RTK多点导航` 日志缺失，nav_status 保持 IDLE 超过 35 秒。
- **修复**:
  - `state_callback`: 收到 AUTO_CLEANING 时同步清除 `waiting_for_next_unloading` + 重置生成器
  - `mode_callback`: 过渡检测改用独立 `_last_mode_msg` 变量，不再读取可能已被 state_callback 修改的 `self.current_control_mode`
- **影响**: `rtk_nav.py` — `state_callback()` + `mode_callback()`。两层兜底，各自独立修好竞态。

## 2026-05-31 滚刷索引覆盖 + 队列支持多段区域

### 47. `elif → if` 导致区域名不更新

- **问题**: 路径注释 `#E19_long_block_start` 中 `'start' in comment` 为 True，走 `if 'start'` 分支后 `elif comment:` 被跳过 → `current_area` 始终不更新。同样 `#*_stop` 也会跳过区域更新。
- **现场证据**: `run20260530.log` 中 `route_id` 已切到 `005-E19-E21`，但 `areas` 字段始终显示上一条路径的 `bridge_18A-19A_mid`。
- **修复**: `elif comment:` → `if comment:`，start/stop 检测和区域名更新独立执行。commit: 5062800
- **影响**: `rtk_nav.py` — 主路径加载的注释解析（line 360）。

### 48. 多个 `#*start` 注释覆盖导致滚刷延迟开启

- **问题**: 同一路径文件中 `#E19_long_block_start`（航点41）和 `#E19_long_block_start_mid`（航点83）都包含 `start` 子串。`brush_start_idx` 为单一标量，后者覆盖前者，滚刷在 83 才开而非 41。
- **现场证据**: `run20260530.log` 中 005-E19-E21 路径加载时输出两次 start 检测（航点41 和 83），`check_and_control_brush` 在 83 才触发开启。
- **修复**:
  - `brush_start_idx/brush_stop_idx` 从单一标量改为 list 队列
  - 加载时 append（支持多段独立滚刷区域），`check_and_control_brush` 逐对 pop 消费
  - 过期清理：含 start/stop 子串的非标记注释（如 `_mid`）产生的冗余索引，随 `current_waypoint_idx` 递增触发 `idx > indices[0]` → pop + warn
  - commit: 1c4ef6b
- **影响**: `rtk_nav.py` — 属性初始化、两处路径加载、两处重置点、`check_and_control_brush()`。

## 2026-05-31 进点末段低速卡目标点

### 49. speed_scale 下限过低，草地里推不动

- **问题**: 接近航点（<1.5m）时 `speed_scale = max(0.2, distance/1.5*0.7)`，最低电机指令 = 10.0×0.2 = 2.0 → 实际速度 0.069 m/s，草地/坡面无法克服摩擦力，机器人卡在距目标 0.2m 处不动。
- **现场证据**: `run20260530.log` 航点99 最后 0.2m 耗时 486+ 秒仍未到达，位置在 ±0.000001° 内波动。需 RC_ENABLE 手动推过。
- **关键误区**: `STANLEY_MIN_SPEED=0.15` 只在 Stanley 公式 `max(v, 0.15)` 中防除零，不控制真实行驶速度。实际最低速 = `LINEAR_SPEED_BASE × speed_scale_min × SPEED_CMD_TO_MPS`。
- **修复**: `speed_scale` 下限 `0.2 → 0.3`，最低电机指令 = 3.0 → 实际速度 0.10 m/s。commit: e024fa2
- **影响**: `rtk_nav.py` — 两处 `speed_scale = max(0.3, distance / LOW_DISTANCE * 0.7)`（INITIAL_MOVE + WAYPOINT_MOVE）。

## 2026-05-31 PAUSE 未停止滚刷 + 恢复路径未检测故障

### 50. publish_stop_speed() 未清除 brush_active 导致 PAUSE 期间滚刷继续

- **问题**: PAUSE 触发后 `publish_stop_speed()` 发送 `(0,0,0)`，但 `self.brush_active` 仍为 True。随后 `rtk_timer_callback` 的 timer 读到 `brush_active=True`，继续发布 `z=RTK_BRUSH_SPEED`，滚刷未停。
- **修复**: `publish_stop_speed()` 内增加 `self.brush_active = False`，停电机时同步清除滚刷标志。
- **注意**: 此修复在 #62 (2026-06-03) 中被**撤销**，改为不在 `publish_stop_speed()` 中清零 `brush_active`，原因见 #62。
- **影响**: `rtk_nav.py` — `publish_stop_speed()`。

### 51. 航向超时恢复缺少状态复原 + 未检查跌落故障

- **问题**: `handle_rtk_data_timeout()` 中航向恢复分支只设 `nav_running=True`，没有恢复 `nav_state`、`pause_reason`、`brush_active`。且未检查 `tilt_fault`，可能在倾斜状态下恢复导航。
- **修复**: 增加完整的 `nav_state`/`pause_reason`/`brush_active` 恢复逻辑；增加 `tilt_fault` 守卫，故障活跃时返回 False 不恢复。
- **影响**: `rtk_nav.py` — `handle_rtk_data_timeout()` 航向恢复分支。

### 52. 生成器 resume 路径无条件恢复 PAUSE

- **问题**: `multi_waypoint_nav_generator()` resume 分支看到 `nav_state==PAUSE` 即无条件恢复到 `pre_pause_state`，不检查故障是否仍活跃。
- **修复**: 增加故障条件检查：`tilt_fault` → 检查 `nav_context["tilt_fault"]`；`heading_timeout` → 检查 `self.heading_timed_out`；`rtk_not_fixed/rtk_timeout` → 检查 `self.rtk_data_timed_out`。任一活跃则保持 PAUSE。
- **影响**: `rtk_nav.py` — resume 分支。

### 53. 生成器主循环缺少 PAUSE 状态处理

- **问题**: `multi_waypoint_nav_generator()` 主循环没有 PAUSE 分支。`nav_state==PAUSE` 时降落到最后的 `yield (0,0)` + `continue`，期间 `nav_running` 可能已为 False 但 generator 仍在空转。
- **修复**: 新增 PAUSE handler，显式 `yield (0.0, 0.0)` 并重新读取 `nav_state`，等待外部恢复。
- **影响**: `rtk_nav.py` — `multi_waypoint_nav_generator()` 主循环。

### 54. state_callback 在故障活跃时仍恢复 AUTO_CLEANING

- **问题**: 手动发送 AUTO_CLEANING 时，`state_callback` 无条件恢复滚刷状态和导航，即使 `tilt_fault=True`。之后 tilt_fault 已 latch，tilt 检测的 `not self.nav_context["tilt_fault"]` 条件阻止重新触发 PAUSE → 机器人倾斜状态下继续行驶。
- **修复**: AUTO_CLEANING 入口增加 `tilt_fault` 检查，活跃时保持 PAUSE 并记录 warn 日志；恢复逻辑移入 else 分支。
- **影响**: `rtk_nav.py` — `state_callback()` AUTO_CLEANING 分支。

### 55. MQTT velocity 从 RTK 计算值切换为电机实际反馈

- **问题**: `velocity` 字段使用 `rtk_nav` 通过 `SPEED_CMD_TO_MPS` 计算的估算速度，与真实车速存在偏差。且 `/rtk/velocity` 订阅增加通信开销。
- **修复**: 移除 `/rtk/velocity` 订阅和 `rtk_velocity_callback`；`velocity` 改用左右电机 `actual_velocity` 平均值 × `MOTOR_RAD_S_TO_MPS` 计算。
- **转换系数**: `MOTOR_RAD_S_TO_MPS = SPEED_CMD_TO_MPS × (10.0 / 22.0)`，22 rad/s → 0.345 m/s → 10.0 电机指令。
- **副作用清理**: `ERROR_RESERVED_3=128` 和 `ERROR_RESERVED_4=256` 合并为 `ERROR_RESERVED_1=256`（bit 8 曾被 `ERROR_UNLOADING_HEADING_TIMEOUT=128` 使用后未清理）。
- **影响**: `motor_control.py` — import/常量/subscription/callback/publish_state。

## 2026-05-31 航向校准超时修复 + 初始航向校验 + 卡滞提速

### 56. 航向校准超时修复 + 卡滞检测 + 初始航向校验(90°±25°) + 校准失败PAUSE

- **问题1 — 校准超时从未触发**: `calibrate_heading_at_waypoint()` 中 `calib_start_time = time.monotonic()` 写在 `while True` 循环**内部**，每帧重置，`ERROR_CALIB_TIMEOUT`(256s) 永远不会到达。
- **问题2 — 草地卡滞**: 航向校准时在草地/坡面上 `min_positive_speed=0.1` 无法推动电机，校准停滞。
- **问题3 — 校准失败后继续导航**: 超时后即使 `heading_err > 5°`，函数仍返回 True，导航继续但航向错误。
- **问题4 — 起步航向无校验**: 出仓后直接切 AUTO_CLEANING，IMU 航向可能偏离 90° 很远，导致 INITIAL_MOVE 立即触发航向异常重校准。
- **修复**:
  - `calib_start_time` 移到 while 循环外部，超时保护真正生效
  - 卡滞检测：左右轮速度指令变化 < 0.3°/帧 持续 5s → 速度从 TURN_SPEED_MID(1.0) 升级到 TURN_SPEED_HIGH(1.5)
  - 超时 + `heading_err > 5°` → 返回 False → 调用方进入 PAUSE
  - `min_positive_speed` 0.1 → 0.5（确保草地上能推动）
  - INITIAL_MOVE 入口新增航向校验：IMU 须在 90°±25° 范围内，不满足则原地等待直到就绪（以 stop_speed yield 形式，不阻塞其他处理）
  - HOLD 和模式切换时清理 `heading_abnormal_start_time`/`heading_timed_out`
- **参数**: `INITIAL_HEADING_TARGET=90.0`, `INITIAL_HEADING_TOLERANCE=25.0`, `ERROR_CALIB_TIMEOUT=256`, `TURN_SPEED_MID=1.0`(原0.7), `TURN_SPEED_SLOW=0.4`(原0.2)
- **影响**: `rtk_nav.py` — `calibrate_heading_at_waypoint()` + `move_to_first_waypoint()` + `mode_callback` + keyboard HOLD handler。

### 58. get_next_path_file 最后文件循环回到第一个

- **问题**: 执行到最后一个路径文件后 `get_next_path_file()` 返回 `None`，任务结束。现场需要循环往复清扫所有区域，而不是跑完一轮就停止。
- **修复**: 最后一个文件（`current_idx >= total_files - 1`）时不再返回 `None`，改为返回 `all_files[0]`（第一个文件），进度从 1/N 重新开始。
- **日志**: `已执行到最后一个路径文件（011-W8-W1.txt）, 循环回到第一个: 001-E1-E8.txt`
- **影响**: `rtk_nav.py` — `get_next_path_file()` 最后文件分支。

## 2026-05-31 跌落后INS稳定等待120s

### 57. 倾斜故障后重新AUTO_CLEANING需等待120s让INS数据稳定

- **问题**: 机器人跌落/倾斜后 INS 传感器融合滤波发散（gyro 饱和 + 加速度计重力参考被污染），恢复后静止状态 Roll≈88°、Pitch≈34°，`ins_flag` 恒为 0 无参考意义。直接重新下发 AUTO_CLEANING 会以错误的 IMU 航向执行导航，导致反复校准或走偏。
- **修复**: 新增 `TILT_STABILIZE_TIMEOUT=120.0` 和 `last_tilt_time` 时间戳：
  - 倾斜故障首次确认时记录 `last_tilt_time = time.monotonic()`
  - **首次启动路径**（IDLE→INITIAL_MOVE）：`last_tilt_time > 0` 且距倾斜不到 120s → while 循环等待（publish_stop_speed + yield），到期后继续航向校验流程
  - **恢复路径**（PAUSE→WAYPOINT_MOVE/ CALIB）：`pause_reason == "tilt_fault"` 且距倾斜不到 120s → 同上等待循环，到期后恢复到 `pre_pause_state`
  - 两条路径覆盖全新启动和清扫中途跌落后恢复
- **参数**: `TILT_STABILIZE_TIMEOUT=120.0`
- **影响**: `rtk_nav.py` — `heading_callback`(记录 last_tilt_time) + `multi_waypoint_nav_generator`(两处等待循环)。

### 参数更新

| 参数 | 值 | 说明 |
|------|-----|------|
| INITIAL_HEADING_TARGET | 90.0° | 初始航向校验目标 |
| INITIAL_HEADING_TOLERANCE | 25.0° | 初始航向允许偏差 |
| TILT_STABILIZE_TIMEOUT | 120.0s | 倾斜后 INS 稳定等待时间 |
| ERROR_CALIB_TIMEOUT | 256s | 航向校准超时 |
| TURN_SPEED_MID | 1.0 (原0.7) | 中速旋转电机指令 |
| TURN_SPEED_SLOW | 0.4 (原0.2) | 低速旋转电机指令 |

## 2026-06-01 航向稳定性检查 + INS 漂移防护

### 59. 全新启动时 IMU 航向严重漂移

- **问题**: 冷启动时 `ins_flag=0`（INS 未完成初始化，gyro bias 未收敛），`ins_heading` 以 3-10°/s 持续漂移。机器人静止状态下 31 秒漂移 300°+。若此时触发 UNLOADING/AUTO_CLEANING，IMU 航向完全不可信，导致 INITIAL_MOVE 立即触发航向异常。
- **现场证据**: `全新启动，角度异常.log` 中 IMU 从 90° 线形漂移到约 -210°，`ins_flag` 始终为 0。
- **修复**: 新增航向稳定性检查——5 秒滑动窗口内 ins_heading 波动 ≤ 3° 且稳定在 89°±15° 范围内，才允许进入清扫：
  - `rtk_nav.py` `heading_callback()`: deque 滑动窗口追踪 ins_heading，每帧发布 `/heading_stable`（Bool）
  - `motor_control.py`: 新增 `/heading_stable` 订阅，`_is_unloading_heading_ready()` 增加 `heading_stable` 条件
  - INITIAL_MOVE 初始航向校验: 要求 `heading_ok AND stable_ok` 同时满足
- **关键设计**:
  - 滑动窗口 5s，用 ≥4.5s 容差 + 最少 20 样本防回调抖动
  - 稳定性追踪**仅在 IDLE/PAUSE/COMPLETED 时更新**，导航中（INITIAL_MOVE/WAYPOINT_MOVE）冻结最后值，避免移动中误报不稳定
  - 初始航向校验容差从 25° 收紧到 15°
- **影响**: `rtk_nav.py` — heading_callback + move_to_first_waypoint + 常量定义；`motor_control.py` — heading_stable 订阅 + _is_unloading_heading_ready。

### 60. 倾斜故障误触发 + 短促颠簸导致 120s 无效等待

- **问题**: angle_y 正常运行时基线在 -7°~-10°（场地自然坡度），10° 阈值几乎没有余量。稍有颠簸 angle_y 突破 10° → 30 帧确认 → 停车 → 恢复后强制等 120s。多次短促颠簸（< 5s）反复浪费数分钟。
- **现场证据**: `倾斜故障误触发.log` 中 3 次触发：angle_y=-10.78°（刚超）、-15.37°（短促）、-10.01°。每次恢复后等待 60-120s。
- **修复**:
  - 倾角阈值 `TILT_ANGLE_THRESHOLD` 从 10° 提升到 **15°**，给 angle_y 正常基线留 5° 余量
  - 新增 `TILT_SHORT_DURATION=10.0s`：倾斜恢复时记录 `last_tilt_duration = 恢复时间 - 确认时间`
  - 恢复路径：若 `last_tilt_duration < 10s`（短促颠簸），**跳过 120s 稳定等待**直接恢复；≥ 10s 才走完整等待
- **影响**: `rtk_nav.py` — 常量 + tilt 恢复逻辑 + 两处跌落稳定等待路径（resume 和首次 AUTO_CLEANING）。

### 参数更新

| 参数 | 原值 | 新值 | 说明 |
|------|------|------|------|
| TILT_ANGLE_THRESHOLD | 10.0° | 15.0° | 倾斜判定阈值 |
| TILT_SHORT_DURATION | — | 10.0s | 短促倾斜跳过稳定等待 |
| INITIAL_HEADING_TOLERANCE | 25.0° | 15.0° | 初始航向校验容差 |
| HEADING_STABILITY_WINDOW | — | 5.0s | 航向稳定检查窗口 |
| HEADING_STABILITY_RANGE | — | 3.0° | 窗口内最大允许波动 |
| HEADING_STABILITY_TARGET | — | 89.0° | 稳定航向目标 |
| HEADING_STABILITY_TOLERANCE | — | 15.0° | 稳定航向容差 |
| UNLOADING_HEADING_TOLERANCE | 25.0° | 15.0° | 出仓航向校验容差 |
| MAX_GPS_WAIT_TIME | 30.0s | 300.0s | GPS 等待超时（给航向稳定留时间） |

## 2026-06-02 calib_stuck 自动重试 + heading_timeout 校准保护

### 61. calib_stuck 恢复路径 heading_abnormal_start_time 未清零导致立即 heading_timeout

- **问题**: calib_stuck 触发 PAUSE 后，`heading_abnormal_start_time` 仍保留着校准期间的累积值（已 >15s）。resume 路径恢复到 WAYPOINT_CALIB 时未清零，导致恢复后的第一个 timer callback 中 `handle_rtk_data_timeout()` 立即触发 heading_timeout PAUSE，杀死刚启动的校准。
- **现场证据**: `卡滞监测，无法纠正.lpg` 中 calib_stuck 后 81s 无恢复——自动恢复失败，但手动 HOLD+AUTO_CLEANING 在 13:00:50 成功（手动路径中 `mode_callback` line 2927 清零了 `heading_abnormal_start_time`，校准能跑满 40s）。
- **根因对比**:
  - 手动 HOLD+AUTO_CLEANING: `mode_callback` → 清零航向状态 → 校准 40s 完整执行 → 成功
  - 自动恢复: 旧 `heading_abnormal_start_time` 已累积 >15s → resume → heading_timeout 立即触发 → 校准被秒杀
- **修复 (三处联动)**:
  1. **resume 路径**: 恢复到 WAYPOINT_CALIB 时清零 `heading_abnormal_start_time`/`heading_timed_out`/`angle_abnormal_count`
  2. **heading_timeout 排除校准**: `handle_rtk_data_timeout()` 中 `nav_state not in [IDLE, PAUSE, COMPLETED]` → 追加 `WAYPOINT_CALIB`——校准有自己的 40s 超时，heading_timeout 打断它是多余的。同时将 `nav_running=False` 等副作用代码移入 if 块内，确保校准中不被意外停止
  3. **calib_stuck 自动重试**: 校准卡滞不再直接 PAUSE，改为最多重试 3 次（每次重建校准生成器、清零航向状态、40s 超时）。3 次后仍未成功才进入 PAUSE 等待人工介入
- **影响**: `rtk_nav.py` — `handle_rtk_data_timeout()` + resume 路径（两处 WAYPOINT_CALIB 恢复）+ calib_stuck 处理段 + `reset_nav_context()` + HOLD/mode_callback 清零。

### 参数更新

| 参数 | 值 | 说明 |
|------|-----|------|
| CALIB_STUCK_MAX_RETRIES | 3 | 校准卡滞最大自动重试次数，超此次数后永久暂停 |
| HEADING_CALIBRATION_TIMEOUT (注) | 40.0s | 每次校准重试独立的超时，3 次共 120s 自动恢复窗口 |

## 2026-06-03 publish_stop_speed 滚刷标志清零导致 RTK 震荡后滚刷永久关闭

### 62. publish_stop_speed() 无条件清零 brush_active → 污染 nav_context 保存链

- **问题**: `publish_stop_speed()` 无条件设置 `self.brush_active = False`。RTK 快速 Float↔固定解震荡时，每次 PAUSE 调用 `publish_stop_speed()` 清零 `self.brush_active`，虽然 `wtrtk_data_callback` 中先保存后清零的顺序保证了 `nav_context["brush_active"]` 第一次保存正确，但**10Hz 定时器在 RTK 回调间隙创建生成器**时，`check_and_control_brush()`（line 2186）若当前航点索引匹配已消费的 stop_index → `publish_brush_speed(0.0)` → `self.brush_active = False` → 下一次 RTK Float 保存进 `nav_context` 的就是 False → 后续所有恢复都拿到 False → 滚刷永久关闭。
- **现场证据**: `run20260602.log` 中 1780452713.336 恢复后 "设置滚刷状态: 开启" ✓，但 1780452714.002（仅 51ms 后）恢复显示 "设置滚刷状态: 关闭" ✗。此后所有恢复都是关闭。HOLD 回调又保存了 False 到 `nav_context`，二次固化。
- **修复**: 移除 `publish_stop_speed()` 中的 `self.brush_active = False`，由各调用方自行决定是否清零。PAUSE 相关路径依赖 nav_context 的 save/restore 闭环：Float 时保存真值 → 恢复时取回真值 → `publish_brush_speed(ON)` 显式恢复。
- **影响**: `rtk_nav.py` — `publish_stop_speed()`。所有显式清零滚刷的位置（`finish_navigation_task`、`reset_nav_context`、路径切换、异常退出）已独立设置 `brush_active = False`，不受此修改影响。

## 2026-06-03 calib_stuck 故障码未上报 — motor_control 屏蔽了 ERROR_CALIB_TIMEOUT

### 63. motor_control 的 rtk_error_callback 和 build_error_code 过滤掉了 bit 256

- **问题**: rtk_nav 中 `ERROR_CALIB_TIMEOUT = 256`，calib_stuck 重试超限后校准生成器返回前设置了该位。但 `motor_control.py` 的 `rtk_error_callback`（line 673）接收 `/rtk/error_status` 时掩码只含 `ERROR_RTK_NOT_FIXED(4) | ERROR_RTK_TIMEOUT(8) | ERROR_TILT_FAULT(64)`，bit 256 被直接 & 掉。`build_error_code()`（line 451）同样不包含。结果 `/robot_state` JSON 的 `error` 字段始终只有 2（`ERROR_LASER_TIMEOUT`），丢了 256。
- **现场证据**: `卡死重试失败,未上报故障码.log` 中 `nav_status: "PAUSE"`、`pause_reason: "calib_stuck"`，但 `error: 2`（仅激光超时），无 calib_stuck 故障码。
- **修复**:
  1. `motor_control.py`: `ERROR_RESERVED_1 = 256` 重命名为 `ERROR_CALIB_TIMEOUT = 256`；`rtk_error_callback` 和 `build_error_code` 掩码均加入 `ERROR_CALIB_TIMEOUT`
  2. `rtk_nav.py`: calib_stuck 进入 PAUSE 时显式调用 `set_rtk_error_bits(ERROR_CALIB_TIMEOUT)`，不再依赖生成器返回前的被动置位
- **影响**: `motor_control.py` — 常量定义 + rtk_error_callback + build_error_code；`rtk_nav.py` — calib_stuck PAUSE 块。修复后 `error = 2 | 256 = 258`，云端可见校准卡滞故障码。

## 2026-06-04 进仓打滑漂移修复 — 旋转/平移分离

### 64. 分段转向边走边转导致 bearing_error 反馈振荡，无法直走到目标点

- **问题**: be90044 引入的分段转向子状态机（ADJUST/PAUSE/POS_CORRECT）将转向和移动叠加在一起输出。转动改变航向→航向改变 bearing_error→bearing_error 改变转向量，形成双输入耦合振荡。靠近目标时微小位移导致 bearing 大幅变化，触发剧烈修正。实际运行左右摆动，无法直走到进仓 GPS 点。
- **根因分析**:
  1. **转向+移动耦合**: `turn_amount` 和 `move_speed` 同时输出，每个动作都在破坏另一个的参考系
  2. **后退转向方向反了**: `bearing_error > 0`（目标右后）时 `turn_sign=1`（左传），应该右转
  3. **转向优先比例不合理**: `abs_be > 30` 时 `turn_ratio=0.8, move_ratio=0.05`，机器人几乎原地打转
  4. **没有分离"对准"和"移动"**: 应先原地旋转对齐 bearing，再直线前进/后退
- **修复** (faafd3a):
  - LOADING_TURN 恢复为昨天版本（纯原地旋转，`get_speed_correction` + `get_adaptive_turn_speed`）
  - 转向完成（yaw 稳定 3 次）后二次 GPS 判距，漂移 >0.3m 进入新阶段 `LOADING_POS_RECOVER`
  - `LOADING_POS_RECOVER`:
    - `ALIGN_BEARING`: 原地纯旋转对齐目标方位（**不移动**）— 目标在前对齐 bearing，目标在后对齐 bearing+180° 准备后退
    - `MOVE_TO_TARGET`: 纯直线移动（**不转向**）— 到达后重新 LOADING_TURN
  - 旋转和平移彻底分离，消除反馈振荡
- **影响**: `motor_control.py` — LOADING_TURN + 新增 LOADING_POS_RECOVER 阶段 + 删除分段转向子状态机（_turn_sub_phase/_finish_turn_to_forward 等）。

### 65. LOADING_NAV_TO_GPS 同样问题：目标在后方时强制前进导致大角度调头打滑

- **问题**: LOADING_NAV_TO_GPS 的 ALIGN/DRIVE 子阶段只支持前进。当进仓 GPS 点在机器人后方时，ALIGN 阶段需要旋转 >90° 掉头，然后 DRIVE 阶段前进行驶。大幅原地旋转同样导致打滑漂移。
- **修复** (66dcb7d):
  - ALIGN 子阶段: `abs(bearing_error) > 90` 时对齐 `bearing + 180°`（反向方位），准备后退
  - DRIVE 子阶段: 后退时 `left=+move+turn, right=-move+turn`，差速修正逻辑与前进一致
  - 增加 `_nav_use_backward` 标志，贯穿 ALIGN → DRIVE
- **电机方向验证**: `turn_speed` 同时对左右轮叠加同号分量（正=左传/负=右转），前进后退相同，无需反转
- **影响**: `motor_control.py` — ALIGN 子阶段（前进/后退判断 + 对齐目标切换）+ DRIVE 子阶段（后退电机方向）。

### 参数更新

| 参数 | 值 | 说明 |
|------|-----|------|
| _pos_recover_gps_drift_limit | 0.3m | 转向后 GPS 漂移阈值，超过触发位置恢复 |
| _pos_recover_timeout | 60.0s | 位置恢复整体超时 |
| loading_turn_time | 60.0s | 角度调整超时（30→60，给位置恢复留余量） |

### 66. 航向校准卡滞重试时增加后退脱困步骤

- **问题**: 航向校准卡滞重试时只重建校准生成器直接重试，不改变物理状态。车轮卡在草皮/坑洞/石子堆里，同样的原地旋转在上一次40s超时中已经证明无法推动，重试只是重复消耗120s等待时间后永久暂停。
- **现场证据**: `E24卡滞.txt` 中3次重试：第1次40s校准超时（误差9.7°），第2次40s校准超时（误差9.5°），第3次才成功（误差0.44°）。两次无效等待共80s。
- **修复** (4f9a294):
  - 第2次及以上重试时，先以差速后退1秒扰动车轮脱离卡点，再重新初始化校准生成器
  - 后退过程中若触发边界传感器，自动切换为边界矫正速度
  - 第1次重试保持原逻辑（不后退），避免偶尔的短时卡滞被不必要的后退扩大偏离
  - 后退速度0.4/-0.4（差速扰动），持续1s后停顿再校准
- **影响**: `rtk_nav.py` — calib_stuck 重试分支（`multi_waypoint_nav_generator` 中 WAYPOINT_CALIB handler）。

### 67. 航向校准卡滞重试使用错误的目标航向（~90° 偏移）

- **问题**: 航向校准卡滞超时后，重试代码调用 `get_path_heading(target_waypoint)` 作为校准目标航向。该函数返回 `waypoint[2] + imu_calibration_offset`（路径文件存储航向），但初始校准时使用的是 `path_direction`（Stanley 计算的路径段方位角）。当路径文件航向与实际路径方向差 ~90° 时，重试校准到错误角度。
- **现场证据**: `0604卡滞转向调整.log` 中航点7：path_dir=179.4°（正确），get_path_heading=89.85°（错误 ~90° 偏移）；航点10：path_dir=90.1°（正确），get_path_heading=179.65°（错误 ~90° 偏移）。重试1/2/3均使用错误目标179.65°，校准完成后 Stanley 检测到 hdg_err≈-89°，航向异常保护重新校准到 ~88.5°。
- **根因**: `get_path_heading()` 读取的是人工打点时记录的航向（waypoint[2]），与 Stanley 计算的路径段方向（起点→终点方位角）是两个独立概念。路径文件中人工记录的航向可能因打点站立角度、imu_calibration_offset 变化等因素与实际路径方向偏差 ~90°。初始校准用 `path_direction`（正确），但卡滞重试的4个读取点直接或间接调用 `get_path_heading()`（错误）。
- **修复** (983f7eb):
  - 新增 `calib_target_heading` 字段到 `nav_context`，所有校准入口保存首次使用的目标航向
  - `start_heading_recalibration()`: 保存 `calib_target_heading = path_direction`
  - 同航点 ≥2 次校准的 fixed_bearing 内联校准: 保存 `fixed_bearing` 为目标
  - 正常航点到达校准: 保存当前 `path_direction` 为目标
  - 卡滞重试的 4 个读取点: 改用 `saved if saved is not None else get_path_heading()`，优先复用保存值
  - 使用 `is not None` 显式检查避免 Python `0.0 or X` 真值陷阱（航向 0.0° 为合法值但 falsy）
- **影响**: `rtk_nav.py` — `nav_context` 初始化 (2处) + `start_heading_recalibration()` + 同航点 fixed_bearing 内联校准 + 正常航点到达校准 + 卡滞重试的 4 个读取点 (L2181, L2196, L2436, L2472)。

### 68. NAV_TO_GPS DRIVE 阶段无航向纠偏导致偏移目标 (a96dec8)

- **问题**: commit 5fa0344 去掉了 DRIVE 阶段的差速转向，改为纯直线行驶（"ALIGN对准后不纠偏，纯直走"）。左右电机不平衡导致航向缓慢漂移（~1-2°/s），机器人逐渐偏航，距离目标从 0.1m 越走越远到 1.9m，再也无法接近。
- **现场证据**: `6月5日1820移动到进仓点近距离没有重对正.txt`:
  - ALIGN 对准完成：航向 164.2°，方位 344.0°，距离 0.6m
  - ~8s 后：航向漂到 170.2°，方位 340.5°，距离 0.4m（已偏）
  - ~30s 后：航向漂到 ~-174°，方位 189°，距离 1.9m
  - 最终：距离卡在 1.9m，速度 5.0（全速），但无法接近目标
  - 根因：航向漂移 25°+，bearing 也跟着变，机器人沿错误方向后退
- **旧版差速转向为何被去掉 (5fa0344)**:
  1. 使用 `get_adaptive_turn_speed()`（原地旋转用）：阶梯跳变 <10°→1.5, ≥10°→4.0，10° 硬边界导致纠偏量突变
  2. 无死区：IMU 噪声（±0.5°）每帧触发微调，左右轮速度来回抖动
  3. 反向耦合：纠偏量过大 → 航向修正过度 → bearing 变化 → 纠偏量再变 → "走不直，来回小幅摆头"
- **新版改进** (a96dec8):
  1. **P-only 替代阶梯函数**：`heading_err × NAV_DRIVE_KP(0.08)`，10° 误差 → 0.8 纠偏（旧版 1.5~4.0），连续平滑无跳变
  2. **1.5° 死区** (`NAV_DRIVE_DEADZONE`)：忽略 IMU 噪声和微小航向波动
  3. **max clamp 2.0** (`NAV_DRIVE_MAX_CORR`)：纠偏量不超过 base_speed(5.0) 的 40%，防止反向耦合
  4. **差速方式叠加**：`correction` 加在左右轮同号位置（后退: left=+speed+corr, right=-speed+corr），确保 `corr>0` → 左更快后退+右更慢后退 → 右转(CW) 纠正正误差
  5. **日志增强**：新增 `航向差` 和 `纠偏` 字段到 2s 日志中，方便现场调参
- **参数**:

| 参数 | 值 | 说明 |
|------|-----|------|
| NAV_DRIVE_KP | 0.08 | 航向纠偏比例增益 |
| NAV_DRIVE_MAX_CORR | 2.0 | 最大纠偏量（motor_control 速度单位） |
| NAV_DRIVE_DEADZONE | 1.5° | 纠偏死区，小于此误差不纠偏 |

- **调参指南**:
  - 航向仍在漂（纠偏不够）→ 增大 KP（0.10→0.12）
  - 来回小幅摆头（过度纠偏）→ 增大死区（2.0°）或减小 KP
  - 单次纠偏太猛 → 减小 MAX_CORR（1.5）
- **影响**: `motor_control.py` — 常量定义(3个) + DRIVE 子阶段纠偏逻辑 + 日志输出。

## 2026-06-22 路径直角转弯优化 + 边缘距离配置补全

### 69. 路径直角转弯改为两次转弯 + config 边缘距离参数补全 (f235f75)

- **问题**: 部分区域（E7→bridge_7A-8B）直角转弯时机器人在草地/坡面上大幅原地旋转，滑移导致位置漂移和航向偏差累积。同时多个 config 区域的 `edge_distance_lat` 和 `edge_distance_lon` 缺失或参数不合理，导致路径覆盖范围不完整。
- **修复**:

  **cleaning_path (航点文件，11个):**
  - E7_mid→bridge_7A-8B: 直角转弯拆分为两段，新增中间航点 (392→393→394) 引导机器人以 heading 89.93° → 153.44° → 179.63° 平缓过渡，避免一次大角度旋转
  - E8 区域航点序列调整：heading 从 269.81→179.81→89.81→179.81 循环过渡，每段 heading 变化 < 90°

  **config (配置文件，11个):**
  - `reverse_end_point`: 001-E1-E8 的 E7_start 新增 `reverse_end_point: True`，终点反向处理
  - `edge_distance_lat` 批量补全:
    - E7_start/W8_start 类区域: `[0.1, 0.5]` — 纵向边界扩展
    - `bridge_8B-9A`: `[-3.2, 2.3]` — 纵向覆盖桥梁路段
    - `bridge_8B-9B` (002): `[-3.2, 0.1]` — 同上适配不同配置
    - `bridge-E9-W9`: `0.0 → [-42.0, 0.0]` — 大幅扩展纵向范围覆盖整条东西桥梁
    - `bridge_W9-W9B` (011): `[0.1, -3.8]` — W区桥梁覆盖
    - `back_7A-7bA`: `[0.6, 0.1]` — 回程区域纵向边界
    - `back_9B-8B`: `[0.1, -3.6]` — 回程覆盖
    - E8/E7_end: `[0.1, 0.2]` — 末端区域修正
  - `edge_distance_lon` 修复:
    - `back_E9-W9` / `back_W9-E9`: `0.0 → [0.0, -1.0]` — 修正横向扩展方向
  - `start_corner` 补全: `back_E9-W9` 新增 `start_corner: "top_right"` (008-011)
  - `edge_distance_lat` 修正: 001 E7_start 从 `[-0.3, 0.1]` 改为 `[0.1, 0.1]`（双向对称）
  - 废弃区域注释: 001 中 `back_6B-5A` / `back_5B-4A` 注释掉
  - 缩进统一: `# [右，左]` 注释缩进修正

- **影响范围**: 22 个文件 (`cleaning_path/` 11 个 `.txt` + `config/` 11 个 `.yaml`)，394 行新增，344 行删除。路径几何更平滑减少转弯打滑风险，边缘距离参数覆盖更完整。

## 2026-07-10 新航段首帧航向门控 + 普通模式超声禁用拦截

### 70. 追加回仓/切换新航段时先发正常速度再触发航向异常

- **问题**: 到达末端航点并追加回仓点后，机器人仍可能带着旧航向进入新航段。`WAYPOINT_MOVE` 先计算 Stanley 轮速，航向误差需要连续 5 帧超过 15° 才进入重校准，因此新航段首帧会先发出前进速度。
- **现场证据**: `run20260708.log` 中航点11完成后追加回仓点，旧航向约 `-115°`，新路段方向 `-33.9°`，首帧 `hdg_err=81.7°` 仍输出 `left=-11.50, right=8.50`，随后才因连续异常进入校准。
- **修复**:
  - `WAYPOINT_MOVE` 新建 `stanley_path_start` 的同一帧标记 `path_initialized_now`
  - 首帧若 `abs(heading_err) > HEADING_ABNORMAL_THRESHOLD` 且不在 `force_bearing_mode`，立即 `start_heading_recalibration()` 并先输出 `(0, 0)`
  - 这样新航段不会等待 5 帧异常计数，也不会先发 Stanley 前进轮速
- **影响**: `rtk_nav.py` — `WAYPOINT_MOVE` 新航段初始化后的航向门控。

### 71. 普通模式超声触发后禁用方向仍可被命令入口短暂响应

- **问题**: 超声/防跌落方向已经被禁用时，MQTT/键盘再次下发同方向运动命令仍会进入状态切换，下一轮 timer 才停车，现场表现为“先正常响应速度，然后瞬间再次触发”。
- **修复**:
  - 普通模式 `io_data_callback()` 恢复快/慢通道确认逻辑，持续触发后更新 `blocked_directions`
  - `_update_blocked_directions()` 发现当前状态落入禁用方向时立即停车
  - `switch_state()` 在状态切换入口拒绝已禁用方向，并保持电机/滚刷停止
- **影响**: `motor_control.py` — 普通模式超声确认、禁用方向立即停车、命令入口门控。

## 2026-07-13 人工介入暂停原航点续扫 + RTK恢复条件拆分

### 72. 人工介入暂停只能重新下发路径，无法从当前航点继续清扫

- **问题**: `calib_stuck`、`force_bearing_limit_cycle`、`force_bearing_diverge` 进入人工锁定 PAUSE 后，原恢复方案要求重新下发路径。`route_change_callback()` 会把 `current_waypoint_idx` 重置为 0，导致任务从头开始，丢失原清扫进度。
- **修复**:
  1. 新增 `manual_intervention_seen` 锁存位。人工故障发生时置为 `False`，普通 RTK/航向恢复和生成器重建不能解除锁定
  2. 切换到 `HOLD`、`NORMAL` 或其他非 `AUTO_CLEANING` 模式时置为 `True`，表示已经人工接管处理
  3. 再次进入 `AUTO_CLEANING` 时调用统一恢复函数，不重新加载路径，保留 `current_waypoint_idx` 和滚刷进度
  4. 恢复状态统一设为 `WAYPOINT_MOVE`，目标为 `waypoints[current_waypoint_idx]`；即使故障发生在 `WAYPOINT_CALIB`，也先重新靠近当前航点，再按正常到点流程进入校准，避免人工移动车辆后在错误位置校准并跳点
  5. 清零校准生成器、卡滞重试、force_bearing 模式、极限环/背离计数和航向异常瞬态，清除 `ERROR_CALIB_TIMEOUT`
  6. 航点索引为 0 时清空 `last_waypoint_cache`，让下一轮使用当前 GPS 作为 Stanley 路径起点，避免“航点0→航点0”的零长度路径
  7. `/motor/state` 与 `/control/mode` 两个入口均复用同一恢复函数，兼容两话题到达顺序不同的情况
- **恢复状态机**: `人工锁定PAUSE → 切出AUTO_CLEANING人工处理 → 再次进入AUTO_CLEANING → WAYPOINT_MOVE(current_waypoint_idx)`。
- **可观测性**: `/rtk/nav_context` 增加 `manual_intervention_seen`，可区分“等待人工接管”和“已处理、等待重新进入自动模式”。
- **影响**: `rtk_nav.py` — `nav_context`、三类人工暂停入口、人工恢复函数、`state_callback()`、`mode_callback()` 和暂停日志。

### 73. rtk_not_fixed 与 rtk_timeout 共用超时标志导致非固定解误恢复

- **问题**: PAUSE 生成器把 `rtk_not_fixed` 和 `rtk_timeout` 合并使用 `rtk_data_timed_out` 判断。RTK 数据持续更新但定位仍为 Float/单点解时，`rtk_data_timed_out=False`，生成器会错误恢复导航，随后下一帧又被非固定解暂停，形成 PAUSE/恢复抖动。
- **修复**:
  1. `rtk_not_fixed` 只使用最新 `fix_status` 判断：`last_gps_status != 4` 时保持 PAUSE，只有 RTK Fixed 才恢复
  2. `rtk_timeout` 只使用 `rtk_data_timed_out` 判断：超过 `RTK_DATA_TIMEOUT` 未收到 `/wtrtk_data` 时保持 PAUSE，数据重新到达后恢复
  3. 初始化并持续更新 `last_gps_status`；`fix_status < 0` 也按非固定解处理，不再因收到无效数据而误判为可恢复
  4. 组合故障处理：若先发生 `rtk_timeout`，数据恢复时定位仍非固定解，则把 `pause_reason` 从 `rtk_timeout` 切换为 `rtk_not_fixed`，保留原 `pre_pause_state` 并继续停车
  5. Fixed 状态回调只直接恢复 `rtk_not_fixed`；`rtk_timeout` 由数据超时标志解除后通过生成器恢复，两个恢复条件不再交叉
- **恢复条件**:

| pause_reason | 保持暂停条件 | 恢复条件 |
|---|---|---|
| `rtk_not_fixed` | `last_gps_status != 4` | `fix_status == 4` |
| `rtk_timeout` | `rtk_data_timed_out == True` | 重新收到 `/wtrtk_data`，超时标志清除 |

- **影响**: `rtk_nav.py` — RTK状态初始化、`heading_callback()` 非固定解/Fixed处理、PAUSE生成器恢复判断。
- **验证**: `python -B -m py_compile src/rtk_nav/rtk_nav/rtk_nav.py` 与 `git diff --check` 均通过。

### 74. 原地旋转校准路径绕过超声保护

- **问题**: 超声触发逻辑已按 `WAYPOINT_MOVE` 和 `WAYPOINT_CALIB` 分流，但仍有多处直接调用 `calibrate_heading_at_waypoint()` 后立即 `yield` 轮速的路径。这些路径虽然实际动作是原地旋转，却不经过 `WAYPOINT_CALIB` 的“旋转打滑 → `_retreat_to_waypoint()` → 重新校准”保护，导致初始航向对准、首航点最终校准、`force_bearing` 原地对准等场景可能绕过超声检测。
- **修复**:
  1. 新增 `_calibrate_with_boundary_retreat(target_heading, target_waypoint, label)`，作为所有原地航向校准的统一保护入口
  2. helper 在校准期间临时把 `nav_state` 和 `calib_target_heading` 设置为 `WAYPOINT_CALIB` 语义，使 `_is_motion_blocked()` 能按当前旋转方向判断左/右侧超声触发
  3. 旋转校准中触发超声且有目标航点时，执行 `_retreat_to_waypoint()` 回到航点附近后重新校准；无目标航点时交给 `get_boundary_correct_speed()` 启动边界纠偏
  4. 撤退过程中继续用 `_is_speed_blocked(left_speed, right_speed)` 检查候选轮速，若撤退动作本身仍被禁止，则切换到边界纠偏
  5. 初始航向对准、第一个航点最终校准、`INITIAL_MOVE` 的 `force_bearing` 原地对准、初始段航向异常校准、`WAYPOINT_MOVE` 的 `force_bearing` 原地对准全部改为走统一 helper
  6. 删除 `move_to_first_waypoint()` 内部旧的“行进触发后 GPS 后退 0.5m”逻辑，避免和外层 `INITIAL_MOVE` 的 `get_boundary_correct_speed()` 重复抢状态
- **行为约束**: 行进态仍由外层 `INITIAL_MOVE` / `WAYPOINT_MOVE` 调用 `get_boundary_correct_speed()` 做方向纠偏；原地旋转态统一执行“超声触发 → GPS撤退归位 → 重新校准”。
- **影响**: `rtk_nav.py` — 原地校准统一保护 helper、初始移动子生成器、主 `WAYPOINT_CALIB`/`force_bearing` 相关校准入口。
- **验证**: `python -B -m py_compile src/rtk_nav/rtk_nav/rtk_nav.py` 与 `git diff --check -- src/rtk_nav/rtk_nav/rtk_nav.py` 均通过。

## 2026-07-13 固定进仓航向收敛 + AUTO入口航向安全门控

### 75. loading_gps 航向存在隐式180度换算

- **问题**: `run.launch.py` 需要把实际进仓车头航向先加 `180°`，`motor_control.py` 再减 `180°` 得到最终目标。配置语义不直观，现场设置 `172.52°` 时容易得到错误的反向目标。
- **修复**:
  1. `loading_gps` 第三项统一定义为“后退进仓时的车头目标航向”
  2. `run.launch.py` 直接配置 `172.52°`，不再写 `172.52 + 180.0`
  3. `motor_control.py` 只将参数归一化到 `[-180°, 180°)`，不再额外反向
- **行为**: 配置 `loading_gps=[lon, lat, 172.52]` 后，进仓转向目标和后退纠偏目标均为 `172.52°`；后退进仓的轮速方向保持不变。
- **影响**: `run.launch.py`、`motor_control.py`。

### 76. 固定进仓点启用后仍保留 /unloading_gps 冗余通道

- **问题**: `rtk_nav` 已使用固定 `loading_gps`，但 `motor_control` 仍在出仓完成后等待 RTK Fixed 和航向稳定，再发布 `/unloading_gps`；`rtk_nav` 仍保留对应订阅和只打印日志的 callback。该通道不再改变进仓点，却延迟出仓完成并保留无效超时错误路径。
- **修复**:
  1. 删除 `motor_control` 的 `/unloading_gps` publisher、消息构造和两处发布逻辑
  2. 删除仅用于该发布的 `heading_stable` 订阅、GPS/航向等待计时、超时错误位和缓存变量
  3. 出仓运动完成后直接完成 `HOLD → AUTO_CLEANING` 模式切换
  4. 删除 `rtk_nav` 的 `/unloading_gps` subscription、`unloading_gps_callback()` 及旧解析注释
- **职责边界**: `motor_control` 负责出仓动作和模式切换；`rtk_nav` 只从 launch 注入的 `loading_gps` 获取固定进仓点，并独立负责 RTK/航向安全判断。
- **影响**: `motor_control.py`、`rtk_nav.py`。

### 77. AUTO_CLEANING 入口可能复用旧航向稳定结果

- **问题**: 删除 `motor_control` 的航向等待后，`rtk_nav` 虽有5秒航向稳定检查，但进入 `AUTO_CLEANING` 时没有清空 `_heading_stability_history` 和 `_last_heading_stable`。若旧结果仍为 `True`，首次启动或暂停恢复可能直接通过门控，无法保证使用切换后的新样本确认无漂移。
- **修复**:
  1. 新增 AUTO 航向门控状态；每次进入 `AUTO_CLEANING` 清空旧样本、强制稳定结果为 `False`
  2. `/motor/state` 和 `/control/mode` 两个入口共用门控准备函数，并用一次性标志避免消息到达顺序不同导致重复重置
  3. `rtk_timer_callback()` 在创建或恢复导航生成器前统一等待新的5秒窗口满足波动 `≤3°`，等待期间持续发布停车速度
  4. 门控覆盖 `IDLE` 首次启动以及 `WAYPOINT_MOVE`、`WAYPOINT_CALIB` 等恢复路径；恢复校准期间也允许在停车状态重新采集稳定样本
  5. `/rtk/nav_context` 增加 `heading_stable`、`auto_heading_gate_pending`、`heading_stability_samples`，便于现场确认等待原因和采样进度
- **预期日志**: `[AUTO航向门控] ...已清空旧航向样本` → `等待航向稳定` → `航向稳定检查通过，允许启动/恢复导航`。
- **影响**: `rtk_nav.py` — 航向稳定追踪、双入口模式回调、10Hz导航启动门控和调试上下文。
- **验证**: bundled Python 对 `motor_control.py`、`rtk_nav.py`、`run.launch.py` 执行 `py_compile` 通过；AUTO门控静态检查和 `git diff --check` 通过。
