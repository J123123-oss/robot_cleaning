# Stanley 控制器修复记录

分支: `stanley` | 日期: 2026-05-21

## 提交历史

```
本次提交 fix: 初始航向校准后强制切换到航点1，避免零长度Stanley路径
b1d106c Handle abnormal Stanley heading recovery
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
