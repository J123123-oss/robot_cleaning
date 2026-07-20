#  RTK循迹 Ros2

1、 打点建图，保存路径，到launch.py中切换对应路径
    地图通过起始点经纬度，倾角，指定轨迹起点、地图间隔等确定。

2、 播放录制bag测试，启动launch中的wtrtk_parse_txt

3、 通过话题发布

    ros2 topic pub /keyboard/control std_msgs/msg/String "{data: 'r'}" -1
    进入RTK导航模式，默认为Normal模式可以键盘发布控制不能RTK导航。

    通过话题发布
    ros2 topic pub /keyboard/control std_msgs/msg/String "{data: 'n'}" -1
    切换正常模式，键盘控制，发布w/s/a/d控制运动，z停止，x使能

    通过话题发布
    ros2 topic pub /keyboard/control std_msgs/msg/String "{data: 'm'}" -1
    进入遥控模式

4、 实时导航测试
    注释wtrtk_parse_txt，启动launch中的wtrtk_serial_driver
    进入实时导航，提前进行步骤1确定路径


## 无线充电
### 开始充电

ros2 service call /start_charging custom_msgs/srv/ChargeControl "{}"

### 停止充电
ros2 service call /stop_charging custom_msgs/srv/ChargeControl "{}"

### 查询电压电流
ros2 service call /query_volt_curr std_srvs/srv/Trigger "{}"

### 实时订阅故障码话题
ros2 topic echo /charging_fault_code std_msgs/msg/Int16


## RTK导航流程图

### 自动清扫简化流程

```mermaid
graph LR
    DISABLE["DISABLE<br/>失能"] -->|"ENABLE + START"| START["START<br/>出仓"]
    START -->|"出仓完成"| AUTO_CLEANING["AUTO_CLEANING<br/>RTK巡航清扫"]
    AUTO_CLEANING -->|"导航完成"| LOADING["LOADING<br/>激光对位进仓"]
    LOADING -->|"进仓完成"| DISABLE

    style DISABLE fill:#ff6b6b,stroke:#333,color:#fff
    style START fill:#6bcb77,stroke:#333
    style AUTO_CLEANING fill:#9b59b6,stroke:#333,color:#fff
    style LOADING fill:#4d96ff,stroke:#333,color:#fff
```

### 主状态机
```mermaid
graph TD
    %% ===== 入口 =====
    STARTUP["上电启动"] --> DISABLE

    %% ===== 电机初始化 =====
    DISABLE -->|"ENABLE"| ENABLE["ENABLE<br/>电机使能"]
    ENABLE -->|"DISABLE"| DISABLE

    %% ===== HOLD 中枢 =====
    ENABLE -->|"HOLD"| HOLD["HOLD<br/>电机停止，保持使能"]
    HOLD -->|"DISABLE"| DISABLE
    HOLD -->|"ENABLE"| ENABLE

    %% ===== 全自动清扫 =====
    HOLD -->|"AUTO_CLEANING"| AUTO_CLEANING["AUTO_CLEANING<br/>RTK巡航清扫"]
    ENABLE -->|"AUTO_CLEANING"| AUTO_CLEANING
    AUTO_CLEANING -->|"HOLD 暂停"| HOLD
    AUTO_CLEANING -->|"DISABLE 失能"| DISABLE
    AUTO_CLEANING -->|"导航完成 COMPLETED<br/>自动触发"| LOADING

    %% ===== 出仓/开始作业 =====
    HOLD -->|"START"| U_CHECK{"电量 ≥ 91%？"}
    U_CHECK -->|"是"| START["START<br/>出仓流程"]
    U_CHECK -->|"否，拒绝执行"| HOLD
    START -->|"出仓完成"| START_DONE["START 完成<br/>complete_state=True<br/>等待后续指令"]
    START -->|"GPS超时 30s"| DISABLE
    START -->|"HOLD 强制中断"| HOLD

    %% ===== 进仓 =====
    HOLD -->|"LOADING"| L_CHECK{"距进仓点 ≤ 10m？"}
    L_CHECK -->|"是"| LOADING["LOADING<br/>激光对位进仓"]
    L_CHECK -->|"否，拒绝执行"| HOLD
    LOADING -->|"进仓完成"| LOADING_DONE["LOADING 完成<br/>complete_state=True<br/>等待后续指令"]
    LOADING -->|"HOLD 强制中断"| HOLD

    %% ===== 进出仓期间保护 =====
    START -.->|"仅接受 HOLD / DISABLE"| START
    LOADING -.->|"仅接受 HOLD / DISABLE"| LOADING

    %% ===== 出仓/进仓完成后续 =====
    START_DONE -->|"航向稳定后<br/>自动切换"| AUTO_CLEANING
    LOADING_DONE -->|"补发3条完成消息"| DISABLE

    %% ===== MQTT 遥控接管流程 =====
    RC_ENABLE["RC_ENABLE<br/>MQTT启用遥控器"] -->|"保存当前状态<br/>立即停车<br/>rc_control=True"| RC_ACTIVE["遥控接管中<br/>使用NORMAL键盘逻辑<br/>优先级高于AUTO_CLEANING"]
    RC_ACTIVE -->|"RC_DISABLE<br/>MQTT关闭遥控器"| RESTORE["恢复原控制模式<br/>继续原流程"]

    %% RC_ENABLE 可从任意状态进入（AUTO_CLEANING下也允许）
    HOLD -.->|"MQTT: RC_ENABLE"| RC_ENABLE
    AUTO_CLEANING -.->|"MQTT: RC_ENABLE"| RC_ENABLE
    LOADING_DONE -.->|"MQTT: RC_ENABLE"| RC_ENABLE
    START_DONE -.->|"MQTT: RC_ENABLE"| RC_ENABLE
    DISABLE -.->|"MQTT: RC_ENABLE"| RC_ENABLE

    RESTORE --> HOLD

    %% ===== 样式 =====
    style STARTUP fill:#e1e1e1,stroke:#999
    style DISABLE fill:#ff6b6b,stroke:#333,color:#fff
    style HOLD fill:#ffd93d,stroke:#333
    style START fill:#6bcb77,stroke:#333
    style START_DONE fill:#b8e6c8,stroke:#6bcb77
    style LOADING fill:#4d96ff,stroke:#333,color:#fff
    style LOADING_DONE fill:#a8d1ff,stroke:#4d96ff
    style AUTO_CLEANING fill:#9b59b6,stroke:#333,color:#fff
    style ENABLE fill:#dfe6e9,stroke:#333
    style U_CHECK fill:#ffeaa7,stroke:#333
    style L_CHECK fill:#ffeaa7,stroke:#333
    style RC_ENABLE fill:#e17055,stroke:#333,color:#fff
    style RC_ACTIVE fill:#fab1a0,stroke:#e17055
    style RESTORE fill:#74b9ff,stroke:#333
```

```mermaid
graph TD
    subgraph 外部事件中断
        GPS_NONFIX["定位或定向非Fixed"] --> PAUSE_AUTO["PAUSE<br/>rtk_not_fixed"]
        RTK_TIMEOUT["RTK数据断流>1s"] --> PAUSE_AUTO2["PAUSE<br/>rtk_timeout"]
        HDG_GATE_TIMEOUT["AUTO航向门控60s未稳定"] --> PAUSE_AUTO3["PAUSE<br/>auto_heading_gate_timeout"]
        PAUSE_AUTO --> GPS_FIX["定位与定向恢复Fixed"] --> RESUME["恢复pre_pause_state"]
        PAUSE_AUTO2 --> RTK_FRESH["/wtrtk_data恢复新鲜"] --> RESUME
        PAUSE_AUTO3 --> HDG_STABLE["新5s航向稳定窗口通过"] --> RESUME

        CALIB_STUCK["校准卡滞重试耗尽"] --> PAUSE_MANUAL["PAUSE<br/>calib_stuck"]
        RETREAT_TIMEOUT["GPS撤退P1/P2超时"] --> PAUSE_MANUAL2["PAUSE<br/>boundary_retreat_timeout"]
        FORCE_LIMIT["force_bearing极限环/背离"] --> PAUSE_MANUAL3["PAUSE<br/>force_bearing_* "]
        PAUSE_MANUAL --> TAKEOVER["切出AUTO_CLEANING<br/>人工处理"]
        PAUSE_MANUAL2 --> TAKEOVER
        PAUSE_MANUAL3 --> TAKEOVER
        TAKEOVER --> REENTER_AUTO["重新进入AUTO_CLEANING"] --> RESUME_WP["WAYPOINT_MOVE<br/>从当前航点恢复"]
        HOLD["电机状态HOLD"] --> STOP_NAV["强制停止导航"]
        MODE_SWITCH["控制模式切换"] --> STOP_NAV
        ROUTE_CHANGE["路径切换/rtk/route_change"] --> RELOAD["重新加载航点文件<br/>清零航向异常计时器"]
        RELOAD --> RESET_IDX["重置航点索引为0"]
    end
```

```mermaid
graph TD
    subgraph 定时器驱动 10Hz
        TIMER["rtk_timer_callback"] --> CHECK_MODE{"控制模式==AUTO_CLEANING?"}
        CHECK_MODE -- 否 --> IDLE_RESET["重置生成器"]
        CHECK_MODE -- 是 --> CHECK_WAYPOINTS{"有航点数据?"}
        CHECK_WAYPOINTS -- 否 --> ERROR_EXIT["报错退出"]
        CHECK_WAYPOINTS -- 是 --> CHECK_GENERATOR{"生成器存在?"}
        CHECK_GENERATOR -- 否 --> CREATE_GEN["创建multi_waypoint_nav_generator"]
        CREATE_GEN --> RUN_GEN["resume判断: IDLE->首次 / 其他->恢复"]
        CHECK_GENERATOR -- 是 --> NEXT_STEP["next()获取速度"]
        NEXT_STEP --> PUB_SPEED["发布motor_speed"]
    end
```
### 导航状态机 
```mermaid
graph TD
    IDLE["IDLE"] --> INIT_MOVE["INITIAL_MOVE"]
    INIT_MOVE --> MOVE_FIRST["初始点->第一个航点"]
    MOVE_FIRST --> STANLEY1["Stanley控制器直线行驶"]
    STANLEY1 --> CHECK_FORCE1{"force_bearing_mode?"}
    CHECK_FORCE1 -- 是 --> BEARING1["方位角直行: path_dir=bearing→target<br/>跳过航向异常检测"]
    BEARING1 --> CHECK_DIST1
    CHECK_FORCE1 -- 否 --> CHECK_HEADING1{"航向误差>15° 连续5帧?"}
    CHECK_HEADING1 -- 是 --> INC_COUNT1["waypoint_recalib_count++"]
    INC_COUNT1 --> CHECK_COUNT1{"同航点校准≥2次?"}
    CHECK_COUNT1 -- 是 --> SET_FORCE1["force_bearing_mode=True<br/>打滑/位置偏移，直行不校准"]
    SET_FORCE1 --> STANLEY1
    CHECK_COUNT1 -- 否 --> RECALIB1_INLINE["内联航向校准 原地旋转至path_dir"]
    RECALIB1_INLINE --> STANLEY1
    CHECK_HEADING1 -- 否 --> CHECK_DIST1{"距离<0.1m 连续5帧?"}
    CHECK_DIST1 -- 否 --> STANLEY1
    CHECK_DIST1 -- 是 --> CALIB1["WAYPOINT_CALIB 航向校准"]
    CALIB1 --> TURN1["原地旋转至目标heading"]
    TURN1 --> CHECK_ANGLE1{"角度误差<1°?"}
    CHECK_ANGLE1 -- 否 --> TURN1
    CHECK_ANGLE1 -- 是 --> RESET_COUNT1["清零 waypoint_recalib_count<br/>current_waypoint_idx++"]
    RESET_COUNT1 --> NEXT_WP["current_waypoint_idx++"]

    NEXT_WP --> WP_MOVE["WAYPOINT_MOVE"]
    WP_MOVE --> GET_TARGET["获取目标航点"]
    GET_TARGET --> CHECK_LAST{"到达最后一个航点?"}
    CHECK_LAST -- 是 --> SWITCH_FILE["自动切换下一路径文件"]
    SWITCH_FILE --> CHECK_NEXT{"有下一文件?"}
    CHECK_NEXT -- 是 --> RESET_IDX2["重置索引=0, 返回新文件首航点<br/>清零 waypoint_recalib_count"]
    RESET_IDX2 --> WP_MOVE
    CHECK_NEXT -- 否 --> CHECK_LOADING{"有出仓点?"}
    CHECK_LOADING -- 是 --> APPEND_LOADING["追加出仓点到航点列表"]
    APPEND_LOADING --> WP_MOVE
    CHECK_LOADING -- 否 --> COMPLETED["COMPLETED 导航完成"]

    CHECK_LAST -- 否 --> STANLEY2["Stanley控制器直线行驶"]
    STANLEY2 --> CHECK_FORCE{"force_bearing_mode?"}
    CHECK_FORCE -- 是 --> BEARING2["方位角直行: path_dir=bearing→target<br/>跳过航向异常检测"]
    BEARING2 --> CHECK_DIST2
    CHECK_FORCE -- 否 --> CHECK_HEADING{"航向误差>15° 连续5帧?"}
    CHECK_HEADING -- 是 --> INC_COUNT["waypoint_recalib_count++"]
    INC_COUNT --> CHECK_COUNT{"同航点校准≥2次?"}
    CHECK_COUNT -- 是 --> SET_FORCE["force_bearing_mode=True<br/>打滑/位置偏移，直行不校准"]
    SET_FORCE --> STANLEY2
    CHECK_COUNT -- 否 --> RECALIB["WAYPOINT_CALIB 重新校准到当前路径方向"]
    RECALIB --> STANLEY2
    CHECK_HEADING -- 否 --> CHECK_DIST2{"距离<0.1m?"}
    CHECK_DIST2 -- 否 --> CHECK_LOW{"距离<1.5m?"}
    CHECK_LOW -- 是 --> SLOW_DOWN["线性减速 speed_scale=0.2~0.7"]
    SLOW_DOWN --> STANLEY2
    CHECK_LOW -- 否 --> STANLEY2
    CHECK_DIST2 -- 是 --> CALIB2["WAYPOINT_CALIB 航向校准"]
    CALIB2 --> TURN2["原地旋转至目标heading"]
    TURN2 --> CHECK_ANGLE2{"角度误差<1°?"}
    CHECK_ANGLE2 -- 否 --> TURN2
    CHECK_ANGLE2 -- 是 --> RESET_COUNT["清零 waypoint_recalib_count<br/>current_waypoint_idx++"]
    RESET_COUNT --> NEXT_WP
```

### 航向校准与边界撤退安全分支

```mermaid
graph TD
    ENTER["进入 WAYPOINT_CALIB<br/>原地旋转校准"] --> ROTATE["calibrate_heading_at_waypoint"]
    ROTATE --> SENSOR{"旋转动作被边界禁止?"}
    SENSOR -- 否 --> ANGLE{"误差小于1°?"}
    ANGLE -- 是 --> SUCCESS["校准成功<br/>推进或继续当前航点"]
    ANGLE -- 否 --> ROTATE

    SENSOR -- 是 --> RETREAT["停车一帧、清除固定纠偏锁定<br/>GPS撤退回目标航点"]
    RETREAT --> P1["P1: 转向远离航点方向<br/>传感器阻挡时先远离"]
    P1 --> RETREAT_TIMEOUT{"总时长超过30s?"}
    RETREAT_TIMEOUT -- 否 --> P2["P2: 倒车归位<br/>距离闭环"]
    P2 --> RETREAT_OK{"距离小于阈值?"}
    RETREAT_OK -- 是 --> ROTATE
    RETREAT_OK -- 否 --> RETREAT_TIMEOUT
    RETREAT_TIMEOUT -- 是 --> PAUSE_RETREAT["PAUSE<br/>boundary_retreat_timeout<br/>持续发布零速度"]
    PAUSE_RETREAT --> MANUAL["切出AUTO_CLEANING人工处理"]
    MANUAL --> RESUME_CURRENT["重新进入AUTO_CLEANING<br/>WAYPOINT_MOVE当前航点"]

    ANGLE -- 校准超时/失败 --> RETRY{"重试次数不超过3?"}
    RETRY -- 否 --> PAUSE_STUCK["PAUSE<br/>calib_stuck"]
    RETRY -- 是 --> RETRY_NUM{"第2次及以上?"}
    RETRY_NUM -- 否 --> RESET_CALIB["重建校准生成器"] --> ROTATE
    RETRY_NUM -- 是 --> BACKUP["后退脱困<br/>速度=+1.0/-1.0，时长=1.5s"]
    BACKUP --> BACKUP_BLOCKED{"候选后退速度被边界禁止?"}
    BACKUP_BLOCKED -- 是 --> BOUNDARY_CORRECT["get_boundary_correct_speed"] --> RESET_CALIB
    BACKUP_BLOCKED -- 否 --> RESET_CALIB

    style PAUSE_RETREAT fill:#e17055,stroke:#333,color:#fff
    style PAUSE_STUCK fill:#e17055,stroke:#333,color:#fff
    style SUCCESS fill:#b8e6c8,stroke:#6bcb77
```

### 边界矫正状态机
```mermaid
graph TD
    subgraph 边界矫正状态机
        SENSOR_TRIG["传感器触发 front/mid"] --> TURNING["TURNING 偏转1.0s"]
        TURNING --> BACKING["BACKING 后退4.0s"]
        BACKING --> RETURNING["RETURNING 反向偏转退回2.0s"]
        RETURNING --> IDLE_BC["IDLE 恢复正常导航"]
    end
```

### Stanley控制器流程
```mermaid
graph TD
    INPUT["输入: current_pos, current_heading, path_start, path_end, path_direction, velocity"] --> STEP1

    subgraph STEP1["计算横向误差"]
        UTM1["UTM坐标转换: 当前位置/起点/终点"] --> CROSS["向量叉积 AP x AB"]
        CROSS --> LATERAL["lateral_error = cross / |AB|"]
        LATERAL --> CLAMP_LE["clamp ±MAX_LATERAL_ERROR(1.0m)"]
    end

    STEP1 --> STEP2
    subgraph STEP2["计算航向误差"]
        HEADING_ERR["heading_error = path_direction - current_heading"]
        HEADING_ERR --> NORMALIZE["归一化到-180°~180°"]
    end

    STEP2 --> STEP3
    subgraph STEP3["Stanley控制律"]
        V_RAW["velocity (电机指令)"] --> CONV["real_v = velocity × 0.0345"]
        CONV --> K["K自适应: <1.3m→0.42, ≥1.3m→0.45"]
        K --> STEERING
        LE["lateral_error"] --> STEERING
        HE["heading_error"] --> STEERING
        STEERING["st_corr = atan(K × lat_err / max(real_v, 0.15))"]
        STEERING --> SUPPRESS{"abs(hdg_err)>4° 且 st_corr与hdg_err同向?"}
        SUPPRESS -- 是 --> HALF["st_corr × 0.5 航向优先抑制"]
        SUPPRESS -- 否 --> TOTAL["total = st_corr - hdg_err"]
        HALF --> TOTAL
        TOTAL --> CLAMP["clamp ±45°"]
    end

    STEP3 --> STEP4
    subgraph STEP4["差速分配"]
        FACTOR["steering_factor = clamped / 45°"] --> DIFF["speed_diff = factor × 1.5"]
        DIFF --> LEFT["left = -velocity + speed_diff"]
        DIFF --> RIGHT["right = velocity + speed_diff"]
    end

    STEP4 --> OUTPUT["输出: left_speed, right_speed + real_velocity发布到/rtk/velocity"]
```

### 航向异常保护

- **连续异常检测**: 导航行驶中航向误差超过 15° 连续 5 帧（0.5s），判定为异常航向，触发重校准。
- **方位角模式排除**: t>1.0（越过投影终点）或 force_bearing_mode 时，航向误差来自侧向接近目标（几何现象而非 IMU 异常），跳过航向异常计数和超时计时。
- **打滑/偏移兜底**: 同航点重校准 ≥2 次后自动切换 `force_bearing_mode`，先原地旋转对准目标方位，之后每帧检查航向偏差：>15° 先原地对准再走，≤15° 用实时方位角直行。既防止 `bearing(current→target)` 每帧重算导致的追尾螺旋，又避免固定缓存中途打滑回不来。航点切换后清零。
- **RTK 数据超时**: `/wtrtk_data` 超过 1s 未更新立即停车 PAUSE；数据新鲜即为恢复条件（不再检查航向误差绝对值，避免 IMU 漂移导致无法恢复）。
- **模式切入清理**: 进入 AUTO_CLEANING / 路径切换时清零 `heading_abnormal_start_time` 和 `heading_timed_out`，防止旧任务污染。
- 单帧跳变只累计一次，下一帧恢复到阈值内会清零计数，避免 RTK/IMU 瞬时抖动误判。
- 异常重校准完成后不推进航点索引，继续前往当前航点；正常到达航点后的校准仍会推进到下一个航点。

### 航点切换与跨文件处理
```mermaid
graph TD
    WP_SWITCH["航点索引切换"] --> CHECK_CALIB{"当前是校准状态?"}
    CHECK_CALIB -- 是 --> KEEP_CALIB["保持WAYPOINT_CALIB状态"]
    CHECK_CALIB -- 否 --> RESET_MOVE["重置为WAYPOINT_MOVE"]

    RESET_MOVE --> CHECK_CROSS{"current_waypoint_idx==0 且有跨文件缓存?"}
    CHECK_CROSS -- 是 --> USE_CROSS["last_waypoint_cache = cross_file_last_waypoint"]
    CHECK_CROSS -- 否 --> CHECK_SAME{"idx-1 >= 0?"}
    CHECK_SAME -- 是 --> USE_SAME["last_waypoint_cache = waypoints[idx-1]"]
    CHECK_SAME -- 否 --> NO_CACHE["无上一个航点缓存"]

    USE_CROSS --> INIT_STANLEY["初始化Stanley路径参数"]
    USE_SAME --> INIT_STANLEY
    INIT_STANLEY --> CALC_BEARING["calculate_path_bearing 预计算路径方向"]
    CALC_BEARING --> SAVE_PATH["保存 stanley_path_start, stanley_path_direction"]
```

### 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| LINEAR_SPEED_BASE | 10.0 | 基础行驶速度（电机指令值，约0.35m/s） |
| SPEED_LIMIT | 1.3 × BASE | 最大速度限制 |
| SPEED_CMD_TO_MPS | 0.0345 | 电机指令→真实速度(m/s)转换系数 |
| RTK_WAYPOINT_TOLERANCE | 0.10 m | 到达航点距离阈值 |
| INITIAL_MOVE_TOLERANCE | 0.10 m | 初始移动到达阈值 |
| LOW_DISTANCE | 1.5 m | 减速触发距离 |
| RTK_HEADING_TOLERANCE | 1.0° | 航向校准精度 |
| STANLEY_K_NEAR (<1.3m) | 0.42 | 近距离自适应 K |
| STANLEY_K_FAR (≥1.3m) | 0.45 | 远距离自适应 K |
| STANLEY_MIN_SPEED | 0.15 m/s | Stanley 计算最小速度（防除零+限幅） |
| MAX_LATERAL_ERROR | 1.0 m | 横向误差钳位上限 |
| STRAIGHT_MAX_CORRECTION | 1.5 | 直线最大差速修正 |
| HEADING_ABNORMAL_THRESHOLD | 15.0° | 航向异常阈值 |
| ANGLE_ABNORMAL_COUNT | 5 | 连续异常帧数（0.5s） |
| 航向优先抑制 | abs(hdg_err)>4° | 横向项与航向项同向时 st_corr 减半 |
| RTK_DATA_TIMEOUT | 1.0 s | RTK 数据断流超时停车 |
| force_bearing_mode | ≥2次 | 同航点重校准次数阈值，触发后自适应循环对准（>15°先旋转, ≤15°实时方位角直行） |
| FORCE_BEARING_REALIGN_THRESHOLD | 15.0° | force_bearing_mode 下航向偏差超此值触发原地重新对准 |

## 批量生成清扫路径

`batch_generate_paths.py` —— 不依赖 ROS2 环境，直接读取 YAML 配置，调用路径规划核心函数生成清扫路径 txt 文件。

```bash
# 基础用法（只生成 txt，不画图）
python batch_generate_paths.py

# 输出目录: src/rtk_nav/rtk_nav/cleaning_path/
# 读取目录: src/rtk_nav/rtk_nav/config/
```

- 遍历 `config/` 下所有 `*.yaml` 配置文件
- 每个配置文件生成一个 `{配置名}.txt` 密集点路径文件
- 图片绘制已注释，需要时取消注释 `plot_multi_area_path` 和 matplotlib 导入即可
