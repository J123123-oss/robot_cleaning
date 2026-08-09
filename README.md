# RTK 循迹 ROS2

## 当前运行版本

系统由 `motor_control` 底盘状态机和 `rtk_nav` 航点导航节点组成。AUTO 清扫的有效路径为：

`AUTO_CLEANING` → 航向门控停车 → `INITIAL_MOVE`/`WAYPOINT_MOVE` → 航点原地校准 → `COMPLETED` → `LOADING`。

进入 AUTO 后，`rtk_nav` 必须同时确认定位 Fixed、定向 Fixed 且 GGA 数据有效，然后连续采集 5 秒短窗和 30 秒收敛窗（分别 ≤1°、≤2°）。运行中 Float ≤3 秒桥接已有 Fixed 历史；超过 3 秒清空窗口，重新完整采集 5 秒短窗和 30 秒收敛窗。

自动恢复型暂停保持底盘 `AUTO_CLEANING`，只输出零速度；`calib_stuck`、航向校准失败、边界 GPS 撤退超时、force bearing 极限环，以及航向门控 600 秒超时属于人工介入暂停，底盘切换 `HOLD`。导航状态通过 `/rtk/nav_state` 发布 JSON，包含 `pause_reason`、`auto_resume` 和递增 `seq`。

暂停协议按结构化字段处理：`auto_resume=true` 时底盘不改变控制模式，只清零左右轮和滚刷速度；`auto_resume=false` 时底盘将 `AUTO_CLEANING` 切换到 `HOLD/NORMAL`，等待人工重新进入 AUTO。底盘按 `seq` 丢弃旧状态，不能仅根据 `nav_state="PAUSE"` 字符串决策。

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
    切换 NORMAL 模式（旧 REMOTE 模式已废弃）

4、 实时导航测试
    注释wtrtk_parse_txt，启动launch中的wtrtk_serial_driver
    进入实时导航，提前进行步骤1确定路径

## 路径切换与区域跳转

路径标识 `route_id` 是清扫路径文件的文件名去掉 `.txt` 后缀，例如
`008-W19-W16` 对应 `cleaning_path/008-W19-W16.txt`。`rtk_nav` 通过
`/rtk/current_route_id` 发布当前实际加载的标识，`motor_control` 将其同步到
`/robot_state` 的 `route_id` 字段；它不是单纯的显示字段，而是当前导航文件的标识。

### 修改 `route_id`（切换整条路径）

通过 `robot_cmd` 发送下列任一 JSON；`route_id` 不带 `.txt` 后缀：

```bash
ros2 topic pub /robot_cmd std_msgs/msg/String \
  '{data: "{\"route_id\":\"008-W19-W16\"}"}' -1

# 兼容写法
ros2 topic pub /robot_cmd std_msgs/msg/String \
  '{data: "{\"command\":\"CHANGE_ROUTE\",\"route_id\":\"008-W19-W16\"}"}' -1
```

限制条件：

- 仅在底盘状态为 `DISABLE` 时受理；`ENABLE`、`HOLD`、`START`、`LOADING` 和 `AUTO_CLEANING` 均会拒绝。
- 文件必须存在于设备配置的 `cleaning_path/{route_id}.txt`；不存在、空 `route_id` 或带错后缀（如传入 `xxx.txt`）不会切换。
- 成功后导航节点清空旧航点、滚刷事件、校准/边界/force-bearing 上下文和航向异常计时器，将航点索引置为 `0`、导航状态置为 `IDLE`，再加载新文件并发布新的 `route_id`。
- 切换路径不自动启动清扫；标准作业流程是随后下发 `START`，出仓结算后进入 `AUTO_CLEANING` 航向门控。

### `skip_to_area`（从指定区域续扫）

区域跳转仅接受如下 JSON：

```bash
ros2 topic pub /robot_cmd std_msgs/msg/String \
  '{data: "{\"skip_to_area\":\"bridge_E3out2-E3\"}"}' -1
```

限制与行为：

- 命令入口与 `rtk_nav` 接收端均要求底盘已确认处于 `HOLD`；任何一个检查发现不是 `HOLD` 都会拒绝。不要在 `AUTO_CLEANING` 行驶时直接跳转。
- 区域名不能为空，且当前路径必须已加载航点及区域标签；否则不改变索引。
- 匹配优先级为**精确匹配**、**前缀匹配**（`area.startswith(target)`）、**双向包含匹配**。匹配到多个不同区域时，按路径文件中的先后顺序选择第一个，并在日志中列出候选区域；因此应优先传完整区域名，避免依赖前缀/包含匹配。
- 跳转到匹配区域的首个航点。若当前位置有效，以当前位置建立接入航段；否则回退使用目标航点的前一航点（首航点则使用自身）。旧 Stanley 路径方向不会沿用。
- 跳转会重置校准、边界撤退和 force-bearing 上下文，清除错误码 `128`，并按目标航点索引重新计算滚刷状态；跳入 `#start` 与 `#stop` 之间时滚刷会在恢复 AUTO 后开启。
- 跳转完成时 `rtk_nav` 会发布计划恢复状态 `WAYPOINT_MOVE`，但底盘仍处于 `HOLD`，不会运动。随后重新进入 `AUTO_CLEANING`，仍必须通过 5 秒/30 秒航向门控并完成必要的路径对齐，才从目标航点开始行驶。

```mermaid
flowchart TD
    CMD["robot_cmd JSON"] --> KIND{"route_id 或 skip_to_area?"}
    KIND -- "route_id" --> DISABLED{"底盘为 DISABLE?"}
    DISABLED -- "否" --> REJECT_ROUTE["拒绝：路径切换仅限 DISABLE"]
    DISABLED -- "是" --> FILE{"cleaning_path/{route_id}.txt 存在?"}
    FILE -- "否" --> REJECT_FILE["拒绝：路径文件不存在/标识无效"]
    FILE -- "是" --> LOAD["重置索引=0与导航上下文<br/>加载新路径，nav_state=IDLE"]

    KIND -- "skip_to_area" --> HOLD{"底盘已确认 HOLD?"}
    HOLD -- "否" --> REJECT_SKIP["拒绝：区域跳转仅限 HOLD"]
    HOLD -- "是" --> AREA{"区域标签已加载且可匹配?"}
    AREA -- "否" --> REJECT_AREA["拒绝：空名称/无航点/未匹配"]
    AREA -- "是" --> REJOIN["选择首个匹配航点<br/>当前位置接入路径，重算滚刷"]
    REJOIN --> WAIT_AUTO["保持 HOLD，等待 AUTO_CLEANING"]
    WAIT_AUTO --> GATE["5秒/30秒航向门控与路径对齐"]
    GATE --> RESUME["从目标航点 WAYPOINT_MOVE 续扫"]
```


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
    START -->|"出仓完成并结算2s"| AUTO_CLEANING["AUTO_CLEANING<br/>航向门控停车"]
    AUTO_CLEANING -->|"5s≤1°且30s≤2°"| INITIAL_MOVE["INITIAL_MOVE / WAYPOINT_MOVE<br/>RTK巡航清扫"]
    INITIAL_MOVE -->|"导航完成"| LOADING["LOADING<br/>GPS/激光对位进仓"]
    LOADING -->|"进仓完成"| DISABLE

    style DISABLE fill:#ff6b6b,stroke:#333,color:#fff
    style START fill:#6bcb77,stroke:#333
    style AUTO_CLEANING fill:#9b59b6,stroke:#333,color:#fff
    style INITIAL_MOVE fill:#9b59b6,stroke:#333,color:#fff
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
    HOLD -->|"AUTO_CLEANING"| AUTO_CLEANING["AUTO_CLEANING<br/>RTK导航/航向门控"]
    ENABLE -->|"AUTO_CLEANING"| AUTO_CLEANING
    AUTO_CLEANING -->|"自动恢复型 PAUSE<br/>保持AUTO_CLEANING，零速度"| AUTO_GATE["航向/RTK门控暂停"]
    AUTO_GATE -->|"Fixed+GGA有效，稳定窗口通过"| AUTO_CLEANING
    AUTO_GATE -->|"600s未通过，错误码64"| HOLD
    AUTO_CLEANING -->|"人工介入型 PAUSE<br/>底盘切换HOLD"| HOLD
    AUTO_CLEANING -->|"DISABLE 失能"| DISABLE
    AUTO_CLEANING -->|"导航完成 COMPLETED<br/>自动触发"| LOADING

    %% ===== 出仓/开始作业 =====
    HOLD -->|"START"| U_CHECK{"电量 ≥ 90%？"}
    U_CHECK -->|"是"| START["START<br/>出仓流程"]
    U_CHECK -->|"否，拒绝执行"| HOLD
    START -->|"出仓完成"| START_DONE["出仓完成<br/>HOLD结算2s"]
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
    START_DONE -->|"结算完成"| AUTO_CLEANING
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
    style AUTO_GATE fill:#9b59b6,stroke:#333,color:#fff
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
        GPS_NONFIX["定位/定向非Fixed<br/>或GGA无效"] --> PAUSE_AUTO["PAUSE<br/>rtk_not_fixed<br/>auto_resume=true"]
        RTK_TIMEOUT["RTK断流或零角度>1s"] --> PAUSE_AUTO2["PAUSE<br/>rtk_timeout<br/>auto_resume=true"]
        HDG_TIMEOUT["运行中航向误差>15°持续15s"] --> PAUSE_AUTO3["PAUSE<br/>heading_timeout<br/>auto_resume=true"]
        PAUSE_AUTO --> GPS_FIX["定位、定向Fixed且GGA有效"] --> GATE["重新通过5s/30s航向门控"] --> RESUME["恢复pre_pause_state"]
        PAUSE_AUTO2 --> RTK_FRESH["/wtrtk_data重新新鲜"] --> RTK_QUALITY{"当前仍为双Fixed且GGA有效?"}
        RTK_QUALITY -- 是 --> RESUME
        RTK_QUALITY -- 否 --> PAUSE_AUTO
        PAUSE_AUTO3 --> HDG_NORMAL["航向数据恢复正常"] --> RESUME

        HDG_GATE_TIMEOUT["AUTO航向门控600s未通过<br/>错误码64"] --> PAUSE_GATE["PAUSE<br/>auto_heading_gate_timeout<br/>auto_resume=false"]
        CALIB_STUCK["校准卡滞/初始对准失败"] --> PAUSE_MANUAL["PAUSE<br/>calib_stuck / initial_heading_calib_failed<br/>auto_resume=false"]
        RETREAT_TIMEOUT["GPS撤退P1/P2或边界周期超时"] --> PAUSE_MANUAL2["PAUSE<br/>boundary_retreat_timeout / boundary_cycle_exhausted<br/>auto_resume=false"]
        FORCE_LIMIT["force_bearing极限环/持续背离"] --> PAUSE_MANUAL3["PAUSE<br/>force_bearing_*<br/>auto_resume=false"]
        PAUSE_GATE --> TAKEOVER["底盘切换HOLD<br/>等待人工重新进入AUTO_CLEANING"]
        PAUSE_MANUAL --> TAKEOVER
        PAUSE_MANUAL2 --> TAKEOVER
        PAUSE_MANUAL3 --> TAKEOVER
        TAKEOVER --> REENTER_AUTO["重新进入AUTO_CLEANING"] --> RESUME_WP["从当前航点重新进入WAYPOINT_MOVE"]
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
        CHECK_MODE -- 是 --> MANUAL_PAUSE{"人工介入暂停?"}
        MANUAL_PAUSE -- 是 --> STOP_HOLD["清空生成器<br/>发布零速度，等待HOLD/重新AUTO"]
        MANUAL_PAUSE -- 否 --> DATA_GUARD["检查RTK数据超时、零角度和运行中航向异常"]
        DATA_GUARD --> QUALITY{"定位/定向Fixed且GGA有效?"}
        QUALITY -- 否 --> STOP_AUTO["自动恢复型PAUSE<br/>保持AUTO_CLEANING并发布零速度"]
        QUALITY -- 是 --> WAYPOINTS{"有航点数据?"}
        WAYPOINTS -- 否 --> ERROR_EXIT["报错并重置为IDLE"]
        WAYPOINTS -- 是 --> HEADING_GATE{"航向门控通过?"}
        HEADING_GATE -- 否 --> STOP_GATE["保持PAUSE/门控等待<br/>600s超时错误码64并转HOLD"]
        HEADING_GATE -- 是 --> ALIGN{"恢复后需要先对齐路径?"}
        ALIGN -- 是 --> WAYPOINT_CALIB["WAYPOINT_CALIB<br/>原地对齐后再启动巡迹"]
        ALIGN -- 否 --> CHECK_GENERATOR{"生成器存在且可运行?"}
        WAYPOINT_CALIB --> CHECK_GENERATOR
        CHECK_GENERATOR -- 否 --> CREATE_GEN["创建multi_waypoint_nav_generator<br/>resume: IDLE首次，其它从当前状态"]
        CREATE_GEN --> NEXT_STEP["next()获取速度"]
        CHECK_GENERATOR -- 是 --> NEXT_STEP
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
    CHECK_FORCE1 -- 是 --> BEARING1["方位角直行: path_dir=bearing→target<br/>抑制固定路径横向项"]
    BEARING1 --> CHECK_DIST1
    CHECK_FORCE1 -- 否 --> CHECK_HEADING1{"航向误差>15° 连续5帧?"}
    CHECK_HEADING1 -- 是 --> INC_COUNT1["waypoint_recalib_count++"]
    INC_COUNT1 --> CHECK_COUNT1{"同航点校准≥2次?"}
    CHECK_COUNT1 -- 是 --> SET_FORCE1["force_bearing_mode=True<br/>打滑/位置偏移，改用方位角兜底"]
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
    CHECK_FORCE -- 是 --> BEARING2["方位角直行: path_dir=bearing→target<br/>抑制固定路径横向项"]
    BEARING2 --> CHECK_DIST2
    CHECK_FORCE -- 否 --> CHECK_HEADING{"航向误差>15° 连续5帧?"}
    CHECK_HEADING -- 是 --> INC_COUNT["waypoint_recalib_count++"]
    INC_COUNT --> CHECK_COUNT{"同航点校准≥2次?"}
    CHECK_COUNT -- 是 --> SET_FORCE["force_bearing_mode=True<br/>打滑/位置偏移，改用方位角兜底"]
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
        STEERING --> TOTAL["total_steering = st_corr - heading_error"]
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

- **局部重校准**: 导航行驶中航向误差超过 15° 连续 5 帧（0.5s），先在当前航点触发重校准；同一航点反复校准后进入 `force_bearing_mode` 兜底。
- **force bearing**: 使用当前位置到目标的实时方位角，关闭固定路径段的横向项；误差超过 15° 仍先原地对准。极限环或持续背离时发布人工介入暂停并使用错误码 128。
- **全局航向异常**: 航向异常连续超过 15s 时设置错误码 8，进入 `heading_timeout` 自动恢复型 PAUSE；恢复判据是航向数据恢复正常，不等同于 AUTO 航向稳定门控通过。
- **RTK 数据超时**: `/wtrtk_data` 超过 1s 未更新，或 `angle_x/angle_y` 持续为 0 超过 1s，设置错误码 8 并停车；数据恢复且质量仍为双Fixed时回到原导航阶段，若仍非Fixed则转入 `rtk_not_fixed` 并重新经过航向门控。
- **模式切入清理**: 进入 `AUTO_CLEANING` 或切换路径时清零 `heading_abnormal_start_time` 和 `heading_timed_out`，防止旧任务污染。
- 单帧跳变只累计一次，下一帧恢复到阈值内会清零计数，避免 RTK/IMU 瞬时抖动误判。
- 异常重校准完成后不推进航点索引，继续前往当前航点；正常到达航点后的校准才推进到下一个航点。

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
| RTK_DATA_TIMEOUT | 1.0 s | RTK 数据断流超时停车 |
| RTK_ZERO_ANGLE_TIMEOUT | 1.0 s | angle_x/angle_y持续为0的异常超时 |
| HEADING_ABNORMAL_TIMEOUT | 15.0 s | 运行中航向误差持续超时，错误码8 |
| HEADING_STABILITY_WINDOW | 5.0 s | AUTO门控短窗，波动≤1° |
| HEADING_STABILITY_RANGE | 1.0° | 5秒短窗最大允许波动 |
| HEADING_STABILITY_SETTLE_WINDOW | 30.0 s | AUTO门控收敛窗口 |
| HEADING_STABILITY_SETTLE_RANGE | 2.0° | 30秒收敛窗口最大允许跨度 |
| HEADING_QUALITY_GAP_MAX | 3.0 s | Float/非Fixed最长桥接间隔；超时清空窗口 |
| HEADING_FIXED_CONFIRM_WINDOW | 1.0 s | 短时恢复双Fixed后的确认停车时间 |
| AUTO_HEADING_GATE_TIMEOUT | 600.0 s | AUTO航向门控超时，错误码64并转HOLD |
| UNLOADING_MIN_BATTERY | 90% | START出仓最低电量 |
| UNLOADING_SETTLE_DURATION | 2.0 s | 出仓完成后HOLD结算时间 |
| force_bearing_mode | ≥2次 | 同航点重校准后启用实时方位角兜底 |
| FORCE_BEARING_REALIGN_THRESHOLD | 15.0° | force_bearing_mode 下航向偏差超此值触发原地重新对准 |

### 当前运行时故障码

以下为导航/任务故障码，按位或组合发布；底盘另有电机故障码 `1` 和激光超时码 `2`。

| 故障码 | 当前含义 | 处理方式 |
|---:|---|---|
| 4 | RTK 定位或定向不是 Fixed，或 GGA 数据无效 | 自动停车并保持 `AUTO_CLEANING`，等待质量恢复后重新通过航向门控 |
| 8 | `/wtrtk_data` 断流超过 1s、`angle_x/angle_y` 持续为 0 超过 1s，或运行中航向误差超过 15° 持续 15s | 自动停车暂停；RTK/航向数据恢复后自动恢复，恢复判据不等同于航向稳定 |
| 16 | START 时电量低于 90% | 拒绝出仓，保持当前状态 |
| 32 | 进仓导航/激光对位流程超时或前置条件失败 | 停车并进入 HOLD，检查位置和进仓状态 |
| 64 | AUTO 航向门控等待 600s 仍未通过 | 设置错误码并进入人工介入 HOLD；重新进入 AUTO 后重新采集稳定窗口 |
| 128 | 航向校准卡滞/超时、初始对准失败、边界 GPS 撤退超时、force bearing 极限环或持续背离 | 停车并进入人工介入 HOLD；人工处理后从当前航点续扫 |

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
