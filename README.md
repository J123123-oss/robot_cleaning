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

### 主状态机
```mermaid
graph TD
    subgraph 外部事件中断
        GPS_NONFIX["GPS非固定解"] --> PAUSE["PAUSE暂停导航"]
        PAUSE --> GPS_FIX["GPS恢复固定解"] --> RESUME["恢复pre_pause_state"]
        HOLD["电机状态HOLD"] --> STOP_NAV["强制停止导航"]
        MODE_SWITCH["控制模式切换"] --> STOP_NAV
        ROUTE_CHANGE["路径切换/rtk/route_change"] --> RELOAD["重新加载航点文件"]
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
    subgraph 导航状态机 
        IDLE["IDLE"] --> INIT_MOVE["INITIAL_MOVE"]
        INIT_MOVE --> MOVE_FIRST["初始点->第一个航点"]
        MOVE_FIRST --> STANLEY1["Stanley控制器直线行驶"]
        STANLEY1 --> CHECK_DIST1{"距离<0.1m 连续5帧?"}
        CHECK_DIST1 -- 否 --> STANLEY1
        CHECK_DIST1 -- 是 --> CALIB1["WAYPOINT_CALIB 航向校准"]
        CALIB1 --> TURN1["原地旋转至目标heading"]
        TURN1 --> CHECK_ANGLE1{"角度误差<1度?"}
        CHECK_ANGLE1 -- 否 --> TURN1
        CHECK_ANGLE1 -- 是 --> NEXT_WP["current_waypoint_idx++"]

        NEXT_WP --> WP_MOVE["WAYPOINT_MOVE"]
        WP_MOVE --> GET_TARGET["获取目标航点"]
        GET_TARGET --> CHECK_LAST{"到达最后一个航点?"}
        CHECK_LAST -- 是 --> SWITCH_FILE["自动切换下一路径文件"]
        SWITCH_FILE --> CHECK_NEXT{"有下一文件?"}
        CHECK_NEXT -- 是 --> RESET_IDX2["重置索引=0, 返回新文件首航点"]
        RESET_IDX2 --> WP_MOVE
        CHECK_NEXT -- 否 --> CHECK_LOADING{"有出仓点?"}
        CHECK_LOADING -- 是 --> APPEND_LOADING["追加出仓点到航点列表"]
        APPEND_LOADING --> WP_MOVE
        CHECK_LOADING -- 否 --> COMPLETED["COMPLETED 导航完成"]

        CHECK_LAST -- 否 --> STANLEY2["Stanley控制器直线行驶"]
        STANLEY2 --> CHECK_DIST2{"距离<0.1m?"}
        CHECK_DIST2 -- 否 --> CHECK_LOW{"距离<1.3m?"}
        CHECK_LOW -- 是 --> SLOW_DOWN["线性减速 speed_scale=0.2~0.7"]
        SLOW_DOWN --> STANLEY2
        CHECK_LOW -- 否 --> STANLEY2
        CHECK_DIST2 -- 是 --> CALIB2["WAYPOINT_CALIB 航向校准"]
        CALIB2 --> TURN2["原地旋转至目标heading"]
        TURN2 --> CHECK_ANGLE2{"角度误差<1度?"}
        CHECK_ANGLE2 -- 否 --> TURN2
        CHECK_ANGLE2 -- 是 --> NEXT_WP
    end
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
        LATERAL --> SIGN["正=偏左 负=偏右"]
    end

    STEP1 --> STEP2
    subgraph STEP2["计算航向误差"]
        HEADING_ERR["heading_error = path_direction - current_heading"]
        HEADING_ERR --> NORMALIZE["归一化到-180度~180度"]
    end

    STEP2 --> STEP3
    subgraph STEP3["Stanley控制律"]
        K["K = STANLEY_K = 2.0"] --> STEERING
        V["velocity"] --> STEERING
        LE["lateral_error"] --> STEERING
        HE["heading_error"] --> STEERING
        STEERING["total_steering = heading_error + atan(K * lateral_error / velocity)"]
        STEERING --> CLAMP["clamp -45度~45度"]
    end

    STEP3 --> STEP4
    subgraph STEP4["差速分配"]
        FACTOR["steering_factor = clamped / 45度"] --> DIFF["speed_diff = factor * MAX_CORRECTION"]
        DIFF --> LEFT["left = -velocity - speed_diff"]
        DIFF --> RIGHT["right = velocity + speed_diff"]
    end

    STEP4 --> OUTPUT["输出: left_speed, right_speed"]
```

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
```mermaid
graph LR
    P1[RTK_WAYPOINT_TOLERANCE=0.1m] --> P1D[到达航点距离阈值]
    P2[INITIAL_MOVE_TOLERANCE=0.1m] --> P2D[初始移动到达阈值]
    P3[LINEAR_SPEED_BASE=10.0] --> P3D[基础行驶速度]
    P4[LOW_DISTANCE=1.3m] --> P4D[减速触发距离]
    P5[STANLEY_K=2.0] --> P5D[横向误差增益]
    P6[MAX_CORRECTION=1.6] --> P6D[最大差速修正]
    P7[RTK_HEADING_TOLERANCE=1.0°] --> P7D[航向校准精度]
    P8[GPS_SMOOTH_WINDOW=5帧] --> P8D[GPS滑动平均窗口]
```