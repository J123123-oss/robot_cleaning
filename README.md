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

```mermaid
graph TD
    A[开机使能 START] --> B[遥控器]
    A -->a[键盘]
    a --> C
    A -->b[MQTT]
    b --> C
    B --> C[出仓 UNLOADING]
    C --> D[定时直行]
    D --> d
    d[转向] --> E[RTK起始点]
    E{角度满足?}
    E -- 是 --> F[切换控制模式]
    c[记录IMU朝向]-->d
    G --> e[重置IMU]
    F --> G[RTK导航开始]
    E -- 否 --> W[超时不满足]
    W --> d
    e --> H[区域1]
    H[区域1] --> I{超声波检测边缘触发?}
    I -- 是 --> J[退回组件] --> K[到达下一个点]
    I -- 否 --> L{角度小于阈值}
    L -->|是/否 都实时检测超声波| I{超声波检测边缘触发?}
    L -- 是 --> f{距离小于阈值}
    f -->|是/否 都实时检测超声波| I{超声波检测边缘触发?}
    f -- 是 --> K[到达下一个点]
    K --> M[......] --> N[区域2]
    N --> O[......]
    O --> P[RTK返回中转点]
    P --> Q[区域n-1]
    Q --> R[区域n]
    R --> S[RTK结束]
    S --> T[进仓LOADING]
    T --> U[转向，后退进仓]
    U --> V[停止STOP]

```
出仓结束后返回:
```mermaid
graph TD
    A[RTK多点导航全部完成] --> B{订阅出仓点GPS}
    B --> D[添加最后一个文件末尾索引
    使用multi_waypoint_nav_generator]
    D -->F
```
ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32088536, longitude: 120.06717012, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1
ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32009074, longitude: 120.06899750, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1
ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32005031, longitude: 120.06719501, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1
序号,经度,纬度,航向角(度)
1,120.07064157,30.32105564,88.51
2,120.07065098,30.32105585,338.51

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32105564, longitude: 120.07064157, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32105585, longitude: 120.07065098, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1


120.07151235,30.32131103,223.65
2,120.07142616,30.32123303,313.93
120.07141285, 30.3212441
4,120.07149904,30.32132210,43.65

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32131103, longitude: 120.07151235, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1


ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32123303, longitude: 120.07142616, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.3212441, longitude: 120.07141285, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1


ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32132210, longitude: 120.07149904, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

1,120.07151235,30.32131103,223.65
2,120.07142616,30.32123303,313.93
3,120.07142350,30.32123525,43.65
4,120.07150969,30.32131325,313.93
5,120.07150703,30.32131546,223.65
6,120.07142084,30.32123746,313.93
7,120.07141817,30.32123967,43.65
8,120.07150437,30.32131767,313.93
9,120.07150170,30.32131988,223.65
10,120.07141551,30.32124189,313.93
11,120.07141285,30.32124410,43.65
12,120.07149904,30.32132210,43.65



































ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.3208108, longitude: 120.07117109, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32073328, longitude: 120.07119762, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32072987, longitude: 120.07118433, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32080767, longitude: 120.07115780, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32080426, longitude: 120.07114451, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32072646, longitude: 120.07117104, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32072305, longitude: 120.07115776, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32080085, longitude: 120.07113123, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

1,120.,30.,73.44
2,120.,30.,163.60
3,120.,30.,253.44
4,120.,30.,163.60
5,120.,30.,73.44
6,120.,30.,163.60
7,120.,30.,253.44
8,120.,30.,163.60
9,120.,30.,73.44
10,120.,30.,73.44
ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32084408, longitude: 120.07111649, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32086709, longitude: 120.07120617, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32085840, longitude: 120.07120912, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32083543, longitude: 120.07111935, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32082679, longitude: 120.07112238, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32084980, longitude: 120.07121207, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32084116, longitude: 120.07121501, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32081814, longitude: 120.07112533, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32080950, longitude: 120.07112828, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: {status: 4, service: 0}, latitude: 30.32083251, longitude: 120.07121796, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}" -r 1