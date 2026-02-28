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
### 出仓结束后返回:
```mermaid
graph TD
    A[RTK多点导航全部完成] --> B{订阅出仓点GPS}
    B --> D[添加最后一个文件末尾索引
    使用multi_waypoint_nav_generator]
    D -->F
```
## 激光对正原逻辑
### 激光对正旧：
激光距离<3000mm:
    if 差值>1000mm: 执行「偏转→后退2s→反向偏转检查」流程
        if 差值<10mm: 直行 下一步
    else 差值≤1000mm → 中等速度纠偏直行
        if 激光距离<530mm → 最终对位判断
            if 差值<2: 结束
        if 激光距离<1000mm
            差值≥5mm → 低速旋转对准
            差值<5mm 直行 下一步
        if 中距离（≥1000mm），判断差值是否<100mm
            差值>100mm：中速旋转对准
            差值<100mm：直行 下一步
激光距离>3000mm:
    旋转寻找

### 激光对正修改版
```mermaid
graph TD
    A[进仓LOADING阶段] --> B{激光距离<3000mm?}
    %% 激光超出有效区分支
    B -- 否 --> C[旋转寻找目标（低速）]
    C --> D{寻找超时>10s?}
    D -- 是 --> E[后退重新定位（base/5）]
    D -- 否 --> C
    E --> B
    %% 激光有效区分支
    B -- 是 --> F{差值>1000mm?}
    
    %% 大幅差值闭环调整（粗调）
    F -- 是 --> G[调整阶段：偏转对准（3s，base/4）]
    G --> H{偏转超时?}
    H -- 是 --> I[调整阶段：后退2s（base/5）]
    I --> J{后退完成?}
    J -- 是 --> K[重新读取激光值]
    K --> L[调整阶段：反向偏转检查（2s，base/6）]
    L --> M{检查超时?}
    M -- 是 --> N{差值仍>1000mm?}
    N -- 是 --> G
    N -- 否 --> O[进入中等差值纠偏]
    M -- 否 --> L
    J -- 否 --> I
    H -- 否 --> G
    
    %% 中等差值分级精调
    F -- 否 --> O
    O --> P{激光距离≥1000mm?}
    %% 中距离（≥1000mm）
    P -- 是 --> Q{差值>100mm?}
    Q -- 是 --> R[中速旋转对准（base/3）]
    Q -- 否 --> S[直行（base/3）→下一步]
    R --> O
    S --> O
    %% 近距离（<1000mm）
    P -- 否 --> T{激光距离≥330mm?}
    T -- 是 --> U{差值≥5mm?}
    U -- 是 --> V[低速旋转对准（base/4）]
    U -- 否 --> W[直行（base/4）→下一步]
    V --> O
    W --> O
    %% 极近距离（<330mm）终调
    T -- 否 --> X{差值<1mm?}
    X -- 否 --> Y[最终微调（base/20）+重置稳定计数]
    Y --> O
    X -- 是 --> Z{连续3次达标?}
    Z -- 否 --> AA[小幅直行（base/10）+稳定计数+1]
    AA --> O
    Z -- 是 --> AB[停止→进仓完成（COMPLETE）]
```