RTK循迹测试 Ros2

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
    进入原遥控解析模式(未测试)

4、 实时导航测试
    注释wtrtk_parse_txt，启动launch中的wtrtk_serial_driver
    进入实时导航，提前进行步骤1确定路径


Ros1 Old Version
1
先单独启动launch运行cleaning_path_planner，生成轨迹，并保存。
2
launch运行MotorControlNode修改"~rtk_path_file"为cleaning_path目录下刚生成的文件
RTK可通过launch中录制的RTK消息解析调试，实地测试使用RTK实时消息解析


通过之前RTK采样数据转换为导航路线测试导航运行情况，流程可以跑通，偏角部分还有问题。
解析遥控器解码的数据并转换为运动控制,待测试。
电机控制订阅/current_speed,并执行
步骤：
启动launch文件后，发布start消息：
rostopic pub /rtk_nav/start std_msgs/String "data: 'start'"
订阅motor/state
motor/current_speed

