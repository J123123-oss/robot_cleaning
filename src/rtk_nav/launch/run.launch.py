from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 全局路径配置
    rtk_path_file = '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/065_20260708_154944.txt'
    # 固定进仓RTK航点：[经度, 纬度, 车头进仓航向角]。现场标定后填写，避免使用出仓漂移后的实时点。
    # loading_gps = [110.647415, 35.605940, 89.80]
    loading_gps = [120.07061963994207, 30.320991691985643, 172.52]

    # 声明robot_ID参数，默认值可自定义（比如"GF-HZ-TEST"）
    declare_robot_id_arg = DeclareLaunchArgument(
        "robot_ID",  # 参数名 和ROS1的 arg name="robot_ID" 对应
        default_value=TextSubstitution(text="HEJIN_Huaxinyuan_old"),  # 默认值
        description="机器人唯一标识ID，用于拼接MQTT主题"
    )

    # 定义电机控制节点
    motor_control_node = Node(
        package='motor_control',  # 对应 ROS1 的 pkg，包名不变
        executable='motor_control',  # 对应 ROS1 的 type，可执行文件名称
        # name='motor_control',  # 对应 ROS1 的 name，节点名称
        output='screen',  # 对应 ROS1 的 output，输出到终端
        parameters=[
            # 对应 ROS1 的 param，使用键值对形式配置参数
            {'rtk_path_file': rtk_path_file},
            {'loading_gps': loading_gps}
        ]
    )
    sensors_485_node = Node(
        package='motor_control',  # 对应 ROS1 的 pkg，包名不变
        executable='sensors_485',  # 对应 ROS1 的 type，可执行文件名称
        # name='sensors_485',  # 对应 ROS1 的 name，节点名称
        # output='screen',  # 对应 ROS1 的 output，输出到终端
        parameters=[
            {'port': '/dev/ttyS4'},
            {'baud': 9600}
        ]
    )
        # 配置激光节点
    laser_node = Node(
        package='motor_control',
        executable='laser_distance',
        name='laser_distance_node',
        parameters=[
            {'serial_port': '/dev/laser'},    # 替换为实际串口设备路径
            {'baud_rate': 115200}
        ]
    )

    # # 充电485节点
    # charging_node = Node(
    #     package='motor_control',
    #     executable='charging',
    #     name='charging_485_node',
    #     parameters=[
    #         {'serial_port': '/dev/battery_charging'},    # 替换为实际串口设备路径
    #         {'slave_addr': 1},
    #         {'battery_addr': 11},
    #         {'timeout': 0.5}
    #     ]
    # )

    # 摄像头遥控器解析节点（串口文本协议 [Web] angle → 差速速度指令）
    camera_remote_control_node = Node(
        package='motor_control',
        executable='camera_remote_control_node',
        name='camera_remote_control_node',
        parameters=[
            {'port': '/dev/camera_remote'},  # 替换为实际串口设备路径
            {'baudrate': 115200},
            {'max_speed': 5.0},              # 最大电机速度指令 (double)
            {'publish_rate': 0.05},          # 发布周期 秒 (double)
            {'angle_timeout': 2.0},         # 超时停车阈值 秒 (double)
            {'disable_dtr_rts': True},       # 禁用 DTR/RTS 避免 ESP32 复位 (bool)
            {'esp32_boot_wait': 0.5},        # ESP32 上电等待时间 秒 (double)
        ]
    )
    rtk_navigator = Node(
        package='rtk_nav',
        executable='rtk_nav',
        name='rtk_nav',
        # output='screen',
        parameters=[
            {'rtk_path_file': rtk_path_file},
            {'loading_gps': loading_gps}
        ]
    )
    # RTK录制的消息解析节点（原ROS1中未注释的节点）
    wtrtk_parse_txt_node = Node(
        package='rtk_nav',
        executable='wtrtk_parse_txt',
        name='wtrtk_parse_txt',
        output='screen',
        parameters=[
            # {'file_path': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/返回.txt'}
            # 原注释的其他参数可取消注释添加
            # {'file_path': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
            {'file_path': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
        ]
    )

    # RTK实时消息解析节点（原ROS1中注释的节点，保留注释结构）
    wtrtk_serial_driver_node = Node(
        package='rtk_nav',
        executable='wtrtk_serial_driver',
        name='wtrtk_serial_driver',
        output='screen',
        parameters=[
            {'port': '/dev/WTRTK'},
            # {'port': '/dev/ttyS3'},
            {'baud': 460800}
            # {'baud': 230400}
        ]
    )

    # ===================== 2. 配置MQTT桥接节点 (对应ROS1的 <node>) =====================
    mqtt_ros_bridge_node = Node(
        package="mqtt_ros2",  # ROS2功能包名（替换为你的实际包名）
        executable="mqtt_ros2_bridge",  # 节点可执行文件名（setup.py中配置的console_scripts名称）
        name="mqtt",  # 节点名称 和ROS1的 name="mqtt_ros_bridge" 对应
        output='log',  # 不输出终端 ✅,  # 日志输出到终端（ROS1的 output="screen"）
        arguments=['--ros-args', '--log-level', 'fatal'],  # 关闭所有日志
        # emulate_tty=True,  # 确保彩色日志、交互正常
        # 传递参数（对应ROS1的 <param>）
        parameters=[
            {
            "broker": "121.40.57.48",  # MQTT服务器地址
            "port": 1883,  # MQTT端口
            "user": "gf-mounted",  # MQTT用户名
            "password": "20230810",  # MQTT密码
            # 拼接参数：robot/$(arg robot_ID)/status → ROS2用LaunchConfiguration
            "topic_status": ["robot/", LaunchConfiguration("robot_ID"), "/status"],
            "topic_dock_status": ["dock/", LaunchConfiguration("robot_ID"), "/status"],
            "topic_cmd": ["robot/", LaunchConfiguration("robot_ID"), "/cmd"],
            "topic_command": ["robot/", LaunchConfiguration("robot_ID"), "/command"],
            "topic_result": ["robot/", LaunchConfiguration("robot_ID"), "/result"],
            "client_id": ["python-mqtt-client-a", LaunchConfiguration("robot_ID")]
            }
        ]
    )

    # 组装所有节点到 LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_robot_id_arg)
    ld.add_action(mqtt_ros_bridge_node)
    ld.add_action(motor_control_node)

    ld.add_action(sensors_485_node)
    ld.add_action(laser_node)
    ld.add_action(camera_remote_control_node)
    # ld.add_action(charging_node)
    # ld.add_action(wtrtk_parse_txt_node)
    ld.add_action(rtk_navigator)
    # 若需要启用注释的节点，取消以下对应行的注释
    ld.add_action(wtrtk_serial_driver_node)

    return ld
