from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 全局路径配置
    rtk_path_file = '/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/001-E1-E8.txt'
    # 固定进仓RTK航点：[经度, 纬度, 航向角]。现场标定后填写，避免使用出仓漂移后的实时点。
    loading_gps = [110.64741424789473, 35.60594097811998, -90.0]

    # 声明robot_ID参数，默认值可自定义（比如"GF-HZ-TEST"）
    declare_robot_id_arg = DeclareLaunchArgument(
        "robot_ID",  # 参数名 和ROS1的 arg name="robot_ID" 对应
        default_value=TextSubstitution(text="HEJIN_Huaxinyuan"),  # 默认值
        description="机器人唯一标识ID，用于拼接MQTT主题"
    )

    declare_visual_correction_arg = DeclareLaunchArgument(
        "enable_visual_correction",
        default_value=TextSubstitution(text="false"),
        description="Enable visual correction; disabled by default",
    )

    declare_stanley_k_path_arg = DeclareLaunchArgument(
        "stanley_k_path",
        default_value=TextSubstitution(text="0.45"),
        description="Stanley lateral gain for normal path tracking",
    )
    declare_stanley_k_near_target_arg = DeclareLaunchArgument(
        "stanley_k_near_target",
        default_value=TextSubstitution(text="0.42"),
        description="Stanley lateral gain within 1.3 m of the target",
    )
    declare_visual_heading_gain_arg = DeclareLaunchArgument(
        "visual_heading_gain",
        default_value=TextSubstitution(text="0.2"),
        description="Visual heading correction gain",
    )
    declare_visual_lateral_gain_arg = DeclareLaunchArgument(
        "visual_lateral_gain",
        default_value=TextSubstitution(text="10.0"),
        description="Visual lateral correction gain",
    )
    declare_visual_max_steering_arg = DeclareLaunchArgument(
        "visual_max_steering_deg",
        default_value=TextSubstitution(text="3.0"),
        description="Maximum visual correction steering angle in degrees",
    )
    declare_visual_confidence_threshold_arg = DeclareLaunchArgument(
        "visual_confidence_threshold",
        default_value=TextSubstitution(text="0.9"),
        description="Minimum visual confidence for correction",
    )
    declare_visual_timeout_arg = DeclareLaunchArgument(
        "visual_timeout_sec",
        default_value=TextSubstitution(text="0.5"),
        description="Visual sample timeout in seconds",
    )
    declare_camera_width_arg = DeclareLaunchArgument(
        "camera_width",
        default_value=TextSubstitution(text="360"),
        description="Camera output width",
    )
    declare_camera_height_arg = DeclareLaunchArgument(
        "camera_height",
        default_value=TextSubstitution(text="640"),
        description="Camera output height",
    )
    declare_camera_fps_arg = DeclareLaunchArgument(
        "camera_fps",
        default_value=TextSubstitution(text="30"),
        description="Camera capture and publish FPS",
    )
    declare_camera_image_path_arg = DeclareLaunchArgument(
        "camera_image_path",
        default_value=TextSubstitution(text=""),
        description="Optional static image path; empty reads /dev/video0",
    )
    declare_camera_jpeg_quality_arg = DeclareLaunchArgument(
        "camera_jpeg_quality",
        default_value=TextSubstitution(text="80"),
        description="JPEG quality for the compressed camera topic",
    )
    declare_detection_fps_arg = DeclareLaunchArgument(
        "detection_fps",
        default_value=TextSubstitution(text="30.0"),
        description="Grid line detection timer FPS",
    )
    declare_publish_debug_images_arg = DeclareLaunchArgument(
        "publish_debug_images",
        default_value=TextSubstitution(text="false"),
        description="Publish grid-line debug images",
    )
    declare_debug_image_fps_arg = DeclareLaunchArgument(
        "debug_image_fps",
        default_value=TextSubstitution(text="1.0"),
        description="Maximum debug image publish FPS",
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
            {'port': '/dev/ttyS1'},
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

    # 充电485节点
    charging_node = Node(
        package='motor_control',
        executable='charging',
        name='charging_485_node',
        parameters=[
            {'serial_port': '/dev/battery_charging'},    # 替换为实际串口设备路径
            {'slave_addr': 1},
            {'battery_addr': 11},
            {'timeout': 0.5}
        ]
    )
    rtk_navigator = Node(
        package='rtk_nav',
        executable='rtk_nav',
        name='rtk_nav',
        # output='screen',
        parameters=[
            {'rtk_path_file': rtk_path_file},
            {'loading_gps': loading_gps},
            {
                'enable_visual_correction': ParameterValue(
                    LaunchConfiguration("enable_visual_correction"),
                    value_type=bool,
                ),
                'stanley_k_path': ParameterValue(
                    LaunchConfiguration("stanley_k_path"), value_type=float
                ),
                'stanley_k_near_target': ParameterValue(
                    LaunchConfiguration("stanley_k_near_target"), value_type=float
                ),
                'visual_heading_gain': ParameterValue(
                    LaunchConfiguration("visual_heading_gain"), value_type=float
                ),
                'visual_lateral_gain': ParameterValue(
                    LaunchConfiguration("visual_lateral_gain"), value_type=float
                ),
                'visual_max_steering_deg': ParameterValue(
                    LaunchConfiguration("visual_max_steering_deg"), value_type=float
                ),
                'visual_confidence_threshold': ParameterValue(
                    LaunchConfiguration("visual_confidence_threshold"), value_type=float
                ),
                'visual_timeout_sec': ParameterValue(
                    LaunchConfiguration("visual_timeout_sec"), value_type=float
                ),
            },
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
            # {'port': '/dev/WTRTK'},
            {'port': '/dev/ttyS2'},
            {'baud': 230400}
        ]
    )

    line_detector_node = Node(
        package='rtk_nav',
        executable='line_detector_node',
        name='grid_line_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visual_correction')),
        parameters=[
            {
                'enable_visual_correction': ParameterValue(
                    LaunchConfiguration('enable_visual_correction'),
                    value_type=bool,
                ),
                'detection_fps': ParameterValue(
                    LaunchConfiguration('detection_fps'), value_type=float
                ),
                'publish_debug_images': ParameterValue(
                    LaunchConfiguration('publish_debug_images'), value_type=bool
                ),
                'debug_image_fps': ParameterValue(
                    LaunchConfiguration('debug_image_fps'), value_type=float
                ),
            },
        ],
    )

    camera_publisher_node = Node(
        package='rtk_nav',
        executable='camera_publisher_node',
        name='camera_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visual_correction')),
        parameters=[
            {
                'width': ParameterValue(
                    LaunchConfiguration('camera_width'), value_type=int
                ),
                'height': ParameterValue(
                    LaunchConfiguration('camera_height'), value_type=int
                ),
                'fps': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=int
                ),
                'image_path': LaunchConfiguration('camera_image_path'),
                'jpeg_quality': ParameterValue(
                    LaunchConfiguration('camera_jpeg_quality'), value_type=int
                ),
            },
        ],
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
    ld.add_action(declare_visual_correction_arg)
    ld.add_action(declare_stanley_k_path_arg)
    ld.add_action(declare_stanley_k_near_target_arg)
    ld.add_action(declare_visual_heading_gain_arg)
    ld.add_action(declare_visual_lateral_gain_arg)
    ld.add_action(declare_visual_max_steering_arg)
    ld.add_action(declare_visual_confidence_threshold_arg)
    ld.add_action(declare_visual_timeout_arg)
    ld.add_action(declare_camera_width_arg)
    ld.add_action(declare_camera_height_arg)
    ld.add_action(declare_camera_fps_arg)
    ld.add_action(declare_camera_image_path_arg)
    ld.add_action(declare_camera_jpeg_quality_arg)
    ld.add_action(declare_detection_fps_arg)
    ld.add_action(declare_publish_debug_images_arg)
    ld.add_action(declare_debug_image_fps_arg)
    ld.add_action(mqtt_ros_bridge_node)
    ld.add_action(motor_control_node)

    ld.add_action(sensors_485_node)
    ld.add_action(laser_node)
    ld.add_action(charging_node)
    # ld.add_action(wtrtk_parse_txt_node)
    ld.add_action(rtk_navigator)
    # 若需要启用注释的节点，取消以下对应行的注释
    ld.add_action(wtrtk_serial_driver_node)
    ld.add_action(line_detector_node)
    ld.add_action(camera_publisher_node)

    return ld
