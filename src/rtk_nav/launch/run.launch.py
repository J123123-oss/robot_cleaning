from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """构建并返回 RTK 导航系统的完整 ROS 2 launch 描述。"""
    # 全局路径配置
    rtk_path_file = '/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/test.txt'
    # 固定进仓RTK航点：[经度, 纬度, 航向角]。现场标定后填写，避免使用出仓漂移后的实时点。
    loading_gps = [110.64741424789473, 35.60594097811998, -90.0]

    # 声明robot_ID参数，默认值可自定义（比如"GF-HZ-TEST"）
    declare_robot_id_arg = DeclareLaunchArgument(
        "robot_ID",  # 参数名 和ROS1的 arg name="robot_ID" 对应
        default_value=TextSubstitution(text="HANGZHOU_1"),  # 默认值
        description="机器人唯一标识ID，用于拼接MQTT主题"
    )

    # 滚刷配置：0表示未安装滚刷，1只控制3号，2控制3、4号。
    declare_brush_motor_count_arg = DeclareLaunchArgument(
        "brush_motor_count",
        default_value=TextSubstitution(text="0"),
        description="滚刷电机数量，只支持0、1或2",
    )
    declare_brush_direction_mode_arg = DeclareLaunchArgument(
        "brush_direction_mode",
        default_value=TextSubstitution(text="same"),
        description="双滚刷方向模式：same或opposite",
    )
    # AIMotor 速度模式参数：加减速度单位为电机侧 Pul/s^2，方向取 1 或 -1。
    declare_profile_acceleration_arg = DeclareLaunchArgument(
        "profile_acceleration",
        default_value=TextSubstitution(text="60000"),
        description="AIMotor 速度模式加速度，单位 Pul/s^2",
    )
    declare_profile_deceleration_arg = DeclareLaunchArgument(
        "profile_deceleration",
        default_value=TextSubstitution(text="445000"),
        description="AIMotor 速度模式减速度，单位 Pul/s^2；启动时通过0x6084回读确认",
    )
    declare_motor_direction_args = tuple(
        DeclareLaunchArgument(
            f"motor_direction_{motor_id}",
            default_value=TextSubstitution(
                text="-1" if motor_id in (1, 2, 4) else "1"
            ),
            description=(
                f"AIMotor 电机 {motor_id} 速度指令方向，"
                f"默认{'反转' if motor_id in (1, 2, 4) else '正转'}，"
                "1 正常，-1 反向"
            ),
        )
        for motor_id in range(1, 5)
    )

    declare_imu_odom_fallback_arg = DeclareLaunchArgument(
        "enable_imu_odom_fallback",
        default_value=TextSubstitution(text="false"),
        description=(
            "Enable short RTK position-loss IMU/odometry hold on an eligible "
            "straight waypoint segment"
        ),
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
    # 以下纠偏参数由 rtk_nav 消费：速度和纠偏量统一使用轮子减速器输出轴 r/min，
    # ratio 为无量纲比例。
    # RTK/Stanley 纠偏比例，1.0 表示使用完整纠偏量。
    declare_rtk_correction_ratio_arg = DeclareLaunchArgument(
        "rtk_correction_ratio",
        default_value=TextSubstitution(text="1.0"),
        description="RTK/Stanley correction ratio applied to wheel output-shaft r/min",
    )
    # RTK/Stanley 纠偏速度上限（轮子输出轴 r/min）。
    declare_rtk_max_correction_arg = DeclareLaunchArgument(
        "rtk_max_correction",
        default_value=TextSubstitution(text="10.026761"),
        description="Maximum RTK correction: 15% of wheel output-shaft speed",
    )
    declare_publish_debug_images_arg = DeclareLaunchArgument(
        "publish_debug_images",
        default_value=TextSubstitution(text="true"),
        description="Publish grid-line debug images",
    )
    declare_always_show_axis_debug_arg = DeclareLaunchArgument(
        "always_show_axis_debug",
        default_value=TextSubstitution(text="true"),
        description=(
            "Show full axis, line-count, and RTK diagnostics in the debug image"
        ),
    )
    declare_coarse_line_min_length_arg = DeclareLaunchArgument(
        "coarse_line_min_length_px",
        default_value=TextSubstitution(text="30.0"),
        description="Minimum coarse tracking-line Hough segment length in pixels",
    )
    declare_coarse_line_min_width_arg = DeclareLaunchArgument(
        "coarse_line_min_width_px",
        default_value=TextSubstitution(text="4.0"),
        description="Minimum coarse tracking-line width metric in pixels",
    )
    declare_coarse_line_min_support_arg = DeclareLaunchArgument(
        "coarse_line_min_support",
        default_value=TextSubstitution(text="0.3"),
        description="Minimum white-mask support for a coarse tracking line",
    )
    declare_coarse_line_merge_gap_arg = DeclareLaunchArgument(
        "coarse_line_merge_gap_px",
        default_value=TextSubstitution(text="30.0"),
        description="Maximum normal gap for merging coarse line edges",
    )
    declare_coarse_line_gap_fill_arg = DeclareLaunchArgument(
        "coarse_line_gap_fill_px",
        default_value=TextSubstitution(text="80.0"),
        description="Maximum along-line reflection gap to bridge for coarse tracking",
    )
    declare_coarse_line_bridge_normal_gap_arg = DeclareLaunchArgument(
        "coarse_line_bridge_normal_gap_px",
        default_value=TextSubstitution(text="10.0"),
        description="Maximum normal gap between coarse line segments to bridge",
    )
    declare_coarse_line_bridge_angle_arg = DeclareLaunchArgument(
        "coarse_line_bridge_angle_tolerance_deg",
        default_value=TextSubstitution(text="3.0"),
        description="Maximum direction difference for coarse line bridging",
    )
    declare_white_line_value_threshold_arg = DeclareLaunchArgument(
        "white_line_value_threshold",
        default_value=TextSubstitution(text="170.0"),
        description="Minimum HSV value for a white grid line",
    )
    declare_white_line_saturation_max_arg = DeclareLaunchArgument(
        "white_line_saturation_max",
        default_value=TextSubstitution(text="100.0"),
        description="Maximum HSV saturation for a white grid line",
    )
    declare_angle_average_center_band_ratio_arg = DeclareLaunchArgument(
        "angle_average_center_band_ratio",
        default_value=TextSubstitution(text="0.8"),
        description="Image center-band ratio used for fine-line angle averaging",
    )
    declare_reference_axis_offset_arg = DeclareLaunchArgument(
        "reference_axis_offset_px",
        default_value=TextSubstitution(text="0.0"),
        description="Fixed line measurement section offset along path axis",
    )

    # 定义电机控制节点
    motor_control_node = Node(
        package='motor_control',  # 对应 ROS1 的 pkg，包名不变
        executable='motor_control',  # 对应 ROS1 的 type，可执行文件名称
        # name='motor_control',  # 对应 ROS1 的 name，节点名称
        output='screen',  # 对应 ROS1 的 output，输出到终端
        arguments=['--ros-args', '--log-level', 'can_motor_driver:=info'],
        parameters=[
            # 对应 ROS1 的 param，使用键值对形式配置参数
            {'rtk_path_file': rtk_path_file},
            {'loading_gps': loading_gps},
            {
                'brush_motor_count': ParameterValue(
                    LaunchConfiguration('brush_motor_count'), value_type=int
                ),
                'brush_direction_mode': LaunchConfiguration(
                    'brush_direction_mode'
                ),
                'profile_acceleration': ParameterValue(
                    LaunchConfiguration('profile_acceleration'), value_type=int
                ),
                'profile_deceleration': ParameterValue(
                    LaunchConfiguration('profile_deceleration'), value_type=int
                ),
                **{
                    f'motor_direction_{motor_id}': ParameterValue(
                        LaunchConfiguration(f'motor_direction_{motor_id}'),
                        value_type=int,
                    )
                    for motor_id in range(1, 5)
                },
            },
        ]
    )
    sensors_485_node = Node(
        package='motor_control',  # 对应 ROS1 的 pkg，包名不变
        executable='sensors_485',  # 对应 ROS1 的 type，可执行文件名称
        # name='sensors_485',  # 对应 ROS1 的 name，节点名称
        # output='screen',  # 对应 ROS1 的 output，输出到终端
        parameters=[
            {'gpio_chip': '/dev/gpiochip6'},
            {'gpio_lines': [9, 8, 7, 6]},
            {'poll_interval': 0.01},
            {'gpio_use_sudo': False},
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
            {'slave_addr': 2},
            {'battery_addr': 1},
            {'battery_baud_rate': 9600},
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
                'enable_imu_odom_fallback': ParameterValue(
                    LaunchConfiguration("enable_imu_odom_fallback"),
                    value_type=bool,
                ),
                'stanley_k_path': ParameterValue(
                    LaunchConfiguration("stanley_k_path"), value_type=float
                ),
                'stanley_k_near_target': ParameterValue(
                    LaunchConfiguration("stanley_k_near_target"), value_type=float
                ),
                'rtk_correction_ratio': ParameterValue(
                    LaunchConfiguration("rtk_correction_ratio"), value_type=float
                ),
                'rtk_max_correction': ParameterValue(
                    LaunchConfiguration("rtk_max_correction"), value_type=float
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
            # {'file_path': '/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/返回.txt'}
            # 原注释的其他参数可取消注释添加
            # {'file_path': '/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
            {'file_path': '/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
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
            {'baud': 460800}
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
    ld.add_action(declare_brush_motor_count_arg)
    ld.add_action(declare_brush_direction_mode_arg)
    ld.add_action(declare_profile_acceleration_arg)
    ld.add_action(declare_profile_deceleration_arg)
    for declare_motor_direction_arg in declare_motor_direction_args:
        ld.add_action(declare_motor_direction_arg)
    ld.add_action(declare_imu_odom_fallback_arg)
    ld.add_action(declare_stanley_k_path_arg)
    ld.add_action(declare_stanley_k_near_target_arg)
    ld.add_action(declare_rtk_correction_ratio_arg)
    ld.add_action(declare_rtk_max_correction_arg)
    ld.add_action(declare_publish_debug_images_arg)
    ld.add_action(declare_always_show_axis_debug_arg)
    ld.add_action(declare_coarse_line_min_length_arg)
    ld.add_action(declare_coarse_line_min_width_arg)
    ld.add_action(declare_coarse_line_min_support_arg)
    ld.add_action(declare_coarse_line_merge_gap_arg)
    ld.add_action(declare_coarse_line_gap_fill_arg)
    ld.add_action(declare_coarse_line_bridge_normal_gap_arg)
    ld.add_action(declare_coarse_line_bridge_angle_arg)
    ld.add_action(declare_white_line_value_threshold_arg)
    ld.add_action(declare_white_line_saturation_max_arg)
    ld.add_action(declare_angle_average_center_band_ratio_arg)
    ld.add_action(declare_reference_axis_offset_arg)
    ld.add_action(mqtt_ros_bridge_node)
    ld.add_action(motor_control_node)

    ld.add_action(sensors_485_node)
    # ld.add_action(laser_node)
    ld.add_action(charging_node)
    # ld.add_action(wtrtk_parse_txt_node)
    ld.add_action(rtk_navigator)
    # # 若需要启用注释的节点，取消以下对应行的注释
    ld.add_action(wtrtk_serial_driver_node)

    return ld
