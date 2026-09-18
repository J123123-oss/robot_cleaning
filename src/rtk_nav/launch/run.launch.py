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

    # 滚刷配置：1只控制3号滚刷，2控制3、4号滚刷；方向模式只影响4号滚刷。
    declare_brush_motor_count_arg = DeclareLaunchArgument(
        "brush_motor_count",
        default_value=TextSubstitution(text="1"),
        description="滚刷电机数量，只支持1或2",
    )
    declare_brush_direction_mode_arg = DeclareLaunchArgument(
        "brush_direction_mode",
        default_value=TextSubstitution(text="same"),
        description="双滚刷方向模式：same或opposite",
    )

    declare_visual_correction_arg = DeclareLaunchArgument(
        "enable_visual_correction",
        default_value=TextSubstitution(text="true"),
        description="Enable visual correction",
    )

    declare_bypass_path_context_gate_arg = DeclareLaunchArgument(
        "bypass_path_context_gate",
        default_value=TextSubstitution(text="true"),
        description="Bypass RTK path context gate for visual line testing",
    )
    declare_fallback_path_axis_arg = DeclareLaunchArgument(
        "fallback_path_axis_image_deg",
        default_value=TextSubstitution(text="-90.0"),
        description=(
            "Fallback motion axis in the image: 0 degrees right, "
            "90 degrees down"
        ),
    )
    declare_angle_reference_axis_arg = DeclareLaunchArgument(
        "angle_reference_axis_image_deg",
        default_value=TextSubstitution(text="-90.0"),
        description="Camera image reference axis for grid-line angle correction",
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
        description="Maximum RTK correction in wheel output-shaft r/min",
    )
    # 视觉纠偏比例，1.0 表示使用完整视觉纠偏量。
    declare_visual_correction_ratio_arg = DeclareLaunchArgument(
        "visual_correction_ratio",
        default_value=TextSubstitution(text="1.0"),
        description="Visual correction ratio applied to wheel output-shaft r/min",
    )
    # 视觉航向误差增益：输入为度，输出为轮速纠偏 r/min。
    declare_visual_heading_gain_arg = DeclareLaunchArgument(
        "visual_heading_gain",
        default_value=TextSubstitution(text="0.334225"),
        description="Visual heading correction gain (r/min per degree)",
    )
    # 视觉横向误差增益：输入为米，输出为轮速纠偏 r/min。
    declare_visual_lateral_gain_arg = DeclareLaunchArgument(
        "visual_lateral_gain",
        default_value=TextSubstitution(text="16.711269"),
        description="Visual lateral correction gain (r/min per meter)",
    )
    # 视觉纠偏对左右轮施加的最大速度偏置（轮子输出轴 r/min）。
    declare_visual_max_correction_arg = DeclareLaunchArgument(
        "visual_max_correction",
        default_value=TextSubstitution(text="3.342254"),
        description="Maximum visual correction in wheel output-shaft r/min",
    )
    declare_visual_confidence_threshold_arg = DeclareLaunchArgument(
        "visual_confidence_threshold",
        default_value=TextSubstitution(text="0.75"),
        description="Minimum visual confidence for correction",
    )
    declare_visual_timeout_arg = DeclareLaunchArgument(
        "visual_timeout_sec",
        default_value=TextSubstitution(text="0.5"),
        description="Visual sample timeout in seconds",
    )
    declare_camera_serial_port_arg = DeclareLaunchArgument(
        "camera_serial_port",
        default_value=TextSubstitution(text="/dev/OpenMV_Cam_H7_Plus"),
        description="OpenMV USB serial device",
    )
    declare_camera_serial_baud_arg = DeclareLaunchArgument(
        "camera_serial_baud",
        default_value=TextSubstitution(text="115200"),
        description="OpenMV serial baudrate",
    )
    declare_camera_serial_timeout_arg = DeclareLaunchArgument(
        "camera_serial_timeout",
        default_value=TextSubstitution(text="0.2"),
        description="OpenMV serial read timeout in seconds",
    )
    declare_camera_serial_no_data_timeout_arg = DeclareLaunchArgument(
        "camera_serial_no_data_timeout",
        default_value=TextSubstitution(text="5.0"),
        description="Reconnect OpenMV after this many seconds without bytes",
    )
    declare_camera_serial_max_frame_arg = DeclareLaunchArgument(
        "camera_serial_max_frame_bytes",
        default_value=TextSubstitution(text="2097152"),
        description="Maximum accepted OpenMV JPEG payload size",
    )
    declare_camera_image_rotation_arg = DeclareLaunchArgument(
        "camera_image_rotation_deg",
        default_value=TextSubstitution(text="0"),
        description="Rotate OpenMV image before publishing: 0, 90, 180, or 270 degrees",
    )
    declare_detection_fps_arg = DeclareLaunchArgument(
        "detection_fps",
        default_value=TextSubstitution(text="30.0"),
        description="Grid line detection timer FPS",
    )
    declare_line_tracking_enabled_arg = DeclareLaunchArgument(
        "line_tracking_enabled",
        default_value=TextSubstitution(text="true"),
        description="Track one parallel line across frames",
    )
    declare_line_tracking_jump_arg = DeclareLaunchArgument(
        "max_line_tracking_jump_px",
        default_value=TextSubstitution(text="30.0"),
        description="Maximum accepted line-normal jump in pixels",
    )
    declare_line_tracking_missed_arg = DeclareLaunchArgument(
        "max_line_tracking_missed_frames",
        default_value=TextSubstitution(text="2"),
        description="Missed frames before constrained line reacquisition",
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
    declare_angle_line_gap_fill_arg = DeclareLaunchArgument(
        "angle_line_gap_fill_px",
        default_value=TextSubstitution(text="25.0"),
        description=(
            "Maximum gap to bridge between collinear grid-line segments"
        ),
    )
    declare_angle_line_bridge_angle_arg = DeclareLaunchArgument(
        "angle_line_bridge_angle_tolerance_deg",
        default_value=TextSubstitution(text="5.0"),
        description=(
            "Maximum direction difference for bridging grid-line segments"
        ),
    )
    declare_angle_line_axis_tolerance_arg = DeclareLaunchArgument(
        "angle_line_axis_tolerance_deg",
        default_value=TextSubstitution(text="25.0"),
        description=(
            "Maximum fine-grid angle difference from the camera reference axis"
        ),
    )
    declare_angle_line_hough_gap_arg = DeclareLaunchArgument(
        "angle_line_hough_gap_px",
        default_value=TextSubstitution(text="2.0"),
        description="Maximum small gap for fine-line Hough detection in pixels",
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
    declare_angle_line_min_length_arg = DeclareLaunchArgument(
        "angle_line_min_length_px",
        default_value=TextSubstitution(text="12.0"),
        description="Minimum fine-line Hough segment length in pixels",
    )
    declare_angle_line_min_width_arg = DeclareLaunchArgument(
        "angle_line_min_width_px",
        default_value=TextSubstitution(text="1.0"),
        description="Minimum estimated fine-line width in pixels",
    )
    declare_angle_line_max_width_arg = DeclareLaunchArgument(
        "angle_line_max_width_px",
        default_value=TextSubstitution(text="6.0"),
        description=(
            "Maximum estimated fine-line width; reject broad reflection bands"
        ),
    )
    declare_angle_line_min_support_arg = DeclareLaunchArgument(
        "angle_line_min_support",
        default_value=TextSubstitution(text="0.20"),
        description="Minimum white-mask support ratio for a fine-line segment",
    )
    declare_angle_line_hough_threshold_arg = DeclareLaunchArgument(
        "angle_line_hough_threshold",
        default_value=TextSubstitution(text="8"),
        description="Hough accumulator threshold for fine-line detection",
    )
    declare_angle_line_min_merged_length_arg = DeclareLaunchArgument(
        "angle_line_min_merged_length_px",
        default_value=TextSubstitution(text="60.0"),
        description="Minimum bridged fine-line length used for angle statistics",
    )
    declare_angle_line_overlay_thickness_arg = DeclareLaunchArgument(
        "angle_line_overlay_thickness_px",
        default_value=TextSubstitution(text="1"),
        description="Fine-line debug overlay thickness in pixels",
    )
    declare_angle_line_overlay_outline_thickness_arg = DeclareLaunchArgument(
        "angle_line_overlay_outline_thickness_px",
        default_value=TextSubstitution(text="2"),
        description="Fine-line debug overlay outline thickness in pixels",
    )
    declare_tracked_line_overlay_thickness_arg = DeclareLaunchArgument(
        "tracked_line_overlay_thickness_px",
        default_value=TextSubstitution(text="2"),
        description="Tracked reference-line debug overlay thickness in pixels",
    )
    declare_enable_grid_line_stream_arg = DeclareLaunchArgument(
        "enable_grid_line_stream",
        default_value=TextSubstitution(text="true"),
        description=(
            "Start the host-side grid-line RTSP streamer; requires "
            "enable_visual_correction and publish_debug_images:=true"
        ),
    )
    declare_camera_angle_offset_arg = DeclareLaunchArgument(
        "camera_angle_offset",
        default_value=TextSubstitution(text="0.0"),
        description="Outdoor camera installation angle correction in degrees",
    )
    declare_grid_line_stream_rtsp_url_arg = DeclareLaunchArgument(
        "grid_line_stream_rtsp_url",
        default_value=TextSubstitution(
            text="rtsp://127.0.0.1:8554/live/grid_line"
        ),
        description="RTSP publish URL for the grid-line detection stream",
    )
    declare_grid_line_stream_fps_arg = DeclareLaunchArgument(
        "grid_line_stream_fps",
        default_value=TextSubstitution(text="10.0"),
        description="Grid-line RTSP stream frame rate",
    )
    declare_grid_line_stream_bitrate_arg = DeclareLaunchArgument(
        "grid_line_stream_bitrate",
        default_value=TextSubstitution(text="800k"),
        description="Grid-line RTSP H.264 target bitrate",
    )
    declare_grid_line_stream_preset_arg = DeclareLaunchArgument(
        "grid_line_stream_preset",
        default_value=TextSubstitution(text="veryfast"),
        description="Grid-line RTSP H.264 encoder preset",
    )
    declare_grid_line_stream_reconnect_arg = DeclareLaunchArgument(
        "grid_line_stream_reconnect_sec",
        default_value=TextSubstitution(text="2.0"),
        description="Grid-line RTSP reconnect delay in seconds",
    )
    declare_ffmpeg_path_arg = DeclareLaunchArgument(
        "ffmpeg_path",
        default_value=TextSubstitution(text="ffmpeg"),
        description="FFmpeg executable used by the grid-line streamer",
    )
    declare_target_line_offset_arg = DeclareLaunchArgument(
        "target_line_offset_m",
        default_value=TextSubstitution(text="nan"),
        description="Calibrated target parallel-line offset from camera center",
    )
    declare_target_line_tolerance_arg = DeclareLaunchArgument(
        "target_line_match_tolerance_m",
        default_value=TextSubstitution(text="0.5"),
        description="Maximum target-line matching error in meters",
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
            },
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
                'rtk_correction_ratio': ParameterValue(
                    LaunchConfiguration("rtk_correction_ratio"), value_type=float
                ),
                'rtk_max_correction': ParameterValue(
                    LaunchConfiguration("rtk_max_correction"), value_type=float
                ),
                'visual_correction_ratio': ParameterValue(
                    LaunchConfiguration("visual_correction_ratio"), value_type=float
                ),
                'visual_heading_gain': ParameterValue(
                    LaunchConfiguration("visual_heading_gain"), value_type=float
                ),
                'visual_lateral_gain': ParameterValue(
                    LaunchConfiguration("visual_lateral_gain"), value_type=float
                ),
                'visual_max_correction': ParameterValue(
                    LaunchConfiguration("visual_max_correction"), value_type=float
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
                'line_tracking_enabled': ParameterValue(
                    LaunchConfiguration('line_tracking_enabled'), value_type=bool
                ),
                'max_line_tracking_jump_px': ParameterValue(
                    LaunchConfiguration('max_line_tracking_jump_px'),
                    value_type=float,
                ),
                'max_line_tracking_missed_frames': ParameterValue(
                    LaunchConfiguration('max_line_tracking_missed_frames'),
                    value_type=int,
                ),
                'publish_debug_images': ParameterValue(
                    LaunchConfiguration('publish_debug_images'), value_type=bool
                ),
                'always_show_axis_debug': ParameterValue(
                    LaunchConfiguration('always_show_axis_debug'), value_type=bool
                ),
                'angle_line_gap_fill_px': ParameterValue(
                    LaunchConfiguration('angle_line_gap_fill_px'), value_type=float
                ),
                'angle_line_bridge_angle_tolerance_deg': ParameterValue(
                    LaunchConfiguration('angle_line_bridge_angle_tolerance_deg'),
                    value_type=float,
                ),
                'angle_line_axis_tolerance_deg': ParameterValue(
                    LaunchConfiguration('angle_line_axis_tolerance_deg'),
                    value_type=float,
                ),
                'coarse_line_min_length_px': ParameterValue(
                    LaunchConfiguration('coarse_line_min_length_px'),
                    value_type=float,
                ),
                'coarse_line_min_width_px': ParameterValue(
                    LaunchConfiguration('coarse_line_min_width_px'),
                    value_type=float,
                ),
                'coarse_line_min_support': ParameterValue(
                    LaunchConfiguration('coarse_line_min_support'),
                    value_type=float,
                ),
                'coarse_line_merge_gap_px': ParameterValue(
                    LaunchConfiguration('coarse_line_merge_gap_px'),
                    value_type=float,
                ),
                'coarse_line_gap_fill_px': ParameterValue(
                    LaunchConfiguration('coarse_line_gap_fill_px'),
                    value_type=float,
                ),
                'coarse_line_bridge_normal_gap_px': ParameterValue(
                    LaunchConfiguration('coarse_line_bridge_normal_gap_px'),
                    value_type=float,
                ),
                'coarse_line_bridge_angle_tolerance_deg': ParameterValue(
                    LaunchConfiguration('coarse_line_bridge_angle_tolerance_deg'),
                    value_type=float,
                ),
                'white_line_value_threshold': ParameterValue(
                    LaunchConfiguration('white_line_value_threshold'),
                    value_type=float,
                ),
                'white_line_saturation_max': ParameterValue(
                    LaunchConfiguration('white_line_saturation_max'),
                    value_type=float,
                ),
                'angle_average_center_band_ratio': ParameterValue(
                    LaunchConfiguration('angle_average_center_band_ratio'),
                    value_type=float,
                ),
                'angle_line_min_length_px': ParameterValue(
                    LaunchConfiguration('angle_line_min_length_px'),
                    value_type=float,
                ),
                'angle_line_min_width_px': ParameterValue(
                    LaunchConfiguration('angle_line_min_width_px'),
                    value_type=float,
                ),
                'angle_line_max_width_px': ParameterValue(
                    LaunchConfiguration('angle_line_max_width_px'),
                    value_type=float,
                ),
                'angle_line_min_support': ParameterValue(
                    LaunchConfiguration('angle_line_min_support'),
                    value_type=float,
                ),
                'angle_line_hough_threshold': ParameterValue(
                    LaunchConfiguration('angle_line_hough_threshold'),
                    value_type=int,
                ),
                'angle_line_hough_gap_px': ParameterValue(
                    LaunchConfiguration('angle_line_hough_gap_px'),
                    value_type=float,
                ),
                'angle_line_min_merged_length_px': ParameterValue(
                    LaunchConfiguration('angle_line_min_merged_length_px'),
                    value_type=float,
                ),
                'angle_line_overlay_thickness_px': ParameterValue(
                    LaunchConfiguration('angle_line_overlay_thickness_px'),
                    value_type=int,
                ),
                'angle_line_overlay_outline_thickness_px': ParameterValue(
                    LaunchConfiguration('angle_line_overlay_outline_thickness_px'),
                    value_type=int,
                ),
                'tracked_line_overlay_thickness_px': ParameterValue(
                    LaunchConfiguration('tracked_line_overlay_thickness_px'),
                    value_type=int,
                ),
                'bypass_path_context_gate': ParameterValue(
                    LaunchConfiguration("bypass_path_context_gate"),
                    value_type=bool,
                ),
                'fallback_path_axis_image_deg': ParameterValue(
                    LaunchConfiguration("fallback_path_axis_image_deg"),
                    value_type=float,
                ),
                'angle_reference_axis_image_deg': ParameterValue(
                    LaunchConfiguration("angle_reference_axis_image_deg"),
                    value_type=float,
                ),
                'camera_angle_offset': ParameterValue(
                    LaunchConfiguration("camera_angle_offset"),
                    value_type=float,
                ),
                'image_rotation_deg': ParameterValue(
                    LaunchConfiguration('camera_image_rotation_deg'),
                    value_type=int,
                ),
                'target_line_offset_m': ParameterValue(
                    LaunchConfiguration("target_line_offset_m"),
                    value_type=float,
                ),
                'target_line_match_tolerance_m': ParameterValue(
                    LaunchConfiguration("target_line_match_tolerance_m"),
                    value_type=float,
                ),
                'reference_axis_offset_px': ParameterValue(
                    LaunchConfiguration("reference_axis_offset_px"),
                    value_type=float,
                ),
            },
        ],
    )

    openmv_serial_publisher_node = Node(
        package='rtk_nav',
        executable='openmv_serial_publisher_node',
        name='openmv_serial_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visual_correction')),
        parameters=[
            {
                'topic': '/camera/color/image/compressed',
                'serial_port': LaunchConfiguration('camera_serial_port'),
                'baudrate': ParameterValue(
                    LaunchConfiguration('camera_serial_baud'), value_type=int
                ),
                'read_timeout_sec': ParameterValue(
                    LaunchConfiguration('camera_serial_timeout'), value_type=float
                ),
                'no_data_timeout_sec': ParameterValue(
                    LaunchConfiguration('camera_serial_no_data_timeout'),
                    value_type=float,
                ),
                'max_frame_bytes': ParameterValue(
                    LaunchConfiguration('camera_serial_max_frame_bytes'),
                    value_type=int,
                ),
                'image_rotation_deg': ParameterValue(
                    LaunchConfiguration('camera_image_rotation_deg'),
                    value_type=int,
                ),
            },
        ],
    )

    grid_line_streamer_node = Node(
        package='rtk_nav',
        executable='grid_line_streamer',
        name='grid_line_streamer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_grid_line_stream')),
        parameters=[
            {
                'image_topic': '/grid_line/detected_image',
                'rtsp_url': LaunchConfiguration('grid_line_stream_rtsp_url'),
                'fps': ParameterValue(
                    LaunchConfiguration('grid_line_stream_fps'),
                    value_type=float,
                ),
                'bitrate': LaunchConfiguration('grid_line_stream_bitrate'),
                'preset': LaunchConfiguration('grid_line_stream_preset'),
                'reconnect_sec': ParameterValue(
                    LaunchConfiguration('grid_line_stream_reconnect_sec'),
                    value_type=float,
                ),
                'ffmpeg_path': LaunchConfiguration('ffmpeg_path'),
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
    ld.add_action(declare_brush_motor_count_arg)
    ld.add_action(declare_brush_direction_mode_arg)
    ld.add_action(declare_visual_correction_arg)
    ld.add_action(declare_bypass_path_context_gate_arg)
    ld.add_action(declare_fallback_path_axis_arg)
    ld.add_action(declare_angle_reference_axis_arg)
    ld.add_action(declare_camera_angle_offset_arg)
    ld.add_action(declare_stanley_k_path_arg)
    ld.add_action(declare_stanley_k_near_target_arg)
    ld.add_action(declare_rtk_correction_ratio_arg)
    ld.add_action(declare_rtk_max_correction_arg)
    ld.add_action(declare_visual_correction_ratio_arg)
    ld.add_action(declare_visual_heading_gain_arg)
    ld.add_action(declare_visual_lateral_gain_arg)
    ld.add_action(declare_visual_max_correction_arg)
    ld.add_action(declare_visual_confidence_threshold_arg)
    ld.add_action(declare_visual_timeout_arg)
    ld.add_action(declare_camera_serial_port_arg)
    ld.add_action(declare_camera_serial_baud_arg)
    ld.add_action(declare_camera_serial_timeout_arg)
    ld.add_action(declare_camera_serial_no_data_timeout_arg)
    ld.add_action(declare_camera_serial_max_frame_arg)
    ld.add_action(declare_camera_image_rotation_arg)
    ld.add_action(declare_detection_fps_arg)
    ld.add_action(declare_line_tracking_enabled_arg)
    ld.add_action(declare_line_tracking_jump_arg)
    ld.add_action(declare_line_tracking_missed_arg)
    ld.add_action(declare_publish_debug_images_arg)
    ld.add_action(declare_always_show_axis_debug_arg)
    ld.add_action(declare_angle_line_gap_fill_arg)
    ld.add_action(declare_angle_line_bridge_angle_arg)
    ld.add_action(declare_angle_line_axis_tolerance_arg)
    ld.add_action(declare_angle_line_hough_gap_arg)
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
    ld.add_action(declare_angle_line_min_length_arg)
    ld.add_action(declare_angle_line_min_width_arg)
    ld.add_action(declare_angle_line_max_width_arg)
    ld.add_action(declare_angle_line_min_support_arg)
    ld.add_action(declare_angle_line_hough_threshold_arg)
    ld.add_action(declare_angle_line_min_merged_length_arg)
    ld.add_action(declare_angle_line_overlay_thickness_arg)
    ld.add_action(declare_angle_line_overlay_outline_thickness_arg)
    ld.add_action(declare_tracked_line_overlay_thickness_arg)
    ld.add_action(declare_enable_grid_line_stream_arg)
    ld.add_action(declare_grid_line_stream_rtsp_url_arg)
    ld.add_action(declare_grid_line_stream_fps_arg)
    ld.add_action(declare_grid_line_stream_bitrate_arg)
    ld.add_action(declare_grid_line_stream_preset_arg)
    ld.add_action(declare_grid_line_stream_reconnect_arg)
    ld.add_action(declare_ffmpeg_path_arg)
    ld.add_action(declare_target_line_offset_arg)
    ld.add_action(declare_target_line_tolerance_arg)
    ld.add_action(declare_reference_axis_offset_arg)
    ld.add_action(mqtt_ros_bridge_node)
    ld.add_action(motor_control_node)

    # ld.add_action(sensors_485_node)
    # ld.add_action(laser_node)
    # ld.add_action(charging_node)
    # ld.add_action(wtrtk_parse_txt_node)
    ld.add_action(rtk_navigator)
    # # 若需要启用注释的节点，取消以下对应行的注释
    ld.add_action(wtrtk_serial_driver_node)
    ld.add_action(line_detector_node)
    ld.add_action(openmv_serial_publisher_node)
    ld.add_action(grid_line_streamer_node)

    return ld
