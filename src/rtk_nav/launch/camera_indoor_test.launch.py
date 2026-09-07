from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动不依赖 RTK 的室内摄像头直线纠偏测试链路。"""
    declare_camera_serial_port_arg = DeclareLaunchArgument(
        'camera_serial_port',
        default_value=TextSubstitution(text='/dev/OpenMV_Cam_H7_Plus'),
        description='OpenMV USB serial device',
    )
    declare_camera_serial_baud_arg = DeclareLaunchArgument(
        'camera_serial_baud',
        default_value=TextSubstitution(text='921600'),
        description='OpenMV serial baudrate',
    )
    declare_camera_serial_timeout_arg = DeclareLaunchArgument(
        'camera_serial_timeout',
        default_value=TextSubstitution(text='0.2'),
        description='OpenMV serial read timeout in seconds',
    )
    declare_camera_serial_no_data_timeout_arg = DeclareLaunchArgument(
        'camera_serial_no_data_timeout',
        default_value=TextSubstitution(text='5.0'),
        description='Reconnect OpenMV after this many seconds without bytes',
    )
    declare_camera_serial_max_frame_arg = DeclareLaunchArgument(
        'camera_serial_max_frame_bytes',
        default_value=TextSubstitution(text='2097152'),
        description='Maximum accepted OpenMV JPEG payload size',
    )
    declare_base_speed_arg = DeclareLaunchArgument(
        'base_speed',
        default_value=TextSubstitution(text='1.0'),
        description='Low indoor forward speed in motor driver units',
    )
    declare_heading_gain_arg = DeclareLaunchArgument(
        'heading_gain',
        default_value=TextSubstitution(text='0.05'),
        description='Heading error gain in motor speed units per degree',
    )
    declare_lateral_gain_arg = DeclareLaunchArgument(
        'lateral_gain',
        default_value=TextSubstitution(text='5.0'),
        description='Lateral error gain in motor speed units per meter',
    )
    declare_max_correction_arg = DeclareLaunchArgument(
        'max_correction',
        default_value=TextSubstitution(text='0.8'),
        description='Maximum differential correction in motor speed units',
    )
    declare_min_confidence_arg = DeclareLaunchArgument(
        'min_confidence',
        default_value=TextSubstitution(text='0.5'),
        description='Minimum visual confidence required to move',
    )
    declare_visual_timeout_arg = DeclareLaunchArgument(
        'visual_timeout_sec',
        default_value=TextSubstitution(text='0.5'),
        description='Stop after this many seconds without fresh visual data',
    )
    declare_publish_debug_images_arg = DeclareLaunchArgument(
        'publish_debug_images',
        default_value=TextSubstitution(text='true'),
        description='Publish annotated and intermediate camera images',
    )
    declare_fallback_path_axis_arg = DeclareLaunchArgument(
        'fallback_path_axis_image_deg',
        default_value=TextSubstitution(text='0.0'),
        description=(
            'Indoor image run axis: 0 degrees right, 90 degrees down'
        ),
    )
    declare_brush_motor_count_arg = DeclareLaunchArgument(
        'brush_motor_count',
        default_value=TextSubstitution(text='2'),
        description='Brush motor count: 1 uses ID 3, 2 uses IDs 3 and 4',
    )

    openmv_camera_node = Node(
        package='rtk_nav',
        executable='openmv_serial_publisher_node',
        name='indoor_camera_publisher',
        output='screen',
        parameters=[
            {
                'topic': '/camera/color/image/compressed',
                'serial_port': LaunchConfiguration('camera_serial_port'),
                'baudrate': ParameterValue(
                    LaunchConfiguration('camera_serial_baud'), value_type=int
                ),
                'read_timeout_sec': ParameterValue(
                    LaunchConfiguration('camera_serial_timeout'),
                    value_type=float,
                ),
                'no_data_timeout_sec': ParameterValue(
                    LaunchConfiguration('camera_serial_no_data_timeout'),
                    value_type=float,
                ),
                'max_frame_bytes': ParameterValue(
                    LaunchConfiguration('camera_serial_max_frame_bytes'),
                    value_type=int,
                ),
            }
        ],
    )

    line_detector_node = Node(
        package='rtk_nav',
        executable='line_detector_node',
        name='indoor_grid_line_detector',
        output='screen',
        parameters=[
            {
                'enable_visual_correction': True,
                'indoor_test_mode': True,
                'bypass_path_context_gate': True,
                'fallback_path_axis_image_deg': ParameterValue(
                    LaunchConfiguration('fallback_path_axis_image_deg'),
                    value_type=float,
                ),
                'publish_debug_images': ParameterValue(
                    LaunchConfiguration('publish_debug_images'), value_type=bool
                ),
            }
        ],
    )

    controller_node = Node(
        package='motor_control',
        executable='camera_indoor_test_controller',
        name='camera_indoor_test_controller',
        output='screen',
        parameters=[
            {
                'base_speed': ParameterValue(
                    LaunchConfiguration('base_speed'), value_type=float
                ),
                'heading_gain': ParameterValue(
                    LaunchConfiguration('heading_gain'), value_type=float
                ),
                'lateral_gain': ParameterValue(
                    LaunchConfiguration('lateral_gain'), value_type=float
                ),
                'max_correction': ParameterValue(
                    LaunchConfiguration('max_correction'), value_type=float
                ),
                'min_confidence': ParameterValue(
                    LaunchConfiguration('min_confidence'), value_type=float
                ),
                'visual_timeout_sec': ParameterValue(
                    LaunchConfiguration('visual_timeout_sec'), value_type=float
                ),
                'brush_motor_count': ParameterValue(
                    LaunchConfiguration('brush_motor_count'), value_type=int
                ),
            }
        ],
    )

    motor_driver_node = Node(
        package='motor_control',
        executable='motor_driver',
        name='indoor_can_motor_driver',
        output='screen',
        parameters=[
            {
                # The driver owns all four nodes: left/right wheels and two
                # brush IDs.  The controller keeps both brush targets at 0.
                'auto_enable': True,
                'command_timeout_sec': 0.8,
                'brush_motor_count': ParameterValue(
                    LaunchConfiguration('brush_motor_count'), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_camera_serial_port_arg,
            declare_camera_serial_baud_arg,
            declare_camera_serial_timeout_arg,
            declare_camera_serial_no_data_timeout_arg,
            declare_camera_serial_max_frame_arg,
            declare_base_speed_arg,
            declare_heading_gain_arg,
            declare_lateral_gain_arg,
            declare_max_correction_arg,
            declare_min_confidence_arg,
            declare_visual_timeout_arg,
            declare_publish_debug_images_arg,
            declare_fallback_path_axis_arg,
            declare_brush_motor_count_arg,
            motor_driver_node,
            openmv_camera_node,
            line_detector_node,
            controller_node,
        ]
    )
