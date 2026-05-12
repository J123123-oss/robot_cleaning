from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    declare_robot_id_arg = DeclareLaunchArgument(
        "robot_ID",
        default_value=TextSubstitution(text="HEJIN_Huaxinyuan"),
        description="Robot ID used for MQTT topics",
    )

    motor_control_node = Node(
        package='motor_control',
        executable='motor_control',
        output='screen',
        parameters=[],
    )

    sensors_485_node = Node(
        package='motor_control',
        executable='sensors_485',
        parameters=[
            {'serial_port': '/dev/ttyS4'},
            {'io_baudrate': 9600},
        ],
    )

    laser_node = Node(
        package='motor_control',
        executable='laser_distance',
        name='laser_distance_node',
        parameters=[
            {'serial_port': '/dev/laser'},
            {'baud_rate': 115200},
        ],
    )

    charging_node = Node(
        package='motor_control',
        executable='charging',
        name='charging_485_node',
        parameters=[
            {'serial_port': '/dev/battery_charging'},
            {'charge_baud_rate': 19200},
            {'battery_baud_rate': 4800},
            {'slave_addr': 1},
            {'battery_addr': 0x0B},
            {'timeout': 0.5},
            {'battery_timeout': 0.2},
        ],
    )

    rtk_navigator = Node(
        package='rtk_nav',
        executable='rtk_nav',
        name='rtk_nav',
        parameters=[
            {'rtk_path_file': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/test_back11-6.txt'}
        ],
    )

    wtrtk_parse_txt_node = Node(
        package='rtk_nav',
        executable='wtrtk_parse_txt',
        name='wtrtk_parse_txt',
        output='screen',
        parameters=[
            {'file_path': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
        ],
    )

    wtrtk_serial_driver_node = Node(
        package='rtk_nav',
        executable='wtrtk_serial_driver',
        name='wtrtk_serial_driver',
        output='screen',
        parameters=[
            {'port': '/dev/WTRTK'},
            {'baud': 460800},
        ],
    )

    mqtt_ros_bridge_node = Node(
        package='mqtt_ros2',
        executable='mqtt_ros2_bridge',
        name='mqtt',
        output='log',
        arguments=['--ros-args', '--log-level', 'fatal'],
        parameters=[{
            "broker": "121.40.57.48",
            "port": 1883,
            "user": "gf-mounted",
            "password": "20230810",
            "topic_status": ["robot/", LaunchConfiguration("robot_ID"), "/status"],
            "topic_dock_status": ["dock/", LaunchConfiguration("robot_ID"), "/status"],
            "topic_cmd": ["robot/", LaunchConfiguration("robot_ID"), "/cmd"],
            "topic_command": ["robot/", LaunchConfiguration("robot_ID"), "/command"],
            "topic_result": ["robot/", LaunchConfiguration("robot_ID"), "/result"],
            "client_id": ["python-mqtt-client-a", LaunchConfiguration("robot_ID")],
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_id_arg)
    ld.add_action(mqtt_ros_bridge_node)
    ld.add_action(motor_control_node)
    ld.add_action(laser_node)
    ld.add_action(charging_node)
    ld.add_action(rtk_navigator)
    ld.add_action(wtrtk_serial_driver_node)

    # Intentionally disabled while debugging the shared battery/charging bus.
    # ld.add_action(sensors_485_node)
    # ld.add_action(wtrtk_parse_txt_node)

    return ld
