from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明robot_ID参数，默认值可自定义（比如"GF-HZ-TEST"）
    declare_robot_id_arg = DeclareLaunchArgument(
        "robot_ID",  # 参数名 和ROS1的 arg name="robot_ID" 对应
        default_value=TextSubstitution(text="ROS2_VM"),  # 默认值
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
            # {'rtk_path_file': '/home/forlinx/robot_cleaning/src/rtk_nav/cleaning_path/cleaning_path_20251226_155005.txt'}
            # 原注释的其他参数可在此处取消注释添加，格式同上
            # {'rtk_path_file': '/home/forlinx/robot_cleaning/src/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt'}
            # {'rtk_path_file': '/home/forlinx/robot_cleaning/src/rtk_nav/cleaning_path/道路4.txt'}
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
    RTKNavigator = Node(
        package='rtk_nav',
        executable='rtk_nav',
        name='rtk_nav',
        output='screen',
        parameters=[
            # {'rtk_path_file': '/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/top_left.txt'}
            # {'rtk_path_file': '/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/top_left.txt'}
            {'rtk_path_file': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/three_path_20260203_161413.txt'}
            # {'rtk_path_file': '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/three_path_20260203_161413.txt'}
            # 原注释的其他参数可取消注释添加
            # {'file_path': '/home/forlinx/robot_cleaning/src/rtk_nav/rtkmsgs/直线往返1.txt'}
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
            {'file_path': '/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
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

    # 清扫路径规划节点（原ROS1中注释的节点，保留注释结构和所有参数）
    cleaning_path_planner_node = Node(
        package='rtk_nav',
        executable='cleaning_path_planner',
        name='cleaning_path_planner',
        output='screen',
        parameters=[
            #120.0711247716332,30.320803806689252
            {'base_point.lon': 120.0711247716332},
            {'base_point.lat': 30.320803806689252},
            {'rect_width': 5.0},
            {'rect_height': 10.0},
            {'rotation_deg': 15.0},
            {'interval': 1.0},
            {'start_corner': 'top_left'},
            {'edge_distance_lon': 0.5},
            {'edge_distance_lat': 0.5},
            {'headless': False}
        ]
    )
    # ===================== 2. 配置MQTT桥接节点 (对应ROS1的 <node>) =====================
    mqtt_ros_bridge_node = Node(
        package="mqtt_ros2",  # ROS2功能包名（替换为你的实际包名）
        executable="mqtt_ros2_bridge",  # 节点可执行文件名（setup.py中配置的console_scripts名称）
        name="mqtt",  # 节点名称 和ROS1的 name="mqtt_ros_bridge" 对应
        output="screen",  # 日志输出到终端（ROS1的 output="screen"）
        emulate_tty=True,  # 确保彩色日志、交互正常
        # 传递参数（对应ROS1的 <param>）
        parameters=[
            {
            "broker": "121.40.57.48",  # MQTT服务器地址
            "port": 1883,  # MQTT端口
            "user": "gf-mounted",  # MQTT用户名
            "password": "20230810",  # MQTT密码
            # 拼接参数：robot/$(arg robot_ID)/status → ROS2用LaunchConfiguration
            "topic_status": ["robot/", LaunchConfiguration("robot_ID"), "/status"],
            "topic_cmd": ["robot/", LaunchConfiguration("robot_ID"), "/cmd"],
            "topic_command": ["robot/", LaunchConfiguration("robot_ID"), "/command"],
            "topic_result": ["robot/", LaunchConfiguration("robot_ID"), "/result"],
            "client_id": ["python-mqtt-client-", LaunchConfiguration("robot_ID")]
            }
        ]
    )
    # 多区域清扫路径规划节点
    full_path_planner = Node(
        package='rtk_nav',
        executable='full_path_planner',  # 注意： executable名称需与脚本文件名一致（或通过setup.py配置）
        name='full_path',
        output='screen',
        parameters=[
            # ---------------------- 全局配置 ----------------------
            {'area_count': 3},  # 区域数量（根据实际需求修改）
            {'default.interval': 2.0},  # 默认路径间隔（可被区域专属参数覆盖）
            {'default.start_corner': 'top_right'},  # 默认起始角点
            {'default.swap_wh_select': True},  # default: inner_width=delta lon, inner_width >= inner_height,up-down
            {'default.edge_distance_lon': 0.5},  # 默认经度方向边界距离
            {'default.edge_distance_lat': 0.5},  # 默认纬度方向边界距离
            {'headless': False},  # 是否无头模式（不显示图片）
            
            # ---------------------- 区域0参数（必填：3个标定点；可选：覆盖默认参数） ----------------------
            # 120.06908157229124,30.320549326045818
            # 120.06934719266512,30.319875087155783
            # 120.06893303174934,30.320515493992254
            {'area_0.calib_point_a.lon': 120.0691124656172},
            {'area_0.calib_point_a.lat': 30.3200001861353},
            {'area_0.calib_point_b.lon': 120.06902223898602},
            {'area_0.calib_point_b.lat': 30.320249484457765},
            {'area_0.calib_point_c.lon': 120.06884603228838},
            {'area_0.calib_point_c.lat': 30.319928373774324},
            {'area_0.interval': 2.3},  # 可选：覆盖默认间隔
            {'area_0.start_corner': 'bottom_left'},  # 可选：覆盖默认起始角点
            # {'area_0.swap_wh_select': True},  # 可选：覆盖默认inner_width=delta lon

            
            # ---------------------- 区域1参数（示例：第二个区域） ----------------------
            # 120.06891577325935,30.320537691107706
            # 120.0691364728377,30.319852150388023
            # 120.06873704947856,30.320508473942272

            {'area_1.calib_point_a.lon': 120.06939750637723},
            {'area_1.calib_point_a.lat': 30.319874909665536},
            {'area_1.calib_point_b.lon': 120.06935553828153},
            {'area_1.calib_point_b.lat': 30.320058864609038},
            {'area_1.calib_point_c.lon': 120.06891137342275},
            {'area_1.calib_point_c.lat': 30.319765710359487},
            {'area_1.interval': 2.4},  # 可选：该区域间隔为1.2m（覆盖默认）
            {'area_1.start_corner': 'bottom_right'},  # 可选：覆盖默认起始角点
            {'area_1.swap_wh_select': False},  # 可选：覆盖默认inner_width=delta lon

            # AERA_1
            # a120.0691124656172,30.3200001861353
            # b120.06902223898602,30.320249484457765
            # c120.06884603228838,30.319928373774324
            # A120.06939750637723,30.319874909665536
            # B120.06935553828153,30.320058864609038
            # C120.06891137342275,30.319765710359487
            # A`120.06922485147298,30.32040989499512
            # B`120.06935553828153,30.320058864609038
            # C`120.06907218060923,30.320421493893136
            {'area_2.calib_point_a.lon': 120.06922485147298},
            {'area_2.calib_point_a.lat': 30.32040989499512},
            {'area_2.calib_point_b.lon': 120.06935553828153},
            {'area_2.calib_point_b.lat': 30.320058864609038},
            {'area_2.calib_point_c.lon': 120.06907218060923},
            {'area_2.calib_point_c.lat': 30.320421493893136},
            {'area_2.interval': 2.0},  # 可选：该区域间隔为1.2m（覆盖默认）
            {'area_2.start_corner': 'bottom_right'},  # 可选：bottom_right need to invert
            # {'area_2.swap_wh_select': True},  # 可选：覆盖默认inner_width=delta lon

        ]
    )
    # 组装所有节点到 LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_robot_id_arg)
    ld.add_action(mqtt_ros_bridge_node)
    ld.add_action(motor_control_node)

    ld.add_action(sensors_485_node)
    # ld.add_action(wtrtk_parse_txt_node)
    ld.add_action(RTKNavigator)
    # 若需要启用注释的节点，取消以下对应行的注释
    ld.add_action(wtrtk_serial_driver_node)
    # ld.add_action(cleaning_path_planner_node)
    # ld.add_action(full_path_planner)

    return ld