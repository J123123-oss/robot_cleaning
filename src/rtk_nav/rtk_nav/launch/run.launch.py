# 导入 ROS2 Launch 相关依赖
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义电机控制节点
    motor_control_node = Node(
        package='motor_control',  # 对应 ROS1 的 pkg，包名不变
        executable='motor_control.py',  # 对应 ROS1 的 type，可执行文件名称
        name='motor_control_node',  # 对应 ROS1 的 name，节点名称
        output='screen',  # 对应 ROS1 的 output，输出到终端
        parameters=[
            # 对应 ROS1 的 param，使用键值对形式配置参数
            {'rtk_path_file': '/home/ubuntu/robot_cleaning/src/rtk_nav/cleaning_path/cleaning_path_20251226_155005.txt'}
            # 原注释的其他参数可在此处取消注释添加，格式同上
            # {'rtk_path_file': '/home/ubuntu/robot_cleaning/src/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt'}
            # {'rtk_path_file': '/home/ubuntu/robot_cleaning/src/rtk_nav/cleaning_path/道路4.txt'}
        ]
    )
    RTKNavigator = Node(
        package='rtk_nav',
        executable='rtk_nav.py',
        name='rtk_nav',
        output='screen',
        parameters=[
            {'rtk_path_file': '"/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"'}
            # 原注释的其他参数可取消注释添加
            # {'file_path': '/home/ubuntu/robot_cleaning/src/rtk_nav/rtkmsgs/直线往返1.txt'}
        ]
    )
    # RTK录制的消息解析节点（原ROS1中未注释的节点）
    wtrtk_parse_txt_node = Node(
        package='rtk_nav',
        executable='wtrtk_parse_txt.py',
        name='wtrtk_parse_txt',
        output='screen',
        parameters=[
            {'file_path': '/home/ubuntu/robot_cleaning/src/rtk_nav/rtkmsgs/道路边轨迹3.txt'}
            # 原注释的其他参数可取消注释添加
            # {'file_path': '/home/ubuntu/robot_cleaning/src/rtk_nav/rtkmsgs/直线往返1.txt'}
        ]
    )

    # RTK实时消息解析节点（原ROS1中注释的节点，保留注释结构）
    # wtrtk_serial_driver_node = Node(
    #     package='rtk_nav',
    #     executable='wtrtk_serial_driver.py',
    #     name='wtrtk_serial_driver',
    #     parameters=[
    #         {'port': '/dev/WTRTK'},
    #         {'baud': 460800}
    #     ]
    # )

    # 清扫路径规划节点（原ROS1中注释的节点，保留注释结构和所有参数）
    # cleaning_path_planner_node = Node(
    #     package='rtk_nav',
    #     executable='cleaning_path_planner.py',
    #     name='cleaning_path_planner',
    #     output='screen',
    #     parameters=[
    #         {'base_point.lon': 120.06910407},
    #         {'base_point.lat': 30.32000295},
    #         {'rect_width': 6.0},
    #         {'rect_height': 15.0},
    #         {'rotation_deg': 20.0},
    #         {'interval': 1.0},
    #         {'start_corner': 'bottom_right'},
    #         {'edge_distance_lon': 0.5},
    #         {'edge_distance_lat': 0.5},
    #         {'headless': False}
    #     ]
    # )

    # 组装所有节点到 LaunchDescription
    ld = LaunchDescription()
    ld.add_action(motor_control_node)
    ld.add_action(wtrtk_parse_txt_node)
    ld.add_action(RTKNavigator)
    # 若需要启用注释的节点，取消以下对应行的注释
    # ld.add_action(wtrtk_serial_driver_node)
    # ld.add_action(cleaning_path_planner_node)

    return ld