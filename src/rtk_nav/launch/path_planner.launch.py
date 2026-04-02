from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    # 多区域清扫路径规划节点
    full_path_planner = Node(
        package='rtk_nav',
        executable='full_path_planner',  # 注意： executable名称需与脚本文件名一致（或通过setup.py配置）
        name='full_path',
        output='screen',
        parameters=[
            # ---------------------- 全局配置 ----------------------
            {'area_count': 2},  # 区域数量（根据实际需求修改）
            {'default.interval': 9.0},  # 默认路径间隔（可被区域专属参数覆盖）
            {'default.start_corner': 'top_left'},  # 默认起始角点
            {'default.end_corner_mode': 'opposite'},  # 默认end    #diagonal / opposite
            {'default.swap_wh_select': True},  # default: inner_width=delta lon, inner_width >= inner_height,up-down
            {'default.edge_distance_lon': 0.5},  # 默认经度方向边界距离
            {'default.edge_distance_lat': 0.5},  # 默认纬度方向边界距离
            {'headless': False},  # 是否无头模式（不显示图片）
            
            # ---------------------- 区域0参数（必填：3个标定点；可选：覆盖默认参数） ----------------------
            {'area_0.calib_point_a.lon': 110.64741299779179},
            {'area_0.calib_point_a.lat': 35.60593376188335},
            {'area_0.calib_point_b.lon': 110.64741219706659},
            {'area_0.calib_point_b.lat': 35.60611366491313},
            {'area_0.calib_point_c.lon': 110.64726667869006},
            {'area_0.calib_point_c.lat': 35.60611161096633},
            # {'area_0.interval': 1.3},  # 可选：覆盖默认间隔
            {'area_0.start_corner': 'top_right'},  # 可选：覆盖默认起始角点
            {'area_0.end_corner_mode': 'diagonal'},  # 可选：覆盖默认起始角点
            # {'area_0.swap_wh_select': True},  # 可选：覆盖默认inner_width=delta lon
            # ---------------------- 区域1参数（示例：第二个区域） ----------------------

            {'area_1.calib_point_a.lon': 110.64741366306355},
            {'area_1.calib_point_a.lat': 35.60571889444015},
            {'area_1.calib_point_b.lon': 110.64741283394737},
            {'area_1.calib_point_b.lat': 35.60589706282963},
            {'area_1.calib_point_c.lon': 110.64726808899731},
            {'area_1.calib_point_c.lat': 35.605895621784235},
            # {'area_1.interval': 1.18},  # 可选：该区域间隔为1.2m（覆盖默认）
            {'area_1.start_corner': 'top_left'},  # 可选：覆盖默认起始角点
            {'area_1.end_corner_mode': 'diagonal'},  # 可选：覆盖默认起始角点

            # {'area_1.swap_wh_select': False},  # 可选：覆盖默认inner_width=delta lon

            
            # ---------------------- 区域1参数（示例：第二个区域） ----------------------

            # {'area_2.calib_point_a.lon': 120.06770772485666},
            # {'area_2.calib_point_a.lat': 30.321238340495807},
            # {'area_2.calib_point_b.lon': 120.06758377060459},
            # {'area_2.calib_point_b.lat': 30.32120704408408},
            # {'area_2.calib_point_c.lon': 120.06761198560712},
            # {'area_2.calib_point_c.lat': 30.321116435931167},
            # {'area_2.interval': 2.0},  # 可选：该区域间隔为1.2m（覆盖默认）
            # {'area_2.start_corner': 'top_left'},  # 可选：覆盖默认起始角点
            # # {'area_1.swap_wh_select': False},  # 可选：覆盖默认inner_width=delta lon

            # # AERA_1
            # {'area_3.calib_point_a.lon': 120.06757631066615},
            # {'area_3.calib_point_a.lat': 30.32110629748197},
            # {'area_3.calib_point_b.lon': 120.0674618551937},
            # {'area_3.calib_point_b.lat': 30.321077905589622},
            # {'area_3.calib_point_c.lon': 120.06744037236592},
            # {'area_3.calib_point_c.lat': 30.321138440383812},
            # {'area_3.interval': 2.5},  # 可选：该区域间隔为1.2m（覆盖默认）
            # {'area_3.start_corner': 'top_right'},  # 可选：bottom_right need to invert
            # # {'area_3.swap_wh_select': True},  # 可选：覆盖默认inner_width=delta lon


            # {'area_4.calib_point_a.lon': 120.06744037236592},
            # {'area_4.calib_point_a.lat': 30.321138440383812},
            # {'area_4.calib_point_b.lon': 120.0679311524236},
            # {'area_4.calib_point_b.lat': 30.321260442816914},
            # {'area_4.calib_point_c.lon': 120.06744037236592},
            # {'area_4.calib_point_c.lat': 30.321138440383812},
            # {'area_4.interval': 1.0},  # 可选：该区域间隔为1.2m（覆盖默认）
            # {'area_4.start_corner': 'top_right'},  # 可选：bottom_right need to invert
            # # {'area_4.swap_wh_select': True},  # 可选：覆盖默认inner_width=delta lon
            

        ]
    )
    # 组装所有节点到 LaunchDescription
    ld = LaunchDescription()
   
    ld.add_action(full_path_planner)

    return ld