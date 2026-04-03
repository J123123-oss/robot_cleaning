from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/config/areas.yaml'
    )
    config_file = os.path.normpath(config_file)
    
    full_path_planner = Node(
        package='rtk_nav',
        executable='full_path_planner',
        name='full_path',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'headless': False,
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(full_path_planner)

    return ld
