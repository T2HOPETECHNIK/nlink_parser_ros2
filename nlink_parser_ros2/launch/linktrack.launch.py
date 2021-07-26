import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    linktrack_param_file=os.path.join(get_package_share_directory('nlink_parser_ros2'),'params','linktrack_init_params.yaml')

    ld = LaunchDescription()
    linktrack_node = Node(
        package="nlink_parser_ros2",
        executable="linktrack",
        output="screen",
        arguments=[linktrack_param_file]
    )
    
    ld.add_action(linktrack_node)
    return ld