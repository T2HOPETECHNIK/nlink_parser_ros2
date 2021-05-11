from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    linktrack_aoa_node = Node(
        package="nlink_parser_ros2",
        executable="linktrack_aoa",
    )
    
    ld.add_action(linktrack_aoa_node)
    return ld