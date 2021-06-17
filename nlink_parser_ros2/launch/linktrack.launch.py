from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    linktrack_node = Node(
        package="nlink_parser_ros2",
        executable="linktrack",
        output="screen",
        # figure this yaml stuff out
        arguments=["linktrack_init_params.yaml"]
    )
    
    ld.add_action(linktrack_node)
    
    nlink_to_r2r2_node = Node(
    package="nlink_parser_ros2",
    executable="nlink_to_r2r2",
    # output="screen",
    # figure this yaml stuff out
    arguments=["linktrack_init_params.yaml"]
    )
    
    ld.add_action(nlink_to_r2r2_node)
    
    return ld