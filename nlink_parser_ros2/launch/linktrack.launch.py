from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    node_ns = LaunchConfiguration('node_namespace', default='r2r2')

    ld = LaunchDescription()
    linktrack_node = Node(
        namespace=node_ns,
        package="nlink_parser_ros2",
        executable="linktrack",
        output="screen",
        # figure this yaml stuff out
        arguments=["linktrack_init_params.yaml"]
    )
    
    ld.add_action(linktrack_node)
    return ld