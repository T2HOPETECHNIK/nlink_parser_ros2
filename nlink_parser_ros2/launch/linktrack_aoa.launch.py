import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    linktrack_aoa_param_file=os.path.join(get_package_share_directory('nlink_parser_ros2'),'params','linktrack_aoa_init_params.yaml')
    ld = LaunchDescription()
    linktrack_aoa_node = Node(
        name="linktrack_aoa",
        package="nlink_parser_ros2",
        executable="linktrack_aoa",
        output="screen",
        parameters=[{"pub_frequency":5.0}],
        arguments=[linktrack_aoa_param_file]
    )
    ld.add_action(linktrack_aoa_node)
    
    # linktrack_aoa_node_2 = Node(
    #     name="linktrack_aoa_2",
    #     package="nlink_parser_ros2",
    #     namespace="linktrack_aoa_2",
    #     executable="linktrack_aoa",
    #     output="screen",
    #     # figure this yaml stuff out
    #     arguments=["linktrack_aoa_init_params1.yaml"]
    # )
    # ld.add_action(linktrack_aoa_node_2)

    # nlink_viz = Node(
    #     package="nlink_parser_ros2",
    #     executable="nlink_viz_ros2",
        # output={'stdout':'log'},
        # output="screen",
        # figure this yaml stuff out
        # arguments=["linktrack_aoa_init_params.yaml"]
    # )
    # ld.add_action(nlink_viz)


    return ld
