cmake_minimum_required(VERSION 3.5.0)
project(nlink_parser_ros2_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
    
## Generate messages in the 'msg' folder

set(msg_files
"msg/LinktrackAnchorframe0.msg"
"msg/LinktrackNode0.msg"
"msg/LinktrackNode1.msg"
"msg/LinktrackNode2.msg"
"msg/LinktrackNodeframe0.msg"
"msg/LinktrackNodeframe1.msg"
"msg/LinktrackNodeframe2.msg"
"msg/LinktrackNodeframe3.msg"
"msg/LinktrackTag.msg"
"msg/LinktrackTagframe0.msg"
"msg/TofsenseCascade.msg"
"msg/TofsenseFrame0.msg"
"msg/LinktrackAoaNode0.msg"
"msg/LinktrackAoaNodeframe0.msg"
"msg/LinktrackNode5.msg"
"msg/LinktrackNodeframe5.msg"
"msg/LinktrackNode6.msg"
"msg/LinktrackNodeframe6.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces 
)

# ament_export_dependencies(rosidl_default_runtime)
ament_package()