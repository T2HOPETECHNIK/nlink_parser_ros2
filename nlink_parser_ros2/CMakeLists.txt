cmake_minimum_required(VERSION 3.5.0)
project(nlink_parser_ros2)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_definitions(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlink_parser_ros2_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(yaml-cpp REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include(GNUInstallDirs)

include_directories(include 
    src/utils
    ${serial_INCLUDE_DIRS}
    include
    )


set(nutils "nutils")
add_library(${nutils}
    src/utils/protocol_extracter/nprotocol_base.cpp
    src/utils/protocol_extracter/nprotocol_extracter.cpp
    src/utils/nlink_unpack/nlink_utils.c
    src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
    src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
    src/utils/nlink_unpack/nlink_tofsense_frame0.c
    src/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
    src/utils/init_serial.cpp
    src/utils/nlink_protocol.cpp
    src/utils/nutils.cpp
)
target_link_libraries(${nutils}
  serial::serial
  rclcpp::rclcpp
  yaml-cpp
)

# target_include_directories(${nutils} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${nutils}
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

set(LINKTRACK_NAME "linktrack")
add_executable(${LINKTRACK_NAME}
    src/linktrack/init.cpp
    src/linktrack/main.cpp
    src/linktrack/protocols.cpp
    )
target_link_libraries(${LINKTRACK_NAME}
   ${catkin_LIBRARIES}
    ${nutils}
    )
# rosidl_target_interfaces(${LINKTRACK_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(${LINKTRACK_NAME}
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
)
# add_dependencies(${LINKTRACK_NAME} ${PROJECT_NAME}_generate_messages_cpp)

# set(LINKTRACK_RVIZ_NAME "linktrack_rviz_converter")
# add_executable(${LINKTRACK_RVIZ_NAME}
#     src/linktrack/main_rviz_converter.cpp
#     )
# target_link_libraries(${LINKTRACK_RVIZ_NAME}
#     ${nutils}
#     rclcpp
#     )
# # add_dependencies(${LINKTRACK_RVIZ_NAME} ${PROJECT_NAME}_generate_messages_cpp)
# # rosidl_target_interfaces(${LINKTRACK_RVIZ_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
# ament_target_dependencies(${LINKTRACK_RVIZ_NAME}
#       rclcpp
#       nlink_parser_ros2_interfaces
#       std_msgs
#       geometry_msgs
#     )

# set(TOFSENSE_NAME "tofsense")
# add_executable(${TOFSENSE_NAME}
#     src/tofsense/init.cpp
#     src/tofsense/main.cpp
#     )
# target_link_libraries(${TOFSENSE_NAME}
# #    ${catkin_LIBRARIES}
#     ${nutils}
#     )
# add_dependencies(${TOFSENSE_NAME} ${PROJECT_NAME}_generate_messages_cpp)
# rosidl_target_interfaces(${TOFSENSE_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
# ament_target_dependencies(${TOFSENSE_NAME}
#       "rclcpp"
#     #   "rclcpp_action"
#     )

set(LINKTRACK_AOA_NAME "linktrack_aoa")
add_executable(${LINKTRACK_AOA_NAME}
    src/linktrack_aoa/init.cpp
    src/linktrack_aoa/main.cpp
    src/linktrack/protocols.cpp
    )
target_link_libraries(${LINKTRACK_AOA_NAME}
    ${nutils}
    )
  
# add_dependencies(${LINKTRACK_AOA_NAME} ${PROJECT_NAME}_generate_messages_cpp)
# rosidl_target_interfaces(${LINKTRACK_AOA_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(${LINKTRACK_AOA_NAME}
      "rclcpp"
      "nlink_parser_ros2_interfaces"
      "std_msgs"
      "geometry_msgs"
    )


add_executable(nlink_viz_ros2
    src/aoa_target.cpp
    )
target_link_libraries(nlink_viz_ros2
    ${nutils}
    )

ament_target_dependencies(nlink_viz_ros2
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

add_executable(nlink_viz_single_ros2
    src/target_single.cpp
    )
target_link_libraries(nlink_viz_single_ros2
    ${nutils}
    )

ament_target_dependencies(nlink_viz_single_ros2
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

# set(NLINK_TO_R2R2 "nlink_to_r2r2")
# add_executable(${NLINK_TO_R2R2}
#     src/r2r2_msg_formatter.cpp
#     )
# target_link_libraries(${NLINK_TO_R2R2}
#     ${nutils}
#     )

# ament_target_dependencies(${NLINK_TO_R2R2}
#   "rclcpp"
#   "nlink_parser_ros2_interfaces"
#   "r2r2_jedi_force_sensor_interfaces"
#   "std_msgs"
#   "geometry_msgs"
# )

#############
## Testing ##
#############
# if(${CATKIN_ENABLE_TESTING})
#     find_package(rostest REQUIRED)
# #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#     ## Add gtest based cpp test target and link libraries
#     catkin_add_gtest(${PROJECT_NAME}-test
#         src/linktrack/init.cpp
#         src/linktrack/protocols.cpp
#         src/tofsense/init.cpp
#         src/linktrack_aoa/init.cpp
#         test/test_nlink_parser.cpp
#         )
# endif()

# if(TARGET ${PROJECT_NAME}-test)
#     target_link_libraries(${PROJECT_NAME}-test  ${nutils})
# endif()

install(TARGETS
${LINKTRACK_AOA_NAME} 
${LINKTRACK_NAME}
nlink_viz_ros2
nlink_viz_single_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS
  ${LINKTRACK_AOA_NAME} 
  ${LINKTRACK_NAME}
  nlink_viz_ros2
  nlink_viz_single_ros2
    DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

