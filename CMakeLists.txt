cmake_minimum_required(VERSION 3.16)
project(second_project)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(goal_publisher_node src/goal_publisher_node.cpp)

ament_target_dependencies(goal_publisher_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS goal_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install everything in launch/, config/, maps/, csv/ into the share directory.
# get_package_share_directory('second_project') resolves to that share directory at runtime.
install(DIRECTORY launch config maps csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
