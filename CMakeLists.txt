cmake_minimum_required(VERSION 3.8)
project(move_to_goal_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

add_library(MoveToGoalNode SHARED
  src/MoveToGoalNode.cpp
)

ament_target_dependencies(MoveToGoalNode
  rclcpp
  rclcpp_action
  nav2_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

install(
  TARGETS MoveToGoalNode
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
