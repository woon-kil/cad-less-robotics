cmake_minimum_required(VERSION 3.16)
project(dofbot_moveit_dry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(urdf REQUIRED)

add_executable(dry_plan_node src/dry_plan_node.cpp)
ament_target_dependencies(dry_plan_node
  rclcpp
  sensor_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  urdf)

install(TARGETS dry_plan_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
