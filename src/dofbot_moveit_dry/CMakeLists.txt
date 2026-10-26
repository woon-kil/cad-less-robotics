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
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_controller REQUIRED)
find_package(vision_path_generator REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(dry_plan_node src/dry_plan_node.cpp)
ament_target_dependencies(dry_plan_node
  rclcpp
  sensor_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  urdf)

add_executable(geometry_motion_dry src/geometry_motion_dry.cpp)
target_link_libraries(geometry_motion_dry
  robot_controller::waypoint_prep
  vision_path_generator::polyline_core
  Eigen3::Eigen)
ament_target_dependencies(geometry_motion_dry
  rclcpp
  sensor_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  urdf)

add_executable(ik_feasibility_dry src/ik_feasibility_dry.cpp)
target_link_libraries(ik_feasibility_dry
  robot_controller::waypoint_prep
  vision_path_generator::polyline_core
  Eigen3::Eigen)
ament_target_dependencies(ik_feasibility_dry
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  urdf
  pluginlib)

add_executable(capability_adapter_dry src/capability_adapter_dry.cpp)
target_link_libraries(capability_adapter_dry
  robot_controller::waypoint_prep
  vision_path_generator::polyline_core
  Eigen3::Eigen)
ament_target_dependencies(capability_adapter_dry
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  urdf
  pluginlib)

install(TARGETS dry_plan_node geometry_motion_dry ik_feasibility_dry capability_adapter_dry
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
