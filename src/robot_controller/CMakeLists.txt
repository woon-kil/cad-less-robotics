cmake_minimum_required(VERSION 3.16)
project(robot_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(waypoint_prep SHARED src/waypoint_prep.cpp)
target_include_directories(waypoint_prep PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(waypoint_prep PUBLIC Eigen3::Eigen)
target_compile_features(waypoint_prep PUBLIC cxx_std_17)

add_executable(motion_planner_node src/motion_planner_node.cpp)
target_link_libraries(motion_planner_node waypoint_prep)
ament_target_dependencies(motion_planner_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs)

install(TARGETS waypoint_prep motion_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_prep test/test_waypoint_prep.cpp)
  target_link_libraries(test_waypoint_prep waypoint_prep)
endif()

ament_package()
