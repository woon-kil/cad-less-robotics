cmake_minimum_required(VERSION 3.16)
project(ur_analytic_ik_quintic_timescale LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(ur_analytic_ik_quintic_timescale quintic_timescale.cpp)
target_link_libraries(ur_analytic_ik_quintic_timescale PRIVATE
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_planning_scene
  moveit_core::moveit_collision_detection
  rclcpp::rclcpp
  urdf::urdf
  srdfdom::srdfdom
  ament_index_cpp::ament_index_cpp
  OpenSSL::Crypto)
