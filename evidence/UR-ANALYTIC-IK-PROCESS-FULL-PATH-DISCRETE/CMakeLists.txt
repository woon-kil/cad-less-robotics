cmake_minimum_required(VERSION 3.16)
project(ur_analytic_ik_full_path LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(descartes_light REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenSSL REQUIRED)

function(add_archived_executable target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} PRIVATE
    descartes::descartes_light
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    moveit_core::moveit_planning_scene
    moveit_core::moveit_collision_detection
    rclcpp::rclcpp
    urdf::urdf
    srdfdom::srdfdom
    ament_index_cpp::ament_index_cpp
    OpenSSL::Crypto)
endfunction()

add_archived_executable(ur_analytic_ik_fullgraph
  ${CMAKE_CURRENT_LIST_DIR}/../UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH-CORRESPONDENCE/full-graph-src.cpp)
add_archived_executable(ur_analytic_ik_restorecheck
  ${CMAKE_CURRENT_LIST_DIR}/../UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH-RESTORE/restore-src.cpp)
