cmake_minimum_required(VERSION 3.16)
project(ur_analytic_ik_reproduce LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED TESSERACT_SRC OR NOT DEFINED TESSERACT_OBJECTS)
  message(FATAL_ERROR "TESSERACT_SRC and TESSERACT_OBJECTS are required")
endif()

set(REPO_SCRIPTS ${CMAKE_CURRENT_LIST_DIR}/../scripts)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(descartes_light REQUIRED)
find_package(OpenMP REQUIRED)

function(add_moveit_tool name)
  add_executable(${name} ${REPO_SCRIPTS}/${name}.cpp)
  target_link_libraries(${name} PRIVATE
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    rclcpp::rclcpp
    urdf::urdf
    srdfdom::srdfdom)
endfunction()

add_moveit_tool(ur_analytic_ik_sampled_path_check)
add_moveit_tool(ur_analytic_ik_process_connect)
target_link_libraries(ur_analytic_ik_process_connect PRIVATE descartes::descartes_light)

add_executable(ur_analytic_ik_process_connect_r2 ${REPO_SCRIPTS}/ur_analytic_ik_process_connect_r2.cpp)
target_link_libraries(ur_analytic_ik_process_connect_r2 PRIVATE
  descartes::descartes_light
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  rclcpp::rclcpp
  OpenMP::OpenMP_CXX)

function(add_ik_tool name)
  add_executable(${name}
    ${REPO_SCRIPTS}/${name}.cpp
    ${TESSERACT_OBJECTS}/ur_inv_kin.o
    ${TESSERACT_OBJECTS}/inverse_kinematics.o
    ${TESSERACT_OBJECTS}/types.o
    ${TESSERACT_OBJECTS}/utils.o
    ${TESSERACT_OBJECTS}/logging.o)
  target_include_directories(${name} PRIVATE
    ${TESSERACT_SRC}/kinematics/ur/include
    ${TESSERACT_SRC}/kinematics/core/include
    ${TESSERACT_SRC}/common/include
    ${TESSERACT_SRC}/scene_graph/include
    /usr/include/eigen3)
  target_link_libraries(${name} PRIVATE
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    rclcpp::rclcpp
    urdf::urdf
    srdfdom::srdfdom
    fmt
    spdlog)
endfunction()

add_ik_tool(ur_analytic_ik_fit)
add_ik_tool(ur_analytic_ik_survey)

add_executable(ur_analytic_ik_process_connect_r3
  ${REPO_SCRIPTS}/ur_analytic_ik_process_connect_r3.cpp
  ${TESSERACT_OBJECTS}/ur_inv_kin.o
  ${TESSERACT_OBJECTS}/inverse_kinematics.o
  ${TESSERACT_OBJECTS}/types.o
  ${TESSERACT_OBJECTS}/utils.o
  ${TESSERACT_OBJECTS}/logging.o)
target_include_directories(ur_analytic_ik_process_connect_r3 PRIVATE
  ${TESSERACT_SRC}/kinematics/ur/include
  ${TESSERACT_SRC}/kinematics/core/include
  ${TESSERACT_SRC}/common/include
  ${TESSERACT_SRC}/scene_graph/include
  /usr/include/eigen3)
target_link_libraries(ur_analytic_ik_process_connect_r3 PRIVATE
  descartes::descartes_light
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  rclcpp::rclcpp
  urdf::urdf
  srdfdom::srdfdom
  OpenMP::OpenMP_CXX
  fmt
  spdlog)
