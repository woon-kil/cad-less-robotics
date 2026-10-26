cmake_minimum_required(VERSION 3.16)
project(vision_path_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(polyline_core SHARED src/polyline.cpp)
target_include_directories(polyline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(polyline_core PUBLIC Eigen3::Eigen)
target_compile_features(polyline_core PUBLIC cxx_std_17)

add_library(path_extractor_core SHARED src/path_extractor.cpp)
target_include_directories(path_extractor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(path_extractor_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(path_extractor_core PUBLIC polyline_core ${PCL_LIBRARIES})
target_compile_features(path_extractor_core PUBLIC cxx_std_17)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(camera_node ${PCL_LIBRARIES})
ament_target_dependencies(camera_node rclcpp sensor_msgs pcl_conversions)

add_executable(path_extractor_node src/path_extractor_node.cpp)
target_include_directories(path_extractor_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(path_extractor_node path_extractor_core ${PCL_LIBRARIES})
ament_target_dependencies(path_extractor_node
  rclcpp sensor_msgs geometry_msgs std_msgs pcl_conversions tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS polyline_core path_extractor_core camera_node path_extractor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_polyline test/test_polyline.cpp)
  target_link_libraries(test_polyline polyline_core)
endif()

ament_package()
