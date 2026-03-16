cmake_minimum_required(VERSION 3.16...3.26)
project(form VERSION 0.1.0 LANGUAGES CXX) # x-release-please-version

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build the core FORM library from the parent directory
# Need PIC since FORM (static) gets linked into a shared library (form_odometry_node)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/form_core)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS 2 node
add_library(form_odometry_node SHARED src/node.cpp)
target_compile_features(form_odometry_node PUBLIC cxx_std_17)
target_include_directories(form_odometry_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(form_odometry_node FORM)
ament_target_dependencies(
  form_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  rcutils
  sensor_msgs
  std_msgs
  tf2_ros)

rclcpp_components_register_node(form_odometry_node PLUGIN "form_ros::EstimatorNode" EXECUTABLE form_node)

install(TARGETS form_odometry_node LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()