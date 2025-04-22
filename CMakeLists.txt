cmake_minimum_required(VERSION 3.16)
project(radar_object_merger)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(
  radar_object_merger_node
  SHARED
  src/radar_object_merger.cpp
)

target_include_directories(
  radar_object_merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(radar_object_merger_node PUBLIC cxx_std_17)

ament_target_dependencies(
  radar_object_merger_node PUBLIC
  rclcpp
  rclcpp_components
  autoware_perception_msgs
  geometry_msgs
  std_msgs
  autoware_utils
)

rclcpp_components_register_node(
  radar_object_merger_node
  PLUGIN radar_object_merger::RadarObjectMerger
  EXECUTABLE radar_object_merger_node_exe
)

install(
  TARGETS radar_object_merger_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS radar_object_merger_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)