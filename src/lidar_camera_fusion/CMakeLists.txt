cmake_minimum_required(VERSION 3.8)
project(lidar_camera_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ─── ROS2 ament packages ────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# ─── System libraries ───────────────────────────────────────────────────────
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)

# ─── scan_assembler_node ────────────────────────────────────────────────────
add_executable(scan_assembler_node
  src/scan_assembler_node.cpp
)

target_include_directories(scan_assembler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(scan_assembler_node PRIVATE
  ${PCL_DEFINITIONS}
)

ament_target_dependencies(scan_assembler_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  std_srvs
  laser_geometry
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

target_link_libraries(scan_assembler_node
  ${PCL_LIBRARIES}
)

# ─── cloud_colorizer_node ───────────────────────────────────────────────────
add_executable(cloud_colorizer_node
  src/cloud_colorizer_node.cpp
)

target_include_directories(cloud_colorizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(cloud_colorizer_node PRIVATE
  ${PCL_DEFINITIONS}
)

ament_target_dependencies(cloud_colorizer_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  cv_bridge
  message_filters
)

target_link_libraries(cloud_colorizer_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# ─── Install ────────────────────────────────────────────────────────────────
install(TARGETS
  scan_assembler_node
  cloud_colorizer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# ─── ament finalization ──────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp sensor_msgs std_msgs geometry_msgs std_srvs
  laser_geometry tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs
  tf2_eigen pcl_conversions cv_bridge message_filters
)
ament_package()
