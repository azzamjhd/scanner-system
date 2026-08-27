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
find_package(ament_cmake_python REQUIRED)
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
# body_preprocess_node needs filters / segmentation / search / kdtree on top
# of the common+io that the other nodes already use.
find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  segmentation
  sample_consensus
  search
  kdtree
)
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

# ─── body_preprocess_node ───────────────────────────────────────────────────
# RANSAC bed-plane removal + Euclidean clustering; reads /scanner/colored_cloud
# and publishes /scanner/body_cloud.
add_executable(body_preprocess_node
  src/body_preprocess_node.cpp
)

target_include_directories(body_preprocess_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(body_preprocess_node PRIVATE
  ${PCL_DEFINITIONS}
)

ament_target_dependencies(body_preprocess_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_link_libraries(body_preprocess_node
  ${PCL_LIBRARIES}
)

# ─── Python module (fusion_gui, manual_segmentation_node) ───────────────────
ament_python_install_package(${PROJECT_NAME})

# ─── Install ────────────────────────────────────────────────────────────────
install(TARGETS
  scan_assembler_node
  cloud_colorizer_node
  body_preprocess_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the Python entry-point so `ros2 run lidar_camera_fusion fusion_gui`/`fusion_tui` works
install(PROGRAMS
  scripts/fusion_gui
  scripts/fusion_tui
  DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ─── Python scripts ─────────────────────────────────────────────────────────
install(PROGRAMS
  scripts/scan_image_recorder_node
  scripts/manual_segmentation_node
  scripts/scan_cycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# ─── ament finalization ──────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp sensor_msgs std_msgs geometry_msgs std_srvs
  laser_geometry tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs
  tf2_eigen pcl_conversions cv_bridge message_filters
)
ament_package()
