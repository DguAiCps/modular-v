cmake_minimum_required(VERSION 3.16)
project(modular_v VERSION 1.0.0)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler optimizations for Jetson
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
  if(CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin
  endif()
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# External libraries
find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# CUDA (optional)
find_package(CUDA QUIET)
if(CUDA_FOUND)
  add_definitions(-DHAS_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
endif()

# Include directories
include_directories(
  core/include
  modules/perception/camera_module/include
  modules/perception/rtabmap_module/include
  modules/perception/detection_module/include
  modules/navigation/path_planner/include
  modules/mobility/base_controller/include
  modules/interaction/audio_interface/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "interfaces/msg/ModuleStatus.msg"
  "interfaces/msg/ObstacleInfo.msg"
  "interfaces/msg/NavigationCommand.msg"
  "interfaces/msg/HapticFeedback.msg"
  "interfaces/srv/SetDestination.srv"
  "interfaces/srv/EmergencyStop.srv"
  "interfaces/srv/SaveMap.srv"
  "interfaces/action/NavigateToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Core library
add_library(modular_v_core SHARED
  core/src/module_manager.cpp
)

ament_target_dependencies(modular_v_core
  rclcpp
  std_msgs
  yaml-cpp
)

target_link_libraries(modular_v_core
  yaml-cpp
)

# Camera module library
add_library(camera_module SHARED
  modules/perception/camera_module/src/zed_camera_wrapper.cpp
)

ament_target_dependencies(camera_module
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(camera_module
  modular_v_core
)

# RTAB-Map module library
add_library(rtabmap_module SHARED
  modules/perception/rtabmap_module/src/rtabmap_wrapper.cpp
)

ament_target_dependencies(rtabmap_module
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(rtabmap_module
  modular_v_core
)

# Navigation module library
add_library(navigation_module SHARED
  modules/navigation/path_planner/src/navigation_wrapper.cpp
)

ament_target_dependencies(navigation_module
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
)

target_link_libraries(navigation_module
  modular_v_core
)

# Motor controller library
add_library(motor_controller SHARED
  modules/mobility/base_controller/src/motor_controller.cpp
)

ament_target_dependencies(motor_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(motor_controller
  modular_v_core
)

# Voice interface library
add_library(voice_interface SHARED
  modules/interaction/audio_interface/src/voice_interface.cpp
)

ament_target_dependencies(voice_interface
  rclcpp
  std_msgs
)

target_link_libraries(voice_interface
  modular_v_core
)

# Main executable
add_executable(modular_v_main
  src/main.cpp
)

ament_target_dependencies(modular_v_main
  rclcpp
)

target_link_libraries(modular_v_main
  modular_v_core
  camera_module
  rtabmap_module
  navigation_module
  motor_controller
  voice_interface
)

# Install libraries
install(TARGETS
  modular_v_core
  camera_module
  rtabmap_module
  navigation_module
  motor_controller
  voice_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS modular_v_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  core/include/
  modules/perception/camera_module/include/
  modules/perception/rtabmap_module/include/
  modules/navigation/path_planner/include/
  modules/mobility/base_controller/include/
  modules/interaction/audio_interface/include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/setup_environment.sh
  scripts/system_health_check.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Add test executables here
endif()

ament_export_include_directories(include)
ament_export_libraries(
  modular_v_core
  camera_module
  rtabmap_module
  navigation_module
  motor_controller
  voice_interface
)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
)

ament_package()