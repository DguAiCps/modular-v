cmake_minimum_required(VERSION 3.8)
project(rtabmap_module)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
# Note: We don't link against rtabmap_slam/sync/util to avoid opencv_aruco dependency
# Instead, we launch rtabmap nodes via ros2 launch and just monitor their output
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(core REQUIRED)

# Include directories
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

# RTABMap wrapper library
add_library(${PROJECT_NAME} SHARED
  src/rtabmap_wrapper.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  rtabmap_msgs
  ament_index_cpp
  core
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  rtabmap_msgs
  ament_index_cpp
  core
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
