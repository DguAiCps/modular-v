cmake_minimum_required(VERSION 3.8)
project(modular_v_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModuleStatus.msg"
  "msg/ObstacleInfo.msg"
  "msg/NavigationCommand.msg"
  "msg/HapticFeedback.msg"
  "srv/SetDestination.srv"
  "srv/EmergencyStop.srv"
  "srv/SaveMap.srv"
  "action/NavigateToGoal.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    unique_identifier_msgs
    action_msgs
)

ament_package()
