# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
Modular-V is a ROS2-based autonomous navigation framework for visually impaired assistance robots, integrating RTAB-Map SLAM, ZED 2i stereo camera, and voice/haptic interfaces on NVIDIA Jetson platforms.

## Key Dependencies
Pre-installed ROS2 packages this framework wraps:
- **rtabmap_ros**: SLAM functionality (rtabmap_slam, rtabmap_launch, etc.)
- **zed_ros2**: ZED camera integration (zed_wrapper, zed_components, etc.)
- **navigation2**: Path planning and control (nav2_bringup, nav2_planner, etc.)

The framework adds safety features, state management, and unified control over these packages.

## Essential Commands

### Build
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
# Jetson optimized: add --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=87
```

### Run
```bash
ros2 launch modular_v system_bringup.launch.py  # Full system
ros2 launch zed_wrapper zed_camera.launch.py    # ZED only (debug)
ros2 launch rtabmap_launch rtabmap.launch.py    # RTAB-Map only (debug)
```

### Monitor
```bash
ros2 topic echo /system/status        # Health monitoring
ros2 node list                        # Active nodes
rviz2 -d config/modular_v.rviz       # Visualization
```

## Architecture

### Core Design
- **Module System**: Plugin-based architecture with `IModule` interface (`core/include/core/module_interface.hpp`)
- **Wrapper Pattern**: Modules wrap existing ROS2 packages (rtabmap_ros, zed_ros2, nav2) adding safety and state management
- **Communication**: ROS2 topics/services with appropriate QoS (sensor: best_effort, control: reliable)
- **State Machine**: UNINITIALIZED → INITIALIZED → RUNNING → TERMINATED
- **Safety**: Emergency stop service, watchdog timers, velocity limits

### Module Dependencies
```
ZED Camera → RTAB-Map → Navigation → Motor Control
```

### Platform Notes
- **Jetson AGX Orin**: CUDA arch 87, TensorRT inference, 32GB unified memory
- **ZED 2i**: 15Hz@720p, ULTRA depth mode, IMU fusion enabled
- **Config Files**: `config/system_config.yaml`, `config/navigation_params.yaml`

## C++ Coding Conventions

### Naming
- **Files**: `snake_case.cpp/hpp`
- **Classes**: `PascalCase`
- **Functions**: `camelCase()`
- **Members**: `member_var_` (trailing underscore)
- **Constants**: `UPPER_SNAKE_CASE` or `kPascalCase`
- **ROS2 topics**: `/snake_case`

### Class Structure Order
1. Constructor/Destructor
2. Deleted/Default functions
3. Public interface (inherited first)
4. Private methods
5. ROS2 callbacks
6. Member variables (ROS2 objects → threads → data → config)

### Key Patterns
```cpp
// ROS2 message callbacks use SharedPtr
void callback(const sensor_msgs::msg::Image::SharedPtr msg);

// Logging
RCLCPP_INFO(node_->get_logger(), "Message: %s", text.c_str());

// QoS profiles
auto sensor_qos = rclcpp::QoS(10).best_effort();
auto control_qos = rclcpp::QoS(10).reliable();

// Thread safety
std::lock_guard<std::mutex> lock(mutex_);
std::atomic<bool> running_{false};
```

### Documentation
Use Doxygen style (`@brief`, `@param`, `@return`) for public APIs