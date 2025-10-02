# Core Framework - Claude Code Guide

## Overview
Core framework provides the foundation for the modular-v system with module lifecycle management, dependency resolution, and health monitoring.

## Key Components

### Module Interface (`module_interface.hpp`)
Base interface all modules must implement:
```cpp
class IModule {
    virtual bool initialize() = 0;
    virtual bool start() = 0;
    virtual bool pause() = 0;
    virtual bool resume() = 0;
    virtual bool stop() = 0;
    virtual bool shutdown() = 0;
    // ... state, config, health methods
};
```

**State Machine**: UNINITIALIZED → INITIALIZED → RUNNING → PAUSED/ERROR → TERMINATED

### BaseModule (`base_module.hpp/cpp`)
Recommended base class for all modules. Provides:
- **Lifecycle management**: Automatic state transitions with logging
- **Watchdog timer**: 5-second timeout (configurable via YAML)
- **Status publishing**: Auto-publishes to `/<module_name>/status`
- **Config handling**: YAML load/save with `loadModuleConfig()` override

**Usage Pattern**:
```cpp
class MyModule : public BaseModule {
public:
    MyModule() : BaseModule("my_module", "1.0.0") {}
protected:
    bool onInitialize() override {
        // Custom initialization
        updateWatchdog();  // Must call periodically
        return true;
    }
    bool onStart() override { /* ... */ }
};
```

### ModuleManager (`module_manager.hpp/cpp`)
Central manager for all system modules:
- **Dependency resolution**: Topological sort with circular dependency detection
- **Auto-recovery**: Monitors health, restarts failed modules (max 3 attempts)
- **ROS2 services**:
  - `/system/status` topic (1Hz)
  - `/system/emergency_stop` service

**Registration Example**:
```cpp
ModuleManager mgr;
auto cam_module = std::make_shared<ZedCameraWrapper>();
mgr.registerModule("camera", cam_module);
mgr.registerModule("rtabmap", rtabmap_module, {"camera"});  // depends on camera
mgr.initializeSystem();
mgr.startSystem();
```

**Initialization order**: Automatically computed via topological sort based on dependencies.

### Utilities (`utils.hpp`)
Header-only utilities:
- **State conversion**: `moduleStateToString()`, `stringToModuleState()`
- **Timing**: `Timer`, `ScopedTimer` (RAII logging)
- **Math helpers**: `clamp()`, `lerp()`, `normalizeAngle()`

## Implementation Checklist

When creating a new module:
1. ✅ Inherit from `BaseModule` (easier) or `IModule` (full control)
2. ✅ Override `onInitialize()`, `onStart()`, `onStop()` at minimum
3. ✅ Call `updateWatchdog()` periodically in main loop/callbacks
4. ✅ Use `loadModuleConfig(YAML::Node)` for custom config
5. ✅ Register with ModuleManager including dependencies
6. ✅ Add to CMakeLists.txt if creating new library

## Error Recovery

**Auto-recovery** (enabled by default):
- Health check every 2 seconds
- Unhealthy modules auto-restarted (max 3 attempts)
- Counter resets when module becomes healthy

**Manual control**:
```cpp
mgr.enableAutoRecovery(false);          // Disable auto-recovery
mgr.setMaxRestartAttempts(5);           // Change max attempts
mgr.restartModule("camera");            // Manual restart
```

## Testing
All modules should validate:
- State transitions (UNINITIALIZED → INITIALIZED → RUNNING → TERMINATED)
- Dependency ordering (dependent modules start after dependencies)
- Health checks (watchdog timeout, custom health checks)
- Config loading/saving

## Common Patterns

**Watchdog in callbacks**:
```cpp
void MyModule::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    updateWatchdog();  // Reset watchdog on activity
    // Process image
}
```

**Custom health check**:
```cpp
bool MyModule::performHealthCheck() override {
    return data_received_ && (rclcpp::Clock().now() - last_msg_time_).seconds() < 5.0;
}
```

**Scoped timing**:
```cpp
void processData() {
    ScopedTimer timer("processData", node_->get_logger());
    // Processing code
}  // Automatically logs: "processData took X.XXX ms"
```
