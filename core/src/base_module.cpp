#include "core/base_module.hpp"
#include <fstream>

namespace modular_v {
namespace core {

BaseModule::BaseModule(const std::string& name, const std::string& version) {
    // Set module info
    name_ = name;
    version_ = version;

    // Create ROS2 node for this module
    node_ = rclcpp::Node::make_shared(name);

    // Initialize watchdog
    last_watchdog_update_ = std::chrono::steady_clock::now();

    // Create status publisher
    std::string status_topic = "/" + name + "/status";
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        status_topic, 10);

    // Create health check timer (check every 1 second)
    health_check_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&BaseModule::healthCheckTimerCallback, this));

    // Create status publishing timer (publish every 2 seconds)
    status_timer_ = node_->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&BaseModule::publishStatus, this));

    health_status_message_ = "Module created";

    RCLCPP_INFO(node_->get_logger(), "Module %s v%s constructed",
        name_.c_str(), version_.c_str());
}

BaseModule::~BaseModule() {
    if (state_ != ModuleState::TERMINATED) {
        shutdown();
    }
}

bool BaseModule::initialize() {
    if (state_ != ModuleState::UNINITIALIZED) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s: Cannot initialize from state %d", name_.c_str(), static_cast<int>(state_));
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Initializing module %s...", name_.c_str());

    if (!onInitialize()) {
        RCLCPP_ERROR(node_->get_logger(), "Module %s initialization failed", name_.c_str());
        state_ = ModuleState::ERROR;
        health_status_message_ = "Initialization failed";
        return false;
    }

    state_ = ModuleState::INITIALIZED;
    health_status_message_ = "Initialized successfully";
    updateWatchdog();

    RCLCPP_INFO(node_->get_logger(), "Module %s initialized", name_.c_str());
    return true;
}

bool BaseModule::start() {
    if (state_ != ModuleState::INITIALIZED && state_ != ModuleState::PAUSED) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s: Cannot start from state %d", name_.c_str(), static_cast<int>(state_));
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Starting module %s...", name_.c_str());

    if (!onStart()) {
        RCLCPP_ERROR(node_->get_logger(), "Module %s start failed", name_.c_str());
        state_ = ModuleState::ERROR;
        health_status_message_ = "Start failed";
        return false;
    }

    state_ = ModuleState::RUNNING;
    health_status_message_ = "Running";
    updateWatchdog();

    RCLCPP_INFO(node_->get_logger(), "Module %s started", name_.c_str());
    return true;
}

bool BaseModule::pause() {
    if (state_ != ModuleState::RUNNING) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s: Cannot pause from state %d", name_.c_str(), static_cast<int>(state_));
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Pausing module %s...", name_.c_str());

    if (!onPause()) {
        RCLCPP_ERROR(node_->get_logger(), "Module %s pause failed", name_.c_str());
        state_ = ModuleState::ERROR;
        health_status_message_ = "Pause failed";
        return false;
    }

    state_ = ModuleState::PAUSED;
    health_status_message_ = "Paused";
    updateWatchdog();

    RCLCPP_INFO(node_->get_logger(), "Module %s paused", name_.c_str());
    return true;
}

bool BaseModule::resume() {
    if (state_ != ModuleState::PAUSED) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s: Cannot resume from state %d", name_.c_str(), static_cast<int>(state_));
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Resuming module %s...", name_.c_str());

    if (!onResume()) {
        RCLCPP_ERROR(node_->get_logger(), "Module %s resume failed", name_.c_str());
        state_ = ModuleState::ERROR;
        health_status_message_ = "Resume failed";
        return false;
    }

    state_ = ModuleState::RUNNING;
    health_status_message_ = "Running";
    updateWatchdog();

    RCLCPP_INFO(node_->get_logger(), "Module %s resumed", name_.c_str());
    return true;
}

bool BaseModule::stop() {
    if (state_ != ModuleState::RUNNING && state_ != ModuleState::PAUSED &&
        state_ != ModuleState::ERROR) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s: Cannot stop from state %d", name_.c_str(), static_cast<int>(state_));
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Stopping module %s...", name_.c_str());

    if (!onStop()) {
        RCLCPP_ERROR(node_->get_logger(), "Module %s stop failed", name_.c_str());
        state_ = ModuleState::ERROR;
        health_status_message_ = "Stop failed";
        return false;
    }

    state_ = ModuleState::INITIALIZED;
    health_status_message_ = "Stopped";
    updateWatchdog();

    RCLCPP_INFO(node_->get_logger(), "Module %s stopped", name_.c_str());
    return true;
}

bool BaseModule::shutdown() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down module %s...", name_.c_str());

    // Stop first if running
    if (state_ == ModuleState::RUNNING || state_ == ModuleState::PAUSED) {
        stop();
    }

    if (!onShutdown()) {
        RCLCPP_ERROR(node_->get_logger(), "Module %s shutdown failed", name_.c_str());
        state_ = ModuleState::ERROR;
        health_status_message_ = "Shutdown failed";
        return false;
    }

    state_ = ModuleState::TERMINATED;
    health_status_message_ = "Terminated";

    // Cancel timers
    if (health_check_timer_) {
        health_check_timer_->cancel();
    }
    if (status_timer_) {
        status_timer_->cancel();
    }

    RCLCPP_INFO(node_->get_logger(), "Module %s shutdown complete", name_.c_str());
    return true;
}

bool BaseModule::loadConfig(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        // Load watchdog timeout if specified
        if (config["watchdog_timeout_seconds"]) {
            watchdog_timeout_ = std::chrono::seconds(
                config["watchdog_timeout_seconds"].as<int>());
        }

        // Call derived class config loader
        if (!loadModuleConfig(config)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Module %s: Failed to load module-specific config", name_.c_str());
            return false;
        }

        config_file_path_ = config_path;
        RCLCPP_INFO(node_->get_logger(),
            "Module %s: Config loaded from %s", name_.c_str(), config_path.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "Module %s: Failed to load config: %s", name_.c_str(), e.what());
        return false;
    }
}

bool BaseModule::saveConfig(const std::string& config_path) {
    try {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "module_name" << YAML::Value << name_;
        out << YAML::Key << "module_version" << YAML::Value << version_;
        out << YAML::Key << "watchdog_timeout_seconds"
            << YAML::Value << watchdog_timeout_.count();

        // Call derived class config saver
        if (!saveModuleConfig(out)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Module %s: Failed to save module-specific config", name_.c_str());
            return false;
        }

        out << YAML::EndMap;

        std::ofstream file(config_path);
        file << out.c_str();
        file.close();

        config_file_path_ = config_path;
        RCLCPP_INFO(node_->get_logger(),
            "Module %s: Config saved to %s", name_.c_str(), config_path.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "Module %s: Failed to save config: %s", name_.c_str(), e.what());
        return false;
    }
}

bool BaseModule::isHealthy() const {
    // Check watchdog timeout
    auto now = std::chrono::steady_clock::now();
    auto last_update = last_watchdog_update_.load();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update);

    if (elapsed > watchdog_timeout_) {
        return false;
    }

    // Check if module is in error state
    if (state_ == ModuleState::ERROR) {
        return false;
    }

    // Perform module-specific health check
    return const_cast<BaseModule*>(this)->performHealthCheck();
}

std::string BaseModule::getHealthStatus() const {
    if (!isHealthy()) {
        auto now = std::chrono::steady_clock::now();
        auto last_update = last_watchdog_update_.load();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update);

        if (elapsed > watchdog_timeout_) {
            return "UNHEALTHY: Watchdog timeout (" +
                   std::to_string(elapsed.count()) + "s since last update)";
        }

        if (state_ == ModuleState::ERROR) {
            return "UNHEALTHY: Error state - " + health_status_message_;
        }
    }

    return health_status_message_;
}

void BaseModule::updateWatchdog() {
    last_watchdog_update_ = std::chrono::steady_clock::now();
}

void BaseModule::healthCheckTimerCallback() {
    // This runs periodically to check health
    // Derived classes should call updateWatchdog() during their operation
    if (!isHealthy() && state_ == ModuleState::RUNNING) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s health check failed: %s",
            name_.c_str(), getHealthStatus().c_str());
    }
}

void BaseModule::publishStatus() {
    auto msg = std_msgs::msg::String();

    std::string state_str;
    switch (state_) {
        case ModuleState::UNINITIALIZED: state_str = "UNINITIALIZED"; break;
        case ModuleState::INITIALIZED: state_str = "INITIALIZED"; break;
        case ModuleState::RUNNING: state_str = "RUNNING"; break;
        case ModuleState::PAUSED: state_str = "PAUSED"; break;
        case ModuleState::ERROR: state_str = "ERROR"; break;
        case ModuleState::TERMINATED: state_str = "TERMINATED"; break;
    }

    msg.data = name_ + " [" + state_str + "]: " + health_status_message_;
    status_publisher_->publish(msg);
}

} // namespace core
} // namespace modular_v
