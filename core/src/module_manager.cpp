#include "core/module_manager.hpp"
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sstream>
#include <thread>

namespace modular_v {
namespace core {

ModuleManager::ModuleManager() {
    node_ = rclcpp::Node::make_shared("module_manager");

    // Create status publisher
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/system/status", 10);

    // Create emergency stop service
    emergency_stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/system/emergency_stop",
        std::bind(&ModuleManager::emergencyStopCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Create status timer (publish every 1 second)
    status_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ModuleManager::statusTimerCallback, this));

    // Create health monitor timer (check every 2 seconds)
    health_monitor_timer_ = node_->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&ModuleManager::healthMonitorTimerCallback, this));

    RCLCPP_INFO(node_->get_logger(), "ModuleManager initialized");
}

ModuleManager::~ModuleManager() {
    if (!modules_.empty()) {
        shutdownSystem();
    }
}

bool ModuleManager::registerModule(const std::string& name,
                                  std::shared_ptr<IModule> module,
                                  const std::vector<std::string>& dependencies) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    if (modules_.find(name) != modules_.end()) {
        RCLCPP_WARN(node_->get_logger(),
            "Module %s already registered", name.c_str());
        return false;
    }

    modules_[name] = module;
    dependencies_[name] = dependencies;

    RCLCPP_INFO(node_->get_logger(),
        "Module %s registered successfully", name.c_str());
    return true;
}

bool ModuleManager::unregisterModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        return false;
    }

    if (it->second->getState() == ModuleState::RUNNING) {
        it->second->stop();
    }

    modules_.erase(it);
    dependencies_.erase(name);
    return true;
}

bool ModuleManager::startModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Module %s not found", name.c_str());
        return false;
    }

    if (!checkDependencies(name)) {
        RCLCPP_ERROR(node_->get_logger(),
            "Dependencies for module %s are not satisfied", name.c_str());
        return false;
    }

    return it->second->start();
}

bool ModuleManager::stopModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        return false;
    }

    return it->second->stop();
}

bool ModuleManager::pauseModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        return false;
    }

    return it->second->pause();
}

bool ModuleManager::resumeModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        return false;
    }

    return it->second->resume();
}

std::shared_ptr<IModule> ModuleManager::getModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        return nullptr;
    }

    return it->second;
}

std::vector<std::string> ModuleManager::getModuleList() const {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    std::vector<std::string> list;
    for (const auto& pair : modules_) {
        list.push_back(pair.first);
    }
    return list;
}

ModuleState ModuleManager::getModuleState(const std::string& name) const {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        return ModuleState::UNINITIALIZED;
    }

    return it->second->getState();
}

bool ModuleManager::initializeSystem() {
    RCLCPP_INFO(node_->get_logger(), "Initializing system...");

    auto init_order = getInitializationOrder();

    // Check if initialization order is valid
    if (init_order.empty() && !modules_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to determine initialization order (circular dependency or other error)");
        return false;
    }

    for (const auto& module_name : init_order) {
        auto module = modules_[module_name];
        if (!module->initialize()) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to initialize module %s", module_name.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
            "Module %s initialized", module_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "System initialization complete");
    return true;
}

bool ModuleManager::startSystem() {
    RCLCPP_INFO(node_->get_logger(), "Starting system...");

    auto init_order = getInitializationOrder();

    // Check if initialization order is valid
    if (init_order.empty() && !modules_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to determine start order (circular dependency or other error)");
        return false;
    }

    for (const auto& module_name : init_order) {
        if (!startModule(module_name)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to start module %s", module_name.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
            "Module %s started", module_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "System started successfully");
    return true;
}

bool ModuleManager::stopSystem() {
    RCLCPP_INFO(node_->get_logger(), "Stopping system...");

    auto init_order = getInitializationOrder();
    std::reverse(init_order.begin(), init_order.end());

    for (const auto& module_name : init_order) {
        stopModule(module_name);
        RCLCPP_INFO(node_->get_logger(),
            "Module %s stopped", module_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "System stopped");
    return true;
}

bool ModuleManager::shutdownSystem() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down system...");

    stopSystem();

    for (auto& pair : modules_) {
        pair.second->shutdown();
    }

    modules_.clear();
    dependencies_.clear();

    RCLCPP_INFO(node_->get_logger(), "System shutdown complete");
    return true;
}

bool ModuleManager::emergencyStop() {
    RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP TRIGGERED!");

    emergency_stop_triggered_ = true;

    // Stop all modules immediately
    for (auto& pair : modules_) {
        pair.second->stop();
    }

    return true;
}

bool ModuleManager::loadSystemConfig(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        // Load configuration for each module
        for (auto& pair : modules_) {
            if (config[pair.first]) {
                // Module-specific config loading would go here
                RCLCPP_INFO(node_->get_logger(),
                    "Loaded config for module %s", pair.first.c_str());
            }
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to load config: %s", e.what());
        return false;
    }
}

bool ModuleManager::saveSystemConfig(const std::string& config_path) {
    // Implementation for saving configuration
    return true;
}

bool ModuleManager::checkSystemHealth() {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    for (const auto& pair : modules_) {
        if (!pair.second->isHealthy()) {
            RCLCPP_WARN(node_->get_logger(),
                "Module %s is unhealthy: %s",
                pair.first.c_str(),
                pair.second->getHealthStatus().c_str());
            return false;
        }
    }

    return true;
}

std::map<std::string, bool> ModuleManager::getModuleHealthStatus() {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    std::map<std::string, bool> health_status;
    for (const auto& pair : modules_) {
        health_status[pair.first] = pair.second->isHealthy();
    }

    return health_status;
}

std::vector<std::string> ModuleManager::getInitializationOrder() {
    std::vector<std::string> order;

    if (!topologicalSort(order)) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to compute initialization order (circular dependency?)");
        // Return empty vector on failure
        order.clear();
    }

    return order;
}

bool ModuleManager::topologicalSort(std::vector<std::string>& sorted_modules) {
    // Check for circular dependencies first
    std::map<std::string, int> visited;
    for (const auto& pair : modules_) {
        visited[pair.first] = 0;  // 0 = unvisited
    }

    for (const auto& pair : modules_) {
        if (hasCircularDependency(pair.first, visited)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Circular dependency detected involving module: %s", pair.first.c_str());
            return false;
        }
    }

    // Kahn's algorithm for topological sort
    std::map<std::string, int> in_degree;
    for (const auto& pair : modules_) {
        in_degree[pair.first] = 0;
    }

    // Calculate in-degrees
    for (const auto& pair : dependencies_) {
        for (const auto& dep : pair.second) {
            if (in_degree.find(dep) != in_degree.end()) {
                in_degree[pair.first]++;
            }
        }
    }

    // Queue for modules with no dependencies
    std::vector<std::string> queue;
    for (const auto& pair : in_degree) {
        if (pair.second == 0) {
            queue.push_back(pair.first);
        }
    }

    // Process modules
    while (!queue.empty()) {
        std::string current = queue.back();
        queue.pop_back();
        sorted_modules.push_back(current);

        // Find modules that depend on current
        for (const auto& pair : dependencies_) {
            bool depends_on_current = false;
            for (const auto& dep : pair.second) {
                if (dep == current) {
                    depends_on_current = true;
                    break;
                }
            }

            if (depends_on_current) {
                in_degree[pair.first]--;
                if (in_degree[pair.first] == 0) {
                    queue.push_back(pair.first);
                }
            }
        }
    }

    // Check if all modules were processed
    if (sorted_modules.size() != modules_.size()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Topological sort failed: %zu modules sorted, %zu total",
            sorted_modules.size(), modules_.size());
        return false;
    }

    return true;
}

bool ModuleManager::hasCircularDependency(const std::string& module,
                                         std::map<std::string, int>& visited,
                                         const std::string& current) {
    std::string check_module = current.empty() ? module : current;

    // If already being visited (gray node), circular dependency found
    if (visited[check_module] == 1) {
        return true;
    }

    // If already completely visited (black node), no circular dependency
    if (visited[check_module] == 2) {
        return false;
    }

    // Mark as being visited (gray)
    visited[check_module] = 1;

    // Check dependencies
    auto deps_it = dependencies_.find(check_module);
    if (deps_it != dependencies_.end()) {
        for (const auto& dep : deps_it->second) {
            if (hasCircularDependency(module, visited, dep)) {
                return true;
            }
        }
    }

    // Mark as completely visited (black)
    visited[check_module] = 2;
    return false;
}

bool ModuleManager::checkDependencies(const std::string& module) {
    auto deps_it = dependencies_.find(module);
    if (deps_it == dependencies_.end()) {
        return true;
    }

    for (const auto& dep : deps_it->second) {
        auto module_it = modules_.find(dep);
        if (module_it == modules_.end() ||
            module_it->second->getState() != ModuleState::RUNNING) {
            return false;
        }
    }

    return true;
}

void ModuleManager::publishSystemStatus() {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    std::stringstream ss;
    ss << "System Status:\n";
    ss << "  Total Modules: " << modules_.size() << "\n";
    ss << "  Emergency Stop: " << (emergency_stop_triggered_ ? "ACTIVE" : "Inactive") << "\n";

    for (const auto& pair : modules_) {
        const auto& module = pair.second;
        ss << "  - " << pair.first << ": ";

        switch (module->getState()) {
            case ModuleState::UNINITIALIZED:
                ss << "UNINITIALIZED";
                break;
            case ModuleState::INITIALIZED:
                ss << "INITIALIZED";
                break;
            case ModuleState::RUNNING:
                ss << "RUNNING";
                break;
            case ModuleState::PAUSED:
                ss << "PAUSED";
                break;
            case ModuleState::ERROR:
                ss << "ERROR";
                break;
            case ModuleState::TERMINATED:
                ss << "TERMINATED";
                break;
        }

        ss << " [" << (module->isHealthy() ? "Healthy" : "Unhealthy") << "]\n";
    }

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    status_publisher_->publish(msg);
}

void ModuleManager::emergencyStopCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request_header;  // Unused
    (void)request;         // Unused

    RCLCPP_WARN(node_->get_logger(), "Emergency stop service called!");

    bool success = emergencyStop();

    response->success = success;
    response->message = success ? "Emergency stop executed successfully"
                                : "Emergency stop failed";
}

void ModuleManager::statusTimerCallback() {
    publishSystemStatus();
}

void ModuleManager::healthMonitorTimerCallback() {
    if (!auto_recovery_enabled_ || emergency_stop_triggered_) {
        return;
    }

    std::lock_guard<std::mutex> lock(modules_mutex_);

    for (const auto& pair : modules_) {
        const auto& module_name = pair.first;
        const auto& module = pair.second;

        // Check if module is unhealthy
        if (!module->isHealthy() && module->getState() == ModuleState::RUNNING) {
            RCLCPP_ERROR(node_->get_logger(),
                "Module %s is unhealthy: %s",
                module_name.c_str(), module->getHealthStatus().c_str());

            // Attempt restart if below max attempts
            if (restart_attempts_[module_name] < max_restart_attempts_) {
                RCLCPP_WARN(node_->get_logger(),
                    "Attempting automatic restart of module %s (attempt %d/%d)",
                    module_name.c_str(),
                    restart_attempts_[module_name] + 1,
                    max_restart_attempts_);

                if (restartModule(module_name)) {
                    RCLCPP_INFO(node_->get_logger(),
                        "Module %s restarted successfully", module_name.c_str());
                } else {
                    RCLCPP_ERROR(node_->get_logger(),
                        "Failed to restart module %s", module_name.c_str());
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(),
                    "Module %s has exceeded maximum restart attempts (%d)",
                    module_name.c_str(), max_restart_attempts_);
            }
        } else if (module->isHealthy() && restart_attempts_[module_name] > 0) {
            // Reset restart counter if module is healthy
            RCLCPP_INFO(node_->get_logger(),
                "Module %s is healthy again, resetting restart counter", module_name.c_str());
            restart_attempts_[module_name] = 0;
        }
    }
}

bool ModuleManager::restartModule(const std::string& name) {
    std::lock_guard<std::mutex> lock(modules_mutex_);

    auto it = modules_.find(name);
    if (it == modules_.end()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Cannot restart module %s: not found", name.c_str());
        return false;
    }

    restart_attempts_[name]++;

    RCLCPP_INFO(node_->get_logger(), "Restarting module %s...", name.c_str());

    // Stop the module
    if (!it->second->stop()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to stop module %s during restart", name.c_str());
        // Try to force shutdown and reinitialize
        it->second->shutdown();
        if (!it->second->initialize()) {
            return false;
        }
    }

    // Wait a bit before restarting
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Reinitialize if needed
    if (it->second->getState() == ModuleState::UNINITIALIZED) {
        if (!it->second->initialize()) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to initialize module %s during restart", name.c_str());
            return false;
        }
    }

    // Start the module
    if (!it->second->start()) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to start module %s during restart", name.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
        "Module %s restarted successfully", name.c_str());
    return true;
}

} // namespace core
} // namespace modular_v