#include "core/module_manager.hpp"
#include <algorithm>
#include <yaml-cpp/yaml.h>

namespace modular_v {
namespace core {

ModuleManager::ModuleManager() {
    node_ = rclcpp::Node::make_shared("module_manager");
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
    // Simple implementation - should use topological sort for dependencies
    std::vector<std::string> order;
    for (const auto& pair : modules_) {
        order.push_back(pair.first);
    }
    return order;
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

} // namespace core
} // namespace modular_v