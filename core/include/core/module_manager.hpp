#ifndef CORE_MODULE_MANAGER_HPP
#define CORE_MODULE_MANAGER_HPP

#include "core/module_interface.hpp"
#include <map>
#include <vector>
#include <memory>
#include <mutex>

namespace modular_v {
namespace core {

/**
 * @brief Manages all modules in the system
 */
class ModuleManager {
public:
    ModuleManager();
    ~ModuleManager();

    // Module registration
    bool registerModule(const std::string& name,
                       std::shared_ptr<IModule> module,
                       const std::vector<std::string>& dependencies = {});
    bool unregisterModule(const std::string& name);

    // Module control
    bool startModule(const std::string& name);
    bool stopModule(const std::string& name);
    bool pauseModule(const std::string& name);
    bool resumeModule(const std::string& name);

    // Module queries
    std::shared_ptr<IModule> getModule(const std::string& name);
    std::vector<std::string> getModuleList() const;
    ModuleState getModuleState(const std::string& name) const;

    // System control
    bool initializeSystem();
    bool startSystem();
    bool stopSystem();
    bool shutdownSystem();
    bool emergencyStop();

    // Configuration
    bool loadSystemConfig(const std::string& config_path);
    bool saveSystemConfig(const std::string& config_path);

    // Health monitoring
    bool checkSystemHealth();
    std::map<std::string, bool> getModuleHealthStatus();

    // ROS2 node
    rclcpp::Node::SharedPtr getNode() const { return node_; }

private:
    // Module storage
    std::map<std::string, std::shared_ptr<IModule>> modules_;
    std::map<std::string, std::vector<std::string>> dependencies_;

    // State tracking
    std::atomic<bool> emergency_stop_triggered_{false};
    mutable std::mutex modules_mutex_;

    // Dependency resolution
    std::vector<std::string> getInitializationOrder();
    bool checkDependencies(const std::string& module);

    // ROS2 node
    rclcpp::Node::SharedPtr node_;
};

} // namespace core
} // namespace modular_v

#endif // CORE_MODULE_MANAGER_HPP