#ifndef CORE_MODULE_MANAGER_HPP
#define CORE_MODULE_MANAGER_HPP

#include "core/module_interface.hpp"
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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

    // Error recovery
    bool restartModule(const std::string& name);
    void enableAutoRecovery(bool enable) { auto_recovery_enabled_ = enable; }
    void setMaxRestartAttempts(int max_attempts) { max_restart_attempts_ = max_attempts; }

    // ROS2 node
    rclcpp::Node::SharedPtr getNode() const { return node_; }

    // Status publishing
    void publishSystemStatus();

private:
    // ROS2 callbacks
    void emergencyStopCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void statusTimerCallback();
    void healthMonitorTimerCallback();

    // Module storage
    std::map<std::string, std::shared_ptr<IModule>> modules_;
    std::map<std::string, std::vector<std::string>> dependencies_;

    // State tracking
    std::atomic<bool> emergency_stop_triggered_{false};
    mutable std::mutex modules_mutex_;

    // Error recovery
    std::map<std::string, int> restart_attempts_;
    bool auto_recovery_enabled_{true};
    int max_restart_attempts_{3};

    // Dependency resolution
    std::vector<std::string> getInitializationOrder();
    bool checkDependencies(const std::string& module);
    bool topologicalSort(std::vector<std::string>& sorted_modules);
    bool hasCircularDependency(const std::string& module,
                              std::map<std::string, int>& visited,
                              const std::string& current = "");

    // ROS2 node and communication
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr health_monitor_timer_;
};

} // namespace core
} // namespace modular_v

#endif // CORE_MODULE_MANAGER_HPP