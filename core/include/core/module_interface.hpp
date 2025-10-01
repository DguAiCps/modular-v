#ifndef CORE_MODULE_INTERFACE_HPP
#define CORE_MODULE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

namespace modular_v {
namespace core {

/**
 * @brief Module state enumeration
 */
enum class ModuleState {
    UNINITIALIZED,  ///< Module has not been initialized
    INITIALIZED,    ///< Module is initialized but not running
    RUNNING,        ///< Module is actively running
    PAUSED,         ///< Module is paused
    ERROR,          ///< Module encountered an error
    TERMINATED      ///< Module has been terminated
};

/**
 * @brief Base interface for all modules in the system
 */
class IModule {
public:
    virtual ~IModule() = default;

    // Lifecycle methods
    virtual bool initialize() = 0;
    virtual bool start() = 0;
    virtual bool pause() = 0;
    virtual bool resume() = 0;
    virtual bool stop() = 0;
    virtual bool shutdown() = 0;

    // State management
    virtual ModuleState getState() const = 0;
    virtual std::string getName() const = 0;
    virtual std::string getVersion() const = 0;

    // Configuration
    virtual bool loadConfig(const std::string& config_path) = 0;
    virtual bool saveConfig(const std::string& config_path) = 0;

    // Health monitoring
    virtual bool isHealthy() const = 0;
    virtual std::string getHealthStatus() const = 0;

    // ROS2 node access
    virtual rclcpp::Node::SharedPtr getNode() const { return node_; }

protected:
    rclcpp::Node::SharedPtr node_;
    ModuleState state_ = ModuleState::UNINITIALIZED;
    std::string name_;
    std::string version_;
};

} // namespace core
} // namespace modular_v

#endif // CORE_MODULE_INTERFACE_HPP