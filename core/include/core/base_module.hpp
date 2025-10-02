#ifndef CORE_BASE_MODULE_HPP
#define CORE_BASE_MODULE_HPP

#include "core/module_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <atomic>

namespace modular_v {
namespace core {

/**
 * @brief Base implementation of IModule interface
 *
 * Provides common functionality for all modules including:
 * - Basic lifecycle management
 * - Health monitoring with watchdog
 * - Configuration loading/saving
 * - Status publishing
 */
class BaseModule : public IModule {
public:
    /**
     * @brief Constructor
     * @param name Module name
     * @param version Module version
     */
    BaseModule(const std::string& name, const std::string& version);
    virtual ~BaseModule();

    // Lifecycle methods (can be overridden)
    bool initialize() override;
    bool start() override;
    bool pause() override;
    bool resume() override;
    bool stop() override;
    bool shutdown() override;

    // State management
    ModuleState getState() const override { return state_; }
    std::string getName() const override { return name_; }
    std::string getVersion() const override { return version_; }

    // Configuration
    bool loadConfig(const std::string& config_path) override;
    bool saveConfig(const std::string& config_path) override;

    // Health monitoring
    bool isHealthy() const override;
    std::string getHealthStatus() const override;

    // Update watchdog (modules should call this periodically)
    void updateWatchdog();

    // Logger access helper
    rclcpp::Logger get_logger() const { return node_->get_logger(); }

protected:
    // Virtual methods for derived classes to implement
    virtual bool onInitialize() { return true; }
    virtual bool onStart() { return true; }
    virtual bool onPause() { return true; }
    virtual bool onResume() { return true; }
    virtual bool onStop() { return true; }
    virtual bool onShutdown() { return true; }

    // Configuration helpers
    virtual bool loadModuleConfig(const YAML::Node& config) { (void)config; return true; }
    virtual bool saveModuleConfig(YAML::Emitter& out) { (void)out; return true; }

    // Health check (override to add custom checks)
    virtual bool performHealthCheck() { return true; }

    // Member variables
    std::string health_status_message_;

private:
    // Health monitoring
    void healthCheckTimerCallback();
    std::atomic<std::chrono::steady_clock::time_point> last_watchdog_update_;
    std::chrono::seconds watchdog_timeout_{5};
    rclcpp::TimerBase::SharedPtr health_check_timer_;

    // Status publishing
    void publishStatus();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Configuration
    std::string config_file_path_;
};

} // namespace core
} // namespace modular_v

#endif // CORE_BASE_MODULE_HPP
