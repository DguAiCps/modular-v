#ifndef RTABMAP_WRAPPER_HPP
#define RTABMAP_WRAPPER_HPP

#include "core/module_interface.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

namespace modular_v {
namespace perception {

/**
 * @brief Wrapper for RTAB-Map ROS2 package
 *
 * This module launches and monitors the rtabmap_slam node,
 * providing configuration and state management
 */
class RTABMapWrapper : public core::IModule {
public:
    RTABMapWrapper();
    ~RTABMapWrapper() override;

    // IModule interface implementation
    bool initialize() override;
    bool start() override;
    bool pause() override;
    bool resume() override;
    bool stop() override;
    bool shutdown() override;

    core::ModuleState getState() const override { return state_; }
    std::string getName() const override { return name_; }
    std::string getVersion() const override { return version_; }

    bool loadConfig(const std::string& config_path) override;
    bool saveConfig(const std::string& config_path) override;
    bool isHealthy() const override;
    std::string getHealthStatus() const override;

    // RTAB-Map specific methods
    bool saveMap(const std::string& path);
    bool loadMap(const std::string& path);
    bool resetMap();
    bool setLocalizationMode(bool enable);

private:
    // Monitoring callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // Health check
    void healthCheckLoop();

    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    // ROS2 service clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resume_client_;

    // Health monitoring
    std::thread health_thread_;
    std::atomic<bool> running_{false};
    std::atomic<std::chrono::steady_clock::time_point> last_map_update_;
    std::atomic<std::chrono::steady_clock::time_point> last_pose_update_;

    // Configuration
    struct Config {
        bool localization_mode = false;
        std::string database_path = "~/.ros/rtabmap.db";
        float update_rate = 1.0;  // Hz
        bool delete_db_on_start = false;
    } config_;
};

} // namespace perception
} // namespace modular_v

#endif // RTABMAP_WRAPPER_HPP