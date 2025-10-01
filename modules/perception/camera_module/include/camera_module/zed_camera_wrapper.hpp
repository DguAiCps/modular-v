#ifndef ZED_CAMERA_WRAPPER_HPP
#define ZED_CAMERA_WRAPPER_HPP

#include "core/module_interface.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <atomic>

namespace modular_v {
namespace perception {

/**
 * @brief Wrapper for ZED ROS2 camera package
 *
 * This module launches and monitors the zed_wrapper node,
 * providing health monitoring and state management
 */
class ZedCameraWrapper : public core::IModule {
public:
    ZedCameraWrapper();
    ~ZedCameraWrapper() override;

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

private:
    // Health monitoring
    void healthCheckLoop();

    // Callbacks for monitoring data flow
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ROS2 subscribers for health monitoring
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

    // Health monitoring thread
    std::thread health_thread_;
    std::atomic<bool> running_{false};

    // Metrics
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<std::chrono::steady_clock::time_point> last_frame_time_;

    // Configuration
    struct Config {
        std::string camera_model = "zed2i";
        int resolution = 2;  // HD720
        int fps = 15;
        bool enable_depth = true;
        bool enable_imu = true;
        bool enable_tracking = true;
    } config_;
};

} // namespace perception
} // namespace modular_v

#endif // ZED_CAMERA_WRAPPER_HPP