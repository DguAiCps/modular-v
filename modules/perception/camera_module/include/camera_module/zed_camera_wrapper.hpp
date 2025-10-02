#ifndef ZED_CAMERA_WRAPPER_HPP
#define ZED_CAMERA_WRAPPER_HPP

#include "core/base_module.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/executors.hpp>
#include <thread>
#include <atomic>
#include <mutex>

namespace modular_v {
namespace perception {

/**
 * @brief Wrapper for ZED ROS2 camera package
 *
 * This module wraps the zed_wrapper ROS2 package, launching and monitoring
 * the zed_wrapper node as a composable component. Provides health monitoring,
 * state management, and configuration handling for ZED 2i stereo camera.
 *
 * Key features:
 * - Launches zed_wrapper as a composable node
 * - Monitors image/depth/pointcloud topics for health status
 * - Manages camera configuration via YAML files
 * - Provides standardized module interface for system integration
 */
class ZedCameraWrapper : public core::BaseModule {
public:
    ZedCameraWrapper();
    ~ZedCameraWrapper() override;

    // Deleted copy/move constructors
    ZedCameraWrapper(const ZedCameraWrapper&) = delete;
    ZedCameraWrapper& operator=(const ZedCameraWrapper&) = delete;
    ZedCameraWrapper(ZedCameraWrapper&&) = delete;
    ZedCameraWrapper& operator=(ZedCameraWrapper&&) = delete;

protected:
    // BaseModule lifecycle hooks
    bool onInitialize() override;
    bool onStart() override;
    bool onStop() override;
    bool onShutdown() override;

    // Configuration hooks
    bool loadModuleConfig(const YAML::Node& config) override;
    bool saveModuleConfig(YAML::Emitter& out) override;

    // Health check
    bool performHealthCheck() override;

private:
    // ZED wrapper node management
    void launchZedWrapper();
    void stopZedWrapper();
    void spinZedNode();

    // Health monitoring
    void healthCheckLoop();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ZED wrapper container
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> zed_executor_;
    rclcpp::Node::SharedPtr zed_wrapper_node_;
    std::thread zed_spin_thread_;

    // ROS2 subscribers for health monitoring
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Health monitoring thread
    std::thread health_thread_;
    std::atomic<bool> monitoring_active_{false};

    // Metrics
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> depth_frames_received_{0};
    std::atomic<uint64_t> pointcloud_received_{0};
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point last_depth_time_;
    std::mutex metrics_mutex_;

    // Configuration
    struct Config {
        std::string camera_name = "zed2i";
        std::string camera_model = "zed2i";
        std::string grab_resolution = "HD720";
        int grab_frame_rate = 15;

        // Depth settings
        std::string depth_mode = "NEURAL_LIGHT";
        float min_depth = 0.3f;
        float max_depth = 10.0f;

        // Features
        bool pos_tracking_enabled = true;
        bool imu_fusion = true;
        bool publish_tf = true;
        bool publish_map_tf = true;

        // Point cloud
        float point_cloud_freq = 10.0f;

        // Topics namespace
        std::string topic_root = "/zed2i";

        // Config files
        std::string config_file_path = "";
    } config_;

    // Diagnostics
    mutable std::mutex diag_mutex_;
    std::string last_error_;
};

} // namespace perception
} // namespace modular_v

#endif // ZED_CAMERA_WRAPPER_HPP