#ifndef RTABMAP_WRAPPER_HPP
#define RTABMAP_WRAPPER_HPP

#include "core/base_module.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rtabmap_msgs/msg/map_graph.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/srv/reset_pose.hpp>
#include <rclcpp/executors.hpp>
#include <thread>
#include <atomic>
#include <mutex>

namespace modular_v {
namespace perception {

/**
 * @brief Wrapper for RTAB-Map ROS2 SLAM package
 *
 * This module wraps the rtabmap_slam ROS2 package, launching and monitoring
 * the rtabmap node for stereo visual SLAM. Provides health monitoring,
 * state management, and configuration handling for RTAB-Map.
 *
 * Key features:
 * - Launches rtabmap_slam node in stereo mode
 * - Subscribes to ZED 2i stereo images and odometry
 * - Monitors map, pose, and graph topics for health status
 * - Provides map save/load/reset functionality
 * - Supports localization mode for pre-mapped environments
 */
class RTABMapWrapper : public core::BaseModule {
public:
    RTABMapWrapper();
    ~RTABMapWrapper() override;

    // Deleted copy/move constructors
    RTABMapWrapper(const RTABMapWrapper&) = delete;
    RTABMapWrapper& operator=(const RTABMapWrapper&) = delete;
    RTABMapWrapper(RTABMapWrapper&&) = delete;
    RTABMapWrapper& operator=(RTABMapWrapper&&) = delete;

    // RTAB-Map specific methods
    bool saveMap(const std::string& path);
    bool resetMap();
    bool setLocalizationMode(bool enable);
    bool pauseMapping();
    bool resumeMapping();

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
    // RTAB-Map node management
    void launchRTABMap();
    void stopRTABMap();
    void spinRTABMapNode();

    // Monitoring callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void mapGraphCallback(const rtabmap_msgs::msg::MapGraph::SharedPtr msg);
    void mapDataCallback(const rtabmap_msgs::msg::MapData::SharedPtr msg);

    // RTAB-Map node container
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rtabmap_executor_;
    rclcpp::Node::SharedPtr rtabmap_node_;
    std::thread rtabmap_spin_thread_;

    // ROS2 subscribers for health monitoring
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<rtabmap_msgs::msg::MapGraph>::SharedPtr graph_sub_;
    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr mapdata_sub_;

    // ROS2 publishers for Nav2 integration
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav2_map_pub_;

    // ROS2 service clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resume_client_;
    rclcpp::Client<rtabmap_msgs::srv::ResetPose>::SharedPtr reset_pose_client_;

    // Health monitoring
    std::atomic<bool> monitoring_active_{false};
    std::atomic<uint64_t> map_updates_received_{0};
    std::atomic<uint64_t> pose_updates_received_{0};
    std::atomic<uint64_t> nodes_in_map_{0};
    std::chrono::steady_clock::time_point last_map_time_;
    std::chrono::steady_clock::time_point last_pose_time_;
    std::mutex metrics_mutex_;

    // Configuration
    struct Config {
        // RTAB-Map mode
        bool localization_mode = false;
        bool delete_db_on_start = false;

        // Database
        std::string database_path = "~/.ros/rtabmap.db";

        // Frame IDs
        std::string frame_id = "base_link";
        std::string odom_frame_id = "odom";
        std::string map_frame_id = "map";

        // Topic subscriptions
        std::string left_image_topic = "/zed2i/zed_node/left/image_rect_color";
        std::string right_image_topic = "/zed2i/zed_node/right/image_rect_gray";
        std::string left_camera_info_topic = "/zed2i/zed_node/left/camera_info";
        std::string right_camera_info_topic = "/zed2i/zed_node/right/camera_info";
        std::string odom_topic = "/zed2i/zed_node/odom";
        std::string imu_topic = "/zed2i/zed_node/imu/data";

        // RTAB-Map parameters
        bool stereo = true;
        bool use_zed_odom = true;  // Use ZED's visual odometry instead of rtabmap's
        bool subscribe_rgbd = false;
        int queue_size = 30;
        bool approx_sync = true;
        float approx_sync_max_interval = 0.01;  // 10ms

        // RTAB-Map SLAM settings
        int Rtabmap_DetectionRate = 1;  // Hz, 0=unlimited
        float Rtabmap_TimeThr = 0;  // Max time allowed for map update (0=disabled)
        int Mem_IncrementalMemory = 1;  // 1=SLAM mode, 0=localization
        int Mem_InitWMWithAllNodes = 0;  // 1=localization mode

        // Memory management (important for Jetson)
        int Mem_ImageKept = 0;  // Don't keep images in RAM
        int Mem_STMSize = 10;  // Short-term memory size
        float Mem_ReduceGraph = 0.0;  // Keep all nodes

        // Loop closure
        bool RGBD_LoopClosureReextractFeatures = true;
        int Vis_MinInliers = 15;
        int Vis_InlierDistance = 5;

        // Optimization
        int RGBD_OptimizeMaxError = 3;
        bool RGBD_ProximityBySpace = true;

        // Performance (Jetson optimization)
        int Vis_MaxFeatures = 400;  // Reduce for Jetson
        int SURF_HessianThreshold = 150;

        // 2D Grid map generation for Nav2
        bool Grid_FromDepth = true;
        float Grid_CellSize = 0.05;  // 5cm resolution
        float Grid_RangeMax = 5.0;   // 5m max range
        float Grid_RangeMin = 0.0;
        float Grid_ClusterRadius = 1.0;
        bool Grid_GroundIsObstacle = false;
        float Grid_MaxObstacleHeight = 2.0;  // 2m
        float Grid_MaxGroundHeight = 0.1;    // 10cm
        bool Grid_NormalsSegmentation = true;
        bool Grid_3D = false;  // Generate 2D grid for Nav2
        int Grid_NoiseFilteringRadius = 4;
        int Grid_NoiseFilteringMinNeighbors = 5;

    } config_;

    // Diagnostics
    mutable std::mutex diag_mutex_;
    std::string last_error_;
    bool is_paused_{false};
};

} // namespace perception
} // namespace modular_v

#endif // RTABMAP_WRAPPER_HPP
