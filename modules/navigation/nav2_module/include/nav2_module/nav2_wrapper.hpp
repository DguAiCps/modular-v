#ifndef NAV2_WRAPPER_HPP
#define NAV2_WRAPPER_HPP

#include "core/base_module.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/empty.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <memory>

namespace modular_v {
namespace navigation {

/**
 * @brief Wrapper for Navigation2 stack
 *
 * Manages the full Nav2 navigation stack including:
 * - Controller server (path following)
 * - Planner server (global path planning)
 * - Behavior server (recovery behaviors)
 * - BT Navigator (behavior tree coordination)
 * - Smoother server (path smoothing)
 * - Velocity smoother
 * - Waypoint follower
 *
 * Integrates with RTAB-Map for mapping and ZED camera for odometry/perception.
 */
class Nav2Wrapper : public core::BaseModule {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2Wrapper();
    ~Nav2Wrapper() override;

    // Delete copy/move constructors
    Nav2Wrapper(const Nav2Wrapper&) = delete;
    Nav2Wrapper& operator=(const Nav2Wrapper&) = delete;

    /**
     * @brief Navigate to a goal pose
     * @param goal Target pose in map frame
     * @return true if goal was accepted
     */
    bool navigateToGoal(const geometry_msgs::msg::PoseStamped& goal);

    /**
     * @brief Cancel current navigation goal
     * @return true if cancellation was successful
     */
    bool cancelNavigation();

    /**
     * @brief Clear all costmaps
     * @return true if costmaps were cleared
     */
    bool clearCostmaps();

    /**
     * @brief Check if currently navigating
     * @return true if navigation is in progress
     */
    bool isNavigating() const;

    /**
     * @brief Get current planned path
     * @return Current path
     */
    nav_msgs::msg::Path getCurrentPath() const;

protected:
    // BaseModule lifecycle hooks
    bool onInitialize() override;
    bool onStart() override;
    bool onStop() override;
    bool onShutdown() override;

    // Configuration management
    bool loadModuleConfig(const YAML::Node& config) override;
    bool saveModuleConfig(YAML::Emitter& out) override;

    // Health monitoring
    bool performHealthCheck() override;

private:
    /**
     * @brief Launch Nav2 navigation stack nodes
     */
    void launchNav2Stack();

    /**
     * @brief Stop Nav2 navigation stack
     */
    void stopNav2Stack();

    /**
     * @brief Thread function to spin Nav2 executor
     */
    void spinNav2Executor();

    // Action client callbacks
    void goalResponseCallback(const GoalHandle::SharedPtr& goal_handle);
    void feedbackCallback(
        GoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandle::WrappedResult& result);

    // Topic callbacks for monitoring
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // ROS2 components for Nav2 interaction
    rclcpp::Node::SharedPtr nav2_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> nav2_executor_;
    std::thread nav2_thread_;

    // Action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    GoalHandle::SharedPtr current_goal_handle_;
    std::mutex goal_mutex_;

    // Service clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_global_costmap_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_local_costmap_client_;

    // Monitoring subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;

    // State tracking
    std::atomic<bool> is_navigating_{false};
    nav_msgs::msg::Path current_path_;
    mutable std::mutex path_mutex_;

    std::chrono::steady_clock::time_point last_cmd_vel_time_;
    std::chrono::steady_clock::time_point last_plan_time_;
    std::mutex time_mutex_;

    // Process management for Nav2 nodes
    std::vector<rclcpp::Node::SharedPtr> nav2_lifecycle_nodes_;

    // Configuration
    struct Config {
        // Navigation parameters
        float max_linear_velocity = 0.5;        // m/s (conservative for indoor)
        float max_angular_velocity = 0.8;       // rad/s
        float xy_goal_tolerance = 0.2;          // meters
        float yaw_goal_tolerance = 0.26;        // radians (~15 degrees)

        // Planner parameters
        std::string planner_plugin = "GridBased";
        float planner_tolerance = 0.3;
        bool use_astar = false;                 // Use Dijkstra (faster)

        // Controller parameters
        int controller_frequency = 10;          // Hz (reduced from 20 for Jetson)
        std::string controller_plugin = "FollowPath";

        // Costmap parameters
        float global_costmap_resolution = 0.05; // meters
        float global_costmap_update_freq = 1.0; // Hz
        float local_costmap_resolution = 0.05;  // meters
        float local_costmap_update_freq = 3.0;  // Hz (reduced from 5)
        float local_costmap_width = 3.0;        // meters
        float local_costmap_height = 3.0;       // meters

        // Robot parameters
        float robot_radius = 0.3;               // meters
        std::string robot_base_frame = "base_link";
        std::string global_frame = "map";
        std::string odom_frame = "odom";

        // Topic names
        std::string odom_topic = "/zed2i/zed_node/odom";
        std::string map_topic = "/rtabmap/grid_map";
        std::string cmd_vel_topic = "/cmd_vel";

        // Behavior parameters
        bool autostart = true;
        bool use_sim_time = false;
    } config_;
};

} // namespace navigation
} // namespace modular_v

#endif // NAV2_WRAPPER_HPP
