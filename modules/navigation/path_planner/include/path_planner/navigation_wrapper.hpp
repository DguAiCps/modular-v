#ifndef NAVIGATION_WRAPPER_HPP
#define NAVIGATION_WRAPPER_HPP

#include "core/module_interface.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace modular_v {
namespace navigation {

/**
 * @brief Wrapper for Nav2 navigation stack
 *
 * This module interfaces with the nav2_bt_navigator,
 * providing simplified navigation commands and monitoring
 */
class NavigationWrapper : public core::IModule {
public:
    NavigationWrapper();
    ~NavigationWrapper() override;

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

    // Navigation commands
    bool navigateToGoal(const geometry_msgs::msg::PoseStamped& goal);
    bool cancelNavigation();
    nav_msgs::msg::Path getCurrentPath() const;
    bool isNavigating() const;

private:
    // Action client callbacks
    void goalResponseCallback(
        std::shared_future<rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::SharedPtr> future);
    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void resultCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::WrappedResult& result);

    // ROS2 action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;

    // Current goal handle
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    // State tracking
    std::atomic<bool> is_navigating_{false};
    nav_msgs::msg::Path current_path_;
    mutable std::mutex path_mutex_;

    // Configuration
    struct Config {
        float xy_goal_tolerance = 0.25;  // meters
        float yaw_goal_tolerance = 0.25;  // radians
        float max_velocity = 1.0;  // m/s
        float max_angular_velocity = 1.0;  // rad/s
    } config_;
};

} // namespace navigation
} // namespace modular_v

#endif // NAVIGATION_WRAPPER_HPP