#include "nav2_module/nav2_wrapper.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace modular_v {
namespace navigation {

Nav2Wrapper::Nav2Wrapper()
    : BaseModule("nav2_wrapper", "1.0.0")
{
    RCLCPP_INFO(get_logger(), "Nav2Wrapper constructed");
}

Nav2Wrapper::~Nav2Wrapper()
{
    if (nav2_thread_.joinable()) {
        nav2_thread_.join();
    }
}

bool Nav2Wrapper::onInitialize()
{
    RCLCPP_INFO(get_logger(), "Initializing Nav2 wrapper...");

    // Create monitoring node (separate from Nav2 stack)
    nav2_node_ = rclcpp::Node::make_shared("nav2_wrapper_monitor");

    // Create action client for navigation
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        nav2_node_, "navigate_to_pose");

    // Create service clients for costmap clearing
    clear_global_costmap_client_ = nav2_node_->create_client<std_srvs::srv::Empty>(
        "/global_costmap/clear_entirely_global_costmap");
    clear_local_costmap_client_ = nav2_node_->create_client<std_srvs::srv::Empty>(
        "/local_costmap/clear_entirely_local_costmap");

    // QoS profiles
    auto reliable_qos = rclcpp::QoS(10).reliable();

    // Subscribe to monitoring topics
    plan_sub_ = nav2_node_->create_subscription<nav_msgs::msg::Path>(
        "/plan", reliable_qos,
        std::bind(&Nav2Wrapper::pathCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = nav2_node_->create_subscription<geometry_msgs::msg::Twist>(
        config_.cmd_vel_topic, reliable_qos,
        std::bind(&Nav2Wrapper::cmdVelCallback, this, std::placeholders::_1));

    global_costmap_sub_ = nav2_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", reliable_qos,
        std::bind(&Nav2Wrapper::globalCostmapCallback, this, std::placeholders::_1));

    // Create executor for Nav2 interaction node
    nav2_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    nav2_executor_->add_node(nav2_node_);

    RCLCPP_INFO(get_logger(), "Nav2 wrapper initialized successfully");
    return true;
}

bool Nav2Wrapper::onStart()
{
    RCLCPP_INFO(get_logger(), "Starting Nav2 navigation stack...");

    // Launch Nav2 stack
    launchNav2Stack();

    // Start executor thread
    nav2_thread_ = std::thread(&Nav2Wrapper::spinNav2Executor, this);

    // Wait for action server to be available
    RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose action server...");
    if (!nav_action_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available!");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Nav2 navigation stack started successfully");
    return true;
}

bool Nav2Wrapper::onStop()
{
    RCLCPP_INFO(get_logger(), "Stopping Nav2 navigation stack...");

    // Cancel any active navigation
    if (is_navigating_) {
        cancelNavigation();
    }

    // Stop Nav2 stack
    stopNav2Stack();

    RCLCPP_INFO(get_logger(), "Nav2 navigation stack stopped");
    return true;
}

bool Nav2Wrapper::onShutdown()
{
    RCLCPP_INFO(get_logger(), "Shutting down Nav2 wrapper...");

    // Stop executor thread
    if (nav2_executor_) {
        nav2_executor_->cancel();
    }

    if (nav2_thread_.joinable()) {
        nav2_thread_.join();
    }

    // Clear node references
    nav2_lifecycle_nodes_.clear();
    nav2_node_.reset();

    RCLCPP_INFO(get_logger(), "Nav2 wrapper shutdown complete");
    return true;
}

void Nav2Wrapper::launchNav2Stack()
{
    RCLCPP_INFO(get_logger(), "Launching Nav2 lifecycle nodes...");

    // Note: In a real implementation, we would use launch files or
    // programmatically create and configure all Nav2 nodes here.
    // For now, we assume Nav2 nodes are launched externally via nav2_bringup.
    // This wrapper focuses on interacting with the Nav2 stack rather than
    // launching it directly.

    // Alternative approach: Use system() to launch nav2_bringup
    // or use launch API to programmatically start nav2_bringup/navigation_launch.py

    RCLCPP_WARN(get_logger(),
        "Nav2 stack should be launched externally using nav2_bringup");
    RCLCPP_INFO(get_logger(),
        "Example: ros2 launch nav2_bringup navigation_launch.py");
}

void Nav2Wrapper::stopNav2Stack()
{
    RCLCPP_INFO(get_logger(), "Stopping Nav2 lifecycle nodes...");

    // Clear lifecycle node references
    nav2_lifecycle_nodes_.clear();

    // Note: If we launched Nav2 nodes programmatically in launchNav2Stack(),
    // we would shutdown those nodes here.
}

void Nav2Wrapper::spinNav2Executor()
{
    RCLCPP_INFO(get_logger(), "Nav2 executor thread started");

    while (rclcpp::ok() && getState() == core::ModuleState::RUNNING) {
        try {
            nav2_executor_->spin_some(100ms);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in Nav2 executor: %s", e.what());
        }
    }

    RCLCPP_INFO(get_logger(), "Nav2 executor thread stopped");
}

bool Nav2Wrapper::navigateToGoal(const geometry_msgs::msg::PoseStamped& goal)
{
    if (!nav_action_client_->wait_for_action_server(1s)) {
        RCLCPP_ERROR(get_logger(), "Navigate action server not available");
        return false;
    }

    // Cancel any existing goal
    if (is_navigating_) {
        RCLCPP_INFO(get_logger(), "Canceling previous navigation goal");
        cancelNavigation();
    }

    // Create goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;

    // Send goal options
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Nav2Wrapper::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&Nav2Wrapper::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&Nav2Wrapper::resultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Sending navigation goal to (%.2f, %.2f, %.2f)",
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    // Send goal asynchronously
    auto goal_future = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

    is_navigating_ = true;
    return true;
}

bool Nav2Wrapper::cancelNavigation()
{
    std::lock_guard<std::mutex> lock(goal_mutex_);

    if (!current_goal_handle_) {
        RCLCPP_WARN(get_logger(), "No active goal to cancel");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Canceling navigation goal");

    auto cancel_future = nav_action_client_->async_cancel_goal(current_goal_handle_);

    // Wait for cancellation to complete
    if (cancel_future.wait_for(2s) == std::future_status::ready) {
        is_navigating_ = false;
        current_goal_handle_.reset();
        RCLCPP_INFO(get_logger(), "Navigation goal canceled successfully");
        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to cancel navigation goal");
        return false;
    }
}

bool Nav2Wrapper::clearCostmaps()
{
    bool success = true;

    // Clear global costmap
    if (clear_global_costmap_client_->wait_for_service(1s)) {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto future = clear_global_costmap_client_->async_send_request(request);

        if (future.wait_for(2s) == std::future_status::ready) {
            RCLCPP_INFO(get_logger(), "Global costmap cleared");
        } else {
            RCLCPP_WARN(get_logger(), "Failed to clear global costmap");
            success = false;
        }
    } else {
        RCLCPP_WARN(get_logger(), "Global costmap clear service not available");
        success = false;
    }

    // Clear local costmap
    if (clear_local_costmap_client_->wait_for_service(1s)) {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto future = clear_local_costmap_client_->async_send_request(request);

        if (future.wait_for(2s) == std::future_status::ready) {
            RCLCPP_INFO(get_logger(), "Local costmap cleared");
        } else {
            RCLCPP_WARN(get_logger(), "Failed to clear local costmap");
            success = false;
        }
    } else {
        RCLCPP_WARN(get_logger(), "Local costmap clear service not available");
        success = false;
    }

    return success;
}

bool Nav2Wrapper::isNavigating() const
{
    return is_navigating_.load();
}

nav_msgs::msg::Path Nav2Wrapper::getCurrentPath() const
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    return current_path_;
}

void Nav2Wrapper::goalResponseCallback(const GoalHandle::SharedPtr& goal_handle)
{
    std::lock_guard<std::mutex> lock(goal_mutex_);

    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        is_navigating_ = false;
        return;
    }

    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by server");
}

void Nav2Wrapper::feedbackCallback(
    GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)goal_handle;  // Unused

    // Log current navigation progress
    auto current_pose = feedback->current_pose.pose;
    auto distance_remaining = feedback->distance_remaining;
    auto eta = feedback->estimated_time_remaining;

    RCLCPP_DEBUG(get_logger(),
        "Navigation progress - Position: (%.2f, %.2f), Distance remaining: %.2f m, ETA: %.1f s",
        current_pose.position.x, current_pose.position.y,
        distance_remaining, rclcpp::Duration(eta).seconds());

    updateWatchdog();
}

void Nav2Wrapper::resultCallback(const GoalHandle::WrappedResult& result)
{
    is_navigating_ = false;

    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_.reset();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Navigation succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Navigation was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Navigation was canceled");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown navigation result code: %d",
                static_cast<int>(result.code));
            break;
    }
}

void Nav2Wrapper::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    current_path_ = *msg;

    {
        std::lock_guard<std::mutex> time_lock(time_mutex_);
        last_plan_time_ = std::chrono::steady_clock::now();
    }

    RCLCPP_DEBUG(get_logger(), "Received new plan with %zu poses", msg->poses.size());
    updateWatchdog();
}

void Nav2Wrapper::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    (void)msg;  // Unused, just for monitoring

    std::lock_guard<std::mutex> lock(time_mutex_);
    last_cmd_vel_time_ = std::chrono::steady_clock::now();

    updateWatchdog();
}

void Nav2Wrapper::globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    (void)msg;  // Unused, just for monitoring

    RCLCPP_DEBUG(get_logger(), "Received global costmap update");
    updateWatchdog();
}

bool Nav2Wrapper::performHealthCheck()
{
    updateWatchdog();

    // Check if action server is still available
    if (!nav_action_client_->action_server_is_ready()) {
        RCLCPP_WARN(get_logger(), "Navigate action server not available");
        health_status_message_ = "Action server unavailable";
        return false;
    }

    // Check if we're receiving updates when navigating
    if (is_navigating_) {
        std::lock_guard<std::mutex> lock(time_mutex_);
        auto now = std::chrono::steady_clock::now();

        // Check cmd_vel updates
        auto time_since_cmd_vel = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_cmd_vel_time_).count();

        if (time_since_cmd_vel > 2000) {  // 2 seconds timeout
            RCLCPP_WARN(get_logger(),
                "No cmd_vel updates for %.1f seconds", time_since_cmd_vel / 1000.0);
            health_status_message_ = "No velocity commands";
            // Don't fail health check - robot might be stationary
        }
    }

    health_status_message_ = "Healthy";
    return true;
}

bool Nav2Wrapper::loadModuleConfig(const YAML::Node& config)
{
    try {
        if (config["navigation"]) {
            auto nav_config = config["navigation"];

            if (nav_config["max_linear_velocity"]) {
                config_.max_linear_velocity = nav_config["max_linear_velocity"].as<float>();
            }
            if (nav_config["max_angular_velocity"]) {
                config_.max_angular_velocity = nav_config["max_angular_velocity"].as<float>();
            }
            if (nav_config["xy_goal_tolerance"]) {
                config_.xy_goal_tolerance = nav_config["xy_goal_tolerance"].as<float>();
            }
            if (nav_config["yaw_goal_tolerance"]) {
                config_.yaw_goal_tolerance = nav_config["yaw_goal_tolerance"].as<float>();
            }
        }

        if (config["planner"]) {
            auto planner_config = config["planner"];

            if (planner_config["plugin"]) {
                config_.planner_plugin = planner_config["plugin"].as<std::string>();
            }
            if (planner_config["tolerance"]) {
                config_.planner_tolerance = planner_config["tolerance"].as<float>();
            }
            if (planner_config["use_astar"]) {
                config_.use_astar = planner_config["use_astar"].as<bool>();
            }
        }

        if (config["controller"]) {
            auto controller_config = config["controller"];

            if (controller_config["frequency"]) {
                config_.controller_frequency = controller_config["frequency"].as<int>();
            }
            if (controller_config["plugin"]) {
                config_.controller_plugin = controller_config["plugin"].as<std::string>();
            }
        }

        if (config["costmaps"]) {
            auto costmap_config = config["costmaps"];

            if (costmap_config["global"]) {
                auto global = costmap_config["global"];
                if (global["resolution"]) {
                    config_.global_costmap_resolution = global["resolution"].as<float>();
                }
                if (global["update_frequency"]) {
                    config_.global_costmap_update_freq = global["update_frequency"].as<float>();
                }
            }

            if (costmap_config["local"]) {
                auto local = costmap_config["local"];
                if (local["resolution"]) {
                    config_.local_costmap_resolution = local["resolution"].as<float>();
                }
                if (local["update_frequency"]) {
                    config_.local_costmap_update_freq = local["update_frequency"].as<float>();
                }
                if (local["width"]) {
                    config_.local_costmap_width = local["width"].as<float>();
                }
                if (local["height"]) {
                    config_.local_costmap_height = local["height"].as<float>();
                }
            }
        }

        if (config["robot"]) {
            auto robot_config = config["robot"];

            if (robot_config["radius"]) {
                config_.robot_radius = robot_config["radius"].as<float>();
            }
            if (robot_config["base_frame"]) {
                config_.robot_base_frame = robot_config["base_frame"].as<std::string>();
            }
        }

        if (config["topics"]) {
            auto topic_config = config["topics"];

            if (topic_config["odom"]) {
                config_.odom_topic = topic_config["odom"].as<std::string>();
            }
            if (topic_config["map"]) {
                config_.map_topic = topic_config["map"].as<std::string>();
            }
            if (topic_config["cmd_vel"]) {
                config_.cmd_vel_topic = topic_config["cmd_vel"].as<std::string>();
            }
        }

        RCLCPP_INFO(get_logger(), "Nav2 configuration loaded successfully");
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load Nav2 config: %s", e.what());
        return false;
    }
}

bool Nav2Wrapper::saveModuleConfig(YAML::Emitter& out)
{
    try {
        out << YAML::Key << "navigation";
        out << YAML::BeginMap;
        out << YAML::Key << "max_linear_velocity" << YAML::Value << config_.max_linear_velocity;
        out << YAML::Key << "max_angular_velocity" << YAML::Value << config_.max_angular_velocity;
        out << YAML::Key << "xy_goal_tolerance" << YAML::Value << config_.xy_goal_tolerance;
        out << YAML::Key << "yaw_goal_tolerance" << YAML::Value << config_.yaw_goal_tolerance;
        out << YAML::EndMap;

        out << YAML::Key << "planner";
        out << YAML::BeginMap;
        out << YAML::Key << "plugin" << YAML::Value << config_.planner_plugin;
        out << YAML::Key << "tolerance" << YAML::Value << config_.planner_tolerance;
        out << YAML::Key << "use_astar" << YAML::Value << config_.use_astar;
        out << YAML::EndMap;

        out << YAML::Key << "controller";
        out << YAML::BeginMap;
        out << YAML::Key << "frequency" << YAML::Value << config_.controller_frequency;
        out << YAML::Key << "plugin" << YAML::Value << config_.controller_plugin;
        out << YAML::EndMap;

        out << YAML::Key << "costmaps";
        out << YAML::BeginMap;

        out << YAML::Key << "global";
        out << YAML::BeginMap;
        out << YAML::Key << "resolution" << YAML::Value << config_.global_costmap_resolution;
        out << YAML::Key << "update_frequency" << YAML::Value << config_.global_costmap_update_freq;
        out << YAML::EndMap;

        out << YAML::Key << "local";
        out << YAML::BeginMap;
        out << YAML::Key << "resolution" << YAML::Value << config_.local_costmap_resolution;
        out << YAML::Key << "update_frequency" << YAML::Value << config_.local_costmap_update_freq;
        out << YAML::Key << "width" << YAML::Value << config_.local_costmap_width;
        out << YAML::Key << "height" << YAML::Value << config_.local_costmap_height;
        out << YAML::EndMap;

        out << YAML::EndMap;  // costmaps

        out << YAML::Key << "robot";
        out << YAML::BeginMap;
        out << YAML::Key << "radius" << YAML::Value << config_.robot_radius;
        out << YAML::Key << "base_frame" << YAML::Value << config_.robot_base_frame;
        out << YAML::EndMap;

        out << YAML::Key << "topics";
        out << YAML::BeginMap;
        out << YAML::Key << "odom" << YAML::Value << config_.odom_topic;
        out << YAML::Key << "map" << YAML::Value << config_.map_topic;
        out << YAML::Key << "cmd_vel" << YAML::Value << config_.cmd_vel_topic;
        out << YAML::EndMap;

        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to save Nav2 config: %s", e.what());
        return false;
    }
}

} // namespace navigation
} // namespace modular_v
