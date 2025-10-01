#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "core/module_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace modular_v {
namespace mobility {

/**
 * @brief Motor controller for differential drive robot
 *
 * Receives velocity commands and controls Dynamixel motors,
 * publishes odometry information
 */
class MotorController : public core::IModule {
public:
    MotorController();
    ~MotorController() override;

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

    // Motion control
    void setVelocity(float linear_x, float angular_z);
    void emergencyStop();

private:
    // Control loop
    void controlLoop();

    // Velocity command callback
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Odometry calculation
    void updateOdometry(double dt);
    void publishOdometry();

    // ROS2 publishers and subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Control thread
    std::thread control_thread_;
    std::atomic<bool> running_{false};

    // Current state
    geometry_msgs::msg::Twist current_cmd_vel_;
    nav_msgs::msg::Odometry odometry_;
    mutable std::mutex cmd_mutex_;

    // Motor parameters
    struct Config {
        float wheel_radius = 0.1;  // meters
        float wheel_base = 0.5;  // meters
        float max_linear_velocity = 1.0;  // m/s
        float max_angular_velocity = 1.0;  // rad/s
        int control_frequency = 50;  // Hz
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
    } config_;

    // Encoder tracking
    int32_t last_left_encoder_ = 0;
    int32_t last_right_encoder_ = 0;
};

} // namespace mobility
} // namespace modular_v

#endif // MOTOR_CONTROLLER_HPP