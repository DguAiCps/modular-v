#include "camera_module/zed_camera_wrapper.hpp"
#include <rclcpp_components/component_manager.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace modular_v {
namespace perception {

ZedCameraWrapper::ZedCameraWrapper()
    : BaseModule("ZED Camera Wrapper", "1.0.0") {
    last_frame_time_ = std::chrono::steady_clock::now();
    last_depth_time_ = std::chrono::steady_clock::now();
}

ZedCameraWrapper::~ZedCameraWrapper() {
    if (state_ == core::ModuleState::RUNNING) {
        stop();
    }
    if (state_ != core::ModuleState::UNINITIALIZED) {
        shutdown();
    }
}

bool ZedCameraWrapper::onInitialize() {
    try {
        // Create monitoring node
        node_ = rclcpp::Node::make_shared("zed_camera_wrapper_monitor");

        RCLCPP_INFO(node_->get_logger(), "Initializing ZED Camera Wrapper for %s",
                    config_.camera_model.c_str());

        // Setup monitoring subscribers
        auto qos = rclcpp::QoS(10).best_effort();

        rgb_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            config_.topic_root + "/zed_node/rgb/image_rect_color", qos,
            std::bind(&ZedCameraWrapper::imageCallback, this, std::placeholders::_1));

        depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            config_.topic_root + "/zed_node/depth/depth_registered", qos,
            std::bind(&ZedCameraWrapper::depthCallback, this, std::placeholders::_1));

        pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.topic_root + "/zed_node/point_cloud/cloud_registered", qos,
            std::bind(&ZedCameraWrapper::pointCloudCallback, this, std::placeholders::_1));

        // Create ZED wrapper executor
        zed_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        RCLCPP_INFO(node_->get_logger(), "ZED Camera Wrapper initialized successfully");
        health_status_message_ = "Initialized";

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Initialization failed: %s", e.what());
        std::lock_guard<std::mutex> lock(diag_mutex_);
        last_error_ = std::string("Init exception: ") + e.what();
        health_status_message_ = last_error_;
        return false;
    }
}

bool ZedCameraWrapper::onStart() {
    try {
        RCLCPP_INFO(node_->get_logger(), "Starting ZED Camera Wrapper");

        // Launch ZED wrapper node
        launchZedWrapper();

        // Start health monitoring
        monitoring_active_ = true;
        health_thread_ = std::thread(&ZedCameraWrapper::healthCheckLoop, this);

        // Start ZED node spinning
        zed_spin_thread_ = std::thread(&ZedCameraWrapper::spinZedNode, this);

        RCLCPP_INFO(node_->get_logger(), "ZED Camera Wrapper started");
        health_status_message_ = "Running";
        updateWatchdog();

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Start failed: %s", e.what());
        std::lock_guard<std::mutex> lock(diag_mutex_);
        last_error_ = std::string("Start exception: ") + e.what();
        health_status_message_ = last_error_;
        return false;
    }
}

bool ZedCameraWrapper::onStop() {
    RCLCPP_INFO(node_->get_logger(), "Stopping ZED Camera Wrapper");

    // Stop health monitoring
    monitoring_active_ = false;
    if (health_thread_.joinable()) {
        health_thread_.join();
    }

    // Stop ZED wrapper
    stopZedWrapper();

    // Stop ZED executor
    if (zed_spin_thread_.joinable()) {
        zed_executor_->cancel();
        zed_spin_thread_.join();
    }

    RCLCPP_INFO(node_->get_logger(), "ZED Camera Wrapper stopped");
    health_status_message_ = "Stopped";

    return true;
}

bool ZedCameraWrapper::onShutdown() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down ZED Camera Wrapper");

    // Cleanup subscribers
    rgb_sub_.reset();
    depth_sub_.reset();
    pc_sub_.reset();
    imu_sub_.reset();

    // Cleanup ZED wrapper
    zed_wrapper_node_.reset();
    zed_executor_.reset();

    RCLCPP_INFO(node_->get_logger(), "ZED Camera Wrapper shutdown complete");
    health_status_message_ = "Terminated";

    return true;
}

void ZedCameraWrapper::launchZedWrapper() {
    try {
        // Get zed_wrapper package path
        std::string zed_wrapper_share;
        try {
            zed_wrapper_share = ament_index_cpp::get_package_share_directory("zed_wrapper");
        } catch (const std::exception& e) {
            throw std::runtime_error("Could not find zed_wrapper package: " + std::string(e.what()));
        }

        // Build config file path
        std::string config_common = zed_wrapper_share + "/config/common_stereo.yaml";
        std::string config_camera = zed_wrapper_share + "/config/" + config_.camera_model + ".yaml";

        // Verify config files exist
        if (!std::filesystem::exists(config_common)) {
            throw std::runtime_error("Common config not found: " + config_common);
        }
        if (!std::filesystem::exists(config_camera)) {
            throw std::runtime_error("Camera config not found: " + config_camera);
        }

        RCLCPP_INFO(node_->get_logger(), "Loading ZED config from:");
        RCLCPP_INFO(node_->get_logger(), "  - %s", config_common.c_str());
        RCLCPP_INFO(node_->get_logger(), "  - %s", config_camera.c_str());

        // Create node options with parameters
        rclcpp::NodeOptions options;
        options.append_parameter_override("general.camera_name", config_.camera_name);
        options.append_parameter_override("general.camera_model", config_.camera_model);
        options.append_parameter_override("general.grab_resolution", config_.grab_resolution);
        options.append_parameter_override("general.grab_frame_rate", config_.grab_frame_rate);
        options.append_parameter_override("depth.depth_mode", config_.depth_mode);
        options.append_parameter_override("depth.min_depth", config_.min_depth);
        options.append_parameter_override("depth.max_depth", config_.max_depth);
        options.append_parameter_override("pos_tracking.pos_tracking_enabled", config_.pos_tracking_enabled);
        options.append_parameter_override("pos_tracking.imu_fusion", config_.imu_fusion);
        options.append_parameter_override("pos_tracking.publish_tf", config_.publish_tf);
        options.append_parameter_override("pos_tracking.publish_map_tf", config_.publish_map_tf);
        options.append_parameter_override("depth.point_cloud_freq", config_.point_cloud_freq);

        // Load config files
        options.arguments({"--ros-args",
                          "--params-file", config_common,
                          "--params-file", config_camera});

        // Create ZED wrapper node
        // Note: This assumes zed_wrapper provides a component. If not, we'll need to use subprocess
        try {
            // Try to load as component
            auto loader = std::make_shared<rclcpp_components::ComponentManager>(zed_executor_, "zed_component_manager");

            // For now, create a simple node that will trigger the actual zed_wrapper launch
            // In production, you would load the actual ZED component here
            zed_wrapper_node_ = rclcpp::Node::make_shared(config_.camera_name + "_node", options);
            zed_executor_->add_node(zed_wrapper_node_);

            RCLCPP_INFO(node_->get_logger(), "ZED wrapper node created successfully");

        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(),
                       "Could not load ZED as component: %s. Consider using external launch.",
                       e.what());
            // In this case, user should launch zed_wrapper separately
            // Our wrapper will just monitor the topics
        }

    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to launch ZED wrapper: " + std::string(e.what()));
    }
}

void ZedCameraWrapper::stopZedWrapper() {
    if (zed_wrapper_node_) {
        RCLCPP_INFO(node_->get_logger(), "Stopping ZED wrapper node");
        if (zed_executor_) {
            zed_executor_->remove_node(zed_wrapper_node_);
        }
        zed_wrapper_node_.reset();
    }
}

void ZedCameraWrapper::spinZedNode() {
    if (zed_executor_) {
        RCLCPP_INFO(node_->get_logger(), "ZED executor spinning...");
        zed_executor_->spin();
    }
}

void ZedCameraWrapper::healthCheckLoop() {
    RCLCPP_INFO(node_->get_logger(), "Health check thread started");

    auto last_log_time = std::chrono::steady_clock::now();

    while (monitoring_active_ && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto now = std::chrono::steady_clock::now();

        // Periodic status logging (every 10 seconds)
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 10) {
            std::lock_guard<std::mutex> lock(metrics_mutex_);

            auto time_since_frame = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_frame_time_).count();
            auto time_since_depth = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_depth_time_).count();

            RCLCPP_INFO(node_->get_logger(),
                       "ZED Health: RGB frames=%lu (last %ldms ago), Depth=%lu (last %ldms ago), PC=%lu",
                       frames_received_.load(), time_since_frame,
                       depth_frames_received_.load(), time_since_depth,
                       pointcloud_received_.load());

            last_log_time = now;
        }

        updateWatchdog();
    }

    RCLCPP_INFO(node_->get_logger(), "Health check thread stopped");
}

void ZedCameraWrapper::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
    frames_received_++;
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    last_frame_time_ = std::chrono::steady_clock::now();
}

void ZedCameraWrapper::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
    depth_frames_received_++;
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    last_depth_time_ = std::chrono::steady_clock::now();
}

void ZedCameraWrapper::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    (void)msg;
    pointcloud_received_++;
}

bool ZedCameraWrapper::performHealthCheck() {
    if (state_ != core::ModuleState::RUNNING) {
        return false;
    }

    std::lock_guard<std::mutex> lock(metrics_mutex_);

    auto now = std::chrono::steady_clock::now();

    // Check if we're receiving RGB frames (max 2 seconds old)
    auto time_since_frame = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_frame_time_).count();

    if (time_since_frame > 2000) {
        health_status_message_ = "No RGB frames received for " +
                                std::to_string(time_since_frame / 1000) + "s";
        return false;
    }

    // Check if we're receiving depth frames
    auto time_since_depth = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_depth_time_).count();

    if (time_since_depth > 2000) {
        health_status_message_ = "No depth frames received for " +
                                std::to_string(time_since_depth / 1000) + "s";
        return false;
    }

    // Healthy
    health_status_message_ = "Healthy - RGB: " + std::to_string(frames_received_.load()) +
                            ", Depth: " + std::to_string(depth_frames_received_.load()) +
                            ", PC: " + std::to_string(pointcloud_received_.load());
    return true;
}

bool ZedCameraWrapper::loadModuleConfig(const YAML::Node& config) {
    try {
        if (config["camera"]) {
            auto cam = config["camera"];
            config_.camera_name = cam["camera_name"].as<std::string>(config_.camera_name);
            config_.camera_model = cam["camera_model"].as<std::string>(config_.camera_model);
            config_.grab_resolution = cam["grab_resolution"].as<std::string>(config_.grab_resolution);
            config_.grab_frame_rate = cam["grab_frame_rate"].as<int>(config_.grab_frame_rate);
        }

        if (config["depth"]) {
            auto depth = config["depth"];
            config_.depth_mode = depth["depth_mode"].as<std::string>(config_.depth_mode);
            config_.min_depth = depth["min_depth"].as<float>(config_.min_depth);
            config_.max_depth = depth["max_depth"].as<float>(config_.max_depth);
        }

        if (config["tracking"]) {
            auto track = config["tracking"];
            config_.pos_tracking_enabled = track["enabled"].as<bool>(config_.pos_tracking_enabled);
            config_.imu_fusion = track["imu_fusion"].as<bool>(config_.imu_fusion);
            config_.publish_tf = track["publish_tf"].as<bool>(config_.publish_tf);
            config_.publish_map_tf = track["publish_map_tf"].as<bool>(config_.publish_map_tf);
        }

        if (config["pointcloud"]) {
            auto pc = config["pointcloud"];
            config_.point_cloud_freq = pc["frequency"].as<float>(config_.point_cloud_freq);
        }

        if (config["topics"]) {
            auto topics = config["topics"];
            config_.topic_root = topics["root"].as<std::string>(config_.topic_root);
        }

        RCLCPP_INFO(node_->get_logger(), "Module configuration loaded");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load module config: %s", e.what());
        return false;
    }
}

bool ZedCameraWrapper::saveModuleConfig(YAML::Emitter& out) {
    try {
        out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "camera_name" << YAML::Value << config_.camera_name;
        out << YAML::Key << "camera_model" << YAML::Value << config_.camera_model;
        out << YAML::Key << "grab_resolution" << YAML::Value << config_.grab_resolution;
        out << YAML::Key << "grab_frame_rate" << YAML::Value << config_.grab_frame_rate;
        out << YAML::EndMap;

        out << YAML::Key << "depth" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "depth_mode" << YAML::Value << config_.depth_mode;
        out << YAML::Key << "min_depth" << YAML::Value << config_.min_depth;
        out << YAML::Key << "max_depth" << YAML::Value << config_.max_depth;
        out << YAML::EndMap;

        out << YAML::Key << "tracking" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "enabled" << YAML::Value << config_.pos_tracking_enabled;
        out << YAML::Key << "imu_fusion" << YAML::Value << config_.imu_fusion;
        out << YAML::Key << "publish_tf" << YAML::Value << config_.publish_tf;
        out << YAML::Key << "publish_map_tf" << YAML::Value << config_.publish_map_tf;
        out << YAML::EndMap;

        out << YAML::Key << "pointcloud" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "frequency" << YAML::Value << config_.point_cloud_freq;
        out << YAML::EndMap;

        out << YAML::Key << "topics" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "root" << YAML::Value << config_.topic_root;
        out << YAML::EndMap;

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to save module config: %s", e.what());
        return false;
    }
}

} // namespace perception
} // namespace modular_v
