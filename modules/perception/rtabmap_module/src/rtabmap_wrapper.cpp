#include "rtabmap_module/rtabmap_wrapper.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace modular_v {
namespace perception {

RTABMapWrapper::RTABMapWrapper()
    : BaseModule("rtabmap_wrapper", "1.0.0")
{
    last_map_time_ = std::chrono::steady_clock::now();
    last_pose_time_ = std::chrono::steady_clock::now();
}

RTABMapWrapper::~RTABMapWrapper() {
    if (getState() == core::ModuleState::RUNNING) {
        stop();
    }
    shutdown();
}

bool RTABMapWrapper::onInitialize() {
    RCLCPP_INFO(get_logger(), "Initializing RTAB-Map wrapper");

    // Create monitoring node
    node_ = rclcpp::Node::make_shared("rtabmap_wrapper_monitor");

    // Setup QoS profiles
    auto reliable_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    auto sensor_qos = rclcpp::QoS(10).best_effort();

    // Subscribe to RTAB-Map output topics for health monitoring
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/rtabmap/grid_map", reliable_qos,
        std::bind(&RTABMapWrapper::mapCallback, this, std::placeholders::_1));

    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rtabmap/localization_pose", reliable_qos,
        std::bind(&RTABMapWrapper::poseCallback, this, std::placeholders::_1));

    graph_sub_ = node_->create_subscription<rtabmap_msgs::msg::MapGraph>(
        "/rtabmap/mapGraph", reliable_qos,
        std::bind(&RTABMapWrapper::mapGraphCallback, this, std::placeholders::_1));

    mapdata_sub_ = node_->create_subscription<rtabmap_msgs::msg::MapData>(
        "/rtabmap/mapData", reliable_qos,
        std::bind(&RTABMapWrapper::mapDataCallback, this, std::placeholders::_1));

    // Create publisher for Nav2 map topic (relay from /rtabmap/grid_map to /map)
    nav2_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1).transient_local().reliable());

    // Create service clients for RTAB-Map control
    reset_client_ = node_->create_client<std_srvs::srv::Empty>("/rtabmap/reset");
    pause_client_ = node_->create_client<std_srvs::srv::Empty>("/rtabmap/pause");
    resume_client_ = node_->create_client<std_srvs::srv::Empty>("/rtabmap/resume");
    reset_pose_client_ = node_->create_client<rtabmap_msgs::srv::ResetPose>("/rtabmap/reset_pose");

    // Create executor for RTAB-Map node
    rtabmap_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    monitoring_active_ = true;

    RCLCPP_INFO(get_logger(), "RTAB-Map wrapper initialized successfully");
    return true;
}

bool RTABMapWrapper::onStart() {
    RCLCPP_INFO(get_logger(), "Starting RTAB-Map wrapper");

    try {
        launchRTABMap();

        // Start monitoring node spin
        rtabmap_spin_thread_ = std::thread(&RTABMapWrapper::spinRTABMapNode, this);

        RCLCPP_INFO(get_logger(), "RTAB-Map wrapper started successfully");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to start RTAB-Map: %s", e.what());
        return false;
    }
}

bool RTABMapWrapper::onStop() {
    RCLCPP_INFO(get_logger(), "Stopping RTAB-Map wrapper");

    monitoring_active_ = false;

    stopRTABMap();

    if (rtabmap_spin_thread_.joinable()) {
        rtabmap_spin_thread_.join();
    }

    RCLCPP_INFO(get_logger(), "RTAB-Map wrapper stopped");
    return true;
}

bool RTABMapWrapper::onShutdown() {
    RCLCPP_INFO(get_logger(), "Shutting down RTAB-Map wrapper");

    // Clean up subscribers
    map_sub_.reset();
    pose_sub_.reset();
    graph_sub_.reset();
    mapdata_sub_.reset();

    // Clean up service clients
    reset_client_.reset();
    pause_client_.reset();
    resume_client_.reset();
    reset_pose_client_.reset();

    rtabmap_executor_.reset();
    rtabmap_node_.reset();

    RCLCPP_INFO(get_logger(), "RTAB-Map wrapper shutdown complete");
    return true;
}

void RTABMapWrapper::launchRTABMap() {
    RCLCPP_INFO(get_logger(), "Launching RTAB-Map SLAM node");

    // Create parameter map for rtabmap node
    std::vector<rclcpp::Parameter> params;

    // Subscription settings
    params.push_back(rclcpp::Parameter("subscribe_stereo", config_.stereo));
    params.push_back(rclcpp::Parameter("subscribe_rgbd", config_.subscribe_rgbd));
    params.push_back(rclcpp::Parameter("subscribe_depth", false));
    params.push_back(rclcpp::Parameter("subscribe_rgb", false));
    params.push_back(rclcpp::Parameter("subscribe_scan", false));
    params.push_back(rclcpp::Parameter("subscribe_scan_cloud", false));
    params.push_back(rclcpp::Parameter("subscribe_user_data", false));
    params.push_back(rclcpp::Parameter("subscribe_odom_info", false));

    // Frame IDs
    params.push_back(rclcpp::Parameter("frame_id", config_.frame_id));
    params.push_back(rclcpp::Parameter("odom_frame_id", config_.odom_frame_id));
    params.push_back(rclcpp::Parameter("map_frame_id", config_.map_frame_id));
    params.push_back(rclcpp::Parameter("publish_tf", true));

    // Database
    params.push_back(rclcpp::Parameter("database_path", config_.database_path));
    if (config_.delete_db_on_start) {
        // Delete database if exists
        std::filesystem::path db_path(config_.database_path);
        if (std::filesystem::exists(db_path)) {
            RCLCPP_WARN(get_logger(), "Deleting existing database: %s", config_.database_path.c_str());
            std::filesystem::remove(db_path);
        }
    }

    // Synchronization
    params.push_back(rclcpp::Parameter("approx_sync", config_.approx_sync));
    params.push_back(rclcpp::Parameter("approx_sync_max_interval", config_.approx_sync_max_interval));
    params.push_back(rclcpp::Parameter("queue_size", config_.queue_size));
    params.push_back(rclcpp::Parameter("sync_queue_size", config_.queue_size));

    // QoS settings
    params.push_back(rclcpp::Parameter("qos_image", 1));  // Reliable
    params.push_back(rclcpp::Parameter("qos_camera_info", 1));  // Reliable
    params.push_back(rclcpp::Parameter("qos_odom", 1));  // Reliable
    params.push_back(rclcpp::Parameter("qos_imu", 1));  // Reliable

    // RTAB-Map parameters (using ~ prefix for private parameters)
    params.push_back(rclcpp::Parameter("Rtabmap/DetectionRate", std::to_string(config_.Rtabmap_DetectionRate)));
    params.push_back(rclcpp::Parameter("Rtabmap/TimeThr", std::to_string(config_.Rtabmap_TimeThr)));

    // Memory management
    params.push_back(rclcpp::Parameter("Mem/IncrementalMemory",
        config_.localization_mode ? "false" : "true"));
    params.push_back(rclcpp::Parameter("Mem/InitWMWithAllNodes",
        config_.localization_mode ? "true" : "false"));
    params.push_back(rclcpp::Parameter("Mem/ImageKept", std::to_string(config_.Mem_ImageKept)));
    params.push_back(rclcpp::Parameter("Mem/STMSize", std::to_string(config_.Mem_STMSize)));
    params.push_back(rclcpp::Parameter("Mem/ReduceGraph", std::to_string(config_.Mem_ReduceGraph)));

    // Loop closure
    params.push_back(rclcpp::Parameter("RGBD/LoopClosureReextractFeatures",
        config_.RGBD_LoopClosureReextractFeatures ? "true" : "false"));
    params.push_back(rclcpp::Parameter("Vis/MinInliers", std::to_string(config_.Vis_MinInliers)));
    params.push_back(rclcpp::Parameter("Vis/InlierDistance", std::to_string(config_.Vis_InlierDistance)));

    // Optimization
    params.push_back(rclcpp::Parameter("RGBD/OptimizeMaxError", std::to_string(config_.RGBD_OptimizeMaxError)));
    params.push_back(rclcpp::Parameter("RGBD/ProximityBySpace",
        config_.RGBD_ProximityBySpace ? "true" : "false"));

    // Performance
    params.push_back(rclcpp::Parameter("Vis/MaxFeatures", std::to_string(config_.Vis_MaxFeatures)));
    params.push_back(rclcpp::Parameter("SURF/HessianThreshold", std::to_string(config_.SURF_HessianThreshold)));

    // Create rtabmap node options
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(params);
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);

    // Create rtabmap node
    rtabmap_node_ = rclcpp::Node::make_shared("rtabmap", "rtabmap", node_options);

    // Setup topic remappings manually by creating subscriptions
    // Note: We can't directly remap topics in node creation, so we'll rely on
    // launch file remappings or the default topic names from ZED

    rtabmap_executor_->add_node(rtabmap_node_);

    RCLCPP_INFO(get_logger(), "RTAB-Map node created and configured");
    RCLCPP_INFO(get_logger(), "  Mode: %s", config_.localization_mode ? "Localization" : "SLAM");
    RCLCPP_INFO(get_logger(), "  Database: %s", config_.database_path.c_str());
    RCLCPP_INFO(get_logger(), "  Left image topic: %s", config_.left_image_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Right image topic: %s", config_.right_image_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Odom topic: %s", config_.odom_topic.c_str());
}

void RTABMapWrapper::stopRTABMap() {
    RCLCPP_INFO(get_logger(), "Stopping RTAB-Map node");

    if (rtabmap_executor_) {
        rtabmap_executor_->cancel();
    }

    if (rtabmap_node_) {
        rtabmap_executor_->remove_node(rtabmap_node_);
    }
}

void RTABMapWrapper::spinRTABMapNode() {
    RCLCPP_INFO(get_logger(), "Starting RTAB-Map node executor");

    while (monitoring_active_ && rclcpp::ok()) {
        rtabmap_executor_->spin_some(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_logger(), "RTAB-Map node executor stopped");
}

bool RTABMapWrapper::performHealthCheck() {
    updateWatchdog();

    auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(metrics_mutex_);

    // Check if we're receiving pose updates (required for SLAM)
    auto time_since_pose = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_pose_time_).count();

    // Allow 5 seconds timeout for pose updates
    if (time_since_pose > 5000) {
        RCLCPP_WARN(get_logger(), "No pose updates for %.1f seconds", time_since_pose / 1000.0);
        return false;
    }

    // Map updates are less critical (only generated when new nodes are added)
    // Just log warning but don't fail health check
    auto time_since_map = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_map_time_).count();
    if (time_since_map > 30000) {  // 30 second warning threshold
        RCLCPP_DEBUG(get_logger(), "No map updates for %.1f seconds (nodes: %lu)",
            time_since_map / 1000.0, nodes_in_map_.load());
    }

    return true;
}

void RTABMapWrapper::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    map_updates_received_++;
    last_map_time_ = std::chrono::steady_clock::now();
    updateWatchdog();

    // Relay 2D occupancy grid to Nav2's /map topic
    if (nav2_map_pub_ && nav2_map_pub_->get_subscription_count() > 0) {
        nav2_map_pub_->publish(*msg);
    }
}

void RTABMapWrapper::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    pose_updates_received_++;
    last_pose_time_ = std::chrono::steady_clock::now();
    updateWatchdog();
}

void RTABMapWrapper::mapGraphCallback(const rtabmap_msgs::msg::MapGraph::SharedPtr msg) {
    nodes_in_map_ = msg->poses.size();
    RCLCPP_DEBUG(get_logger(), "Map graph updated: %lu nodes, %lu links",
        msg->poses.size(), msg->links.size());
    updateWatchdog();
}

void RTABMapWrapper::mapDataCallback(const rtabmap_msgs::msg::MapData::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Map data updated: %lu nodes", msg->nodes.size());
    updateWatchdog();
}

bool RTABMapWrapper::loadModuleConfig(const YAML::Node& config) {
    try {
        // Mode settings
        if (config["mode"]) {
            auto mode = config["mode"];
            if (mode["localization"]) config_.localization_mode = mode["localization"].as<bool>();
            if (mode["delete_db_on_start"]) config_.delete_db_on_start = mode["delete_db_on_start"].as<bool>();
        }

        // Database
        if (config["database"]) {
            auto db = config["database"];
            if (db["path"]) config_.database_path = db["path"].as<std::string>();
        }

        // Frame IDs
        if (config["frames"]) {
            auto frames = config["frames"];
            if (frames["base_link"]) config_.frame_id = frames["base_link"].as<std::string>();
            if (frames["odom"]) config_.odom_frame_id = frames["odom"].as<std::string>();
            if (frames["map"]) config_.map_frame_id = frames["map"].as<std::string>();
        }

        // Topics
        if (config["topics"]) {
            auto topics = config["topics"];
            if (topics["left_image"]) config_.left_image_topic = topics["left_image"].as<std::string>();
            if (topics["right_image"]) config_.right_image_topic = topics["right_image"].as<std::string>();
            if (topics["left_camera_info"]) config_.left_camera_info_topic = topics["left_camera_info"].as<std::string>();
            if (topics["right_camera_info"]) config_.right_camera_info_topic = topics["right_camera_info"].as<std::string>();
            if (topics["odom"]) config_.odom_topic = topics["odom"].as<std::string>();
            if (topics["imu"]) config_.imu_topic = topics["imu"].as<std::string>();
        }

        // RTAB-Map parameters
        if (config["rtabmap"]) {
            auto rtab = config["rtabmap"];
            if (rtab["queue_size"]) config_.queue_size = rtab["queue_size"].as<int>();
            if (rtab["approx_sync"]) config_.approx_sync = rtab["approx_sync"].as<bool>();
            if (rtab["approx_sync_max_interval"]) config_.approx_sync_max_interval = rtab["approx_sync_max_interval"].as<float>();
        }

        // SLAM settings
        if (config["slam"]) {
            auto slam = config["slam"];
            if (slam["detection_rate"]) config_.Rtabmap_DetectionRate = slam["detection_rate"].as<int>();
            if (slam["time_threshold"]) config_.Rtabmap_TimeThr = slam["time_threshold"].as<float>();
        }

        // Memory settings
        if (config["memory"]) {
            auto mem = config["memory"];
            if (mem["image_kept"]) config_.Mem_ImageKept = mem["image_kept"].as<int>();
            if (mem["stm_size"]) config_.Mem_STMSize = mem["stm_size"].as<int>();
            if (mem["reduce_graph"]) config_.Mem_ReduceGraph = mem["reduce_graph"].as<float>();
        }

        // Loop closure
        if (config["loop_closure"]) {
            auto lc = config["loop_closure"];
            if (lc["reextract_features"]) config_.RGBD_LoopClosureReextractFeatures = lc["reextract_features"].as<bool>();
            if (lc["min_inliers"]) config_.Vis_MinInliers = lc["min_inliers"].as<int>();
            if (lc["inlier_distance"]) config_.Vis_InlierDistance = lc["inlier_distance"].as<int>();
        }

        // Performance
        if (config["performance"]) {
            auto perf = config["performance"];
            if (perf["max_features"]) config_.Vis_MaxFeatures = perf["max_features"].as<int>();
            if (perf["hessian_threshold"]) config_.SURF_HessianThreshold = perf["hessian_threshold"].as<int>();
        }

        // Grid map generation for Nav2
        if (config["grid"]) {
            auto grid = config["grid"];
            if (grid["from_depth"]) config_.Grid_FromDepth = grid["from_depth"].as<bool>();
            if (grid["cell_size"]) config_.Grid_CellSize = grid["cell_size"].as<float>();
            if (grid["range_max"]) config_.Grid_RangeMax = grid["range_max"].as<float>();
            if (grid["range_min"]) config_.Grid_RangeMin = grid["range_min"].as<float>();
            if (grid["cluster_radius"]) config_.Grid_ClusterRadius = grid["cluster_radius"].as<float>();
            if (grid["ground_is_obstacle"]) config_.Grid_GroundIsObstacle = grid["ground_is_obstacle"].as<bool>();
            if (grid["max_obstacle_height"]) config_.Grid_MaxObstacleHeight = grid["max_obstacle_height"].as<float>();
            if (grid["max_ground_height"]) config_.Grid_MaxGroundHeight = grid["max_ground_height"].as<float>();
            if (grid["normals_segmentation"]) config_.Grid_NormalsSegmentation = grid["normals_segmentation"].as<bool>();
            if (grid["3d"]) config_.Grid_3D = grid["3d"].as<bool>();
            if (grid["noise_filtering_radius"]) config_.Grid_NoiseFilteringRadius = grid["noise_filtering_radius"].as<int>();
            if (grid["noise_filtering_min_neighbors"]) config_.Grid_NoiseFilteringMinNeighbors = grid["noise_filtering_min_neighbors"].as<int>();
        }

        RCLCPP_INFO(get_logger(), "Configuration loaded successfully");
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load configuration: %s", e.what());
        return false;
    }
}

bool RTABMapWrapper::saveModuleConfig(YAML::Emitter& out) {
    try {
        out << YAML::BeginMap;

        // Mode
        out << YAML::Key << "mode" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "localization" << YAML::Value << config_.localization_mode;
        out << YAML::Key << "delete_db_on_start" << YAML::Value << config_.delete_db_on_start;
        out << YAML::EndMap;

        // Database
        out << YAML::Key << "database" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "path" << YAML::Value << config_.database_path;
        out << YAML::EndMap;

        // Frames
        out << YAML::Key << "frames" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "base_link" << YAML::Value << config_.frame_id;
        out << YAML::Key << "odom" << YAML::Value << config_.odom_frame_id;
        out << YAML::Key << "map" << YAML::Value << config_.map_frame_id;
        out << YAML::EndMap;

        // Topics
        out << YAML::Key << "topics" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "left_image" << YAML::Value << config_.left_image_topic;
        out << YAML::Key << "right_image" << YAML::Value << config_.right_image_topic;
        out << YAML::Key << "left_camera_info" << YAML::Value << config_.left_camera_info_topic;
        out << YAML::Key << "right_camera_info" << YAML::Value << config_.right_camera_info_topic;
        out << YAML::Key << "odom" << YAML::Value << config_.odom_topic;
        out << YAML::Key << "imu" << YAML::Value << config_.imu_topic;
        out << YAML::EndMap;

        // RTAB-Map
        out << YAML::Key << "rtabmap" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "queue_size" << YAML::Value << config_.queue_size;
        out << YAML::Key << "approx_sync" << YAML::Value << config_.approx_sync;
        out << YAML::Key << "approx_sync_max_interval" << YAML::Value << config_.approx_sync_max_interval;
        out << YAML::EndMap;

        out << YAML::EndMap;

        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to save configuration: %s", e.what());
        return false;
    }
}

// Public RTAB-Map control methods

bool RTABMapWrapper::saveMap(const std::string& path) {
    RCLCPP_INFO(get_logger(), "Saving map to: %s", path.c_str());

    // RTAB-Map automatically saves to database on shutdown
    // For explicit save, we would need to use the /rtabmap/save_map service
    // which is not implemented in this basic wrapper

    RCLCPP_WARN(get_logger(), "Map auto-saves to database: %s", config_.database_path.c_str());
    return true;
}

bool RTABMapWrapper::resetMap() {
    RCLCPP_INFO(get_logger(), "Resetting RTAB-Map");

    if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Reset service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = reset_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Map reset successfully");

        // Reset metrics
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        map_updates_received_ = 0;
        pose_updates_received_ = 0;
        nodes_in_map_ = 0;

        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to reset map");
        return false;
    }
}

bool RTABMapWrapper::setLocalizationMode(bool enable) {
    RCLCPP_INFO(get_logger(), "Switching to %s mode", enable ? "localization" : "SLAM");

    // Changing mode requires restarting RTAB-Map
    config_.localization_mode = enable;

    if (getState() == core::ModuleState::RUNNING) {
        RCLCPP_WARN(get_logger(), "Mode change requires restart. Restarting RTAB-Map...");
        stop();
        start();
    }

    return true;
}

bool RTABMapWrapper::pauseMapping() {
    RCLCPP_INFO(get_logger(), "Pausing RTAB-Map mapping");

    if (!pause_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Pause service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = pause_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        is_paused_ = true;
        RCLCPP_INFO(get_logger(), "Mapping paused");
        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to pause mapping");
        return false;
    }
}

bool RTABMapWrapper::resumeMapping() {
    RCLCPP_INFO(get_logger(), "Resuming RTAB-Map mapping");

    if (!resume_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Resume service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = resume_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        is_paused_ = false;
        RCLCPP_INFO(get_logger(), "Mapping resumed");
        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to resume mapping");
        return false;
    }
}

} // namespace perception
} // namespace modular_v
