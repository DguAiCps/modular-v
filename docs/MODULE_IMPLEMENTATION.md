# 모듈 구현 가이드

## 목차
1. [모듈 개발 개요](#1-모듈-개발-개요)
2. [카메라 모듈](#2-카메라-모듈)
3. [SLAM 모듈](#3-slam-모듈)
4. [내비게이션 모듈](#4-내비게이션-모듈)
5. [사용자 인터페이스 모듈](#5-사용자-인터페이스-모듈)
6. [모터 제어 모듈](#6-모터-제어-모듈)

## 1. 모듈 개발 개요

### 1.1 모듈 템플릿

```cpp
// modules/template/include/template/template_module.hpp
#ifndef TEMPLATE_MODULE_HPP
#define TEMPLATE_MODULE_HPP

#include "core/module_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>

namespace modular_v {
namespace modules {

class TemplateModule : public core::IModule {
public:
    TemplateModule();
    ~TemplateModule() override;

    // IModule interface
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

protected:
    // 주요 처리 루프
    virtual void processLoop();

    // ROS2 노드
    rclcpp::Node::SharedPtr node_;

    // 상태 변수
    std::atomic<core::ModuleState> state_{core::ModuleState::UNINITIALIZED};
    std::atomic<bool> running_{false};

    // 메타데이터
    std::string name_;
    std::string version_;

    // 처리 스레드
    std::thread process_thread_;

    // 설정
    struct Config {
        // 모듈별 설정 추가
    } config_;
};

} // namespace modules
} // namespace modular_v

#endif // TEMPLATE_MODULE_HPP
```

## 2. 카메라 모듈

### 2.1 ZED 2i 카메라 모듈 전체 구현

```cpp
// modules/perception/camera_module/include/camera_module/zed_camera_module.hpp
#ifndef ZED_CAMERA_MODULE_HPP
#define ZED_CAMERA_MODULE_HPP

#include "core/module_interface.hpp"
#include <sl/Camera.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace modular_v {
namespace perception {

class ZedCameraModule : public core::IModule {
public:
    ZedCameraModule();
    ~ZedCameraModule() override;

    // IModule implementation
    bool initialize() override;
    bool start() override;
    bool stop() override;
    bool shutdown() override;
    bool pause() override { return true; }
    bool resume() override { return true; }

    core::ModuleState getState() const override { return state_; }
    std::string getName() const override { return name_; }
    std::string getVersion() const override { return version_; }

    bool loadConfig(const std::string& config_path) override;
    bool saveConfig(const std::string& config_path) override;
    bool isHealthy() const override;
    std::string getHealthStatus() const override;

private:
    // 주요 처리 함수
    void captureLoop();
    void processFrame();
    void publishImage(const sl::Mat& zed_image, bool is_depth = false);
    void publishPointCloud(const sl::Mat& point_cloud);
    void publishIMU();
    void publishPose();

    // 헬퍼 함수
    sensor_msgs::msg::Image::SharedPtr convertToROSImage(const sl::Mat& zed_image);
    sensor_msgs::msg::PointCloud2::SharedPtr convertToROSPointCloud(const sl::Mat& point_cloud);

    // ZED SDK
    std::unique_ptr<sl::Camera> zed_;
    sl::InitParameters init_params_;
    sl::RuntimeParameters runtime_params_;
    sl::PositionalTrackingParameters tracking_params_;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

    // TF2
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 이미지 트랜스포트
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher it_rgb_pub_;
    image_transport::Publisher it_depth_pub_;

    // 처리 스레드
    std::thread capture_thread_;
    std::atomic<bool> running_{false};

    // 상태 관리
    std::atomic<core::ModuleState> state_{core::ModuleState::UNINITIALIZED};
    std::string name_ = "ZED 2i Camera Module";
    std::string version_ = "1.0.0";

    // 성능 메트릭
    struct Metrics {
        std::atomic<uint64_t> frames_captured{0};
        std::atomic<uint64_t> frames_dropped{0};
        std::atomic<double> avg_processing_time{0.0};
        std::chrono::steady_clock::time_point last_frame_time;
    } metrics_;

    // 설정
    struct Config {
        // 카메라 설정
        int resolution = 2;  // HD720
        int fps = 15;
        int depth_mode = 1;  // ULTRA
        float depth_minimum_distance = 0.3f;
        float depth_maximum_distance = 20.0f;

        // 기능 설정
        bool enable_imu = true;
        bool enable_tracking = true;
        bool enable_point_cloud = true;
        bool enable_object_detection = false;

        // 토픽 이름
        std::string camera_name = "zed2i";
        std::string frame_id = "zed2i_base_link";
    } config_;

    // 진단 정보
    mutable std::mutex diagnostics_mutex_;
    std::string last_error_message_;
};

} // namespace perception
} // namespace modular_v

#endif // ZED_CAMERA_MODULE_HPP
```

```cpp
// modules/perception/camera_module/src/zed_camera_module.cpp
#include "camera_module/zed_camera_module.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace modular_v {
namespace perception {

ZedCameraModule::ZedCameraModule() {
    name_ = "ZED 2i Camera Module";
    version_ = "1.0.0";
}

ZedCameraModule::~ZedCameraModule() {
    if (state_ == core::ModuleState::RUNNING) {
        stop();
    }
    if (state_ != core::ModuleState::UNINITIALIZED) {
        shutdown();
    }
}

bool ZedCameraModule::initialize() {
    try {
        // ROS2 노드 생성
        node_ = rclcpp::Node::make_shared(config_.camera_name + "_module");

        // Publisher 설정
        std::string base_topic = "/" + config_.camera_name;

        rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            base_topic + "/image_raw", rclcpp::QoS(10).best_effort());

        depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            base_topic + "/depth/image_rect", rclcpp::QoS(10).best_effort());

        info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            base_topic + "/camera_info", rclcpp::QoS(10).reliable());

        if (config_.enable_point_cloud) {
            pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                base_topic + "/point_cloud", rclcpp::QoS(5).best_effort());
        }

        if (config_.enable_imu) {
            imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
                base_topic + "/imu/data", rclcpp::QoS(100).best_effort());
        }

        if (config_.enable_tracking) {
            pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                base_topic + "/pose", rclcpp::QoS(10).best_effort());
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        }

        // Image Transport 초기화
        it_ = std::make_shared<image_transport::ImageTransport>(node_);
        it_rgb_pub_ = it_->advertise(base_topic + "/image_raw", 1);
        it_depth_pub_ = it_->advertise(base_topic + "/depth/image_rect", 1);

        // ZED 카메라 초기화
        zed_ = std::make_unique<sl::Camera>();

        // 초기화 파라미터 설정
        init_params_.camera_resolution = static_cast<sl::RESOLUTION>(config_.resolution);
        init_params_.camera_fps = config_.fps;
        init_params_.coordinate_units = sl::UNIT::METER;
        init_params_.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        init_params_.depth_mode = static_cast<sl::DEPTH_MODE>(config_.depth_mode);
        init_params_.depth_minimum_distance = config_.depth_minimum_distance;
        init_params_.depth_maximum_distance = config_.depth_maximum_distance;
        init_params_.depth_stabilization = true;
        init_params_.enable_image_enhancement = true;

        // 카메라 열기
        auto err = zed_->open(init_params_);
        if (err != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to open ZED camera: %s", sl::toString(err).c_str());
            last_error_message_ = "Camera open failed: " + sl::toString(err);
            return false;
        }

        // 런타임 파라미터 설정
        runtime_params_.sensing_mode = sl::SENSING_MODE::FILL;
        runtime_params_.enable_depth = true;
        runtime_params_.confidence_threshold = 95;
        runtime_params_.texture_confidence_threshold = 100;

        // Tracking 초기화
        if (config_.enable_tracking) {
            tracking_params_.enable_area_memory = true;
            tracking_params_.enable_imu_fusion = config_.enable_imu;
            tracking_params_.enable_pose_smoothing = true;

            err = zed_->enablePositionalTracking(tracking_params_);
            if (err != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(),
                    "Failed to enable tracking: %s", sl::toString(err).c_str());
                config_.enable_tracking = false;
            }
        }

        // Object Detection 초기화
        if (config_.enable_object_detection) {
            sl::ObjectDetectionParameters obj_params;
            obj_params.enable_tracking = true;
            obj_params.detection_model = sl::DETECTION_MODEL::MULTI_CLASS_BOX;

            err = zed_->enableObjectDetection(obj_params);
            if (err != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(),
                    "Failed to enable object detection: %s", sl::toString(err).c_str());
                config_.enable_object_detection = false;
            }
        }

        state_ = core::ModuleState::INITIALIZED;
        RCLCPP_INFO(node_->get_logger(), "ZED Camera Module initialized successfully");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Initialization failed: %s", e.what());
        last_error_message_ = std::string("Init exception: ") + e.what();
        return false;
    }
}

bool ZedCameraModule::start() {
    if (state_ != core::ModuleState::INITIALIZED) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot start: module not initialized");
        return false;
    }

    running_ = true;
    capture_thread_ = std::thread(&ZedCameraModule::captureLoop, this);
    state_ = core::ModuleState::RUNNING;

    RCLCPP_INFO(node_->get_logger(), "ZED Camera Module started");
    return true;
}

bool ZedCameraModule::stop() {
    if (state_ != core::ModuleState::RUNNING) {
        return false;
    }

    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    state_ = core::ModuleState::INITIALIZED;
    RCLCPP_INFO(node_->get_logger(), "ZED Camera Module stopped");
    return true;
}

bool ZedCameraModule::shutdown() {
    if (state_ == core::ModuleState::RUNNING) {
        stop();
    }

    if (zed_ && zed_->isOpened()) {
        if (config_.enable_object_detection) {
            zed_->disableObjectDetection();
        }
        if (config_.enable_tracking) {
            zed_->disablePositionalTracking();
        }
        zed_->close();
    }

    state_ = core::ModuleState::TERMINATED;
    RCLCPP_INFO(node_->get_logger(), "ZED Camera Module shutdown complete");
    return true;
}

void ZedCameraModule::captureLoop() {
    sl::Mat zed_image, zed_depth, zed_point_cloud;
    sl::Pose camera_pose;
    sl::SensorsData sensors_data;

    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    while (running_ && rclcpp::ok()) {
        // 프레임 캡처
        if (zed_->grab(runtime_params_) == sl::ERROR_CODE::SUCCESS) {
            metrics_.last_frame_time = std::chrono::steady_clock::now();
            frame_count++;

            // RGB 이미지
            zed_->retrieveImage(zed_image, sl::VIEW::LEFT);
            publishImage(zed_image, false);

            // Depth 이미지
            zed_->retrieveImage(zed_depth, sl::VIEW::DEPTH);
            publishImage(zed_depth, true);

            // Point Cloud
            if (config_.enable_point_cloud) {
                zed_->retrieveMeasure(zed_point_cloud, sl::MEASURE::XYZRGBA);
                publishPointCloud(zed_point_cloud);
            }

            // IMU 데이터
            if (config_.enable_imu) {
                zed_->getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE);
                publishIMU();
            }

            // Tracking 데이터
            if (config_.enable_tracking) {
                zed_->getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
                publishPose();
            }

            metrics_.frames_captured++;

            // FPS 계산
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_fps_time).count();
            if (duration >= 1) {
                double fps = frame_count / static_cast<double>(duration);
                RCLCPP_DEBUG(node_->get_logger(), "FPS: %.1f", fps);
                frame_count = 0;
                last_fps_time = now;
            }
        } else {
            metrics_.frames_dropped++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void ZedCameraModule::publishImage(const sl::Mat& zed_image, bool is_depth) {
    auto ros_image = convertToROSImage(zed_image);
    if (!ros_image) return;

    ros_image->header.stamp = node_->get_clock()->now();
    ros_image->header.frame_id = config_.frame_id;

    if (is_depth) {
        depth_pub_->publish(*ros_image);
        it_depth_pub_.publish(ros_image);
    } else {
        rgb_pub_->publish(*ros_image);
        it_rgb_pub_.publish(ros_image);
    }
}

void ZedCameraModule::publishPointCloud(const sl::Mat& point_cloud) {
    auto ros_pc = convertToROSPointCloud(point_cloud);
    if (!ros_pc) return;

    ros_pc->header.stamp = node_->get_clock()->now();
    ros_pc->header.frame_id = config_.frame_id;
    pc_pub_->publish(*ros_pc);
}

sensor_msgs::msg::Image::SharedPtr ZedCameraModule::convertToROSImage(
    const sl::Mat& zed_image) {
    try {
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = node_->get_clock()->now();
        cv_image.header.frame_id = config_.frame_id;

        sl::Mat zed_image_cpu;
        zed_image.copyTo(zed_image_cpu, sl::MEM::CPU);

        int cv_type = CV_8UC4;
        if (zed_image.getDataType() == sl::MAT_TYPE::F32_C1) {
            cv_type = CV_32FC1;
            cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        } else {
            cv_image.encoding = sensor_msgs::image_encodings::BGRA8;
        }

        cv::Mat cv_mat(zed_image_cpu.getHeight(), zed_image_cpu.getWidth(),
                      cv_type, zed_image_cpu.getPtr<sl::uchar1>());
        cv_image.image = cv_mat;

        return cv_image.toImageMsg();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Image conversion failed: %s", e.what());
        return nullptr;
    }
}

sensor_msgs::msg::PointCloud2::SharedPtr ZedCameraModule::convertToROSPointCloud(
    const sl::Mat& point_cloud) {
    try {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        sl::Mat point_cloud_cpu;
        point_cloud.copyTo(point_cloud_cpu, sl::MEM::CPU);

        float* pc_data = point_cloud_cpu.getPtr<float>();
        int width = point_cloud_cpu.getWidth();
        int height = point_cloud_cpu.getHeight();

        pcl_cloud->width = width;
        pcl_cloud->height = height;
        pcl_cloud->is_dense = false;
        pcl_cloud->points.resize(width * height);

        for (int i = 0; i < width * height; i++) {
            pcl::PointXYZRGB& pt = pcl_cloud->points[i];
            pt.x = pc_data[i * 4 + 0];
            pt.y = pc_data[i * 4 + 1];
            pt.z = pc_data[i * 4 + 2];

            float color = pc_data[i * 4 + 3];
            uint32_t color_uint = *reinterpret_cast<uint32_t*>(&color);
            pt.r = (color_uint >> 16) & 0xFF;
            pt.g = (color_uint >> 8) & 0xFF;
            pt.b = color_uint & 0xFF;
        }

        auto ros_pc = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pcl_cloud, *ros_pc);
        return ros_pc;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "PointCloud conversion failed: %s", e.what());
        return nullptr;
    }
}

bool ZedCameraModule::loadConfig(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["camera"]) {
            auto cam = config["camera"];
            config_.resolution = cam["resolution"].as<int>(config_.resolution);
            config_.fps = cam["fps"].as<int>(config_.fps);
            config_.depth_mode = cam["depth_mode"].as<int>(config_.depth_mode);
            config_.depth_minimum_distance = cam["depth_min"].as<float>(config_.depth_minimum_distance);
            config_.depth_maximum_distance = cam["depth_max"].as<float>(config_.depth_maximum_distance);
        }

        if (config["features"]) {
            auto feat = config["features"];
            config_.enable_imu = feat["imu"].as<bool>(config_.enable_imu);
            config_.enable_tracking = feat["tracking"].as<bool>(config_.enable_tracking);
            config_.enable_point_cloud = feat["point_cloud"].as<bool>(config_.enable_point_cloud);
            config_.enable_object_detection = feat["object_detection"].as<bool>(config_.enable_object_detection);
        }

        RCLCPP_INFO(node_->get_logger(), "Configuration loaded from %s", config_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load config: %s", e.what());
        return false;
    }
}

bool ZedCameraModule::saveConfig(const std::string& config_path) {
    try {
        YAML::Node config;

        config["camera"]["resolution"] = config_.resolution;
        config["camera"]["fps"] = config_.fps;
        config["camera"]["depth_mode"] = config_.depth_mode;
        config["camera"]["depth_min"] = config_.depth_minimum_distance;
        config["camera"]["depth_max"] = config_.depth_maximum_distance;

        config["features"]["imu"] = config_.enable_imu;
        config["features"]["tracking"] = config_.enable_tracking;
        config["features"]["point_cloud"] = config_.enable_point_cloud;
        config["features"]["object_detection"] = config_.enable_object_detection;

        std::ofstream fout(config_path);
        fout << config;
        fout.close();

        RCLCPP_INFO(node_->get_logger(), "Configuration saved to %s", config_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to save config: %s", e.what());
        return false;
    }
}

bool ZedCameraModule::isHealthy() const {
    if (state_ != core::ModuleState::RUNNING) {
        return false;
    }

    // 최근 프레임 체크
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_frame = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - metrics_.last_frame_time).count();

    if (time_since_last_frame > 1000) {  // 1초 이상 프레임 없음
        return false;
    }

    // 드롭 프레임 비율 체크
    if (metrics_.frames_captured > 0) {
        double drop_rate = static_cast<double>(metrics_.frames_dropped) /
                          static_cast<double>(metrics_.frames_captured + metrics_.frames_dropped);
        if (drop_rate > 0.1) {  // 10% 이상 드롭
            return false;
        }
    }

    return true;
}

std::string ZedCameraModule::getHealthStatus() const {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);

    if (!last_error_message_.empty()) {
        return "ERROR: " + last_error_message_;
    }

    if (state_ != core::ModuleState::RUNNING) {
        return "Module not running";
    }

    std::stringstream ss;
    ss << "Frames: " << metrics_.frames_captured
       << ", Dropped: " << metrics_.frames_dropped
       << ", FPS: " << config_.fps;

    return ss.str();
}

} // namespace perception
} // namespace modular_v
```

## 3. SLAM 모듈

### 3.1 RTAB-Map 통합 모듈

```cpp
// modules/perception/rtabmap_module/include/rtabmap_module/rtabmap_module.hpp
#ifndef RTABMAP_MODULE_HPP
#define RTABMAP_MODULE_HPP

#include "core/module_interface.hpp"
#include <rtabmap_ros/CoreWrapper.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace modular_v {
namespace perception {

class RTABMapModule : public core::IModule {
public:
    RTABMapModule();
    ~RTABMapModule() override;

    // IModule implementation
    bool initialize() override;
    bool start() override;
    bool stop() override;
    bool shutdown() override;
    // ... 기타 필수 메서드들

private:
    // Callbacks
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Map management
    void updateMap();
    void publishMap();
    void saveMap(const std::string& path);
    void loadMap(const std::string& path);

    // Localization
    void updateLocalization();
    void publishPose();

    // Loop closure
    void detectLoopClosure();

    // Core
    std::unique_ptr<rtabmap::Rtabmap> rtabmap_;

    // ROS2
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Configuration
    struct Config {
        // SLAM parameters
        bool localization_mode = false;
        std::string database_path = "~/.ros/rtabmap.db";
        bool delete_db_on_start = false;

        // Processing parameters
        float update_rate = 1.0;  // Hz
        int odom_sensor = 0;  // 0: odometry, 1: visual

        // Map parameters
        float grid_cell_size = 0.05;  // meters
        float map_update_distance = 1.0;  // meters
        float map_update_angle = 30.0;  // degrees

        // Memory management
        int max_memory_size = 0;  // 0 = unlimited
        float memory_threshold = 0;  // 0 = disabled
    } config_;
};

} // namespace perception
} // namespace modular_v

#endif // RTABMAP_MODULE_HPP
```

## 4. 내비게이션 모듈

### 4.1 통합 내비게이션 모듈

```cpp
// modules/navigation/include/navigation/navigation_module.hpp
#ifndef NAVIGATION_MODULE_HPP
#define NAVIGATION_MODULE_HPP

#include "core/module_interface.hpp"
#include <nav2_core/behavior_tree_navigator.hpp>
#include <nav2_core/planner.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <actionlib/server/simple_action_server.hpp>

namespace modular_v {
namespace navigation {

class NavigationModule : public core::IModule {
public:
    NavigationModule();
    ~NavigationModule() override;

    // IModule implementation
    bool initialize() override;
    bool start() override;
    bool stop() override;
    // ... 기타 메서드

    // Navigation API
    bool navigateToGoal(const geometry_msgs::msg::PoseStamped& goal);
    bool cancelNavigation();
    nav_msgs::msg::Path getCurrentPath() const;
    geometry_msgs::msg::PoseStamped getCurrentGoal() const;
    NavigationState getNavigationState() const;

private:
    // Navigation components
    std::unique_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_;
    std::unique_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_;

    std::unique_ptr<nav2_core::GlobalPlanner> global_planner_;
    std::unique_ptr<nav2_core::LocalPlanner> local_planner_;
    std::unique_ptr<nav2_core::RecoveryBehavior> recovery_behavior_;

    // Path planning
    bool makePlan(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  nav_msgs::msg::Path& plan);

    // Motion control
    void computeVelocityCommands();
    bool isGoalReached();

    // Obstacle handling
    void updateObstacles();
    bool checkCollision(const geometry_msgs::msg::Twist& cmd_vel);

    // Recovery
    void executeRecovery();

    // State management
    enum class NavigationState {
        IDLE,
        PLANNING,
        EXECUTING,
        RECOVERY,
        SUCCEEDED,
        FAILED
    };

    NavigationState nav_state_{NavigationState::IDLE};
    geometry_msgs::msg::PoseStamped current_goal_;
    nav_msgs::msg::Path current_path_;

    // ROS2
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

    // Action server
    rclcpp_action::Server<NavigateToGoal>::SharedPtr action_server_;

    // Configuration
    struct Config {
        // Planner parameters
        std::string global_planner = "navfn";
        std::string local_planner = "dwa";

        // Robot parameters
        float max_vel_x = 1.0;  // m/s
        float max_vel_theta = 1.0;  // rad/s
        float acc_lim_x = 2.5;  // m/s^2
        float acc_lim_theta = 3.2;  // rad/s^2

        // Goal tolerance
        float xy_goal_tolerance = 0.25;  // meters
        float yaw_goal_tolerance = 0.25;  // radians

        // Safety
        float min_obstacle_distance = 0.5;  // meters
        float emergency_stop_distance = 0.3;  // meters
    } config_;
};

} // namespace navigation
} // namespace modular_v

#endif // NAVIGATION_MODULE_HPP
```

## 5. 사용자 인터페이스 모듈

### 5.1 음성 인터페이스 모듈

```cpp
// modules/interaction/audio_interface/include/audio_interface/voice_module.hpp
#ifndef VOICE_MODULE_HPP
#define VOICE_MODULE_HPP

#include "core/module_interface.hpp"
#include <std_msgs/msg/string.hpp>
#include <queue>
#include <mutex>

namespace modular_v {
namespace interaction {

class VoiceModule : public core::IModule {
public:
    VoiceModule();
    ~VoiceModule() override;

    // IModule implementation
    bool initialize() override;
    bool start() override;
    // ... 기타 메서드

    // Voice API
    void speak(const std::string& text, int priority = 0);
    void speakImmediate(const std::string& text);
    void stopSpeaking();

    // Command recognition
    bool startListening();
    bool stopListening();
    std::string getLastCommand() const;

private:
    // TTS (Text-to-Speech)
    void ttsWorker();
    void synthesizeSpeech(const std::string& text);
    void playAudio(const std::vector<uint8_t>& audio_data);

    // STT (Speech-to-Text)
    void sttWorker();
    void processAudioInput();
    std::string recognizeSpeech(const std::vector<int16_t>& audio_buffer);

    // Audio I/O
    void initAudioDevices();
    void captureAudio(std::vector<int16_t>& buffer);

    // Wake word detection
    bool detectWakeWord(const std::vector<int16_t>& audio);

    // Command processing
    void processCommand(const std::string& command);

    // TTS queue
    struct TTSRequest {
        std::string text;
        int priority;
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
    };

    std::priority_queue<TTSRequest> tts_queue_;
    std::mutex tts_mutex_;
    std::condition_variable tts_cv_;

    // Threads
    std::thread tts_thread_;
    std::thread stt_thread_;
    std::atomic<bool> tts_running_{false};
    std::atomic<bool> stt_running_{false};
    std::atomic<bool> is_speaking_{false};
    std::atomic<bool> is_listening_{false};

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_request_sub_;

    // Audio devices
    void* audio_input_device_;
    void* audio_output_device_;

    // Configuration
    struct Config {
        // TTS settings
        std::string tts_engine = "google";  // google, naver, offline
        std::string tts_language = "ko-KR";
        float tts_speed = 1.0;
        float tts_pitch = 1.0;

        // STT settings
        std::string stt_engine = "google";
        std::string stt_language = "ko-KR";
        std::string wake_word = "안내 로봇";

        // Audio settings
        int sample_rate = 16000;
        int channels = 1;
        int buffer_size = 1024;
    } config_;
};

} // namespace interaction
} // namespace modular_v

#endif // VOICE_MODULE_HPP
```

### 5.2 햅틱 피드백 모듈

```cpp
// modules/interaction/haptic_interface/include/haptic_interface/haptic_module.hpp
#ifndef HAPTIC_MODULE_HPP
#define HAPTIC_MODULE_HPP

#include "core/module_interface.hpp"
#include <modular_v/msg/haptic_feedback.hpp>

namespace modular_v {
namespace interaction {

class HapticModule : public core::IModule {
public:
    HapticModule();
    ~HapticModule() override;

    // Haptic patterns
    enum class Pattern {
        SINGLE_PULSE,
        DOUBLE_PULSE,
        TRIPLE_PULSE,
        CONTINUOUS,
        SOS,
        WARNING,
        SUCCESS,
        ERROR
    };

    // Haptic API
    void vibrate(Pattern pattern, int intensity = 50, int duration_ms = 100);
    void vibrateCustom(const std::vector<int>& pattern);
    void stopVibration();

    // Direction indication
    void indicateDirection(float angle_rad);
    void indicateObstacle(float distance, float angle);

private:
    // Hardware control
    void initHapticDevices();
    void setMotorPWM(int motor_id, int pwm_value);
    void executePattern(Pattern pattern);

    // Pattern generation
    std::vector<int> generatePattern(Pattern pattern);

    // Worker thread
    void hapticWorker();

    // Hardware
    std::vector<int> motor_pins_;
    void* gpio_device_;

    // Thread management
    std::thread haptic_thread_;
    std::queue<std::function<void()>> haptic_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_{false};

    // ROS2
    rclcpp::Subscription<modular_v::msg::HapticFeedback>::SharedPtr haptic_sub_;

    // Configuration
    struct Config {
        int num_motors = 4;
        std::vector<int> motor_pins = {17, 27, 22, 23};  // GPIO pins
        int pwm_frequency = 1000;  // Hz
        int default_intensity = 50;  // 0-100
    } config_;
};

} // namespace interaction
} // namespace modular_v

#endif // HAPTIC_MODULE_HPP
```

## 6. 모터 제어 모듈

### 6.1 베이스 컨트롤러 모듈

```cpp
// modules/mobility/base_controller/include/base_controller/base_controller_module.hpp
#ifndef BASE_CONTROLLER_MODULE_HPP
#define BASE_CONTROLLER_MODULE_HPP

#include "core/module_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace modular_v {
namespace mobility {

class BaseControllerModule : public core::IModule {
public:
    BaseControllerModule();
    ~BaseControllerModule() override;

    // IModule implementation
    bool initialize() override;
    bool start() override;
    // ... 기타 메서드

    // Motion control API
    void setVelocity(float linear_x, float angular_z);
    void stop();
    void emergencyStop();

    // Odometry
    nav_msgs::msg::Odometry getOdometry() const;

private:
    // Motor control
    void initMotors();
    void setMotorSpeed(int motor_id, float speed_rpm);
    void updateMotorControl();

    // Kinematics
    void computeWheelVelocities(float linear, float angular,
                                float& left_vel, float& right_vel);
    void computeOdometry(float left_vel, float right_vel, float dt);

    // Control loop
    void controlLoop();

    // PID control
    struct PIDController {
        float kp, ki, kd;
        float integral = 0;
        float prev_error = 0;

        float compute(float setpoint, float measured, float dt);
    };

    PIDController left_pid_, right_pid_;

    // Motor drivers
    void* motor_driver_;  // Dynamixel or other driver
    std::vector<int> motor_ids_;

    // State
    geometry_msgs::msg::Twist cmd_vel_;
    nav_msgs::msg::Odometry odometry_;
    std::mutex cmd_mutex_;

    // Threads
    std::thread control_thread_;
    std::atomic<bool> running_{false};

    // ROS2
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Configuration
    struct Config {
        // Robot parameters
        float wheel_radius = 0.1;  // meters
        float wheel_base = 0.5;  // meters
        float max_linear_vel = 1.0;  // m/s
        float max_angular_vel = 1.0;  // rad/s

        // Motor parameters
        int motor_left_id = 1;
        int motor_right_id = 2;
        float gear_ratio = 50.0;
        float encoder_resolution = 4096;

        // Control parameters
        float control_frequency = 50.0;  // Hz
        PIDController left_pid = {1.0, 0.1, 0.01};
        PIDController right_pid = {1.0, 0.1, 0.01};
    } config_;
};

} // namespace mobility
} // namespace modular_v

#endif // BASE_CONTROLLER_MODULE_HPP
```

### 6.2 오도메트리 모듈 구현 예제

```cpp
// modules/mobility/base_controller/src/base_controller_module.cpp (일부)
#include "base_controller/base_controller_module.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace modular_v {
namespace mobility {

void BaseControllerModule::computeOdometry(float left_vel, float right_vel, float dt) {
    // 차동 구동 로봇의 오도메트리 계산
    float linear_vel = (left_vel + right_vel) / 2.0;
    float angular_vel = (right_vel - left_vel) / config_.wheel_base;

    // 위치 업데이트 (2D)
    float delta_x = linear_vel * cos(odometry_.pose.pose.orientation.z) * dt;
    float delta_y = linear_vel * sin(odometry_.pose.pose.orientation.z) * dt;
    float delta_theta = angular_vel * dt;

    odometry_.pose.pose.position.x += delta_x;
    odometry_.pose.pose.position.y += delta_y;

    // Quaternion 업데이트
    tf2::Quaternion q;
    q.setRPY(0, 0, odometry_.pose.pose.orientation.z + delta_theta);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    // 속도 업데이트
    odometry_.twist.twist.linear.x = linear_vel;
    odometry_.twist.twist.angular.z = angular_vel;

    // 타임스탬프
    odometry_.header.stamp = node_->get_clock()->now();
    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";

    // Covariance 설정
    // Position covariance
    odometry_.pose.covariance[0] = 0.01;  // x
    odometry_.pose.covariance[7] = 0.01;  // y
    odometry_.pose.covariance[35] = 0.01;  // yaw

    // Velocity covariance
    odometry_.twist.covariance[0] = 0.01;  // linear x
    odometry_.twist.covariance[35] = 0.01;  // angular z
}

void BaseControllerModule::controlLoop() {
    auto last_time = std::chrono::steady_clock::now();

    // Dynamixel 초기화
    dynamixel::PortHandler* port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    dynamixel::PacketHandler* packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    port_handler->openPort();
    port_handler->setBaudRate(1000000);

    while (running_ && rclcpp::ok()) {
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;

        // 명령 속도 가져오기
        geometry_msgs::msg::Twist cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            cmd = cmd_vel_;
        }

        // 휠 속도 계산
        float left_vel, right_vel;
        computeWheelVelocities(cmd.linear.x, cmd.angular.z, left_vel, right_vel);

        // 엔코더 읽기
        int32_t left_encoder, right_encoder;
        packet_handler->read4ByteTxRx(port_handler, config_.motor_left_id,
                                     132, (uint32_t*)&left_encoder);  // Present Position
        packet_handler->read4ByteTxRx(port_handler, config_.motor_right_id,
                                     132, (uint32_t*)&right_encoder);

        // 실제 속도 계산
        static int32_t prev_left_encoder = left_encoder;
        static int32_t prev_right_encoder = right_encoder;

        float left_actual = (left_encoder - prev_left_encoder) * 2 * M_PI /
                           (config_.encoder_resolution * config_.gear_ratio * dt);
        float right_actual = (right_encoder - prev_right_encoder) * 2 * M_PI /
                            (config_.encoder_resolution * config_.gear_ratio * dt);

        prev_left_encoder = left_encoder;
        prev_right_encoder = right_encoder;

        // PID 제어
        float left_cmd = left_pid_.compute(left_vel, left_actual, dt);
        float right_cmd = right_pid_.compute(right_vel, right_actual, dt);

        // 모터 명령 전송
        int32_t left_velocity = static_cast<int32_t>(left_cmd * 60.0 /
                                                     (2 * M_PI * config_.wheel_radius));
        int32_t right_velocity = static_cast<int32_t>(right_cmd * 60.0 /
                                                      (2 * M_PI * config_.wheel_radius));

        packet_handler->write4ByteTxRx(port_handler, config_.motor_left_id,
                                      104, left_velocity);  // Goal Velocity
        packet_handler->write4ByteTxRx(port_handler, config_.motor_right_id,
                                      104, -right_velocity);  // 반대 방향

        // 오도메트리 업데이트
        computeOdometry(left_actual * config_.wheel_radius,
                       right_actual * config_.wheel_radius, dt);

        // 오도메트리 발행
        odom_pub_->publish(odometry_);

        // TF 발행
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odometry_.header;
        transform.child_frame_id = odometry_.child_frame_id;
        transform.transform.translation.x = odometry_.pose.pose.position.x;
        transform.transform.translation.y = odometry_.pose.pose.position.y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odometry_.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);

        // 제어 주기
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                static_cast<int>(1000.0 / config_.control_frequency)));
    }

    // 정지
    packet_handler->write4ByteTxRx(port_handler, config_.motor_left_id, 104, 0);
    packet_handler->write4ByteTxRx(port_handler, config_.motor_right_id, 104, 0);

    port_handler->closePort();
}

float BaseControllerModule::PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    integral += error * dt;
    float derivative = (error - prev_error) / dt;

    float output = kp * error + ki * integral + kd * derivative;

    prev_error = error;

    return output;
}

} // namespace mobility
} // namespace modular_v
```

## 7. 모듈 통합 및 시작

### 7.1 메인 어플리케이션

```cpp
// src/main.cpp
#include <rclcpp/rclcpp.hpp>
#include "core/module_manager.hpp"
#include "camera_module/zed_camera_module.hpp"
#include "rtabmap_module/rtabmap_module.hpp"
#include "navigation/navigation_module.hpp"
#include "audio_interface/voice_module.hpp"
#include "haptic_interface/haptic_module.hpp"
#include "base_controller/base_controller_module.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 모듈 매니저 생성
    auto manager = std::make_shared<modular_v::core::ModuleManager>();

    // 모듈 등록
    manager->registerModule("camera",
        std::make_shared<modular_v::perception::ZedCameraModule>());

    manager->registerModule("slam",
        std::make_shared<modular_v::perception::RTABMapModule>(),
        {"camera"});  // 의존성

    manager->registerModule("navigation",
        std::make_shared<modular_v::navigation::NavigationModule>(),
        {"slam"});

    manager->registerModule("voice",
        std::make_shared<modular_v::interaction::VoiceModule>());

    manager->registerModule("haptic",
        std::make_shared<modular_v::interaction::HapticModule>());

    manager->registerModule("base_controller",
        std::make_shared<modular_v::mobility::BaseControllerModule>());

    // 설정 로드
    std::string config_path = "config/system_config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }
    manager->loadSystemConfig(config_path);

    // 시스템 초기화 및 시작
    if (!manager->initializeSystem()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize system");
        return 1;
    }

    if (!manager->startSystem()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to start system");
        return 1;
    }

    // 시스템 실행
    RCLCPP_INFO(rclcpp::get_logger("main"), "Modular-V system running...");

    // 시그널 핸들러 설정
    signal(SIGINT, [](int) {
        rclcpp::shutdown();
    });

    // 메인 루프
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manager->getNode());

    // 각 모듈의 노드 추가
    for (const auto& module_name : manager->getModuleList()) {
        auto module = manager->getModule(module_name);
        if (module && module->getNode()) {
            executor.add_node(module->getNode());
        }
    }

    executor.spin();

    // 시스템 종료
    manager->stopSystem();
    manager->shutdownSystem();

    rclcpp::shutdown();
    return 0;
}
```

### 7.2 CMakeLists.txt (모듈별)

```cmake
# modules/perception/camera_module/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(camera_module)

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)

# ZED SDK
find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)

# OpenCV
find_package(OpenCV 4 REQUIRED)

# PCL
find_package(PCL 1.12 REQUIRED)

# 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/zed_camera_module.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  tf2_ros
)

# 설치
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs cv_bridge
  image_transport pcl_ros tf2_ros
)

ament_package()
```

이 문서는 각 핵심 모듈의 상세 구현 가이드를 제공합니다. 각 모듈은 독립적으로 개발 및 테스트가 가능하며, 표준 인터페이스를 통해 통합됩니다.