#include <rclcpp/rclcpp.hpp>
#include "core/module_manager.hpp"
#include <signal.h>
#include <memory>

// Global pointer for signal handler
std::shared_ptr<modular_v::core::ModuleManager> g_manager = nullptr;

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Interrupt signal (%d) received.", signum);

    if (g_manager) {
        g_manager->emergencyStop();
        g_manager->shutdownSystem();
    }

    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Create module manager
    g_manager = std::make_shared<modular_v::core::ModuleManager>();

    // Load system configuration
    std::string config_path = "config/system_config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    RCLCPP_INFO(rclcpp::get_logger("main"),
        "Loading configuration from: %s", config_path.c_str());

    if (!g_manager->loadSystemConfig(config_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
            "Failed to load system configuration");
        return 1;
    }

    // Note: Module registration would typically happen here
    // Since we're using wrappers for existing ROS2 packages,
    // the actual nodes are launched via launch files

    RCLCPP_INFO(rclcpp::get_logger("main"),
        "Modular-V system starting...");

    // Create executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(g_manager->getNode());

    // Spin
    RCLCPP_INFO(rclcpp::get_logger("main"),
        "System running. Press Ctrl+C to shutdown.");
    executor.spin();

    // Cleanup
    g_manager->shutdownSystem();
    rclcpp::shutdown();

    return 0;
}