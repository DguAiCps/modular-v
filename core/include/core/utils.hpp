#ifndef CORE_UTILS_HPP
#define CORE_UTILS_HPP

#include "core/module_interface.hpp"
#include <string>
#include <chrono>

namespace modular_v {
namespace core {
namespace utils {

/**
 * @brief Convert ModuleState enum to string
 * @param state Module state
 * @return String representation of state
 */
inline std::string moduleStateToString(ModuleState state) {
    switch (state) {
        case ModuleState::UNINITIALIZED: return "UNINITIALIZED";
        case ModuleState::INITIALIZED: return "INITIALIZED";
        case ModuleState::RUNNING: return "RUNNING";
        case ModuleState::PAUSED: return "PAUSED";
        case ModuleState::ERROR: return "ERROR";
        case ModuleState::TERMINATED: return "TERMINATED";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Convert string to ModuleState enum
 * @param state_str String representation of state
 * @return Module state enum
 */
inline ModuleState stringToModuleState(const std::string& state_str) {
    if (state_str == "UNINITIALIZED") return ModuleState::UNINITIALIZED;
    if (state_str == "INITIALIZED") return ModuleState::INITIALIZED;
    if (state_str == "RUNNING") return ModuleState::RUNNING;
    if (state_str == "PAUSED") return ModuleState::PAUSED;
    if (state_str == "ERROR") return ModuleState::ERROR;
    if (state_str == "TERMINATED") return ModuleState::TERMINATED;
    return ModuleState::UNINITIALIZED;
}

/**
 * @brief Simple timer class for measuring execution time
 */
class Timer {
public:
    Timer() : start_time_(std::chrono::steady_clock::now()) {}

    void reset() {
        start_time_ = std::chrono::steady_clock::now();
    }

    double elapsedSeconds() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - start_time_);
        return duration.count();
    }

    double elapsedMilliseconds() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_);
        return static_cast<double>(duration.count());
    }

    double elapsedMicroseconds() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now - start_time_);
        return static_cast<double>(duration.count());
    }

private:
    std::chrono::steady_clock::time_point start_time_;
};

/**
 * @brief RAII-style scoped timer that logs execution time
 */
class ScopedTimer {
public:
    ScopedTimer(const std::string& name, rclcpp::Logger logger)
        : name_(name), logger_(logger) {
        timer_.reset();
    }

    ~ScopedTimer() {
        RCLCPP_DEBUG(logger_, "%s took %.3f ms",
            name_.c_str(), timer_.elapsedMilliseconds());
    }

private:
    std::string name_;
    rclcpp::Logger logger_;
    Timer timer_;
};

/**
 * @brief Format duration in human-readable format
 * @param seconds Duration in seconds
 * @return Formatted string (e.g., "1h 23m 45s")
 */
inline std::string formatDuration(double seconds) {
    int hrs = static_cast<int>(seconds) / 3600;
    int mins = (static_cast<int>(seconds) % 3600) / 60;
    int secs = static_cast<int>(seconds) % 60;

    std::string result;
    if (hrs > 0) {
        result += std::to_string(hrs) + "h ";
    }
    if (mins > 0 || hrs > 0) {
        result += std::to_string(mins) + "m ";
    }
    result += std::to_string(secs) + "s";

    return result;
}

/**
 * @brief Clamp value between min and max
 */
template<typename T>
inline T clamp(T value, T min_value, T max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief Linear interpolation
 */
template<typename T>
inline T lerp(T a, T b, double t) {
    return a + (b - a) * t;
}

/**
 * @brief Map value from one range to another
 */
template<typename T>
inline T mapRange(T value, T in_min, T in_max, T out_min, T out_max) {
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

/**
 * @brief Degrees to radians conversion
 */
inline double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

/**
 * @brief Radians to degrees conversion
 */
inline double radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

/**
 * @brief Normalize angle to [-pi, pi]
 */
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace utils
} // namespace core
} // namespace modular_v

#endif // CORE_UTILS_HPP
