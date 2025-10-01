#ifndef VOICE_INTERFACE_HPP
#define VOICE_INTERFACE_HPP

#include "core/module_interface.hpp"
#include <std_msgs/msg/string.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace modular_v {
namespace interaction {

/**
 * @brief Voice interface for TTS and STT
 *
 * Provides text-to-speech and speech-to-text functionality
 * for user interaction
 */
class VoiceInterface : public core::IModule {
public:
    VoiceInterface();
    ~VoiceInterface() override;

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

    // Voice interface methods
    void speak(const std::string& text, int priority = 0);
    void speakImmediate(const std::string& text);
    void stopSpeaking();

    bool startListening();
    bool stopListening();
    std::string getLastCommand() const;

private:
    // TTS worker thread
    void ttsWorker();

    // STT worker thread
    void sttWorker();

    // TTS request structure
    struct TTSRequest {
        std::string text;
        int priority;
        std::chrono::time_point<std::chrono::steady_clock> timestamp;

        bool operator<(const TTSRequest& other) const {
            return priority < other.priority;
        }
    };

    // TTS queue management
    std::priority_queue<TTSRequest> tts_queue_;
    mutable std::mutex tts_mutex_;
    std::condition_variable tts_cv_;

    // Worker threads
    std::thread tts_thread_;
    std::thread stt_thread_;
    std::atomic<bool> tts_running_{false};
    std::atomic<bool> stt_running_{false};
    std::atomic<bool> is_speaking_{false};
    std::atomic<bool> is_listening_{false};

    // ROS2 publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_request_sub_;

    // Last recognized command
    mutable std::mutex command_mutex_;
    std::string last_command_;

    // Configuration
    struct Config {
        std::string tts_language = "ko-KR";
        std::string stt_language = "ko-KR";
        float tts_speed = 1.0;
        float tts_pitch = 1.0;
        std::string wake_word = "안내 로봇";
    } config_;
};

} // namespace interaction
} // namespace modular_v

#endif // VOICE_INTERFACE_HPP