#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

/**
 * @file drive_controller.cpp
 * @brief Drive controller node for CUB-URC rover
 * 
 * Hardware Compatibility:
 * - Jetson Orin Nano Super Dev Kit (primary)
 * - Jetson Nano Dev Kit (secondary)
 * 
 * Motor controllers:
 * The BILDA 2x40A controllers read rc-servo pulses (about 50 Hz): 1500 us is stopped,
 * shorter is reverse, longer is forward. Pulse widths are set in drive_config.yaml.
 *
 * PWM Hardware Note:
 * PWM interface uses Linux sysfs (/sys/class/pwm/). The exact PWM chip and channel
 * numbering depends on the Jetson device. If PWM initialization fails:
 * 1. Check actual PWM devices: ls /sys/class/pwm/
 * 2. Update drive_config.yaml with correct chip numbers
 * 3. Ensure device tree overlay enables PWM if needed
 */

class DriveController : public rclcpp::Node
{
public:
    DriveController() : Node("drive_controller")
    {
        // Load configuration
        if (!load_config()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load drive configuration");
            rclcpp::shutdown();
            return;
        }

        // Initialize PWM interfaces
        if (!init_pwm()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize PWM");
            rclcpp::shutdown();
            return;
        }

        // Create command subscriber
        // Topic: /drive/cmd_vel - Standardized command velocity topic (geometry_msgs/Twist)
        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/drive/cmd_vel",
            10,
            std::bind(&DriveController::cmd_callback, this, std::placeholders::_1));

        // Start the timeout clock on the node's own clock. A default-constructed Time uses a
        // different clock source, and subtracting it from now() throws
        last_command_time_ = this->now();

        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&DriveController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Drive controller initialized. Subscribing to /drive/cmd_vel");
    }

    ~DriveController()
    {
        // Stop all motors (neutral pulse), and give the controllers a few frames to see it
        set_all_neutral();
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        // Cleanup PWM
        cleanup_pwm();
    }

private:
    struct PWMChannel {
        std::string name;
        int chip;
        int channel;
        int frequency;
        bool reversed = false;
        std::string path;
    };

    struct MotorCommand {
        float left_front_middle;
        float left_back;
        float right_front_middle;
        float right_back;
    };

    // Configuration
    std::map<std::string, PWMChannel> pwm_channels_;
    std::map<std::string, PWMChannel> motor_config_;
    float max_speed_;
    float acceleration_;
    float deadzone_;
    float front_middle_gain_;
    float back_gain_;
    float min_pulse_us_;
    float center_us_;
    float max_pulse_us_;
    MotorCommand current_command_{0.0f, 0.0f, 0.0f, 0.0f};
    MotorCommand target_command_{0.0f, 0.0f, 0.0f, 0.0f};
    rclcpp::Time last_command_time_;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config()
    {
        try {
            // Get package share directory
            std::string package_share_directory =
                ament_index_cpp::get_package_share_directory("drive");
            std::string config_file = package_share_directory + "/config/drive_config.yaml";

            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

            YAML::Node config = YAML::LoadFile(config_file);

            // Load PWM channels
            auto pwm_cfg = config["drive"]["pwm_channels"];
            for (auto it = pwm_cfg.begin(); it != pwm_cfg.end(); ++it) {
                std::string key = it->first.as<std::string>();
                PWMChannel channel;
                channel.name = it->second["name"].as<std::string>();
                channel.chip = it->second["chip"].as<int>();
                channel.channel = it->second["channel"].as<int>();
                channel.frequency = it->second["frequency"].as<int>();
                if (channel.frequency <= 0) {
                    throw std::runtime_error("pwm frequency must be > 0 for " + key);
                }
                channel.reversed = it->second["reversed"].as<bool>(false);
                channel.path = "/sys/class/pwm/pwmchip" + std::to_string(channel.chip) +
                               "/pwm" + std::to_string(channel.channel);
                pwm_channels_[key] = channel;
            }

            // Load command parameters
            max_speed_ = config["drive"]["command"]["max_speed"].as<float>(1.0f);
            acceleration_ = config["drive"]["command"]["acceleration"].as<float>(0.1f);
            deadzone_ = config["drive"]["command"]["deadzone"].as<float>(0.05f);
            front_middle_gain_ = config["drive"]["command"]["front_middle_gain"].as<float>(1.0f);
            back_gain_ = config["drive"]["command"]["back_gain"].as<float>(1.0f);
            min_pulse_us_ = config["drive"]["command"]["min_pulse_us"].as<float>(1100.0f);
            center_us_ = config["drive"]["command"]["center_us"].as<float>(1500.0f);
            max_pulse_us_ = config["drive"]["command"]["max_pulse_us"].as<float>(1900.0f);

            if (!(min_pulse_us_ < center_us_ && center_us_ < max_pulse_us_)) {
                throw std::runtime_error("need min_pulse_us < center_us < max_pulse_us");
            }
            for (const auto &entry : pwm_channels_) {
                const float period_us = 1e6f / entry.second.frequency;
                if (max_pulse_us_ >= period_us) {
                    throw std::runtime_error("max_pulse_us does not fit in the pwm period for " +
                                             entry.first + " (frequency should be about 50 Hz)");
                }
            }

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Config loading error: %s", e.what());
            return false;
        }
    }

    bool init_pwm()
    {
        try {
            for (auto &[key, channel] : pwm_channels_) {
                // Export PWM channel if not already exported
                std::string export_path = "/sys/class/pwm/pwmchip" + std::to_string(channel.chip) + "/export";
                std::ofstream export_file(export_path);
                if (export_file.is_open()) {
                    export_file << channel.channel;
                    export_file.close();
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                        "Could not export PWM channel %s. Path may not exist: %s", 
                        key.c_str(), export_path.c_str());
                }

                // Set frequency (period in nanoseconds)
                long period_ns = 1000000000 / channel.frequency;
                std::string period_path = channel.path + "/period";
                std::ofstream period_file(period_path);
                if (period_file.is_open()) {
                    period_file << period_ns;
                    period_file.close();
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                        "Could not set PWM period for %s. Check if PWM chip exists: %s", 
                        key.c_str(), period_path.c_str());
                }

                // Start at neutral so the controller arms stopped
                set_pwm_pulse_us(key, center_us_);

                // Enable PWM
                std::string enable_path = channel.path + "/enable";
                std::ofstream enable_file(enable_path);
                if (enable_file.is_open()) {
                    enable_file << "1";
                    enable_file.close();
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                        "Could not enable PWM for %s: %s", 
                        key.c_str(), enable_path.c_str());
                }

                RCLCPP_INFO(this->get_logger(), "Initialized PWM channel: %s (chip %d, ch %d)", 
                    key.c_str(), channel.chip, channel.channel);
            }
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "PWM initialization error: %s", e.what());
            return false;
        }
    }

    void cleanup_pwm()
    {
        for (auto &[key, channel] : pwm_channels_) {
            try {
                // Disable PWM
                std::string enable_path = channel.path + "/enable";
                std::ofstream enable_file(enable_path);
                if (enable_file.is_open()) {
                    enable_file << "0";
                    enable_file.close();
                }

                // Unexport PWM channel
                std::string unexport_path = "/sys/class/pwm/pwmchip" + std::to_string(channel.chip) + "/unexport";
                std::ofstream unexport_file(unexport_path);
                if (unexport_file.is_open()) {
                    unexport_file << channel.channel;
                    unexport_file.close();
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Cleanup error for %s: %s", key.c_str(), e.what());
            }
        }
    }

    void set_pwm_pulse_us(const std::string &channel_name, double pulse_us)
    {
        const auto it = pwm_channels_.find(channel_name);
        if (it == pwm_channels_.end()) {
            RCLCPP_WARN(this->get_logger(), "PWM channel not found: %s", channel_name.c_str());
            return;
        }

        const PWMChannel &channel = it->second;
        const long period_ns = 1000000000L / channel.frequency;
        long pulse_ns = std::lround(pulse_us * 1000.0);
        pulse_ns = std::max(0L, std::min(period_ns, pulse_ns));

        std::ofstream duty_file(channel.path + "/duty_cycle");
        if (duty_file.is_open()) {
            duty_file << pulse_ns;
        }
    }

    void set_all_neutral()
    {
        for (const auto &entry : pwm_channels_) {
            set_pwm_pulse_us(entry.first, center_us_);
        }
    }

    // signed speed (-1 to 1) -> pulse width in us. zero is the center (stopped)
    static double pulse_us_for_speed(float speed, float min_us, float center_us, float max_us)
    {
        if (!std::isfinite(speed)) {
            speed = 0.0f;
        }
        speed = std::max(-1.0f, std::min(1.0f, speed));
        const float span_us = speed >= 0.0f ? (max_us - center_us) : (center_us - min_us);
        return center_us + speed * span_us;
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract linear (forward/backward) velocity
        float linear_velocity = msg->linear.x;   // Forward/backward
        float angular_velocity = msg->angular.z; // Rotation

        // Differential drive kinematics for 4 independent channels:
        // Left side motors (front/middle + back) get the same base speed
        // Right side motors (front/middle + back) get the same base speed
        // Angular velocity adjusts left vs right speeds for turning
        
        float left_base_speed = linear_velocity - angular_velocity;
        float right_base_speed = linear_velocity + angular_velocity;

        // Normalize to -1.0 to 1.0 range
        float max_val = std::max(std::abs(left_base_speed), std::abs(right_base_speed));
        if (max_val > 1.0f) {
            left_base_speed /= max_val;
            right_base_speed /= max_val;
        }

        // Apply gains for front/middle vs back motors (can have different speeds)
        target_command_.left_front_middle = left_base_speed * front_middle_gain_;
        target_command_.left_back = left_base_speed * back_gain_;
        target_command_.right_front_middle = right_base_speed * front_middle_gain_;
        target_command_.right_back = right_base_speed * back_gain_;

        last_command_time_ = this->now();
    }

    void control_loop()
    {
        // Check for command timeout
        auto time_since_cmd = (this->now() - last_command_time_).seconds();
        if (time_since_cmd > 1.0) {
            target_command_ = {0.0f, 0.0f, 0.0f, 0.0f};
        }

        // Apply acceleration limits to all four channels
        float left_fm_delta = target_command_.left_front_middle - current_command_.left_front_middle;
        float left_back_delta = target_command_.left_back - current_command_.left_back;
        float right_fm_delta = target_command_.right_front_middle - current_command_.right_front_middle;
        float right_back_delta = target_command_.right_back - current_command_.right_back;

        if (std::abs(left_fm_delta) > acceleration_) {
            current_command_.left_front_middle += (left_fm_delta > 0 ? acceleration_ : -acceleration_);
        } else {
            current_command_.left_front_middle = target_command_.left_front_middle;
        }

        if (std::abs(left_back_delta) > acceleration_) {
            current_command_.left_back += (left_back_delta > 0 ? acceleration_ : -acceleration_);
        } else {
            current_command_.left_back = target_command_.left_back;
        }

        if (std::abs(right_fm_delta) > acceleration_) {
            current_command_.right_front_middle += (right_fm_delta > 0 ? acceleration_ : -acceleration_);
        } else {
            current_command_.right_front_middle = target_command_.right_front_middle;
        }

        if (std::abs(right_back_delta) > acceleration_) {
            current_command_.right_back += (right_back_delta > 0 ? acceleration_ : -acceleration_);
        } else {
            current_command_.right_back = target_command_.right_back;
        }

        // Apply deadzone to all channels
        if (std::abs(current_command_.left_front_middle) < deadzone_) {
            current_command_.left_front_middle = 0.0f;
        }
        if (std::abs(current_command_.left_back) < deadzone_) {
            current_command_.left_back = 0.0f;
        }
        if (std::abs(current_command_.right_front_middle) < deadzone_) {
            current_command_.right_front_middle = 0.0f;
        }
        if (std::abs(current_command_.right_back) < deadzone_) {
            current_command_.right_back = 0.0f;
        }

        // Set all PWM channels
        set_pwm_speed("motor_left_front_middle", current_command_.left_front_middle);
        set_pwm_speed("motor_left_back", current_command_.left_back);
        set_pwm_speed("motor_right_front_middle", current_command_.right_front_middle);
        set_pwm_speed("motor_right_back", current_command_.right_back);
    }

    void set_pwm_speed(const std::string &channel_name, float speed)
    {
        const auto it = pwm_channels_.find(channel_name);
        if (it == pwm_channels_.end()) {
            RCLCPP_WARN(this->get_logger(), "PWM channel not found: %s", channel_name.c_str());
            return;
        }

        if (it->second.reversed) {
            speed = -speed;
        }
        set_pwm_pulse_us(channel_name, pulse_us_for_speed(speed, min_pulse_us_, center_us_, max_pulse_us_));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
