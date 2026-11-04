#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <sys/stat.h>
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
 * Motor controller signal (goBILDA 2x40A):
 * The controllers take RC-servo style pulses, not a variable duty cycle:
 * - 50 Hz frame (20 ms period)
 * - 1500 us = stop (neutral), 1050 us = full reverse, 1950 us = full forward
 * - They need ~1 s of neutral pulses after power-up before they will drive (arming)
 * All of these are set in drive_config.yaml (drive.pulse_us, drive.command.arm_time).
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
        // Both times must come from this node's clock: subtracting an
        // rclcpp::Time with a different clock type throws and kills the node.
        last_command_time_ = this->now();
        start_time_ = this->now();

        // Load configuration
        if (!load_config()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load drive configuration");
            rclcpp::shutdown();
            return;
        }

        // Initialize PWM interfaces (outputs start at neutral = stopped)
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

        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&DriveController::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
            "Drive controller initialized. Subscribing to /drive/cmd_vel. "
            "Pulses %d/%d/%d us (rev/stop/fwd), arming for %.1f s",
            pulse_min_us_, pulse_neutral_us_, pulse_max_us_, arm_time_);
    }

    ~DriveController()
    {
        // Stop all motors: send neutral (stop) pulses, then disable the outputs
        for (const auto &entry : pwm_channels_) {
            set_pulse_us(entry.first, pulse_neutral_us_);
        }

        // Cleanup PWM
        cleanup_pwm();
    }

private:
    struct PWMChannel {
        std::string name;
        int chip;
        int channel;
        int frequency;
        bool invert;
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
    float max_speed_;
    float acceleration_;
    float deadzone_;
    float front_middle_gain_;
    float back_gain_;
    float cmd_timeout_{1.0f};
    float arm_time_{1.0f};
    int pulse_min_us_{1050};
    int pulse_neutral_us_{1500};
    int pulse_max_us_{1950};
    MotorCommand current_command_{0.0f, 0.0f, 0.0f, 0.0f};
    MotorCommand target_command_{0.0f, 0.0f, 0.0f, 0.0f};
    rclcpp::Time last_command_time_;
    rclcpp::Time start_time_;
    bool armed_logged_{false};

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
                channel.name = it->second["name"].as<std::string>(key);
                channel.chip = it->second["chip"].as<int>();
                channel.channel = it->second["channel"].as<int>();
                channel.frequency = it->second["frequency"].as<int>(50);
                channel.invert = it->second["invert"].as<bool>(false);
                channel.path = "/sys/class/pwm/pwmchip" + std::to_string(channel.chip) +
                               "/pwm" + std::to_string(channel.channel);
                if (channel.frequency <= 0 || channel.frequency > 400) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Channel %s: frequency %d Hz is not a servo rate (use 50)",
                        key.c_str(), channel.frequency);
                    return false;
                }
                pwm_channels_[key] = channel;
            }

            // Load command parameters
            auto cmd = config["drive"]["command"];
            max_speed_ = cmd["max_speed"].as<float>(1.0f);
            acceleration_ = cmd["acceleration"].as<float>(0.1f);
            deadzone_ = cmd["deadzone"].as<float>(0.05f);
            front_middle_gain_ = cmd["front_middle_gain"].as<float>(1.0f);
            back_gain_ = cmd["back_gain"].as<float>(1.0f);
            cmd_timeout_ = cmd["timeout"].as<float>(1.0f);
            arm_time_ = cmd["arm_time"].as<float>(1.0f);

            // Servo pulse widths for the motor controllers
            auto pulse = config["drive"]["pulse_us"];
            pulse_min_us_ = pulse["min"].as<int>(1050);
            pulse_neutral_us_ = pulse["neutral"].as<int>(1500);
            pulse_max_us_ = pulse["max"].as<int>(1950);
            if (!(pulse_min_us_ < pulse_neutral_us_ && pulse_neutral_us_ < pulse_max_us_)) {
                RCLCPP_ERROR(this->get_logger(),
                    "pulse_us must satisfy min < neutral < max (got %d/%d/%d)",
                    pulse_min_us_, pulse_neutral_us_, pulse_max_us_);
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Config loading error: %s", e.what());
            return false;
        }
    }

    static bool write_sysfs(const std::string &path, long value)
    {
        std::ofstream f(path);
        if (!f.is_open()) {
            return false;
        }
        f << value;
        f.close();
        return !f.fail();
    }

    static bool path_exists(const std::string &path)
    {
        struct stat st;
        return stat(path.c_str(), &st) == 0;
    }

    static long period_ns(const PWMChannel &channel)
    {
        return 1000000000L / channel.frequency;
    }

    bool init_pwm()
    {
        try {
            for (auto &[key, channel] : pwm_channels_) {
                // Export PWM channel if not already exported
                if (!path_exists(channel.path)) {
                    std::string export_path = "/sys/class/pwm/pwmchip" + std::to_string(channel.chip) + "/export";
                    if (!write_sysfs(export_path, channel.channel)) {
                        RCLCPP_WARN(this->get_logger(),
                            "Could not export PWM channel %s. Path may not exist: %s",
                            key.c_str(), export_path.c_str());
                    }
                    // The kernel creates pwmN/ asynchronously; give it a moment
                    for (int i = 0; i < 20 && !path_exists(channel.path); ++i) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }

                // Order matters: the period must be set before a duty cycle that fits
                // inside it, and the output should already be neutral when enabled.
                if (!write_sysfs(channel.path + "/period", period_ns(channel))) {
                    RCLCPP_WARN(this->get_logger(),
                        "Could not set PWM period for %s. Check if PWM chip exists: %s",
                        key.c_str(), (channel.path + "/period").c_str());
                }

                set_pulse_us(key, pulse_neutral_us_);

                if (!write_sysfs(channel.path + "/enable", 1)) {
                    RCLCPP_WARN(this->get_logger(),
                        "Could not enable PWM for %s: %s",
                        key.c_str(), (channel.path + "/enable").c_str());
                }

                RCLCPP_INFO(this->get_logger(), "Initialized PWM channel: %s (chip %d, ch %d, %d Hz%s)",
                    key.c_str(), channel.chip, channel.channel, channel.frequency,
                    channel.invert ? ", inverted" : "");
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
                // Disable PWM (no pulses -> the motor controller stops)
                write_sysfs(channel.path + "/enable", 0);

                // Unexport PWM channel
                write_sysfs("/sys/class/pwm/pwmchip" + std::to_string(channel.chip) + "/unexport",
                            channel.channel);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Cleanup error for %s: %s", key.c_str(), e.what());
            }
        }
    }

    // Write one servo pulse width (microseconds) to a channel.
    void set_pulse_us(const std::string &channel_name, int pulse_us)
    {
        auto it = pwm_channels_.find(channel_name);
        if (it == pwm_channels_.end()) {
            RCLCPP_WARN(this->get_logger(), "PWM channel not found: %s", channel_name.c_str());
            return;
        }
        const PWMChannel &channel = it->second;

        pulse_us = std::max(pulse_min_us_, std::min(pulse_max_us_, pulse_us));
        long duty_ns = std::min(static_cast<long>(pulse_us) * 1000L, period_ns(channel));

        if (!write_sysfs(channel.path + "/duty_cycle", duty_ns)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Could not write duty_cycle for %s", channel_name.c_str());
        }
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract linear (forward/backward) velocity
        float linear_velocity = msg->linear.x;   // Forward/backward
        float angular_velocity = msg->angular.z; // Rotation (+ = turn left, ROS convention)

        // Reject NaN/inf so a bad message can never cause motion
        if (!std::isfinite(linear_velocity)) linear_velocity = 0.0f;
        if (!std::isfinite(angular_velocity)) angular_velocity = 0.0f;

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

        // Overall speed cap from config
        left_base_speed *= max_speed_;
        right_base_speed *= max_speed_;

        // Apply gains for front/middle vs back motors (can have different speeds)
        target_command_.left_front_middle = left_base_speed * front_middle_gain_;
        target_command_.left_back = left_base_speed * back_gain_;
        target_command_.right_front_middle = right_base_speed * front_middle_gain_;
        target_command_.right_back = right_base_speed * back_gain_;

        last_command_time_ = this->now();
    }

    static void ramp(float &current, float target, float step)
    {
        float delta = target - current;
        if (std::abs(delta) > step) {
            current += (delta > 0 ? step : -step);
        } else {
            current = target;
        }
    }

    void control_loop()
    {
        const rclcpp::Time now = this->now();

        // Arming: hold neutral for arm_time after start so the controllers arm
        if ((now - start_time_).seconds() < arm_time_) {
            target_command_ = {0.0f, 0.0f, 0.0f, 0.0f};
            current_command_ = {0.0f, 0.0f, 0.0f, 0.0f};
        } else if (!armed_logged_) {
            RCLCPP_INFO(this->get_logger(), "Motor controllers armed; accepting commands");
            armed_logged_ = true;
        }

        // Check for command timeout
        if ((now - last_command_time_).seconds() > cmd_timeout_) {
            target_command_ = {0.0f, 0.0f, 0.0f, 0.0f};
        }

        // Apply acceleration limits to all four channels
        ramp(current_command_.left_front_middle, target_command_.left_front_middle, acceleration_);
        ramp(current_command_.left_back, target_command_.left_back, acceleration_);
        ramp(current_command_.right_front_middle, target_command_.right_front_middle, acceleration_);
        ramp(current_command_.right_back, target_command_.right_back, acceleration_);

        // Apply deadzone (to the output only, so ramping is not affected)
        auto dz = [this](float v) { return std::abs(v) < deadzone_ ? 0.0f : v; };

        // Set all PWM channels
        set_pwm_speed("motor_left_front_middle", dz(current_command_.left_front_middle));
        set_pwm_speed("motor_left_back", dz(current_command_.left_back));
        set_pwm_speed("motor_right_front_middle", dz(current_command_.right_front_middle));
        set_pwm_speed("motor_right_back", dz(current_command_.right_back));
    }

    // Signed speed (-1 reverse .. 0 stop .. +1 forward) -> servo pulse.
    void set_pwm_speed(const std::string &channel_name, float speed)
    {
        auto it = pwm_channels_.find(channel_name);
        if (it == pwm_channels_.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "PWM channel not found: %s", channel_name.c_str());
            return;
        }

        // Clamp speed to -1.0 to 1.0
        speed = std::max(-1.0f, std::min(1.0f, speed));
        if (it->second.invert) {
            speed = -speed;   // motor mounted/wired the other way round
        }

        // Forward and reverse spans may differ if the pulse range is asymmetric
        const float span = speed >= 0.0f
            ? static_cast<float>(pulse_max_us_ - pulse_neutral_us_)
            : static_cast<float>(pulse_neutral_us_ - pulse_min_us_);
        const int pulse_us = pulse_neutral_us_ + static_cast<int>(std::lround(speed * span));

        set_pulse_us(channel_name, pulse_us);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveController>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
