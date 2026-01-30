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
#include <string>
#include <vector>

/**
 * @file drive_controller.cpp
 * @brief Drive controller node for CUB-URC rover
 * 
 * Hardware Compatibility:
 * - Jetson Orin Nano Super Dev Kit (primary)
 * - Jetson Nano Dev Kit (secondary)
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

        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&DriveController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Drive controller initialized. Subscribing to /drive/cmd_vel");
    }

    ~DriveController()
    {
        // Stop all motors
        set_pwm("motor_left_front_middle", 0.0);
        set_pwm("motor_left_back", 0.0);
        set_pwm("motor_right_front_middle", 0.0);
        set_pwm("motor_right_back", 0.0);
        
        // Cleanup PWM
        cleanup_pwm();
    }

private:
    struct PWMChannel {
        std::string name;
        int chip;
        int channel;
        int frequency;
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

    void set_pwm(const std::string &channel_name, float duty_cycle)
    {
        try {
            if (pwm_channels_.find(channel_name) == pwm_channels_.end()) {
                RCLCPP_WARN(this->get_logger(), "PWM channel not found: %s", channel_name.c_str());
                return;
            }

            const PWMChannel &channel = pwm_channels_[channel_name];
            
            // Clamp duty cycle between 0 and 1
            duty_cycle = std::max(0.0f, std::min(1.0f, duty_cycle));

            // Calculate pulse width in nanoseconds
            long period_ns = 1000000000 / channel.frequency;
            long duty_ns = static_cast<long>(period_ns * duty_cycle);

            // Write duty cycle
            std::string duty_path = channel.path + "/duty_cycle";
            std::ofstream duty_file(duty_path);
            if (duty_file.is_open()) {
                duty_file << duty_ns;
                duty_file.close();
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "PWM set error for %s: %s", channel_name.c_str(), e.what());
        }
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
        // Clamp speed to -1.0 to 1.0
        speed = std::max(-1.0f, std::min(1.0f, speed));

        // Convert signed speed to PWM duty cycle (always positive)
        // For bipolar motors, use direction control if available
        // For now, just use absolute value and let motor controller handle direction
        float duty_cycle = std::abs(speed);
        
        set_pwm(channel_name, duty_cycle);

        // TODO: Add direction control logic if using separate direction pins
        // if (speed < 0.0f) { /* set direction pin to reverse */ }
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
