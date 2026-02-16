#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

/**
 * @file arm_controller_node.cpp
 * @brief Drive controller node for CUB-URC rover
 * 
 * Hardware Compatibility:
 * - Jetson Orin Nano Super Dev Kit (primary)
 * - Jetson Nano Dev Kit (secondary)
 * - Drivers: DM542T, DM320T
 */


class ArmController : public rclcpp::Node
{
public:
    
    ArmController() : Node("arm_controller")
        {
            // Load configuration
            if (!load_config()) {
                RCLCPP_ERROR(this->get_logger(), "Configuration load failed; shutting down arm controller.");
                rclcpp::shutdown();
                return;
            }

            // Create command subscriber
            // Topic: /arm/cmd_vel - Standardized command velocity topic (geometry_msgs/Twist)
            cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                command_input_topic_,
                10,
                std::bind(&ArmController::cmd_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to command topic: %s", command_input_topic_.c_str());

            joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
                joint_states_output_topic_,
                10);
            RCLCPP_INFO(this->get_logger(), "Publishing joint states to topic: %s",
                joint_states_output_topic_.c_str());

            // Create timer for control loop
            control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100 Hz
            std::bind(&ArmController::control_loop, this));

            last_command_time_ = this->now();
            last_update_time_ = this->now();
        }

    ~ArmController() {
        cleanup_gpio();
    }

private:

    struct JointConfig {
    
        std::string name;
        int step_pin;
        int dir_pin;
        double gear_ratio;
        int microsteps;
        double amps;           
        bool reversed;
        
        // motion state
        double max_speed;      // steps/sec
        double max_accel;      // steps/sec^2
        double current_speed;  // steps/sec
        double target_speed;   // steps/sec
        double current_position; // radians
        
        // hardware state
        bool enabled;
    };

    // GPIO + PWM sysfs helpers (mirrors drive module style)
    struct GPIOPin {
        std::string name;
        int pin;
        std::string path; // /sys/class/gpio/gpioN
    };

    struct StepPwm {
        std::string name;
        int chip;
        int channel;
        int frequency;     // Hz
        std::string path;  // /sys/class/pwm/pwmchipX/pwmY
    };

    // Configuration
    std::map<std::string, JointConfig> joints_;
    std::vector<std::string> joint_order_;
    std::map<std::string, GPIOPin> dir_pins_;
    std::map<std::string, GPIOPin> enable_pins_;
    std::map<std::string, StepPwm> step_pwms_;

    std::map<int, int> gpio_value_fds_;  // pin -> fd
    std::map<int, int> last_dir_level_;  // dir pin -> last level
    std::vector<int> exported_pins_;     // pins this module exported (for teardown)

    double input_timeout_;
    double deadzone_;
    double scale_speed_;
    std::string command_input_topic_;
    std::string joint_states_output_topic_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_update_time_;
    geometry_msgs::msg::Twist last_command_;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config()
    {
        try {
            // Get package share directory
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("arm");
            std::string config_file = package_share_directory + "/config/arm_config.yaml";
            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

            YAML::Node config = YAML::LoadFile(config_file);

            // Load joints
            YAML::Node hardware = config["arm"]["hardware"];
            if (!hardware || !hardware.IsMap()) {
                throw std::runtime_error("Missing arm.hardware section in config");
            }
            YAML::Node limits = config["arm"]["limits"];
            if (!limits || !limits.IsMap()) {
                throw std::runtime_error("Missing arm.limits section in config");
            }

            for (auto it = hardware.begin(); it != hardware.end(); ++it) {
                const std::string joint_key = it->first.as<std::string>();
                const YAML::Node joint = it->second;
                const YAML::Node joint_limits = limits[joint_key];
                if (!joint_limits || !joint_limits.IsMap()) {
                    throw std::runtime_error("Missing limits for joint: " + joint_key);
                }

                JointConfig jc;
                jc.name = joint["name"].as<std::string>();
                jc.step_pin = joint["step_pin"].as<int>();
                jc.dir_pin = joint["dir_pin"].as<int>();
                jc.gear_ratio = joint["gear_ratio"].as<double>();
                jc.microsteps = joint["microsteps"].as<int>();
                jc.amps = joint["amps"].as<double>();
                jc.reversed = joint["reversed"].as<bool>();

                jc.max_speed = joint_limits["max_speed"].as<double>();
                jc.max_accel = joint_limits["max_accel"].as<double>();
                jc.current_speed = 0.0;
                jc.target_speed = 0.0;
                jc.current_position = 0.0;
                jc.enabled = false;

                joints_[joint_key] = jc;
                joint_order_.push_back(joint_key);
            }

            // Load command parameters
            YAML::Node command = config["arm"]["command"];
            if (!command || !command.IsMap()) {
                throw std::runtime_error("Missing arm.command section in config");
            }
            input_timeout_ = command["input_timeout"].as<double>();
            deadzone_ = command["deadzone"].as<double>();
            scale_speed_ = command["scale_speed"].as<double>(1.0);

            YAML::Node ros2 = config["arm"]["ros2"];
            if (!ros2 || !ros2.IsMap()) {
                throw std::runtime_error("Missing arm.ros2 section in config");
            }
            YAML::Node topics = ros2["topics"];
            if (!topics || !topics.IsMap()) {
                throw std::runtime_error("Missing arm.ros2.topics section in config");
            }
            // Load the command input topic used by this node's /Twist subscriber.
            if (!topics["command_input"] || !topics["command_input"].IsScalar()) {
                throw std::runtime_error("Missing arm.ros2.topics.command_input in config");
            }
            YAML::Node joint_states_output = topics["joint_states_output"];
            if (!joint_states_output || !joint_states_output.IsScalar()) {
                // Backward compatibility for older config key.
                joint_states_output = topics["status_output"];
            }
            if (!joint_states_output || !joint_states_output.IsScalar()) {
                throw std::runtime_error(
                    "Missing arm.ros2.topics.joint_states_output in config");
            }

            command_input_topic_ = topics["command_input"].as<std::string>();
            joint_states_output_topic_ = joint_states_output.as<std::string>();
            if (command_input_topic_.empty()) {
                throw std::runtime_error("arm.ros2.topics.command_input must not be empty");
            }
            if (joint_states_output_topic_.empty()) {
                throw std::runtime_error(
                    "arm.ros2.topics.joint_states_output must not be empty");
            }

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Config loading error: %s", e.what());
            return false;
        }
    }

    bool init_gpio(){
        // TODO
        return false;
    }



    static inline void write_gpio_fd(int fd, int v) {
        const char c = v ? '1' : '0';
        lseek(fd, 0, SEEK_SET);
        (void)::write(fd, &c, 1);
    }

    static bool write_text(const std::string &path, const std::string &value)
    {
        std::ofstream out(path);
        if (!out.is_open()) {
            return false;
        }
        out << value;
        return true;
    }

    enum class Direction { Forward, Reverse };

    void set_gpio(const JointConfig& joint, float hz, Direction dir, double dt_s) {
        if (joint.dir_pin < 0 || joint.step_pin < 0) return;
        if (hz < 0.0f) hz = 0.0f;
        if (dt_s <= 0.0) dt_s = 0.01;

        // Get dir_level for writing to pin
        Direction effective_dir = dir;
        if (joint.reversed) {
            effective_dir = (dir == Direction::Forward) ? Direction::Reverse : Direction::Forward;
        }
        const int dir_level = (effective_dir == Direction::Forward) ? 1 : 0;


        int step_fd = gpio_value_fds_.at(joint.step_pin);
        int dir_fd  = gpio_value_fds_.at(joint.dir_pin);
        
        // Set direction; always write on first use of this pin.
        const auto it = last_dir_level_.find(joint.dir_pin);
        const bool dir_changed = (it == last_dir_level_.end()) || (it->second != dir_level);
        if (dir_changed) {
            write_gpio_fd(dir_fd, dir_level);
            last_dir_level_[joint.dir_pin] = dir_level;
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }

        // Treat very low rates as zero to avoid creep.
        static std::map<int, double> step_fraction;  // step pin -> fractional carry
        if (hz < 1.0f) {
            write_gpio_fd(step_fd, 0);
            step_fraction[joint.step_pin] = 0.0;
            return;
        }

        // Generate pulses for one control slice based on real loop dt.
        const double kControlSliceS = std::clamp(dt_s, 0.001, 0.05);
        const int kControlSliceUs = std::max(
            1, static_cast<int>(std::llround(kControlSliceS * 1e6)));

        // DM542T requires >= 2.5us, DM320T requires >= 7.5us, so we use 8 here.
        constexpr int kMinPulseUs = 8;

        const double pulses_exact = static_cast<double>(hz) * kControlSliceS +
                                    step_fraction[joint.step_pin];
        int pulses = static_cast<int>(std::floor(pulses_exact));
        step_fraction[joint.step_pin] = pulses_exact - static_cast<double>(pulses);

        if (pulses <= 0) {
            write_gpio_fd(step_fd, 0);
            return;
        }

        const int period_us = std::max(2 * kMinPulseUs, kControlSliceUs / pulses);
        const int high_us = std::max(kMinPulseUs, period_us / 2); // 50% duty cycle
        const int low_us = std::max(kMinPulseUs, period_us - high_us);

        for (int i = 0; i < pulses; ++i) {
            write_gpio_fd(step_fd, 1);
            std::this_thread::sleep_for(std::chrono::microseconds(high_us));
            write_gpio_fd(step_fd, 0);
            std::this_thread::sleep_for(std::chrono::microseconds(low_us));
        }
    }

    void cleanup_gpio() {
        // first, stop pulses
        for (const auto& [k, j] : joints_) {
            const auto it = gpio_value_fds_.find(j.step_pin);
            if (it != gpio_value_fds_.end() && it->second >= 0) {
                write_gpio_fd(it->second, 0);
            }
        }

        // Close open fds
        for (auto& [pin, fd] : gpio_value_fds_) {
            if (fd >= 0) ::close(fd);
        }
        gpio_value_fds_.clear();
        last_dir_level_.clear();

        // Unexport only what we exported
        for (int pin : exported_pins_) {
            write_text("/sys/class/gpio/unexport", std::to_string(pin));
        }
        exported_pins_.clear();
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_command_time_ = this->now();
        last_command_ = *msg;
        last_command_.linear.x *= scale_speed_;
        last_command_.linear.y *= scale_speed_;
        last_command_.linear.z *= scale_speed_;
        last_command_.angular.x *= scale_speed_;
        last_command_.angular.y *= scale_speed_;
        last_command_.angular.z *= scale_speed_;
    }

    void control_loop()
    {
        const auto now = this->now();
        double dt = (now - last_update_time_).seconds();
        if (dt <= 0.0) {
            dt = 0.01;
        }
        last_update_time_ = now;

        if ((now - last_command_time_).seconds() > input_timeout_) {
            last_command_ = geometry_msgs::msg::Twist();
        }

        apply_command_to_targets();
        update_joint_states(dt);
        publish_joint_states(now);
    }

    void set_motor_speed(JointConfig &joint, double speed, double dt)
    {
        const Direction dir = (speed >= 0.0) ? Direction::Forward : Direction::Reverse;
        set_gpio(joint, static_cast<float>(std::abs(speed)), dir, dt);
    }

    static double steps_to_radians(const JointConfig &joint, double steps)
    {
        constexpr double kTwoPi = 6.28318530717958647692;
        const double steps_per_output_rev =
            static_cast<double>(joint.microsteps) * joint.gear_ratio;
        if (steps_per_output_rev <= 0.0) {
            return 0.0;
        }
        return steps * kTwoPi / steps_per_output_rev;
    }

    void set_target_with_limits(const std::string &joint_key, double target)
    {
        auto it = joints_.find(joint_key);
        if (it == joints_.end()) {
            return;
        }

        if (std::abs(target) < deadzone_) {
            target = 0.0;
        }

        JointConfig &joint = it->second;
        target = std::max(-joint.max_speed, std::min(joint.max_speed, target));
        joint.target_speed = target;
    }

    void apply_command_to_targets()
    {
        // Current axis mapping:
        // angular.z -> base, angular.y -> shoulder, angular.x -> elbow.
        set_target_with_limits("base", last_command_.angular.z);
        set_target_with_limits("shoulder", last_command_.angular.y);
        set_target_with_limits("elbow", last_command_.angular.x);
    }

    void update_joint_states(double dt)
    {
        for (auto &entry : joints_) {
            JointConfig &joint = entry.second;

            const double delta = joint.target_speed - joint.current_speed;
            const double max_change = joint.max_accel * dt;
            if (std::abs(delta) > max_change) {
                joint.current_speed += (delta > 0.0 ? max_change : -max_change);
            } else {
                joint.current_speed = joint.target_speed;
            }

            joint.current_position += steps_to_radians(joint, joint.current_speed * dt);
            set_motor_speed(joint, joint.current_speed, dt);
        }
    }

    void publish_joint_states(const rclcpp::Time &stamp)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name.reserve(joint_order_.size());
        msg.position.reserve(joint_order_.size());
        msg.velocity.reserve(joint_order_.size());

        for (const auto &key : joint_order_) {
            auto it = joints_.find(key);
            if (it == joints_.end()) {
                continue;
            }

            const JointConfig &joint = it->second;
            msg.name.push_back(key);
            msg.position.push_back(joint.current_position);
            msg.velocity.push_back(steps_to_radians(joint, joint.current_speed));
        }

        joint_states_publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());
    rclcpp::shutdown();
    return 0;
}
