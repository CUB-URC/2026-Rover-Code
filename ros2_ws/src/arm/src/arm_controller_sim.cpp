#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @file arm_controller_sim.cpp
 * @brief Simulated arm controller for testing and development
 *
 * This is a simulation version that doesn't access GPIO or PWM hardware.
 * It loads the same YAML configuration and simulates joint speed behavior.
 */

struct SimulatedJoint {
    std::string name;
    double max_speed;
    double max_accel;
    double gear_ratio;
    int microsteps;
    double current_speed = 0.0;
    double target_speed = 0.0;
    double current_position = 0.0;

    void update(double dt_s) {
        double delta = target_speed - current_speed;
        double max_change = max_accel * dt_s;

        if (std::abs(delta) > max_change) {
            current_speed += (delta > 0.0 ? max_change : -max_change);
        } else {
            current_speed = target_speed;
        }

        current_position += steps_to_radians(current_speed * dt_s);
    }

    double steps_to_radians(double steps) const {
        constexpr double kTwoPi = 6.28318530717958647692;
        const double steps_per_output_rev = static_cast<double>(microsteps) * gear_ratio;
        if (steps_per_output_rev <= 0.0) {
            return 0.0;
        }
        return steps * kTwoPi / steps_per_output_rev;
    }
};

class ArmControllerSim : public rclcpp::Node {
  public:
    ArmControllerSim() : Node("arm_controller_sim") {
        if (!load_config()) {
            RCLCPP_ERROR(this->get_logger(), "Configuration load failed; shutting down arm simulator.");
            rclcpp::shutdown();
            return;
        }

        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(command_input_topic_, 10, std::bind(&ArmControllerSim::cmd_callback, this, std::placeholders::_1));

        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_output_topic_, 10);

        const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(control_period_s_));
        control_timer_ = this->create_wall_timer(control_period, std::bind(&ArmControllerSim::control_loop, this));

        last_command_time_ = this->now();
        last_update_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Simulated Arm Controller initialized (SIM MODE)");
        RCLCPP_INFO(this->get_logger(), "Subscribing to %s", command_input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing joint states to %s", joint_states_output_topic_.c_str());
    }

  private:
    std::map<std::string, SimulatedJoint> joints_;
    std::vector<std::string> joint_order_;

    double input_timeout_ = 0.5;
    double deadzone_ = 0.05;
    double scale_speed_ = 1.0;
    double control_period_s_ = 0.01;
    std::string command_input_topic_;
    std::string joint_states_output_topic_;

    geometry_msgs::msg::Twist last_command_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_update_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config() {
        try {
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("arm");
            std::string config_file = package_share_directory + "/config/arm_config.yaml";

            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

            YAML::Node config = YAML::LoadFile(config_file);

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

                SimulatedJoint sim_joint;
                sim_joint.name = joint["name"].as<std::string>();
                sim_joint.max_speed = joint_limits["max_speed"].as<double>();
                sim_joint.max_accel = joint_limits["max_accel"].as<double>();
                sim_joint.gear_ratio = joint["gear_ratio"].as<double>();
                sim_joint.microsteps = joint["microsteps"].as<int>();
                validate_joint_config(joint_key, sim_joint);

                joints_[joint_key] = sim_joint;
                joint_order_.push_back(joint_key);
            }

            YAML::Node command = config["arm"]["command"];
            if (!command || !command.IsMap()) {
                throw std::runtime_error("Missing arm.command section in config");
            }
            input_timeout_ = command["input_timeout"].as<double>();
            deadzone_ = command["deadzone"].as<double>();
            scale_speed_ = command["scale_speed"].as<double>(1.0);
            if (!std::isfinite(input_timeout_) || input_timeout_ < 0.0) {
                throw std::runtime_error("arm.command.input_timeout must be finite and >= 0");
            }
            if (!std::isfinite(deadzone_) || deadzone_ < 0.0) {
                throw std::runtime_error("arm.command.deadzone must be finite and >= 0");
            }
            if (!std::isfinite(scale_speed_) || scale_speed_ < 0.0) {
                throw std::runtime_error("arm.command.scale_speed must be finite and >= 0");
            }

            YAML::Node ros2 = config["arm"]["ros2"];
            if (!ros2 || !ros2.IsMap()) {
                throw std::runtime_error("Missing arm.ros2 section in config");
            }
            const double loop_rate_hz = ros2["loop_rate"].as<double>(100.0);
            if (!std::isfinite(loop_rate_hz) || loop_rate_hz <= 0.0) {
                throw std::runtime_error("arm.ros2.loop_rate must be finite and > 0");
            }
            control_period_s_ = 1.0 / loop_rate_hz;
            YAML::Node topics = ros2["topics"];
            if (!topics || !topics.IsMap()) {
                throw std::runtime_error("Missing arm.ros2.topics section in config");
            }
            // Load command input and joint states output topics from YAML.
            if (!topics["command_input"] || !topics["command_input"].IsScalar()) {
                throw std::runtime_error("Missing arm.ros2.topics.command_input in config");
            }
            YAML::Node joint_states_output = topics["joint_states_output"];
            if (!joint_states_output || !joint_states_output.IsScalar()) {
                // Backward compatibility for older config key.
                joint_states_output = topics["status_output"];
            }
            if (!joint_states_output || !joint_states_output.IsScalar()) {
                throw std::runtime_error("Missing arm.ros2.topics.joint_states_output in config");
            }

            command_input_topic_ = topics["command_input"].as<std::string>();
            joint_states_output_topic_ = joint_states_output.as<std::string>();
            if (command_input_topic_.empty()) {
                throw std::runtime_error("arm.ros2.topics.command_input must not be empty");
            }
            if (joint_states_output_topic_.empty()) {
                throw std::runtime_error("arm.ros2.topics.joint_states_output must not be empty");
            }

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Config loading error: %s", e.what());
            return false;
        }
    }

    static void validate_joint_config(const std::string &joint_key, const SimulatedJoint &joint) {
        if (joint.microsteps <= 0) {
            throw std::runtime_error("Joint " + joint_key + " must have microsteps > 0");
        }
        if (!std::isfinite(joint.gear_ratio) || joint.gear_ratio <= 0.0) {
            throw std::runtime_error("Joint " + joint_key + " must have gear_ratio > 0");
        }
        if (!std::isfinite(joint.max_speed) || joint.max_speed < 0.0) {
            throw std::runtime_error("Joint " + joint_key + " must have max_speed >= 0");
        }
        if (!std::isfinite(joint.max_accel) || joint.max_accel < 0.0) {
            throw std::runtime_error("Joint " + joint_key + " must have max_accel >= 0");
        }
    }

    double command_to_target_speed(const SimulatedJoint &joint, double command) const {
        if (!std::isfinite(command)) {
            return 0.0;
        }

        const double scaled_command = command * scale_speed_;
        if (std::abs(scaled_command) < deadzone_) {
            return 0.0;
        }
        return std::clamp(scaled_command, -joint.max_speed, joint.max_speed);
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_command_time_ = this->now();
        last_command_ = *msg;
    }

    void control_loop() {
        const auto now = this->now();
        double dt = (now - last_update_time_).seconds();
        if (dt <= 0.0) {
            dt = control_period_s_;
        }
        last_update_time_ = now;

        if ((now - last_command_time_).seconds() > input_timeout_) {
            last_command_ = geometry_msgs::msg::Twist();
        }

        apply_command_to_targets();
        update_joints(dt);
        publish_joint_states(now);
    }

    void apply_command_to_targets() {
        // TODO: Confirm joint-to-axis mapping. Current mapping uses angular axes.
        double base_cmd = last_command_.angular.z;
        double shoulder_cmd = last_command_.angular.y;
        double elbow_cmd = last_command_.angular.x;

        set_target_with_limits("base", base_cmd);
        set_target_with_limits("shoulder", shoulder_cmd);
        set_target_with_limits("elbow", elbow_cmd);
    }

    void set_target_with_limits(const std::string &joint_key, double target) {
        if (joints_.find(joint_key) == joints_.end()) {
            return;
        }

        SimulatedJoint &joint = joints_[joint_key];
        joint.target_speed = command_to_target_speed(joint, target);
    }

    void update_joints(double dt) {
        for (auto &entry : joints_) {
            entry.second.update(dt);
        }
    }

    void publish_joint_states(const rclcpp::Time &stamp) {
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

            const SimulatedJoint &joint = it->second;
            msg.name.push_back(key);
            msg.position.push_back(joint.current_position);
            msg.velocity.push_back(joint.steps_to_radians(joint.current_speed));
        }

        joint_states_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControllerSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
