#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

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
    double current_speed = 0.0;
    double target_speed = 0.0;

    void update(double dt_s) {
        double delta = target_speed - current_speed;
        double max_change = max_accel * dt_s;

        if (std::abs(delta) > max_change) {
            current_speed += (delta > 0.0 ? max_change : -max_change);
        } else {
            current_speed = target_speed;
        }
    }
};

class ArmControllerSim : public rclcpp::Node
{
public:
    ArmControllerSim() : Node("arm_controller_sim")
    {
        if (!load_config()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load arm configuration");
            rclcpp::shutdown();
            return;
        }

        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/arm/cmd_vel",
            10,
            std::bind(&ArmControllerSim::cmd_callback, this, std::placeholders::_1));

        status_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/arm/sim_status",
            10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ArmControllerSim::control_loop, this));

        last_command_time_ = this->now();
        last_update_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Simulated Arm Controller initialized (SIM MODE)");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /arm/cmd_vel");
    }

private:
    std::map<std::string, SimulatedJoint> joints_;
    std::vector<std::string> joint_order_;

    double input_timeout_ = 0.5;
    double deadzone_ = 0.05;
    double scale_speed_ = 1.0;

    geometry_msgs::msg::Twist last_command_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_update_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config()
    {
        try {
            std::string package_share_directory =
                ament_index_cpp::get_package_share_directory("arm");
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

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Config loading error: %s", e.what());
            return false;
        }
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
        update_joints(dt);
        publish_status();
    }

    void apply_command_to_targets()
    {
        // TODO: Confirm joint-to-axis mapping. Current mapping uses angular axes.
        double base_cmd = last_command_.angular.z;
        double shoulder_cmd = last_command_.angular.y;
        double elbow_cmd = last_command_.angular.x;

        set_target_with_limits("base", base_cmd);
        set_target_with_limits("shoulder", shoulder_cmd);
        set_target_with_limits("elbow", elbow_cmd);
    }

    void set_target_with_limits(const std::string &joint_key, double target)
    {
        if (joints_.find(joint_key) == joints_.end()) {
            return;
        }

        if (std::abs(target) < deadzone_) {
            target = 0.0;
        }

        SimulatedJoint &joint = joints_[joint_key];
        target = std::max(-joint.max_speed, std::min(joint.max_speed, target));
        joint.target_speed = target;
    }

    void update_joints(double dt)
    {
        for (auto &entry : joints_) {
            entry.second.update(dt);
        }
    }

    void publish_status()
    {
        auto status_msg = std_msgs::msg::String();
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "Joint Speeds: ";

        bool first = true;
        for (const auto &key : joint_order_) {
            auto it = joints_.find(key);
            if (it == joints_.end()) {
                continue;
            }
            if (!first) {
                ss << " ";
            }
            ss << it->second.name << "=" << it->second.current_speed;
            first = false;
        }

        status_msg.data = ss.str();
        status_publisher_->publish(status_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControllerSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
