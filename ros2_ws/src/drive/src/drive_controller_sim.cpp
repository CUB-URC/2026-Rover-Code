#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>

/**
 * @file drive_controller_sim.cpp
 * @brief Simulated drive controller for testing and development
 * 
 * This is a simulation version that doesn't access actual PWM hardware.
 * Instead, it logs all motor commands and simulates motor responses.
 * 
 * Use this to verify:
 * - Parameter loading from YAML
 * - ROS2 topic communication
 * - Control logic without hardware
 * - Parameter tuning before deployment
 */

/**
 * Simulated Motor for testing
 */
struct SimulatedMotor {
    std::string name;
    float current_speed_ = 0.0f;
    float target_speed_ = 0.0f;
    float acceleration_;
    
    SimulatedMotor(const std::string &name, float accel) 
        : name(name), acceleration_(accel) {}
    
    void update(float delta_time_s) {
        float delta = target_speed_ - current_speed_;
        float max_change = acceleration_ * delta_time_s;
        
        if (std::abs(delta) > max_change) {
            current_speed_ += (delta > 0.0f ? max_change : -max_change);
        } else {
            current_speed_ = target_speed_;
        }
    }
};

class DriveControllerSim : public rclcpp::Node
{
public:
    DriveControllerSim() : Node("drive_controller_sim")
    {
        // Load configuration
        if (!load_config()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load drive configuration");
            rclcpp::shutdown();
            return;
        }

        // Initialize simulated motors
        motors_["motor_left_front_middle"] = 
            std::make_shared<SimulatedMotor>("Left Front/Middle", acceleration_);
        motors_["motor_left_back"] = 
            std::make_shared<SimulatedMotor>("Left Back", acceleration_);
        motors_["motor_right_front_middle"] = 
            std::make_shared<SimulatedMotor>("Right Front/Middle", acceleration_);
        motors_["motor_right_back"] = 
            std::make_shared<SimulatedMotor>("Right Back", acceleration_);

        // Create command subscriber
        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/drive/cmd_vel",
            10,
            std::bind(&DriveControllerSim::cmd_callback, this, std::placeholders::_1));

        // Create motor status publisher
        status_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/drive/sim_status",
            10);

        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&DriveControllerSim::control_loop, this));

        RCLCPP_INFO(this->get_logger(), 
            "Simulated Drive Controller initialized (SIM MODE)");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /drive/cmd_vel");
    }

    ~DriveControllerSim() {}

private:
    struct MotorCommand {
        float left_front_middle;
        float left_back;
        float right_front_middle;
        float right_back;
    };

    // Configuration
    float max_speed_;
    float acceleration_;
    float deadzone_;
    float front_middle_gain_;
    float back_gain_;
    
    MotorCommand current_command_{0.0f, 0.0f, 0.0f, 0.0f};
    MotorCommand target_command_{0.0f, 0.0f, 0.0f, 0.0f};
    rclcpp::Time last_command_time_;
    
    // Simulated motors
    std::map<std::string, std::shared_ptr<SimulatedMotor>> motors_;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config()
    {
        try {
            std::string package_share_directory =
                ament_index_cpp::get_package_share_directory("drive");
            std::string config_file = package_share_directory + "/config/drive_config.yaml";

            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

            YAML::Node config = YAML::LoadFile(config_file);

            // Load command parameters
            max_speed_ = config["drive"]["command"]["max_speed"].as<float>(1.0f);
            acceleration_ = config["drive"]["command"]["acceleration"].as<float>(0.1f);
            deadzone_ = config["drive"]["command"]["deadzone"].as<float>(0.05f);
            front_middle_gain_ = config["drive"]["command"]["front_middle_gain"].as<float>(1.0f);
            back_gain_ = config["drive"]["command"]["back_gain"].as<float>(1.0f);

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
            RCLCPP_INFO(this->get_logger(), "Max Speed: %.2f, Acceleration: %.2f, Deadzone: %.2f",
                max_speed_, acceleration_, deadzone_);
            RCLCPP_INFO(this->get_logger(), 
                "Front/Middle Gain: %.2f, Back Gain: %.2f",
                front_middle_gain_, back_gain_);

            last_command_time_ = this->now();
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Config loading error: %s", e.what());
            return false;
        }
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float linear_velocity = msg->linear.x;
        float angular_velocity = msg->angular.z;

        float left_base_speed = linear_velocity - angular_velocity;
        float right_base_speed = linear_velocity + angular_velocity;

        // Normalize to -1.0 to 1.0 range
        float max_val = std::max(std::abs(left_base_speed), std::abs(right_base_speed));
        if (max_val > 1.0f) {
            left_base_speed /= max_val;
            right_base_speed /= max_val;
        }

        // Apply gains
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

        // Apply acceleration limits
        apply_acceleration_limits();
        apply_deadzone();

        // Update simulated motors
        update_simulated_motors();

        // Publish status
        publish_status();
    }

    void apply_acceleration_limits()
    {
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
    }

    void apply_deadzone()
    {
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
    }

    void update_simulated_motors()
    {
        // Set target speeds for simulated motors
        motors_["motor_left_front_middle"]->target_speed_ = current_command_.left_front_middle;
        motors_["motor_left_back"]->target_speed_ = current_command_.left_back;
        motors_["motor_right_front_middle"]->target_speed_ = current_command_.right_front_middle;
        motors_["motor_right_back"]->target_speed_ = current_command_.right_back;

        // Update all motors (0.02s per control loop)
        for (auto &[name, motor] : motors_) {
            motor->update(0.02f);
        }
    }

    void publish_status()
    {
        auto status_msg = std_msgs::msg::String();
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "Motor States: ";
        ss << "LFM=" << motors_["motor_left_front_middle"]->current_speed_ << " ";
        ss << "LB=" << motors_["motor_left_back"]->current_speed_ << " ";
        ss << "RFM=" << motors_["motor_right_front_middle"]->current_speed_ << " ";
        ss << "RB=" << motors_["motor_right_back"]->current_speed_;
        
        status_msg.data = ss.str();
        status_publisher_->publish(status_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControllerSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
