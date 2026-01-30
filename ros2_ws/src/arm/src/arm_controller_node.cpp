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
                RCLCPP_ERROR(this->get_logger(), "Failed to configure joints.");
                rclcpp::shutdown();
                return;
            }
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
        
        // hardware state
        bool enabled;
    };

    // Configuration
    std::map<std::string, JointConfig> joints_;

    double input_timeout_;
    double deadzone_;
    rclcpp::Time last_command_time_;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config()
    {

    }

    bool init_gpio(){
        
    }

    void cleanup_gpio(){
        
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

    }

    void control_loop()
    {

    }

    void set_motor_speed(JointConfig &joint, double speed)
    {
        // determine direction
        bool forward = (speed > 0);
        if (joint.reversed) forward = !forward;

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());
    rclcpp::shutdown();
    return 0;
}