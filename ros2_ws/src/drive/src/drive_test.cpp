#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <thread>

/**
 * @file drive_test.cpp
 * @brief Test node for drive controller
 * 
 * Publishes test commands to /drive/cmd_vel to verify drive controller functionality.
 * Run this node to test: ros2 run drive drive_test_node
 */

class DriveTestNode : public rclcpp::Node
{
public:
    DriveTestNode() : Node("drive_test")
    {
        // Create publisher for drive commands
        // Topic: /drive/cmd_vel - Standardized command velocity topic (geometry_msgs/Twist)
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/drive/cmd_vel",
            10);

        RCLCPP_INFO(this->get_logger(), "Drive test node started");
        RCLCPP_INFO(this->get_logger(), "Publishing test commands to /drive/cmd_vel topic");
    }

    void test_forward()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // Move forward at 50% speed
        msg.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Sending FORWARD command");
        cmd_publisher_->publish(msg);
    }

    void test_backward()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = -0.5;  // Move backward at 50% speed
        msg.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Sending BACKWARD command");
        cmd_publisher_->publish(msg);
    }

    void test_left()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.angular.z = 0.5;  // Turn left
        
        RCLCPP_INFO(this->get_logger(), "Sending LEFT TURN command");
        cmd_publisher_->publish(msg);
    }

    void test_right()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.angular.z = -0.5;  // Turn right
        
        RCLCPP_INFO(this->get_logger(), "Sending RIGHT TURN command");
        cmd_publisher_->publish(msg);
    }

    void test_stop()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Sending STOP command");
        cmd_publisher_->publish(msg);
    }

    void test_sequence()
    {
        using namespace std::chrono_literals;
        
        RCLCPP_INFO(this->get_logger(), "Starting test sequence...");
        
        test_forward();
        std::this_thread::sleep_for(2s);
        
        test_stop();
        std::this_thread::sleep_for(1s);
        
        test_backward();
        std::this_thread::sleep_for(2s);
        
        test_stop();
        std::this_thread::sleep_for(1s);
        
        test_left();
        std::this_thread::sleep_for(2s);
        
        test_stop();
        std::this_thread::sleep_for(1s);
        
        test_right();
        std::this_thread::sleep_for(2s);
        
        test_stop();
        
        RCLCPP_INFO(this->get_logger(), "Test sequence complete");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveTestNode>();

    // Run test sequence
    node->test_sequence();

    rclcpp::shutdown();
    return 0;
}
