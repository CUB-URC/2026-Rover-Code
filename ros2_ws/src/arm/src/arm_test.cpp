#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

/**
 * @file arm_test.cpp
 * @brief Test node for arm controller
 *
 * Publishes test commands to /arm/cmd_vel to verify arm controller functionality.
 * Run this node with: ros2 run arm arm_test_node
 */

class ArmTestNode : public rclcpp::Node
{
public:
    ArmTestNode() : Node("arm_test")
    {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/arm/cmd_vel",
            10);

        RCLCPP_INFO(this->get_logger(), "Arm test node started");
        RCLCPP_INFO(this->get_logger(), "Publishing test commands to /arm/cmd_vel");
    }

    void test_base(double speed)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = speed;  // base rotation
        RCLCPP_INFO(this->get_logger(), "Base cmd: %.2f", speed);
        cmd_publisher_->publish(msg);
    }

    void test_shoulder(double speed)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.y = speed;  // shoulder lift
        RCLCPP_INFO(this->get_logger(), "Shoulder cmd: %.2f", speed);
        cmd_publisher_->publish(msg);
    }

    void test_elbow(double speed)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.x = speed;  // elbow flex
        RCLCPP_INFO(this->get_logger(), "Elbow cmd: %.2f", speed);
        cmd_publisher_->publish(msg);
    }

    void test_stop()
    {
        auto msg = geometry_msgs::msg::Twist();
        RCLCPP_INFO(this->get_logger(), "Stop cmd");
        cmd_publisher_->publish(msg);
    }

    void test_sequence()
    {
        using namespace std::chrono_literals;

        RCLCPP_INFO(this->get_logger(), "Starting arm test sequence...");

        test_base(0.3);
        std::this_thread::sleep_for(2s);
        test_stop();
        std::this_thread::sleep_for(1s);

        test_shoulder(0.2);
        std::this_thread::sleep_for(2s);
        test_stop();
        std::this_thread::sleep_for(1s);

        test_elbow(-0.25);
        std::this_thread::sleep_for(2s);
        test_stop();

        RCLCPP_INFO(this->get_logger(), "Arm test sequence complete");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTestNode>();

    node->test_sequence();

    rclcpp::shutdown();
    return 0;
}
