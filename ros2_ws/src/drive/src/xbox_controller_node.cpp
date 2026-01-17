#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/joystick.h>
#include <thread>
#include <chrono>

/**
 * @file xbox_controller_node.cpp
 * @brief Xbox controller input node for rover drive control
 * 
 * Reads from Xbox controller via Linux joystick interface (/dev/input/jsX)
 * and publishes geometry_msgs/Twist commands to /drive/cmd_vel
 * 
 * Joystick Axis Mapping (Xbox Controller):
 * - Axis 1: Left stick Y (forward/backward) - 0=forward, 1=backward
 * - Axis 0: Left stick X (steering/rotation) - negative=left, positive=right
 * - Axis 4: Left trigger (LT)
 * - Axis 5: Right trigger (RT)
 * - Axis 6: D-pad X
 * - Axis 7: D-pad Y
 * 
 * Button Mapping:
 * - Button 0: A
 * - Button 1: B
 * - Button 2: X
 * - Button 3: Y
 * - Button 4: LB
 * - Button 5: RB
 * - Button 6: Back
 * - Button 7: Start
 * - Button 8: Left stick press
 * - Button 9: Right stick press
 * 
 * Usage:
 * 1. Connect Xbox controller via USB
 * 2. Find device: ls /dev/input/js*
 * 3. Run: ros2 run drive xbox_controller_node --ros-args -p device:=/dev/input/js0
 * 4. In another terminal: ros2 topic echo /drive/cmd_vel
 */

class XboxControllerNode : public rclcpp::Node
{
public:
    XboxControllerNode() : Node("xbox_controller")
    {
        // Declare parameters
        this->declare_parameter<std::string>("device", "/dev/input/js0");
        this->declare_parameter<float>("max_linear_speed", 1.0f);
        this->declare_parameter<float>("max_angular_speed", 1.0f);
        this->declare_parameter<float>("deadzone", 0.1f);
        
        // Get parameters
        device_path_ = this->get_parameter("device").as_string();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        deadzone_ = this->get_parameter("deadzone").as_double();
        
        // Create publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
        
        // Initialize joystick
        if (!init_joystick()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize joystick");
            rclcpp::shutdown();
            return;
        }
        
        // Start joystick reading thread
        joystick_thread_ = std::thread(&XboxControllerNode::read_joystick, this);
        
        RCLCPP_INFO(this->get_logger(), 
            "Xbox controller node initialized on device: %s", device_path_.c_str());
        RCLCPP_INFO(this->get_logger(), 
            "Max linear speed: %.2f, Max angular speed: %.2f, Deadzone: %.2f",
            max_linear_speed_, max_angular_speed_, deadzone_);
    }
    
    ~XboxControllerNode()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
        if (joystick_thread_.joinable()) {
            joystick_thread_.join();
        }
    }

private:
    std::string device_path_;
    int fd_{-1};
    float max_linear_speed_;
    float max_angular_speed_;
    float deadzone_;
    
    float left_stick_x_{0.0f};
    float left_stick_y_{0.0f};
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::thread joystick_thread_;
    
    bool init_joystick()
    {
        fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot open device: %s (Error: %s)", 
                device_path_.c_str(), strerror(errno));
            RCLCPP_INFO(this->get_logger(), 
                "Available devices: ls /dev/input/js*");
            return false;
        }
        
        // Get device name
        char name[128];
        if (ioctl(fd_, JSIOCGNAME(sizeof(name)), name) < 0) {
            strncpy(name, "Unknown", sizeof(name));
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to joystick: %s", name);
        return true;
    }
    
    void read_joystick()
    {
        struct js_event event;
        
        while (rclcpp::ok()) {
            // Try to read event
            ssize_t bytes = read(fd_, &event, sizeof(event));
            
            if (bytes == sizeof(event)) {
                // Handle axis motion (analog sticks, triggers, etc.)
                if (event.type & JS_EVENT_AXIS) {
                    // Normalize value from [-32767, 32767] to [-1.0, 1.0]
                    float normalized = event.value / 32767.0f;
                    
                    // Left stick Y-axis (forward/backward)
                    if (event.number == 1) {
                        left_stick_y_ = -normalized;  // Negative because up is negative
                    }
                    // Left stick X-axis (left/right rotation)
                    else if (event.number == 0) {
                        left_stick_x_ = normalized;
                    }
                }
                // Handle button presses
                else if (event.type & JS_EVENT_BUTTON) {
                    RCLCPP_DEBUG(this->get_logger(), "Button %d: %s", 
                        event.number, event.value ? "pressed" : "released");
                }
            }
            
            // Publish at 50 Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            publish_twist();
        }
    }
    
    void publish_twist()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        // Apply deadzone
        float linear_x = (std::abs(left_stick_y_) > deadzone_) ? left_stick_y_ : 0.0f;
        float angular_z = (std::abs(left_stick_x_) > deadzone_) ? left_stick_x_ : 0.0f;
        
        // Scale to max speeds
        msg.linear.x = linear_x * max_linear_speed_;
        msg.angular.z = angular_z * max_angular_speed_;
        
        cmd_vel_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XboxControllerNode>());
    rclcpp::shutdown();
    return 0;
}
