#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

class ArmController : public rclcpp::Node {
  public:
    ArmController() : Node("arm_controller") {
        if (!load_config()) {
            RCLCPP_ERROR(this->get_logger(), "Configuration load failed; shutting down arm controller.");
            rclcpp::shutdown();
            return;
        }

        if (!init_gpio()) {
            RCLCPP_ERROR(this->get_logger(), "GPIO initialization failed; shutting down arm controller.");
            rclcpp::shutdown();
            return;
        }

        start_joint_workers();

        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(command_input_topic_, 10, std::bind(&ArmController::cmd_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to command topic: %s", command_input_topic_.c_str());

        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_output_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "Publishing joint states to topic: %s", joint_states_output_topic_.c_str());

        const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(control_period_s_));
        control_timer_ = this->create_wall_timer(control_period, std::bind(&ArmController::control_loop, this));

        last_command_time_ = this->now();
        last_update_time_ = this->now();
    }

    ~ArmController() override { cleanup_gpio(); }

  private:
    struct JointRuntime {
        std::string key;
        std::string name;
        int step_pin = -1;
        int dir_pin = -1;
        double gear_ratio = 1.0;
        int microsteps = 1;
        bool reversed = false;
        double max_speed = 0.0; // steps/sec
        double max_accel = 0.0; // steps/sec^2

        int step_fd = -1;
        int dir_fd = -1;

        std::atomic<double> commanded_velocity_hz{0.0}; // signed steps/sec
        std::atomic<double> active_velocity_hz{0.0};    // signed steps/sec
        double current_position_rad = 0.0;

        std::atomic<bool> stop_worker{false};
        std::thread worker;
    };

    std::map<std::string, std::unique_ptr<JointRuntime>> joints_;
    std::vector<std::string> joint_order_;

    std::map<int, int> gpio_value_fds_;
    std::vector<int> exported_pins_;

    double input_timeout_ = 0.5;
    double deadzone_ = 0.05;
    double scale_speed_ = 1.0;
    double control_period_s_ = 0.01;
    std::string command_input_topic_;
    std::string joint_states_output_topic_;

    geometry_msgs::msg::Twist last_command_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_update_time_;
    std::mutex command_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool load_config() {
        try {
            const std::string package_share_directory = ament_index_cpp::get_package_share_directory("arm");
            const std::string config_file = package_share_directory + "/config/arm_config.yaml";
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

            joints_.clear();
            joint_order_.clear();
            for (auto it = hardware.begin(); it != hardware.end(); ++it) {
                const std::string joint_key = it->first.as<std::string>();
                const YAML::Node joint = it->second;
                const YAML::Node joint_limits = limits[joint_key];
                if (!joint_limits || !joint_limits.IsMap()) {
                    throw std::runtime_error("Missing limits for joint: " + joint_key);
                }

                auto runtime = std::make_unique<JointRuntime>();
                runtime->key = joint_key;
                runtime->name = joint["name"].as<std::string>();
                runtime->step_pin = joint["step_pin"].as<int>();
                runtime->dir_pin = joint["dir_pin"].as<int>();
                runtime->gear_ratio = joint["gear_ratio"].as<double>();
                runtime->microsteps = joint["microsteps"].as<int>();
                runtime->reversed = joint["reversed"].as<bool>();
                runtime->max_speed = joint_limits["max_speed"].as<double>();
                runtime->max_accel = joint_limits["max_accel"].as<double>();
                validate_joint_config(*runtime);

                joints_[joint_key] = std::move(runtime);
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

            if (!topics["command_input"] || !topics["command_input"].IsScalar()) {
                throw std::runtime_error("Missing arm.ros2.topics.command_input in config");
            }
            YAML::Node joint_states_output = topics["joint_states_output"];
            if (!joint_states_output || !joint_states_output.IsScalar()) {
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

    static bool write_text(const std::string &path, const std::string &value) {
        std::ofstream out(path);
        if (!out.is_open()) {
            return false;
        }
        out << value;
        out.flush();
        return out.good();
    }

    static inline void write_gpio_fd(int fd, int value) {
        if (fd < 0) {
            return;
        }
        const char c = value ? '1' : '0';
        ::lseek(fd, 0, SEEK_SET); // set offset to start of file for each write
        (void)::write(fd, &c, 1);
    }

    static void validate_joint_config(const JointRuntime &joint) {
        if (joint.microsteps <= 0) {
            throw std::runtime_error("Joint " + joint.key + " must have microsteps > 0");
        }
        if (!std::isfinite(joint.gear_ratio) || joint.gear_ratio <= 0.0) {
            throw std::runtime_error("Joint " + joint.key + " must have gear_ratio > 0");
        }
        if (!std::isfinite(joint.max_speed) || joint.max_speed < 0.0) {
            throw std::runtime_error("Joint " + joint.key + " must have max_speed >= 0");
        }
        if (!std::isfinite(joint.max_accel) || joint.max_accel < 0.0) {
            throw std::runtime_error("Joint " + joint.key + " must have max_accel >= 0");
        }
    }

    double command_to_target_hz(const JointRuntime &joint, double command) const {
        if (!std::isfinite(command)) {
            return 0.0;
        }

        const double scaled_command = command * scale_speed_;
        if (std::abs(scaled_command) < deadzone_) {
            return 0.0;
        }
        return std::clamp(scaled_command, -joint.max_speed, joint.max_speed);
    }

    bool init_gpio() {
        std::set<int> unique_pins;

        for (const auto &entry : joints_) {
            const JointRuntime &joint = *entry.second;
            if (joint.step_pin < 0 || joint.dir_pin < 0) {
                RCLCPP_ERROR(this->get_logger(), "Invalid GPIO config for joint '%s': step_pin=%d, dir_pin=%d", joint.key.c_str(), joint.step_pin, joint.dir_pin);
                return false;
            }
            if (!unique_pins.insert(joint.step_pin).second || !unique_pins.insert(joint.dir_pin).second) {
                RCLCPP_ERROR(this->get_logger(), "Duplicate GPIO pin detected in arm configuration");
                return false;
            }
        }

        for (int pin : unique_pins) {
            const std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(pin);
            const std::string direction_path = gpio_path + "/direction";
            const std::string value_path = gpio_path + "/value";

            if (::access(gpio_path.c_str(), F_OK) != 0) {
                if (!write_text("/sys/class/gpio/export", std::to_string(pin))) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to export GPIO %d: %s", pin, std::strerror(errno));
                    cleanup_gpio();
                    return false;
                }
                exported_pins_.push_back(pin);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            if (!write_text(direction_path, "out")) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO %d direction at %s", pin, direction_path.c_str());
                cleanup_gpio();
                return false;
            }
            if (!write_text(value_path, "0")) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO %d low at %s", pin, value_path.c_str());
                cleanup_gpio();
                return false;
            }

            const int fd = ::open(value_path.c_str(), O_WRONLY | O_CLOEXEC);
            if (fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO %d value file at %s: %s", pin, value_path.c_str(), std::strerror(errno));
                cleanup_gpio();
                return false;
            }
            gpio_value_fds_[pin] = fd;
        }

        for (const auto &entry : joints_) {
            JointRuntime &joint = *entry.second;
            joint.step_fd = gpio_value_fds_[joint.step_pin];
            joint.dir_fd = gpio_value_fds_[joint.dir_pin];
        }

        RCLCPP_INFO(this->get_logger(), "Initialized %zu GPIO pins", unique_pins.size());
        return true;
    }

    void start_joint_workers() {
        for (const auto &key : joint_order_) {
            JointRuntime &joint = *joints_.at(key);
            joint.stop_worker.store(false);
            joint.worker = std::thread(&ArmController::joint_worker_loop, this, &joint);
        }
    }

    void stop_joint_workers() {
        for (const auto &entry : joints_) {
            entry.second->stop_worker.store(true);
        }
        for (const auto &entry : joints_) {
            auto &worker = entry.second->worker;
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void joint_worker_loop(JointRuntime *joint) {
        constexpr int kMinPulseUs = 8;
        int last_dir_level = -1;

        while (!joint->stop_worker.load()) {
            const double active_hz = joint->active_velocity_hz.load();
            if (std::abs(active_hz) < 1e-6) {
                write_gpio_fd(joint->step_fd, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            bool forward = active_hz >= 0.0;
            if (joint->reversed) {
                forward = !forward;
            }
            const int dir_level = forward ? 1 : 0;
            if (dir_level != last_dir_level) {
                write_gpio_fd(joint->dir_fd, dir_level);
                last_dir_level = dir_level;
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }

            const double hz = std::abs(active_hz);
            const int period_us = std::max(2 * kMinPulseUs, static_cast<int>(std::llround(1e6 / hz)));
            const int high_us = std::max(kMinPulseUs, period_us / 2);
            const int low_us = std::max(kMinPulseUs, period_us - high_us);

            write_gpio_fd(joint->step_fd, 1);
            std::this_thread::sleep_for(std::chrono::microseconds(high_us));
            write_gpio_fd(joint->step_fd, 0);
            std::this_thread::sleep_for(std::chrono::microseconds(low_us));
        }

        write_gpio_fd(joint->step_fd, 0);
    }

    void cleanup_gpio() {
        stop_joint_workers();

        for (const auto &entry : joints_) {
            const JointRuntime &joint = *entry.second;
            if (joint.step_fd >= 0) {
                write_gpio_fd(joint.step_fd, 0);
            }
        }

        for (auto &entry : gpio_value_fds_) {
            if (entry.second >= 0) {
                ::close(entry.second);
            }
        }
        gpio_value_fds_.clear();

        for (int pin : exported_pins_) {
            (void)write_text("/sys/class/gpio/unexport", std::to_string(pin));
        }
        exported_pins_.clear();
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_time_ = this->now();
        last_command_ = *msg;
    }

    void set_target_with_limits(const std::string &joint_key, double command) {
        auto it = joints_.find(joint_key);
        if (it == joints_.end()) {
            return;
        }

        JointRuntime &joint = *it->second;
        joint.commanded_velocity_hz.store(command_to_target_hz(joint, command));
    }

    void apply_command_to_targets(const geometry_msgs::msg::Twist &command) {
        set_target_with_limits("base", command.angular.z);
        set_target_with_limits("shoulder", command.angular.y);
        set_target_with_limits("elbow", command.angular.x);
    }

    static double steps_to_radians(const JointRuntime &joint, double steps) {
        constexpr double kTwoPi = 6.28318530717958647692;
        const double steps_per_output_rev = static_cast<double>(joint.microsteps) * joint.gear_ratio;
        if (steps_per_output_rev <= 0.0) {
            return 0.0;
        }
        return steps * kTwoPi / steps_per_output_rev;
    }

    void update_joint_states(double dt) {
        for (const auto &key : joint_order_) {
            JointRuntime &joint = *joints_.at(key);
            double current_hz = joint.active_velocity_hz.load();
            const double target_hz = joint.commanded_velocity_hz.load();

            const double max_change = std::max(0.0, joint.max_accel) * dt;
            const double delta = target_hz - current_hz;
            if (max_change <= 0.0) {
                current_hz = target_hz;
            } else if (std::abs(delta) > max_change) {
                current_hz += (delta > 0.0 ? max_change : -max_change);
            } else {
                current_hz = target_hz;
            }

            joint.active_velocity_hz.store(current_hz);
            joint.current_position_rad += steps_to_radians(joint, current_hz * dt);
        }
    }

    void publish_joint_states(const rclcpp::Time &stamp) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name.reserve(joint_order_.size());
        msg.position.reserve(joint_order_.size());
        msg.velocity.reserve(joint_order_.size());

        for (const auto &key : joint_order_) {
            const JointRuntime &joint = *joints_.at(key);
            msg.name.push_back(key);
            msg.position.push_back(joint.current_position_rad);
            msg.velocity.push_back(steps_to_radians(joint, joint.active_velocity_hz.load()));
        }

        joint_states_publisher_->publish(msg);
    }

    void control_loop() {
        const auto now = this->now();
        geometry_msgs::msg::Twist command;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if ((now - last_command_time_).seconds() > input_timeout_) {
                last_command_ = geometry_msgs::msg::Twist();
            }
            command = last_command_;
        }

        double dt = (now - last_update_time_).seconds();
        if (dt <= 0.0) {
            dt = control_period_s_;
        }
        last_update_time_ = now;

        apply_command_to_targets(command);
        update_joint_states(dt);
        publish_joint_states(now);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());
    rclcpp::shutdown();
    return 0;
}
