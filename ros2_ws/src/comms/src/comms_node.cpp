#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {
static constexpr uint8_t kPacketStartByte = 255;
static constexpr uint8_t kPacketEndByte = '\n'; // 10
static constexpr int kSerialBaudRate = 57600;
static constexpr speed_t kSerialBaudRateTermios = B57600;

enum class PacketType : uint8_t {
    CONTROL = 0,
    STICKS = 1,
};
} // namespace

float byte_to_float(uint8_t value) { return (static_cast<float>(value) / 254.0F) * 2.0F - 1.0F; }

class PacketParser {
  public:
    void reset() {
        state_ = 0;
        tmp_var1_byte_ = 0;
        tmp_var2_byte_ = 0;
    }

    bool parse_byte(uint8_t byte, uint8_t &var1_byte, uint8_t &var2_byte) {
        switch (state_) {
        case 0:
            state_ = (byte == kPacketStartByte) ? 1 : 0;
            break;
        case 1:
            if (byte == static_cast<uint8_t>(PacketType::STICKS)) {
                state_ = 2;
            } else {
                state_ = (byte == kPacketStartByte) ? 1 : 0;
            }
            break;
        case 2:
            tmp_var1_byte_ = byte;
            state_ = 3;
            break;
        case 3:
            tmp_var2_byte_ = byte;
            state_ = 4;
            break;
        case 4:
            if (byte == kPacketEndByte) {
                var1_byte = tmp_var1_byte_;
                var2_byte = tmp_var2_byte_;
                state_ = 0;
                return true;
            }
            state_ = (byte == kPacketStartByte) ? 1 : 0;
            break;
        default:
            state_ = 0;
            break;
        }
        return false;
    }

  private:
    int state_{0};
    uint8_t tmp_var1_byte_{0};
    uint8_t tmp_var2_byte_{0};
};

class CommsNode : public rclcpp::Node {
  public:
    CommsNode() : Node("comms_node") {
        this->declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
        this->declare_parameter<int>("read_chunk_size", 64);
        this->declare_parameter<int>("reconnect_interval_ms", 1000);

        serial_device_ = this->get_parameter("serial_device").as_string();
        read_chunk_size_ = this->get_parameter("read_chunk_size").as_int();
        reconnect_interval_ms_ = this->get_parameter("reconnect_interval_ms").as_int();

        if (read_chunk_size_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "read_chunk_size must be > 0. Using default 64.");
            read_chunk_size_ = 64;
        }
        if (reconnect_interval_ms_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "reconnect_interval_ms must be > 0. Using default 1000.");
            reconnect_interval_ms_ = 1000;
        }

        read_buffer_.resize(static_cast<size_t>(read_chunk_size_));
        vars_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/comms/vars", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CommsNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Comms node started. device=%s baud=%d read_chunk_size=%d reconnect_interval_ms=%d", serial_device_.c_str(), kSerialBaudRate, read_chunk_size_, reconnect_interval_ms_);
    }

    ~CommsNode() override { close_serial_port(); }

  private:
    int open_serial_port(const std::string &device) {
        int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial device %s: %s", device.c_str(), std::strerror(errno));
            return -1;
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed for %s: %s", device.c_str(), std::strerror(errno));
            ::close(fd);
            return -1;
        }

        cfmakeraw(&tty);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
#ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;
#endif
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        if (cfsetispeed(&tty, kSerialBaudRateTermios) != 0 || cfsetospeed(&tty, kSerialBaudRateTermios) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate %d for %s: %s", kSerialBaudRate, device.c_str(), std::strerror(errno));
            ::close(fd);
            return -1;
        }

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed for %s: %s", device.c_str(), std::strerror(errno));
            ::close(fd);
            return -1;
        }

        tcflush(fd, TCIFLUSH);
        return fd;
    }

    void close_serial_port() {
        parser_.reset();
        if (serial_fd_ >= 0) {
            ::close(serial_fd_);
            serial_fd_ = -1;
            RCLCPP_WARN(this->get_logger(), "Serial connection closed.");
        }
    }

    void try_connect() {
        const auto now = std::chrono::steady_clock::now();
        if (last_connect_attempt_.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_connect_attempt_).count();
            if (elapsed_ms < reconnect_interval_ms_) {
                return;
            }
        }

        last_connect_attempt_ = now;
        serial_fd_ = open_serial_port(serial_device_);
        if (serial_fd_ >= 0) {
            parser_.reset();
            RCLCPP_INFO(this->get_logger(), "Connected to serial device %s @ %d baud.", serial_device_.c_str(), kSerialBaudRate);
        }
    }

    void publish_vars(float var1, float var2) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {var1, var2};
        vars_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Received var1=%.4f var2=%.4f", var1, var2);
    }

    void timer_callback() {
        if (serial_fd_ < 0) {
            try_connect();
            return;
        }

        const ssize_t bytes_read = ::read(serial_fd_, read_buffer_.data(), read_buffer_.size());
        if (bytes_read > 0) {
            for (ssize_t i = 0; i < bytes_read; ++i) {
                uint8_t var1_byte = 0;
                uint8_t var2_byte = 0;
                if (parser_.parse_byte(read_buffer_[static_cast<size_t>(i)], var1_byte, var2_byte)) {
                    publish_vars(byte_to_float(var1_byte), byte_to_float(var2_byte));
                }
            }
            return;
        }

        if (bytes_read == 0) {
            return;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return;
        }

        RCLCPP_ERROR(this->get_logger(), "Serial read error on %s: %s", serial_device_.c_str(), std::strerror(errno));
        close_serial_port();
    }

    std::string serial_device_;
    int read_chunk_size_{64};
    int reconnect_interval_ms_{1000};
    int serial_fd_{-1};
    std::chrono::steady_clock::time_point last_connect_attempt_{};
    std::vector<uint8_t> read_buffer_;
    PacketParser parser_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vars_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto comms_node = std::make_shared<CommsNode>();
    rclcpp::spin(comms_node);
    rclcpp::shutdown();
    return 0;
}
