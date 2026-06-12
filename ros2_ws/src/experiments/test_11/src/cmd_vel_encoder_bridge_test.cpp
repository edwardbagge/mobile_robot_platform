// cmd_vel_encoder_bridge_test.cpp
// ROS 2 Jazzy C++ combined cmd_vel serial control and encoder feedback test
//
// Purpose:
// Send serial motor commands to the ESP32 from /cmd_vel and parse encoder
// feedback lines into ROS topics.
//
// Expected ESP32 feedback format:
// FEEDBACK | LEFT ticks = 791 | RIGHT ticks = 713
//
// Subscribes:
// /cmd_vel : geometry_msgs/msg/Twist
//
// Publishes:
// /left_encoder_ticks  : std_msgs/msg/Int64
// /right_encoder_ticks : std_msgs/msg/Int64

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <regex>
#include <string>
#include <thread>

class CmdVelEncoderBridgeNode : public rclcpp::Node
{
public:
  CmdVelEncoderBridgeNode()
  : Node("cmd_vel_encoder_bridge_test"), running_(true)
  {
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_rate_ = B115200;
    linear_threshold_ = this->declare_parameter<double>("linear_threshold", 0.01);
    angular_threshold_ = this->declare_parameter<double>("angular_threshold", 0.01);
    suppress_repeated_commands_ = this->declare_parameter<bool>(
      "suppress_repeated_commands",
      false
    );

    left_publisher_ = this->create_publisher<std_msgs::msg::Int64>(
      "/left_encoder_ticks",
      10
    );

    right_publisher_ = this->create_publisher<std_msgs::msg::Int64>(
      "/right_encoder_ticks",
      10
    );

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&CmdVelEncoderBridgeNode::cmdVelCallback, this, std::placeholders::_1)
    );

    openSerialPort();

    read_thread_ = std::thread(
      &CmdVelEncoderBridgeNode::readSerialLoop,
      this
    );

    RCLCPP_INFO(this->get_logger(), "CMD_VEL encoder bridge test started.");
    RCLCPP_INFO(this->get_logger(), "Listening on /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Publishing /left_encoder_ticks and /right_encoder_ticks");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Suppress repeated commands: %s",
      suppress_repeated_commands_ ? "true" : "false"
    );
  }

  ~CmdVelEncoderBridgeNode()
  {
    running_ = false;

    if (serial_fd_ >= 0) {
      sendSerialCommand("x");
    }

    if (read_thread_.joinable()) {
      read_thread_.join();
    }

    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  int serial_fd_ = -1;
  std::string serial_port_;
  speed_t baud_rate_;
  double linear_threshold_;
  double angular_threshold_;
  bool suppress_repeated_commands_;
  std::string last_sent_command_;

  std::atomic<bool> running_;
  std::thread read_thread_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_publisher_;

  void openSerialPort()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open serial port: %s",
        serial_port_.c_str()
      );
      return;
    }

    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes.");
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    cfsetispeed(&tty, baud_rate_);
    cfsetospeed(&tty, baud_rate_);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes.");
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::string command = twistToCommand(*msg);

    if (!suppress_repeated_commands_ || command != last_sent_command_) {
      if (sendSerialCommand(command)) {
        last_sent_command_ = command;
      }
    }
  }

  std::string twistToCommand(const geometry_msgs::msg::Twist & twist) const
  {
    double linear_x = twist.linear.x;
    double angular_z = twist.angular.z;

    if (std::abs(linear_x) < linear_threshold_ &&
      std::abs(angular_z) < angular_threshold_)
    {
      return "x";
    }

    if (std::abs(linear_x) >= std::abs(angular_z)) {
      if (linear_x > linear_threshold_) {
        return "f";
      } else if (linear_x < -linear_threshold_) {
        return "b";
      }
    } else {
      if (angular_z > angular_threshold_) {
        return "l";
      } else if (angular_z < -angular_threshold_) {
        return "r";
      }
    }

    return "x";
  }

  bool sendSerialCommand(const std::string & command)
  {
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
      return false;
    }

    std::string command_with_newline = command + "\n";

    ssize_t bytes_written = write(
      serial_fd_,
      command_with_newline.c_str(),
      command_with_newline.length()
    );

    if (bytes_written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write command to serial port.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Sent command to ESP32: %s", command.c_str());
    return true;
  }

  void readSerialLoop()
  {
    char buffer[256];
    std::string line_buffer;

    while (running_) {
      if (serial_fd_ >= 0) {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
          buffer[bytes_read] = '\0';
          line_buffer += std::string(buffer);

          size_t newline_position;

          while ((newline_position = line_buffer.find('\n')) != std::string::npos) {
            std::string line = line_buffer.substr(0, newline_position);
            line_buffer.erase(0, newline_position + 1);

            parseLine(line);
          }
        }
      }

      usleep(10000);
    }
  }

  void parseLine(const std::string & raw_line)
  {
    std::string line = raw_line;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    if (line.empty()) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "ESP32: %s", line.c_str());

    static const std::regex feedback_regex(
      R"(FEEDBACK \| LEFT ticks = (-?\d+) \| RIGHT ticks = (-?\d+))"
    );

    std::smatch match;

    if (std::regex_search(line, match, feedback_regex)) {
      long left_ticks = std::stol(match[1].str());
      long right_ticks = std::stol(match[2].str());

      publishTicks(left_ticks, right_ticks);

      RCLCPP_INFO(
        this->get_logger(),
        "Parsed encoder ticks -> LEFT: %ld, RIGHT: %ld",
        left_ticks,
        right_ticks
      );
    }
  }

  void publishTicks(long left_ticks, long right_ticks)
  {
    std_msgs::msg::Int64 left_msg;
    std_msgs::msg::Int64 right_msg;

    left_msg.data = left_ticks;
    right_msg.data = right_ticks;

    left_publisher_->publish(left_msg);
    right_publisher_->publish(right_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelEncoderBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
