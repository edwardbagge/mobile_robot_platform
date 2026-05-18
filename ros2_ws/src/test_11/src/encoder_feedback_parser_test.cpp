// 11_encoder_feedback_parser_test.cpp
// ROS 2 Jazzy C++ encoder feedback parser test
//
// Purpose:
// Read ESP32 serial feedback lines and parse encoder tick values.
//
// Expected ESP32 feedback format:
// FEEDBACK | LEFT ticks = 791 | RIGHT ticks = 713
//
// Publishes:
// /left_encoder_ticks  : std_msgs/msg/Int64
// /right_encoder_ticks : std_msgs/msg/Int64

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <regex>
#include <string>
#include <thread>

class EncoderFeedbackParserNode : public rclcpp::Node
{
public:
  EncoderFeedbackParserNode()
  : Node("encoder_feedback_parser_test"), running_(true)
  {
    serial_port_ = "/dev/ttyUSB0";
    baud_rate_ = B115200;

    left_publisher_ = this->create_publisher<std_msgs::msg::Int64>(
      "/left_encoder_ticks",
      10
    );

    right_publisher_ = this->create_publisher<std_msgs::msg::Int64>(
      "/right_encoder_ticks",
      10
    );

    openSerialPort();

    read_thread_ = std::thread(
      &EncoderFeedbackParserNode::readSerialLoop,
      this
    );

    RCLCPP_INFO(this->get_logger(), "Encoder feedback parser test started.");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
  }

  ~EncoderFeedbackParserNode()
  {
    running_ = false;

    if (read_thread_.joinable())
    {
      read_thread_.join();
    }

    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
    }
  }

private:
  int serial_fd_ = -1;
  std::string serial_port_;
  speed_t baud_rate_;

  std::atomic<bool> running_;
  std::thread read_thread_;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_publisher_;

  void openSerialPort()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open serial port: %s",
        serial_port_.c_str()
      );
      return;
    }

    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd_, &tty) != 0)
    {
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

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes.");
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
  }

  void readSerialLoop()
  {
    char buffer[256];
    std::string line_buffer;

    while (running_)
    {
      if (serial_fd_ >= 0)
      {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
          buffer[bytes_read] = '\0';
          line_buffer += std::string(buffer);

          size_t newline_position;

          while ((newline_position = line_buffer.find('\n')) != std::string::npos)
          {
            std::string line = line_buffer.substr(0, newline_position);
            line_buffer.erase(0, newline_position + 1);

            parseLine(line);
          }
        }
      }

      usleep(10000);
    }
  }

  void parseLine(const std::string & line)
  {
    RCLCPP_INFO(this->get_logger(), "ESP32: %s", line.c_str());

    std::regex feedback_regex(
      R"(FEEDBACK \| LEFT ticks = (-?\d+) \| RIGHT ticks = (-?\d+))"
    );

    std::smatch match;

    if (std::regex_search(line, match, feedback_regex))
    {
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
  auto node = std::make_shared<EncoderFeedbackParserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}