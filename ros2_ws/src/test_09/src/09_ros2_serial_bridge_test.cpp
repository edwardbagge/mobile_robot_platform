// 09_ros2_serial_bridge_test.cpp
// ROS 2 Jazzy C++ serial bridge test
//
// Purpose:
// Subscribe to /robot_command and send commands to ESP32 over USB serial.
//
// Topic:
// /robot_command : std_msgs/msg/String
//
// Valid commands:
// f = forward
// b = backward
// l = turn left
// r = turn right
// x = stop
// s = status
// z = reset encoder counts

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <string>
#include <thread>

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode()
  : Node("serial_bridge_test"), running_(true)
  {
    serial_port_ = "/dev/ttyUSB0";
    baud_rate_ = B115200;

    openSerialPort();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_command",
      10,
      std::bind(&SerialBridgeNode::commandCallback, this, std::placeholders::_1)
    );

    read_thread_ = std::thread(&SerialBridgeNode::readSerialLoop, this);

    RCLCPP_INFO(this->get_logger(), "ROS 2 serial bridge test started.");
    RCLCPP_INFO(this->get_logger(), "Listening on /robot_command");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
  }

  ~SerialBridgeNode()
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

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void openSerialPort()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
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

  void commandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
      return;
    }

    std::string command = msg->data;

    if (!isValidCommand(command))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid command: %s", command.c_str());
      return;
    }

    command += "\n";

    ssize_t bytes_written = write(serial_fd_, command.c_str(), command.length());

    if (bytes_written < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write command to serial port.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Sent command to ESP32: %s", msg->data.c_str());
    }
  }

  bool isValidCommand(const std::string & command)
  {
    return command == "f" ||
           command == "b" ||
           command == "l" ||
           command == "r" ||
           command == "x" ||
           command == "s" ||
           command == "z";
  }

  void readSerialLoop()
  {
    char buffer[256];

    while (running_)
    {
      if (serial_fd_ >= 0)
      {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
          buffer[bytes_read] = '\0';

          std::string received(buffer);
          RCLCPP_INFO(this->get_logger(), "ESP32: %s", received.c_str());
        }
      }

      usleep(10000);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}