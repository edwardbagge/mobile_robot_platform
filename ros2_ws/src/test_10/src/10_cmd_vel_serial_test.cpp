// 10_cmd_vel_serial_test.cpp
// ROS 2 Jazzy C++ cmd_vel to ESP32 serial command test
//
// Purpose:
// Subscribe to /cmd_vel and convert geometry_msgs/msg/Twist messages
// into simple ESP32 serial commands.
//
// Mapping:
// linear.x > 0      -> f
// linear.x < 0      -> b
// angular.z > 0     -> l
// angular.z < 0     -> r
// stopped/zero cmd  -> x
//
// Serial commands sent to ESP32:
// f = forward
// b = backward
// l = turn left
// r = turn right
// x = stop

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

class CmdVelSerialTestNode : public rclcpp::Node
{
public:
  CmdVelSerialTestNode()
  : Node("cmd_vel_serial_test"), running_(true)
  {
    serial_port_ = "/dev/ttyUSB0";
    baud_rate_ = B115200;

    linear_threshold_ = 0.01;
    angular_threshold_ = 0.01;

    last_sent_command_ = "";

    openSerialPort();

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&CmdVelSerialTestNode::cmdVelCallback, this, std::placeholders::_1)
    );

    read_thread_ = std::thread(&CmdVelSerialTestNode::readSerialLoop, this);

    RCLCPP_INFO(this->get_logger(), "CMD_VEL serial test node started.");
    RCLCPP_INFO(this->get_logger(), "Listening on /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
  }

  ~CmdVelSerialTestNode()
  {
    running_ = false;

    if (serial_fd_ >= 0)
    {
      sendSerialCommand("x");
    }

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

  double linear_threshold_;
  double angular_threshold_;

  std::string last_sent_command_;

  std::atomic<bool> running_;
  std::thread read_thread_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

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

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::string command = twistToCommand(*msg);

    if (command != last_sent_command_)
    {
      sendSerialCommand(command);
      last_sent_command_ = command;
    }
  }

  std::string twistToCommand(const geometry_msgs::msg::Twist & twist)
  {
    double linear_x = twist.linear.x;
    double angular_z = twist.angular.z;

    if (std::abs(linear_x) < linear_threshold_ &&
        std::abs(angular_z) < angular_threshold_)
    {
      return "x";
    }

    if (std::abs(linear_x) >= std::abs(angular_z))
    {
      if (linear_x > linear_threshold_)
      {
        return "f";
      }
      else if (linear_x < -linear_threshold_)
      {
        return "b";
      }
    }
    else
    {
      if (angular_z > angular_threshold_)
      {
        return "l";
      }
      else if (angular_z < -angular_threshold_)
      {
        return "r";
      }
    }

    return "x";
  }

  void sendSerialCommand(const std::string & command)
  {
    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
      return;
    }

    std::string command_with_newline = command + "\n";

    ssize_t bytes_written = write(
      serial_fd_,
      command_with_newline.c_str(),
      command_with_newline.length()
    );

    if (bytes_written < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write command to serial port.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Sent command to ESP32: %s", command.c_str());
    }
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
  auto node = std::make_shared<CmdVelSerialTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}