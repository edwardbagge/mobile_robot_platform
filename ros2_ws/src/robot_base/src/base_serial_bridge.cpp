// base_serial_bridge.cpp
// ROS 2 bridge between robot motion commands and the ESP32.
//
// Receives motion commands from ROS 2, converts them to left/right motor PWM,
// sends them to the ESP32 over USB serial, and publishes encoder feedback.

// These are related to ROS 2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>

// These are are Linux/system headers mainly used for the USB serial connection
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// These are standard C++ headers
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

class BaseSerialBridgeNode : public rclcpp::Node // ROS 2 node that handles serial communication with the ESP32
{
public:
  // The constructor configures the node parameters, sets up the ROS 2
  // subscriptions and publishers, opens the serial connection to the ESP32,
  // starts a background reader thread for firmware feedback, and schedules a
  // periodic timer that repeatedly sends the latest command to the robot base.
  BaseSerialBridgeNode()
  : Node("base_serial_bridge"), running_(true)
  {
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");    // Serial device used for communication with the ESP32
    command_source_ = this->declare_parameter<std::string>("command_source", "cmd_vel");   // Select whether commands come from /cmd_vel or /wheel_pwm_cmd
    command_rate_hz_ = this->declare_parameter<double>("command_rate_hz", 20.0);           // Number of motor commands sent to the ESP32 per second
    command_timeout_s_ = this->declare_parameter<double>("command_timeout_s", 0.25);       // Stop the robot if no new command is received within this time
    wheel_base_m_ = this->declare_parameter<double>("wheel_base_m", 0.16);                 // Distance between the left and right wheels
    max_wheel_speed_mps_ = this->declare_parameter<double>("max_wheel_speed_mps", 0.25);   // Wheel speed corresponding to the maximum PWM command
    max_pwm_ = this->declare_parameter<int>("max_pwm", 80);                                // Maximum PWM value sent to the ESP32
    min_pwm_ = this->declare_parameter<int>("min_pwm", 0);                                 // Minimum non-zero PWM value
    left_pwm_scale_ = this->declare_parameter<double>("left_pwm_scale", 1.0);              // Scale factor applied to the left motor PWM
    right_pwm_scale_ = this->declare_parameter<double>("right_pwm_scale", 1.0);            // Scale factor applied to the right motor PWM
    linear_deadband_mps_ = this->declare_parameter<double>("linear_deadband_mps", 0.005);  // Treat very small linear velocity commands as zero
    angular_deadband_rps_ = this->declare_parameter<double>("angular_deadband_rps", 0.01); // Treat very small angular velocity commands as zero
    active_brake_on_stop_ = this->declare_parameter<bool>("active_brake_on_stop", true);   // Use active braking when the commanded motion is zero
    startup_delay_s_ = this->declare_parameter<double>("startup_delay_s", 2.5);            // Delay before communicating with the ESP32 after opening the serial port

    max_pwm_ = std::clamp(max_pwm_, 0, 255);                   // Keep PWM limits within the 8-bit range supported by the ESP32 firmware
    min_pwm_ = std::clamp(min_pwm_, 0, max_pwm_);              // Keep the minimum PWM between zero and the configured maximum

    // Limit motor-specific scaling factors to a reasonable range
    left_pwm_scale_ = std::clamp(left_pwm_scale_, 0.0, 2.0);
    right_pwm_scale_ = std::clamp(right_pwm_scale_, 0.0, 2.0);

    // These publishers expose the encoder feedback from the ESP32 as ROS 2
    // topics. Other nodes can subscribe to them for odometry or debugging.
    left_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/left_encoder_ticks", 10);
    right_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/right_encoder_ticks", 10);

    // The node can operate in two modes:
    // 1. Receive /cmd_vel (geometry_msgs/Twist) and convert it into wheel PWM.
    // 2. Receive /wheel_pwm_cmd (std_msgs/Int32MultiArray) when a higher-level
    //    controller already computed the left/right PWM values.
    if (command_source_ == "wheel_pwm_cmd") 
    {
      pwm_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/wheel_pwm_cmd",
        10,
        std::bind(&BaseSerialBridgeNode::wheelPwmCallback, this, std::placeholders::_1));
    } 
    else 
    {
      command_source_ = "cmd_vel";
      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&BaseSerialBridgeNode::cmdVelCallback, this, std::placeholders::_1));
    }

    openSerialPort();      // Open the USB serial connection to the ESP32

    waitForEsp32Startup(); // Wait for the ESP32 to finish starting before communication begins

    read_thread_ = std::thread(&BaseSerialBridgeNode::readSerialLoop, this); // Start a separate thread that continuously reads serial feedback from the ESP32

    const auto period_s = 1.0 / std::max(command_rate_hz_, 1.0); // Calculate how long to wait between motor commands
    command_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&BaseSerialBridgeNode::sendCommandTimerCallback, this)); // Call sendCommandTimerCallback repeatedly at the configured command rate

    // Print the active configuration when the node starts
    RCLCPP_INFO(this->get_logger(), "Robot base serial bridge started.");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "command_source = %s", command_source_.c_str());
    RCLCPP_INFO(this->get_logger(), "command_rate_hz = %.2f", command_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "command_timeout_s = %.2f", command_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "wheel_base_m = %.3f", wheel_base_m_);
    RCLCPP_INFO(this->get_logger(), "max_wheel_speed_mps = %.3f", max_wheel_speed_mps_);
    RCLCPP_INFO(this->get_logger(), "max_pwm = %d", max_pwm_);
    RCLCPP_INFO(this->get_logger(), "min_pwm = %d", min_pwm_);
    RCLCPP_INFO(this->get_logger(), "left_pwm_scale = %.3f", left_pwm_scale_);
    RCLCPP_INFO(this->get_logger(), "right_pwm_scale = %.3f", right_pwm_scale_);
    RCLCPP_INFO(
      this->get_logger(),
      "active_brake_on_stop = %s",
      active_brake_on_stop_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "startup_delay_s = %.2f", startup_delay_s_);
  }

  ~BaseSerialBridgeNode() // Shut down the serial bridge safely when the node is destroyed
  {
    running_ = false; // Tell the serial-reading thread to stop

    if (serial_fd_ >= 0) {
      sendSerialLine("X"); // Actively brake the motors before shutting down
    }

    if (read_thread_.joinable()) {
      read_thread_.join(); // Wait for the serial-reading thread to finish
    }

    if (serial_fd_ >= 0) {
      close(serial_fd_); // Close the serial connection
    }
  }

private:
  int serial_fd_ = -1;            // File descriptor for the opened serial port
  std::string serial_port_;       // Serial device used for the ESP32 connection
  std::string command_source_;    // Selects whether commands come from /cmd_vel or /wheel_pwm_cmd
  speed_t baud_rate_ = B115200;   // Serial communication speed

  // Configurable motion and timing parameters
  double command_rate_hz_;
  double command_timeout_s_;
  double wheel_base_m_;
  double max_wheel_speed_mps_;
  int max_pwm_;
  int min_pwm_;
  double left_pwm_scale_;
  double right_pwm_scale_;
  double linear_deadband_mps_;
  double angular_deadband_rps_;
  bool active_brake_on_stop_;
  double startup_delay_s_;

  // Latest desired motor PWM values
  int target_left_pwm_ = 0;
  int target_right_pwm_ = 0;

  // Most recent PWM values successfully sent to the ESP32
  int last_sent_left_pwm_ = 0;
  int last_sent_right_pwm_ = 0;

  
  std::chrono::steady_clock::time_point last_cmd_vel_time_ = std::chrono::steady_clock::now(); // Time when the latest motion command was received
  std::mutex command_mutex_; // Protect command values that can be accessed from different threads

  std::atomic<bool> running_; // Controls whether the serial-reading thread should keep running
  std::thread read_thread_;   // Background thread used for reading ESP32 serial feedback

  rclcpp::TimerBase::SharedPtr command_timer_; // ROS 2 timer for periodically sending motor commands

  // ROS 2 subscriptions for the two possible command sources
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_subscription_;

  // ROS 2 publishers for left and right encoder ticks
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_publisher_;

  // Open the serial device with a simple terminal configuration suitable for the
  // ESP32 firmware's 115200 baud text protocol.
  void openSerialPort()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // Open the serial device for reading and writing without blocking

    if (serial_fd_ < 0) // Stop here if the serial port could not be opened
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      return;
    }

    // Create a structure for storing the serial-port settings
    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd_, &tty) != 0) // Read the current serial-port settings
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes.");
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    // Set both input and output communication speed to 115200 baud
    cfsetispeed(&tty, baud_rate_);
    cfsetospeed(&tty, baud_rate_);

    tty.c_cflag |= (CLOCAL | CREAD); // Enable reading and ignore modem-control lines

    // Use 8 data bits
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // Disable parity, extra stop bits, and hardware flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Disable terminal-style processing so serial data is read directly
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    tty.c_oflag &= ~OPOST;                          // Send output data without modifying it

    // Configure non-blocking reads with a short timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) // Apply the new serial-port settings
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes.");
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
  }

  // Give the ESP32 firmware a short startup window before sending commands.
  void waitForEsp32Startup()
  {
    if (serial_fd_ < 0) // Do nothing if the serial port is not open
    {
      return;
    }

    const auto delay = std::chrono::duration<double>(std::max(startup_delay_s_, 0.0)); // Convert the configured startup delay into a time duration
    
    RCLCPP_INFO(
      this->get_logger(),
      "Waiting %.2f s for ESP32 serial startup.",
      startup_delay_s_);
    
    std::this_thread::sleep_for(delay); // Pause this thread for the configured startup delay
    tcflush(serial_fd_, TCIOFLUSH);     // Clear any old serial data that arrived during startup
  }

  // Store the latest high-level command so it can be forwarded on the timer.
  // A Twist message contains linear and angular velocity components, which are
  // converted here into left and right wheel commands for differential drive.
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const auto pwm_pair = twistToPwm(*msg); // Convert the received Twist message into left and right PWM values

    std::lock_guard<std::mutex> lock(command_mutex_); // Protect the shared command variables while updating them

    // Store the new motor targets
    target_left_pwm_ = pwm_pair.first;
    target_right_pwm_ = pwm_pair.second;
    
    last_cmd_vel_time_ = std::chrono::steady_clock::now(); // Remember when the latest command was received
  }

  void wheelPwmCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) // Ignore messages that do not contain both left and right PWM values
    {
      RCLCPP_WARN(this->get_logger(), "Ignoring /wheel_pwm_cmd with fewer than 2 values.");
      return;
    }

    std::lock_guard<std::mutex> lock(command_mutex_); // Protect the shared command variables while updating them

    // Store the PWM values while keeping them within the allowed range
    target_left_pwm_ = std::clamp(static_cast<int>(msg->data[0]), -max_pwm_, max_pwm_);
    target_right_pwm_ = std::clamp(static_cast<int>(msg->data[1]), -max_pwm_, max_pwm_);
    
    last_cmd_vel_time_ = std::chrono::steady_clock::now(); // Remember when the latest command was received
  }

  // Convert a Twist message into left/right wheel PWM values.
  // The message's linear.x and angular.z components are interpreted as the
  // desired forward/backward motion and rotation of the robot. The node then
  // computes the required velocity for each wheel and maps that into a PWM
  // command suitable for the ESP32 firmware.
  std::pair<int, int> twistToPwm(const geometry_msgs::msg::Twist & twist) const
  {
    // Treat very small linear and angular commands as zero
    double linear_x = std::abs(twist.linear.x) < linear_deadband_mps_ ? 0.0 : twist.linear.x;
    double angular_z = std::abs(twist.angular.z) < angular_deadband_rps_ ? 0.0 : twist.angular.z;

    // Calculate the desired speed of each wheel for differential-drive motion
    const double left_wheel_mps = linear_x - (angular_z * wheel_base_m_ * 0.5);
    const double right_wheel_mps = linear_x + (angular_z * wheel_base_m_ * 0.5);

    // Convert wheel speeds to PWM values and apply motor-specific scaling
    const int left_pwm = scalePwm(wheelSpeedToPwm(left_wheel_mps), left_pwm_scale_);
    const int right_pwm = scalePwm(wheelSpeedToPwm(right_wheel_mps), right_pwm_scale_);

    return {left_pwm, right_pwm};
  }

  int wheelSpeedToPwm(double wheel_speed_mps) const
  {
    if (std::abs(wheel_speed_mps) < linear_deadband_mps_ || max_wheel_speed_mps_ <= 0.0) // Treat very small wheel speeds as zero
    {
      return 0;
    }

    int pwm = static_cast<int>(
      std::lround((wheel_speed_mps / max_wheel_speed_mps_) * static_cast<double>(max_pwm_))); // Map wheel speed proportionally to the configured PWM range
    
    pwm = std::clamp(pwm, -max_pwm_, max_pwm_); // Keep the PWM value within the allowed range

    if (pwm != 0 && std::abs(pwm) < min_pwm_) // Raise small non-zero commands to the configured minimum PWM
    {
      pwm = pwm > 0 ? min_pwm_ : -min_pwm_;
    }

    return pwm;
  }

  int scalePwm(int pwm, double scale) const
  {
    int scaled_pwm = static_cast<int>(
      std::lround(static_cast<double>(pwm) * scale)); // Multiply the PWM value by the configured scale factor
    
    scaled_pwm = std::clamp(scaled_pwm, -max_pwm_, max_pwm_); // Keep the scaled value within the allowed PWM range

    if (scaled_pwm != 0 && std::abs(scaled_pwm) < min_pwm_) // Raise small non-zero commands to the configured minimum PWM
    {
      return scaled_pwm > 0 ? min_pwm_ : -min_pwm_;
    }

    return scaled_pwm;
  }

  // Periodically transmit the most recent command, or stop the motors if the
  // command has timed out. This timer is the bridge between the ROS 2 command
  // stream and the periodic update rate expected by the ESP32 firmware.
  void sendCommandTimerCallback()
  {
    int left_pwm = 0;
    int right_pwm = 0;
    bool timed_out = false;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);                       // Protect the shared command variables while reading them
      const auto age = std::chrono::steady_clock::now() - last_cmd_vel_time_; // Check how long it has been since the latest motion command
      timed_out = age > std::chrono::duration<double>(command_timeout_s_);    // Mark the command as timed out if it is too old

      if (!timed_out) // Use the latest PWM targets only if the command is still valid
      {
        left_pwm = target_left_pwm_;
        right_pwm = target_right_pwm_;
      }
    }

    const bool had_nonzero_command = (last_sent_left_pwm_ != 0 || last_sent_right_pwm_ != 0); // Check whether the previously sent command was moving the robot

    sendMotorCommand(left_pwm, right_pwm); // Send the selected motor command to the ESP32

    if (timed_out && had_nonzero_command) // Print a warning when a moving command expires
    {
      RCLCPP_WARN(this->get_logger(), "Command timeout. Stopping motors.");
    }
  }

  // Send a full motor command to the ESP32 using the simple text protocol.
  // The firmware expects commands of the form "M left_pwm right_pwm", where the
  // values are signed PWM values for the left and right motors.
  void sendMotorCommand(int left_pwm, int right_pwm)
  {
    if (left_pwm == 0 && right_pwm == 0 && active_brake_on_stop_) // Use the ESP32 active-brake command when both motor commands are zero
    {
      sendStopCommand();
      return;
    }

    // Build the serial command in the form: M <left_pwm> <right_pwm>
    std::ostringstream line;
    line << "M " << left_pwm << " " << right_pwm;

    if (sendSerialLine(line.str())) // Send the command over serial
    {
      if (left_pwm != last_sent_left_pwm_ || right_pwm != last_sent_right_pwm_) // Print a message only when the commanded PWM values change
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Sent PWM command -> LEFT: %d, RIGHT: %d",
          left_pwm,
          right_pwm);
      }

      // Store the most recently sent PWM values
      last_sent_left_pwm_ = left_pwm;
      last_sent_right_pwm_ = right_pwm;
    }
  }

  void sendStopCommand()
  {
    if (last_sent_left_pwm_ == 0 && last_sent_right_pwm_ == 0) // Do nothing if the last sent command was already zero
    {
      return;
    }

    if (sendSerialLine("X"))  // Send the active-brake command
    {
      RCLCPP_INFO(this->get_logger(), "Sent active brake stop command.");

      // Store that both motors are now stopped
      last_sent_left_pwm_ = 0;
      last_sent_right_pwm_ = 0;
    }
  }

  // Write a newline-terminated command to the serial port and retry on transient
  // write errors.
  bool sendSerialLine(const std::string & line)
  {
    if (serial_fd_ < 0) // Stop if the serial port is not open
    {
      return false;
    }

    const std::string serial_line = line + "\n"; // Add the newline expected by the ESP32 command parser
    size_t total_bytes_written = 0;              // Keep track of how much of the command has already been written

    while (total_bytes_written < serial_line.size()) {

      // Try to write the remaining part of the command
      const ssize_t bytes_written = write(
        serial_fd_,
        serial_line.c_str() + total_bytes_written,
        serial_line.size() - total_bytes_written);

      if (bytes_written > 0) // Continue until the complete command has been written
      {
        total_bytes_written += static_cast<size_t>(bytes_written);
        continue;
      }

      if (bytes_written == 0) // Wait briefly if no bytes were written
      {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      if (errno == EINTR) // Retry if the write was interrupted
      {
        continue;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) // Retry shortly if the non-blocking serial port is temporarily busy
      {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      // Report any other serial write error
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write to serial port: %s",
        std::strerror(errno));
      return false;
    }

    return true; // The complete command was sent successfully
  }

  // Read firmware feedback asynchronously and split it into newline-delimited lines.
  void readSerialLoop()
  {
    std::string line_buffer; // Store received serial data until complete lines are available
    char buffer[256];        // Temporary buffer for each serial read

    while (running_) {
      if (serial_fd_ < 0) // Wait and retry if the serial port is not open
      {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1); // Read available bytes from the serial port

      if (bytes_read > 0) {
        buffer[bytes_read] = '\0'; // End the received data as a C-style string
        line_buffer += buffer;     // Add the new data to the existing incomplete line

        size_t newline_position = std::string::npos;

        while ((newline_position = line_buffer.find('\n')) != std::string::npos) // Process every complete newline-terminated line
        {
          std::string line = line_buffer.substr(0, newline_position); // Extract one complete line
          line_buffer.erase(0, newline_position + 1);                 // Remove the processed line from the buffer

          if (!line.empty() && line.back() == '\r') // Remove a carriage return if one is present
          {
            line.pop_back();
          }

          handleSerialLine(line); // Process the completed serial line
        }
      } 
      else if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) // Wait briefly if no serial data is currently available
      {
        std::this_thread::sleep_for(5ms);
      } 
      else if (errno == EINTR) // Retry if the read operation was interrupted
      {
        continue;
      } 
      else // Report other serial read errors
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to read from serial port: %s",
          std::strerror(errno));
        std::this_thread::sleep_for(50ms);
      }
    }
  }

  // Parse feedback from the ESP32 and publish encoder ticks as ROS topics.
  // The firmware sends lines such as "FEEDBACK | LEFT ticks = ... | RIGHT ticks = ...".
  // This node extracts those numbers and republishes them as standard Int64
  // messages so that odometry and other monitoring nodes can consume them.
  void handleSerialLine(const std::string & line)
  {
    static const std::regex feedback_regex(
      R"(FEEDBACK \| LEFT ticks = (-?\d+) \| RIGHT ticks = (-?\d+))"); // Match encoder feedback lines and capture the left and right tick values

    std::smatch match;

    if (std::regex_search(line, match, feedback_regex)) // Check whether the line contains encoder feedback
    {
      // Convert the captured tick values from text to integers
      std_msgs::msg::Int64 left_msg;
      std_msgs::msg::Int64 right_msg;

      // Publish the encoder counts as ROS 2 topics
      left_msg.data = std::stoll(match[1].str());
      right_msg.data = std::stoll(match[2].str());

      left_publisher_->publish(left_msg);
      right_publisher_->publish(right_msg);
      
    } 
    else if (!line.empty()) // Print other ESP32 messages to the ROS 2 log
    {
      RCLCPP_INFO(this->get_logger(), "ESP32: %s", line.c_str());
    }
  }
};

int main(int argc, char ** argv) // Start ROS 2 and run the base serial bridge node
{
  rclcpp::init(argc, argv);                                // Initialize ROS 2
  rclcpp::spin(std::make_shared<BaseSerialBridgeNode>());  // Create the node and keep processing ROS 2 callbacks
  rclcpp::shutdown();                                      // Shut down ROS 2 when the node stops
  return 0;
}
