// wheel_velocity_controller.cpp
// ROS 2 wheel velocity controller for the differential-drive robot.
//
// Uses /cmd_vel and wheel encoder feedback to calculate left and right
// motor PWM commands, which are published on /wheel_pwm_cmd.

// Standard C++ libraries used for calculations, timing, memory, and thread safety
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

class WheelVelocityControllerNode : public rclcpp::Node // ROS 2 node that control wheel velocity using encoder feedback
{
public:
  // The constructor sets up the subscriptions, publisher, and periodic control
  // timer. The node then runs continuously, updating the motor commands at a
  // fixed frequency based on the latest feedback.
  WheelVelocityControllerNode()
  : Node("wheel_velocity_controller")
  {
    controller_rate_hz_ = this->declare_parameter<double>("controller_rate_hz", 20.0);        // Number of control-loop updates per second
    command_timeout_s_ = this->declare_parameter<double>("command_timeout_s", 0.25);          // Stop using a /cmd_vel command if it is older than this
    encoder_timeout_s_ = this->declare_parameter<double>("encoder_timeout_s", 0.25);          // Stop the controller if encoder feedback is older than this
    wheel_base_m_ = this->declare_parameter<double>("wheel_base_m", 0.245);                   // Distance between the left and right wheels
    wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.041);               // Radius of the drive wheels
    ticks_per_revolution_ = this->declare_parameter<double>("ticks_per_revolution", 2800.0);  // Number of encoder ticks produced by one wheel revolution
    max_wheel_speed_mps_ = this->declare_parameter<double>("max_wheel_speed_mps", 0.10);      // Maximum expected wheel speed
    max_tick_jump_scale_ = this->declare_parameter<double>("max_tick_jump_scale", 3.0);       // Allowed multiple of the expected maximum encoder change
    max_pwm_ = this->declare_parameter<int>("max_pwm", 100);                                  // Maximum PWM value the controller can command
    min_pwm_ = this->declare_parameter<int>("min_pwm", 85);                                   // Minimum non-zero PWM value
    kp_pwm_per_mps_ = this->declare_parameter<double>("kp_pwm_per_mps", 120.0);               // Proportional gain used to correct wheel-speed error
    linear_deadband_mps_ = this->declare_parameter<double>("linear_deadband_mps", 0.005);     // Treat very small linear velocity commands as zero
    angular_deadband_rps_ = this->declare_parameter<double>("angular_deadband_rps", 0.01);    // Treat very small angular velocity commands as zero

    max_pwm_ = std::clamp(max_pwm_, 0, 255);                   // Keep the maximum PWM within the 8-bit range supported by the ESP32
    min_pwm_ = std::clamp(min_pwm_, 0, max_pwm_);              // Keep the minimum PWM between zero and the configured maximum
    controller_rate_hz_ = std::max(controller_rate_hz_, 1.0);  // Prevent the control-loop rate from going below 1 Hz

    // This publisher sends the computed left/right PWM values to the serial
    // bridge, which forwards them to the ESP32 firmware.
    pwm_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_pwm_cmd", 10);

    // Receive the high-level motion command from the navigation or teleop layer.
    // A Twist message contains a desired linear velocity and angular velocity.
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&WheelVelocityControllerNode::cmdVelCallback, this, std::placeholders::_1));

    left_ticks_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/left_encoder_ticks",
      10,
      std::bind(&WheelVelocityControllerNode::leftTicksCallback, this, std::placeholders::_1)); // Subscribe to left encoder tick feedback

    right_ticks_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/right_encoder_ticks",
      10,
      std::bind(&WheelVelocityControllerNode::rightTicksCallback, this, std::placeholders::_1)); // Suscribe to right encoder tick feedback

    const auto period_s = 1.0 / controller_rate_hz_; // Calculate the time between controller updates
    
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&WheelVelocityControllerNode::controlTimerCallback, this)); // Run the control loop repeatedly at the configured rate

    RCLCPP_INFO(this->get_logger(), "Wheel velocity controller started.");
    RCLCPP_INFO(this->get_logger(), "controller_rate_hz = %.2f", controller_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "wheel_base_m = %.3f", wheel_base_m_);
    RCLCPP_INFO(this->get_logger(), "wheel_radius_m = %.3f", wheel_radius_m_);
    RCLCPP_INFO(this->get_logger(), "ticks_per_revolution = %.1f", ticks_per_revolution_);
    RCLCPP_INFO(this->get_logger(), "encoder_timeout_s = %.2f", encoder_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "max_tick_jump_scale = %.2f", max_tick_jump_scale_);
    RCLCPP_INFO(this->get_logger(), "max_pwm = %d", max_pwm_);
    RCLCPP_INFO(this->get_logger(), "min_pwm = %d", min_pwm_);
    RCLCPP_INFO(this->get_logger(), "kp_pwm_per_mps = %.2f", kp_pwm_per_mps_);
  }

private:
  // Controller timing parameters
  double controller_rate_hz_;
  double command_timeout_s_;
  double encoder_timeout_s_;

  // Robot geometry and encoder parameters
  double wheel_base_m_;
  double wheel_radius_m_;
  double ticks_per_revolution_;

  // Limits used by the controller and encoder validity check
  double max_wheel_speed_mps_;
  double max_tick_jump_scale_;
  int max_pwm_;
  int min_pwm_;

  double kp_pwm_per_mps_; // Proportional feedback gain

  // Ignore very small commanded velocities
  double linear_deadband_mps_;
  double angular_deadband_rps_;

  // Desired left and right wheel speeds
  double target_left_mps_ = 0.0;
  double target_right_mps_ = 0.0;

  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME}; // Time when the latest /cmd_vel command was received

  // Latest encoder tick counts 
  int64_t current_left_ticks_ = 0;
  int64_t current_right_ticks_ = 0;

  // Encoder tick counts used during                                 
  int64_t last_left_ticks_ = 0;
  int64_t last_right_ticks_ = 0;

  // Track whether encoder data has been received                                  
  bool left_ticks_received_ = false;
  bool right_ticks_received_ = false;
                                  
  bool velocity_initialized_ = false; // Track whether the first encoder reference values have been stored
  
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME}; // Time of the previous controller updates

  // Time when the latest left and right encoder messages was received
  rclcpp::Time last_left_ticks_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_ticks_time_{0, 0, RCL_ROS_TIME};

  std::mutex state_mutex_; // Protect shared controller and encoder state

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_publisher_;      // Publisher for left and right PWM commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_; // Subscription to /cmd_vel
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_ticks_subscription_;   // Subscription to left encoder tick feedback
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_ticks_subscription_;  // Subscription to right encoder tick feedback
  rclcpp::TimerBase::SharedPtr timer_;                                              // Timer used to run the controller periodically

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) // Convert the desired robot motion into a target speed for each wheel
  {
    const double linear_x =
      std::abs(msg->linear.x) < linear_deadband_mps_ ? 0.0 : msg->linear.x;    // Treat very small linear velocity commands as zero
    const double angular_z =
      std::abs(msg->angular.z) < angular_deadband_rps_ ? 0.0 : msg->angular.z; // Treat very small angular velocity commands as zero

    std::lock_guard<std::mutex> lock(state_mutex_); // Protect the shared controller state while updating it

    // Convert linear and angular velocity into left and right wheel target speeds
    target_left_mps_ = linear_x - (angular_z * wheel_base_m_ * 0.5);  
    target_right_mps_ = linear_x + (angular_z * wheel_base_m_ * 0.5);
    
    last_cmd_vel_time_ = this->get_clock()->now(); // Remember when the latest /cmd_vel command was received
  }

  void leftTicksCallback(const std_msgs::msg::Int64::SharedPtr msg) // Store the latest left encoder count and when it was received
  {
    std::lock_guard<std::mutex> lock(state_mutex_); // Protect the shared controller state while updating it
    
    current_left_ticks_ = msg->data;
    left_ticks_received_ = true;
    last_left_ticks_time_ = this->get_clock()->now();
  }

  void rightTicksCallback(const std_msgs::msg::Int64::SharedPtr msg) // Store the latest right encoder count and when it was received
  {
    std::lock_guard<std::mutex> lock(state_mutex_); // Protect the shared controller state while updating it
    
    current_right_ticks_ = msg->data;
    right_ticks_received_ = true;
    last_right_ticks_time_ = this->get_clock()->now();
  }

  // This is the main control loop. It repeatedly compares the desired wheel
  // speed with the measured wheel speed and adjusts the PWM output accordingly.
  void controlTimerCallback()
  {
    const rclcpp::Time now = this->get_clock()->now(); // Get the current ROS 2 time

    // Local copies of the target and measured wheel speeds
    double target_left_mps = 0.0;
    double target_right_mps = 0.0;
    double actual_left_mps = 0.0;
    double actual_right_mps = 0.0;
    
    bool publish_stop = false; // Becomes true if the controller should command both motors to stop

    {
      std::lock_guard<std::mutex> lock(state_mutex_); // Protect the shared controller and encoder state while reading or updating it

      if (!left_ticks_received_ || !right_ticks_received_) // Stop until encoder feedback has been received from both wheels
      {
        publish_stop = true;
      } 
      else 
      {
        const bool encoder_timed_out =
          (now - last_left_ticks_time_).seconds() > encoder_timeout_s_ ||
          (now - last_right_ticks_time_).seconds() > encoder_timeout_s_;    // Check whether either encoder has stopped providing recent feedback

        if (encoder_timed_out) // Stop the controller if encoder feedback is too old
        {
          velocity_initialized_ = false;
          publish_stop = true;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Encoder feedback timed out. Stopping wheel controller output.");
        } 
        else if (!velocity_initialized_) // Store the first encoder values as the starting reference 
        {
          last_left_ticks_ = current_left_ticks_;
          last_right_ticks_ = current_right_ticks_;
          last_control_time_ = now;
          velocity_initialized_ = true;
          
          publish_stop = true; // Do NOT command motion until a valid speed measurement can be calculated
        } 
        else
        {
          const double dt = (now - last_control_time_).seconds(); // Calculate the time since the previous controller update
          
          if (dt <= 0.0) // Ignore invalid time intervals
          {
            return;
          }

          // Calculate how many encoder ticks each wheel moved since the previous update
          const int64_t delta_left_ticks = current_left_ticks_ - last_left_ticks_;
          const int64_t delta_right_ticks = current_right_ticks_ - last_right_ticks_;

          if (tickJumpLooksInvalid(delta_left_ticks, dt) || tickJumpLooksInvalid(delta_right_ticks, dt)) // Reject unrealistic encoder changes
          {
            // Reset the encoder reference values so the bad jump is not reused
            last_left_ticks_ = current_left_ticks_;
            last_right_ticks_ = current_right_ticks_;
            last_control_time_ = now;
            
            publish_stop = true; // Stop the motors for this control update
            
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              2000,
              "Ignoring implausible encoder tick jump. Resetting controller velocity state.");
          } 
          else 
          {
            // Convert encoder movement into actual wheel speeds
            actual_left_mps = ticksToMeters(delta_left_ticks) / dt;
            actual_right_mps = ticksToMeters(delta_right_ticks) / dt;

            const bool timed_out =
              (now - last_cmd_vel_time_).seconds() > command_timeout_s_; // Check whether the latest /cmd_vel command is too old

            if (!timed_out) // Use the target wheel speeds only while the command is still valid
            {
              target_left_mps = target_left_mps_;
              target_right_mps = target_right_mps_;
            }

            // Store the current encoder values and time for the next control update
            last_left_ticks_ = current_left_ticks_;
            last_right_ticks_ = current_right_ticks_;
            last_control_time_ = now;
          }
        }
      }
    }

    if (publish_stop) // Stop both motors immediately if the controller detected a problem
    {
      publishPwmCommand(0, 0);
      return;
    }

    // Calculate the final PWM command for each wheel
    const int left_pwm = computePwm(target_left_mps, actual_left_mps);
    const int right_pwm = computePwm(target_right_mps, actual_right_mps);
    
    publishPwmCommand(left_pwm, right_pwm); // Publish the PWM commands for the serial bridge
  }

  // Convert an encoder tick change into wheel travel distance in meters
  double ticksToMeters(int64_t ticks) const
  {
    return static_cast<double>(ticks) *
      (2.0 * M_PI * wheel_radius_m_) / ticks_per_revolution_;
  }

  // Check whether an encoder tick change is unrealistically large
  bool tickJumpLooksInvalid(int64_t delta_ticks, double dt) const
  {    
    if (dt <= 0.0 || ticks_per_revolution_ <= 0.0 || wheel_radius_m_ <= 0.0) // Treat invalid timing or wheel parameters as an invalid result
    {
      return true;
    }

    const double max_ticks =
      (std::abs(max_wheel_speed_mps_) * std::max(max_tick_jump_scale_, 1.0) * dt) /
      ticksToMeters(1); // Calculate the largest believable encoder change for this time interval
    
    return static_cast<double>(std::abs(delta_ticks)) > std::max(1.0, max_ticks); // Return true if the measured tick change is larger than the allowed limit
  }

  // Create a base PWM command from the desired wheel speed. This gives the
  // motor a first approximation of the needed effort before any correction is
  // applied based on feedback.
  int feedforwardPwm(double target_mps) const 
  {
    if (std::abs(target_mps) < linear_deadband_mps_ || max_wheel_speed_mps_ <= 0.0) // Return zero for very small target speeds or an invalid maximum speed
    {
      return 0;
    }

    int pwm = static_cast<int>(
      std::lround((target_mps / max_wheel_speed_mps_) * static_cast<double>(max_pwm_))); // Map the target wheel speed proportionally to the PWM range
    pwm = std::clamp(pwm, -max_pwm_, max_pwm_);                                          // Keep the PWM value within the allowed range
    return enforceMinPwm(pwm);                                                           // Apply the configured minimum non-zero PWM
  }

  // Compute the final PWM command by combining a feedforward term with a
  // feedback correction term. The feedback part reduces the difference between
  // the target speed and the measured speed.
  int computePwm(double target_mps, double actual_mps) const
  {
    if (std::abs(target_mps) < linear_deadband_mps_) // Stop the wheel when the target speed is within the deadband
    {
      return 0;
    }

    const int base_pwm = feedforwardPwm(target_mps);                                  // Calculate the initial PWM from the target speed
    const double error_mps = target_mps - actual_mps;                                 // Calculate the difference between target and measured wheel speed
    int pwm = base_pwm + static_cast<int>(std::lround(kp_pwm_per_mps_ * error_mps));  // Correct the PWM according to the speed error
    pwm = std::clamp(pwm, -max_pwm_, max_pwm_);                                       // Keep the final PWM within the allowed range
    return enforceMinPwm(pwm);                                                        // Apply the configured minimum non-zero PWM
  }

  int enforceMinPwm(int pwm) const // Make sure any non-zero PWM command is strong enough to move the motor
  {
    // If the command is non-zero but below the minimum usable PWM,
    // increase it to the configured minimum while keeping its direction
    if (pwm != 0 && std::abs(pwm) < min_pwm_) 
    {
      return pwm > 0 ? min_pwm_ : -min_pwm_;
    }
    
    return pwm; // Leave zero and sufficiently large PWM values unchanged
  }

  // Publish the computed left/right PWM values as a two-element array.
  // The serial bridge subscribes to this topic and forwards the values to the
  // ESP32 firmware over serial.
  void publishPwmCommand(int left_pwm, int right_pwm)
  {
    std_msgs::msg::Int32MultiArray msg;  // Create a ROS 2 message for the two wheel commands
    msg.data = {left_pwm, right_pwm};    // Store the commands in the order: left wheel, right wheel
    pwm_publisher_->publish(msg);        // Publish the PWM commands to the wheel controller topic
  }
};

int main(int argc, char ** argv) // Start ROS 2 and run the wheel velocity controller node
{
  rclcpp::init(argc, argv);                                     // Initialize ROS 2
  auto node = std::make_shared<WheelVelocityControllerNode>();  // Create the wheel velocity controller node
  rclcpp::spin(node);                                           // Keep the node running and process its callbacks 
  rclcpp::shutdown();                                           // Shut down ROS 2 when the node stops
  return 0;
}
