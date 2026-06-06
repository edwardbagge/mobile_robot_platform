#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

class WheelVelocityControllerNode : public rclcpp::Node
{
public:
  WheelVelocityControllerNode()
  : Node("wheel_velocity_controller")
  {
    controller_rate_hz_ = this->declare_parameter<double>("controller_rate_hz", 20.0);
    command_timeout_s_ = this->declare_parameter<double>("command_timeout_s", 0.25);
    encoder_timeout_s_ = this->declare_parameter<double>("encoder_timeout_s", 0.25);
    wheel_base_m_ = this->declare_parameter<double>("wheel_base_m", 0.245);
    wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.041);
    ticks_per_revolution_ = this->declare_parameter<double>("ticks_per_revolution", 2800.0);
    max_wheel_speed_mps_ = this->declare_parameter<double>("max_wheel_speed_mps", 0.10);
    max_tick_jump_scale_ = this->declare_parameter<double>("max_tick_jump_scale", 3.0);
    max_pwm_ = this->declare_parameter<int>("max_pwm", 100);
    min_pwm_ = this->declare_parameter<int>("min_pwm", 85);
    kp_pwm_per_mps_ = this->declare_parameter<double>("kp_pwm_per_mps", 120.0);
    linear_deadband_mps_ = this->declare_parameter<double>("linear_deadband_mps", 0.005);
    angular_deadband_rps_ = this->declare_parameter<double>("angular_deadband_rps", 0.01);

    max_pwm_ = std::clamp(max_pwm_, 0, 255);
    min_pwm_ = std::clamp(min_pwm_, 0, max_pwm_);
    controller_rate_hz_ = std::max(controller_rate_hz_, 1.0);

    pwm_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_pwm_cmd", 10);

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&WheelVelocityControllerNode::cmdVelCallback, this, std::placeholders::_1));

    left_ticks_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/left_encoder_ticks",
      10,
      std::bind(&WheelVelocityControllerNode::leftTicksCallback, this, std::placeholders::_1));

    right_ticks_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/right_encoder_ticks",
      10,
      std::bind(&WheelVelocityControllerNode::rightTicksCallback, this, std::placeholders::_1));

    const auto period_s = 1.0 / controller_rate_hz_;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&WheelVelocityControllerNode::controlTimerCallback, this));

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
  double controller_rate_hz_;
  double command_timeout_s_;
  double encoder_timeout_s_;
  double wheel_base_m_;
  double wheel_radius_m_;
  double ticks_per_revolution_;
  double max_wheel_speed_mps_;
  double max_tick_jump_scale_;
  int max_pwm_;
  int min_pwm_;
  double kp_pwm_per_mps_;
  double linear_deadband_mps_;
  double angular_deadband_rps_;

  double target_left_mps_ = 0.0;
  double target_right_mps_ = 0.0;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};

  int64_t current_left_ticks_ = 0;
  int64_t current_right_ticks_ = 0;
  int64_t last_left_ticks_ = 0;
  int64_t last_right_ticks_ = 0;
  bool left_ticks_received_ = false;
  bool right_ticks_received_ = false;
  bool velocity_initialized_ = false;
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_left_ticks_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_ticks_time_{0, 0, RCL_ROS_TIME};

  std::mutex state_mutex_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_ticks_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_ticks_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double linear_x =
      std::abs(msg->linear.x) < linear_deadband_mps_ ? 0.0 : msg->linear.x;
    const double angular_z =
      std::abs(msg->angular.z) < angular_deadband_rps_ ? 0.0 : msg->angular.z;

    std::lock_guard<std::mutex> lock(state_mutex_);
    target_left_mps_ = linear_x - (angular_z * wheel_base_m_ * 0.5);
    target_right_mps_ = linear_x + (angular_z * wheel_base_m_ * 0.5);
    last_cmd_vel_time_ = this->get_clock()->now();
  }

  void leftTicksCallback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_left_ticks_ = msg->data;
    left_ticks_received_ = true;
    last_left_ticks_time_ = this->get_clock()->now();
  }

  void rightTicksCallback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_right_ticks_ = msg->data;
    right_ticks_received_ = true;
    last_right_ticks_time_ = this->get_clock()->now();
  }

  void controlTimerCallback()
  {
    const rclcpp::Time now = this->get_clock()->now();
    double target_left_mps = 0.0;
    double target_right_mps = 0.0;
    double actual_left_mps = 0.0;
    double actual_right_mps = 0.0;
    bool publish_stop = false;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (!left_ticks_received_ || !right_ticks_received_) {
        publish_stop = true;
      } else {
        const bool encoder_timed_out =
          (now - last_left_ticks_time_).seconds() > encoder_timeout_s_ ||
          (now - last_right_ticks_time_).seconds() > encoder_timeout_s_;

        if (encoder_timed_out) {
          velocity_initialized_ = false;
          publish_stop = true;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Encoder feedback timed out. Stopping wheel controller output.");
        } else if (!velocity_initialized_) {
          last_left_ticks_ = current_left_ticks_;
          last_right_ticks_ = current_right_ticks_;
          last_control_time_ = now;
          velocity_initialized_ = true;
          publish_stop = true;
        } else {
          const double dt = (now - last_control_time_).seconds();
          if (dt <= 0.0) {
            return;
          }

          const int64_t delta_left_ticks = current_left_ticks_ - last_left_ticks_;
          const int64_t delta_right_ticks = current_right_ticks_ - last_right_ticks_;

          if (tickJumpLooksInvalid(delta_left_ticks, dt) || tickJumpLooksInvalid(delta_right_ticks, dt)) {
            last_left_ticks_ = current_left_ticks_;
            last_right_ticks_ = current_right_ticks_;
            last_control_time_ = now;
            publish_stop = true;
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              2000,
              "Ignoring implausible encoder tick jump. Resetting controller velocity state.");
          } else {
            actual_left_mps = ticksToMeters(delta_left_ticks) / dt;
            actual_right_mps = ticksToMeters(delta_right_ticks) / dt;

            const bool timed_out =
              (now - last_cmd_vel_time_).seconds() > command_timeout_s_;

            if (!timed_out) {
              target_left_mps = target_left_mps_;
              target_right_mps = target_right_mps_;
            }

            last_left_ticks_ = current_left_ticks_;
            last_right_ticks_ = current_right_ticks_;
            last_control_time_ = now;
          }
        }
      }
    }

    if (publish_stop) {
      publishPwmCommand(0, 0);
      return;
    }

    const int left_pwm = computePwm(target_left_mps, actual_left_mps);
    const int right_pwm = computePwm(target_right_mps, actual_right_mps);
    publishPwmCommand(left_pwm, right_pwm);
  }

  double ticksToMeters(int64_t ticks) const
  {
    return static_cast<double>(ticks) *
      (2.0 * M_PI * wheel_radius_m_) / ticks_per_revolution_;
  }

  bool tickJumpLooksInvalid(int64_t delta_ticks, double dt) const
  {
    if (dt <= 0.0 || ticks_per_revolution_ <= 0.0 || wheel_radius_m_ <= 0.0) {
      return true;
    }

    const double max_ticks =
      (std::abs(max_wheel_speed_mps_) * std::max(max_tick_jump_scale_, 1.0) * dt) /
      ticksToMeters(1);
    return static_cast<double>(std::abs(delta_ticks)) > std::max(1.0, max_ticks);
  }

  int feedforwardPwm(double target_mps) const
  {
    if (std::abs(target_mps) < linear_deadband_mps_ || max_wheel_speed_mps_ <= 0.0) {
      return 0;
    }

    int pwm = static_cast<int>(
      std::lround((target_mps / max_wheel_speed_mps_) * static_cast<double>(max_pwm_)));
    pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
    return enforceMinPwm(pwm);
  }

  int computePwm(double target_mps, double actual_mps) const
  {
    if (std::abs(target_mps) < linear_deadband_mps_) {
      return 0;
    }

    const int base_pwm = feedforwardPwm(target_mps);
    const double error_mps = target_mps - actual_mps;
    int pwm = base_pwm + static_cast<int>(std::lround(kp_pwm_per_mps_ * error_mps));
    pwm = std::clamp(pwm, -max_pwm_, max_pwm_);
    return enforceMinPwm(pwm);
  }

  int enforceMinPwm(int pwm) const
  {
    if (pwm != 0 && std::abs(pwm) < min_pwm_) {
      return pwm > 0 ? min_pwm_ : -min_pwm_;
    }
    return pwm;
  }

  void publishPwmCommand(int left_pwm, int right_pwm)
  {
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {left_pwm, right_pwm};
    pwm_publisher_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocityControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
