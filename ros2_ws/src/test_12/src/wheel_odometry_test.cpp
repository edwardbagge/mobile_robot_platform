#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>

class WheelOdometryTestNode : public rclcpp::Node
{
public:
  WheelOdometryTestNode()
  : Node("wheel_odometry_test")
  {
    ticks_per_revolution_ = this->declare_parameter<double>("ticks_per_revolution", 700.0);
    wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.03);
    wheel_base_m_ = this->declare_parameter<double>("wheel_base_m", 0.16);
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    left_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/left_encoder_ticks",
      10,
      std::bind(&WheelOdometryTestNode::leftTicksCallback, this, std::placeholders::_1));

    right_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/right_encoder_ticks",
      10,
      std::bind(&WheelOdometryTestNode::rightTicksCallback, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&WheelOdometryTestNode::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "Wheel odometry test started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to /left_encoder_ticks and /right_encoder_ticks");
    RCLCPP_INFO(this->get_logger(), "Publishing /odom");
    RCLCPP_INFO(this->get_logger(), "ticks_per_revolution = %.3f", ticks_per_revolution_);
    RCLCPP_INFO(this->get_logger(), "wheel_radius_m = %.3f", wheel_radius_m_);
    RCLCPP_INFO(this->get_logger(), "wheel_base_m = %.3f", wheel_base_m_);
    RCLCPP_INFO(this->get_logger(), "publish_tf = %s", publish_tf_ ? "true" : "false");
  }

private:
  double ticks_per_revolution_;
  double wheel_radius_m_;
  double wheel_base_m_;
  bool publish_tf_;
  double publish_rate_hz_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  int64_t current_left_ticks_ = 0;
  int64_t current_right_ticks_ = 0;
  int64_t last_processed_left_ticks_ = 0;
  int64_t last_processed_right_ticks_ = 0;

  bool left_ticks_received_ = false;
  bool right_ticks_received_ = false;
  bool odom_initialized_ = false;

  double x_m_ = 0.0;
  double y_m_ = 0.0;
  double yaw_rad_ = 0.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void leftTicksCallback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    current_left_ticks_ = msg->data;
    left_ticks_received_ = true;
  }

  void rightTicksCallback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    current_right_ticks_ = msg->data;
    right_ticks_received_ = true;
  }

  void publishOdometry()
  {
    if (!left_ticks_received_ || !right_ticks_received_) {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();

    if (!odom_initialized_) {
      last_processed_left_ticks_ = current_left_ticks_;
      last_processed_right_ticks_ = current_right_ticks_;
      last_publish_time_ = now;
      odom_initialized_ = true;
      return;
    }

    const double dt = (now - last_publish_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    const int64_t delta_left_ticks = current_left_ticks_ - last_processed_left_ticks_;
    const int64_t delta_right_ticks = current_right_ticks_ - last_processed_right_ticks_;

    const double meters_per_tick =
      (2.0 * M_PI * wheel_radius_m_) / ticks_per_revolution_;

    const double left_distance_m = static_cast<double>(delta_left_ticks) * meters_per_tick;
    const double right_distance_m = static_cast<double>(delta_right_ticks) * meters_per_tick;
    const double center_distance_m = 0.5 * (left_distance_m + right_distance_m);
    const double delta_yaw_rad = (right_distance_m - left_distance_m) / wheel_base_m_;

    // Use midpoint heading to reduce integration error during turns.
    const double heading_mid_rad = yaw_rad_ + (0.5 * delta_yaw_rad);
    x_m_ += center_distance_m * std::cos(heading_mid_rad);
    y_m_ += center_distance_m * std::sin(heading_mid_rad);
    yaw_rad_ = normalizeAngle(yaw_rad_ + delta_yaw_rad);

    const double linear_velocity_mps = center_distance_m / dt;
    const double angular_velocity_rps = delta_yaw_rad / dt;

    publishOdometryMessage(now, linear_velocity_mps, angular_velocity_rps);

    if (publish_tf_) {
      publishTransform(now);
    }

    last_processed_left_ticks_ = current_left_ticks_;
    last_processed_right_ticks_ = current_right_ticks_;
    last_publish_time_ = now;
  }

  void publishOdometryMessage(
    const rclcpp::Time & stamp,
    double linear_velocity_mps,
    double angular_velocity_rps)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = x_m_;
    odom_msg.pose.pose.position.y = y_m_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = yawToQuaternion(yaw_rad_);

    odom_msg.twist.twist.linear.x = linear_velocity_mps;
    odom_msg.twist.twist.angular.z = angular_velocity_rps;

    odom_publisher_->publish(odom_msg);
  }

  void publishTransform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = odom_frame_id_;
    transform_msg.child_frame_id = base_frame_id_;

    transform_msg.transform.translation.x = x_m_;
    transform_msg.transform.translation.y = y_m_;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = yawToQuaternion(yaw_rad_);

    tf_broadcaster_->sendTransform(transform_msg);
  }

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad) const
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_rad);

    geometry_msgs::msg::Quaternion msg;
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();
    msg.w = quaternion.w();
    return msg;
  }

  double normalizeAngle(double angle_rad) const
  {
    while (angle_rad > M_PI) {
      angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
      angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelOdometryTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
