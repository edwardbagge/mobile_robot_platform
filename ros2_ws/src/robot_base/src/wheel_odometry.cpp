// wheel_odometry.cpp
// ROS 2 odometry node for the differential-drive robot.
//
// Uses left and right wheel encoder counts to estimate the robot's position,
// orientation, and velocity, and publishes the result as /odom.

// Standard C++ libraries used for calculations, memory management, and strings
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class WheelOdometryNode : public rclcpp::Node // ROS 2 node that calculates odometry from wheel encoder counts
{
public:
  // The constructor sets up the subscriptions, publisher, timer, and optional TF
  // broadcaster. The node then updates the robot pose at a fixed rate using the
  // latest encoder information.
  WheelOdometryNode()
  : Node("wheel_odometry")
  {
    ticks_per_revolution_ = this->declare_parameter<double>("ticks_per_revolution", 700.0); // Number of encoder ticks produced by one full wheel revolution
    wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.03);              // Radius of each drive wheel in meters
    wheel_base_m_ = this->declare_parameter<double>("wheel_base_m", 0.16);                  // Distance between the left and right wheels
    max_wheel_speed_mps_ = this->declare_parameter<double>("max_wheel_speed_mps", 0.10);    // Maximum expected wheel speed
    max_tick_jump_scale_ = this->declare_parameter<double>("max_tick_jump_scale", 3.0);     // Allow encoder changes up to this multiple of the expected maximum
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);                        // Choose whether to publish the odom -> base_link TF transform
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);            // Number of odometry updates published per second
    publish_rate_hz_ = std::max(publish_rate_hz_, 1.0);                                     // Prevent the publishing rate from going below 1 Hz
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");         // Name of the odometry reference frame
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");    // Name of the robot body frame

    // Publish the estimated robot state as a standard ROS 2 odometry message.
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    if (publish_tf_) // Create the TF broadcaster if TF publishing is enabled
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    left_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/left_encoder_ticks",
      10,
      std::bind(&WheelOdometryNode::leftTicksCallback, this, std::placeholders::_1)); // Subscribe to left encoder tick counts

    right_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/right_encoder_ticks",
      10,
      std::bind(&WheelOdometryNode::rightTicksCallback, this, std::placeholders::_1)); // Subscribe to right encoder tick counts

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);// Calculate the time between odometry updates
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&WheelOdometryNode::publishOdometry, this)); // Call publishOdometry repeatedly at the configured rate

    RCLCPP_INFO(this->get_logger(), "Wheel odometry started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to /left_encoder_ticks and /right_encoder_ticks");
    RCLCPP_INFO(this->get_logger(), "Publishing /odom");
    RCLCPP_INFO(this->get_logger(), "ticks_per_revolution = %.3f", ticks_per_revolution_);
    RCLCPP_INFO(this->get_logger(), "wheel_radius_m = %.3f", wheel_radius_m_);
    RCLCPP_INFO(this->get_logger(), "wheel_base_m = %.3f", wheel_base_m_);
    RCLCPP_INFO(this->get_logger(), "max_wheel_speed_mps = %.3f", max_wheel_speed_mps_);
    RCLCPP_INFO(this->get_logger(), "max_tick_jump_scale = %.2f", max_tick_jump_scale_);
    RCLCPP_INFO(this->get_logger(), "publish_tf = %s", publish_tf_ ? "true" : "false");
  }

private:

  // Encoder and robot geomtry parameters
  double ticks_per_revolution_;
  double wheel_radius_m_;
  double wheel_base_m_;

  // Limits used for rejecting unrealistic encoder jumps
  double max_wheel_speed_mps_;
  double max_tick_jump_scale_;

  // Odometry publishing settings
  bool publish_tf_;
  double publish_rate_hz_;

  // Names of the coordinate frames used for odometry
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // Latest encoder tick counts received
  int64_t current_left_ticks_ = 0;
  int64_t current_right_ticks_ = 0;

  // Encoder counts used during the previous odometry update
  int64_t last_processed_left_ticks_ = 0;
  int64_t last_processed_right_ticks_ = 0;

  // Track whether encoder data has been received
  bool left_ticks_received_ = false;
  bool right_ticks_received_ = false;

  bool odom_initialized_ = false; // Track whether the first odometry reference values have been stored

  // Current estimated robot position and heading
  double x_m_ = 0.0;
  double y_m_ = 0.0;
  double yaw_rad_ = 0.0;

  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME}; // Time of the previous odometry update
                                  
  // ROS 2 subscriptions for encoder tick counts
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_subscription_;
                               
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_; // ROS 2 publisher for odometry 
  rclcpp::TimerBase::SharedPtr timer_;                                   // Timer used for periodic odometry updates
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;        // Optional broadcaster for the odom -> base_link transform

  void leftTicksCallback(const std_msgs::msg::Int64::SharedPtr msg) // Store the latest encoder count for the left wheel
  {
    current_left_ticks_ = msg->data;
    left_ticks_received_ = true;
  }

  void rightTicksCallback(const std_msgs::msg::Int64::SharedPtr msg) // Store the latest encoder count for the right wheel
  {
    current_right_ticks_ = msg->data;
    right_ticks_received_ = true;
  }

  // Main odometry update loop. It computes how much the robot moved since the
  // previous update and then updates the estimated position and heading.
  void publishOdometry()
  {
    if (!left_ticks_received_ || !right_ticks_received_) // Wait until both encoder topics have provided data
    {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now(); // Get the current ROS 2 time

    if (!odom_initialized_) // Store the first encoder values as the starting reference
    {
      last_processed_left_ticks_ = current_left_ticks_;
      last_processed_right_ticks_ = current_right_ticks_;
      last_publish_time_ = now;
      odom_initialized_ = true;
      return;
    }

    const double dt = (now - last_publish_time_).seconds(); // Calculate the time since the previous odometry update
    
    if (dt <= 0.0) // Ignore invalid time intervals
    {
      return;
    }

    // The difference between the current encoder count and the last processed
    // count gives the wheel movement since the previous update.
    const int64_t delta_left_ticks = current_left_ticks_ - last_processed_left_ticks_;
    const int64_t delta_right_ticks = current_right_ticks_ - last_processed_right_ticks_;

    if (tickJumpLooksInvalid(delta_left_ticks, dt) || tickJumpLooksInvalid(delta_right_ticks, dt)) // Reject unrealistically large encoder changes
    {
      // Reset the reference counts so the bad jump is not used later
      last_processed_left_ticks_ = current_left_ticks_;
      last_processed_right_ticks_ = current_right_ticks_;
      last_publish_time_ = now;

      // Print a warning, but not more often than once every two seconds
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Ignoring implausible encoder tick jump. Resetting odometry delta state.");

      publishOdometryMessage(now, 0.0, 0.0); // Publish the current pose with zero velocity
      
      if (publish_tf_) 
      {
        publishTransform(now); // Publish the current TF transform if enabled
      }
      
      return;
    }

    const double meters_per_tick =
      (2.0 * M_PI * wheel_radius_m_) / ticks_per_revolution_;

    // Convert encoder tick changes into physical wheel distances.
    // The average of the two wheel distances gives the robot's forward travel,
    // while the difference between them gives the change in heading.
    const double left_distance_m = static_cast<double>(delta_left_ticks) * meters_per_tick;
    const double right_distance_m = static_cast<double>(delta_right_ticks) * meters_per_tick;
    const double center_distance_m = 0.5 * (left_distance_m + right_distance_m);
    const double delta_yaw_rad = (right_distance_m - left_distance_m) / wheel_base_m_;

    // Use the midpoint heading to reduce integration error during turns.
    const double heading_mid_rad = yaw_rad_ + (0.5 * delta_yaw_rad);
    
    // Update the estimated pose in the world frame based on the robot's motion.
    // The robot moves forward along its current heading and rotates by the
    // computed yaw change.
    x_m_ += center_distance_m * std::cos(heading_mid_rad);
    y_m_ += center_distance_m * std::sin(heading_mid_rad);
    yaw_rad_ = normalizeAngle(yaw_rad_ + delta_yaw_rad);

    const double linear_velocity_mps = center_distance_m / dt;
    const double angular_velocity_rps = delta_yaw_rad / dt;

    publishOdometryMessage(now, linear_velocity_mps, angular_velocity_rps); // Publish the updated odometry message

    if (publish_tf_) // Publish the odom -> base_link transform if enabled
    {
      publishTransform(now);
    }

    // STore the current encoder counts for the next update
    last_processed_left_ticks_ = current_left_ticks_;
    last_processed_right_ticks_ = current_right_ticks_;
    
    last_publish_time_ = now; // Store the current time for the next velocity calculation
  }

  // Publish the estimated pose and velocity in the standard nav_msgs/Odometry
  // format so that navigation and visualization tools can consume it.
  void publishOdometryMessage(
    const rclcpp::Time & stamp,
    double linear_velocity_mps,
    double angular_velocity_rps)
  {
    nav_msgs::msg::Odometry odom_msg;

    // Set the message timestamp and coordinate frames
    odom_msg.header.stamp = stamp;             
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    // Store the estimated robot position
    odom_msg.pose.pose.position.x = x_m_;
    odom_msg.pose.pose.position.y = y_m_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = yawToQuaternion(yaw_rad_); // Store the estimated robot orientation

    // Store the estimated linear and angular velocities
    odom_msg.twist.twist.linear.x = linear_velocity_mps;
    odom_msg.twist.twist.angular.z = angular_velocity_rps;

    odom_publisher_->publish(odom_msg); // Publish the completed odometry message
  }

  // Publish the TF transform that links the odom frame to the robot body frame.
  // This allows other ROS 2 nodes to reason about the robot pose in a spatial
  // coordinate system.
  void publishTransform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform_msg;

    // Set the timestamp and coordinate frames
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = odom_frame_id_;
    transform_msg.child_frame_id = base_frame_id_;

    // Store the robot position in the transform
    transform_msg.transform.translation.x = x_m_;
    transform_msg.transform.translation.y = y_m_;
    transform_msg.transform.translation.z = 0.0;
    
    transform_msg.transform.rotation = yawToQuaternion(yaw_rad_); // Store the robot orientation in the transform

    tf_broadcaster_->sendTransform(transform_msg); // Broad the transform to the ROS 2 TF system
  }

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad) const // Convert the robot's yaw angle into a quaternion for ROS 2 messages
  {
    tf2::Quaternion quaternion;
    
    quaternion.setRPY(0.0, 0.0, yaw_rad); // Create a rotation with zero roll, zero pitch, and the current yaw angle

    // Copy the quaternion values into a ROS 2 message
    geometry_msgs::msg::Quaternion msg;
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();
    msg.w = quaternion.w();
    
    return msg;
  }

  // Reject sudden, unrealistic jumps in encoder counts so that bad feedback does
  // not create large fake motions in the odometry estimate.
  bool tickJumpLooksInvalid(int64_t delta_ticks, double dt) const
  {
    if (dt <= 0.0 || ticks_per_revolution_ <= 0.0 || wheel_radius_m_ <= 0.0) // Treat invalid timing or wheel parameters as an invalid result
    {
      return true;
    }

    const double max_ticks =
      (std::abs(max_wheel_speed_mps_) * std::max(max_tick_jump_scale_, 1.0) * dt) /
      metersPerTick(); // Calculate the largest reasonable number of ticks for this time interval
    
    return static_cast<double>(std::abs(delta_ticks)) > std::max(1.0, max_ticks); // Return true if measured tick change is larger than the allowed limit
  }

  double metersPerTick() const // Calculate how far the wheel travels for once encoder tick
  {
    return (2.0 * M_PI * wheel_radius_m_) / ticks_per_revolution_;
  }

  double normalizeAngle(double angle_rad) const // Keep the yaw angle within the range -pi to +pi
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

int main(int argc, char ** argv) // Start ROS 2 and run the wheel odometry node
{
  rclcpp::init(argc, argv);                           // Initialize ROS 2
  auto node = std::make_shared<WheelOdometryNode>();  // Create the wheel odometry node
  rclcpp::spin(node);                                 // Keep the node running and process its callbacks
  rclcpp::shutdown();                                 // Shut down ROS 2 when the node stops
  return 0;
}
