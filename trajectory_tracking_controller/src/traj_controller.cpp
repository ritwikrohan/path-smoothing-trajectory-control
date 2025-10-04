#include <chrono>
#include <cmath>
#include <algorithm>

#include "trajectory_tracking_controller/traj_controller.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

namespace bot_control
{

PDTrajectoryController::PDTrajectoryController()
: Node("pd_traj_controller_node"),
  prev_lin_error_(0.0),
  prev_ang_error_(0.0),
  active_traj_(false),
  emergency_stop_(false)
{
  // Declaring PD controller parameters 
  this->declare_parameter<double>("kp", 2.0); // default value = 2.0
  this->declare_parameter<double>("kd", 0.1); // default value = 0.1
  this->declare_parameter<double>("max_linear_velocity", 0.3);
  this->declare_parameter<double>("max_angular_velocity", 1.0);

  // Loading them into member variables 
  kp_ = this->get_parameter("kp").as_double();
  kd_ = this->get_parameter("kd").as_double();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // I subscribe to the trajectory to follow
  traj_sub_ = create_subscription<bot_msgs::msg::Trajectory>(
    "/trajectory", 10,
    std::bind(&PDTrajectoryController::trajectoryCallback, this, std::placeholders::_1));

  // I subscribe to an emergency stop flag (true = stop immediately)
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10,
    std::bind(&PDTrajectoryController::emergencyStopCallback, this, std::placeholders::_1));

  // Publisher for velocity commands
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Publisher for visualizing the target pose along trajectory
  target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd_traj/target_pose", 10);

  // Service to cancel trajectory externally
  cancel_service_ = create_service<std_srvs::srv::Trigger>(
    "/cancel_trajectory",
    std::bind(&PDTrajectoryController::cancelTrajectoryCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Control loop runs every 100 ms
  control_loop_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PDTrajectoryController::controlLoop, this));

  last_cycle_time_ = now();
  
  RCLCPP_INFO(get_logger(), "PDTrajectoryController initialized with cancel service");
}

void PDTrajectoryController::trajectoryCallback(const bot_msgs::msg::Trajectory::SharedPtr traj)
{
  if (traj->points.empty()) return;

  // When I receive a new trajectory, I clear any previous emergency stop
  emergency_stop_ = false;
  
  // I replace the old trajectory with the new one
  global_traj_ = *traj;
  traj_start_time_ = now();
  prev_lin_error_ = 0.0;
  prev_ang_error_ = 0.0;
  last_cycle_time_ = now();
  active_traj_ = true;

  RCLCPP_INFO(get_logger(), "Received new trajectory with %zu points", global_traj_.points.size());
}

void PDTrajectoryController::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Emergency stop ON
  if (msg->data && !emergency_stop_) {
    RCLCPP_WARN(get_logger(), "EMERGENCY STOP activated!");
    emergency_stop_ = true;
    
    // Immediately stop robot by publishing zero velocity
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);

  // Emergency stop OFF
  } else if (!msg->data && emergency_stop_) {
    RCLCPP_INFO(get_logger(), "Emergency stop cleared");
    emergency_stop_ = false;
  }
}

void PDTrajectoryController::cancelTrajectoryCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // not used
  
  if (active_traj_) {
    // Stop following the trajectory
    active_traj_ = false;
    emergency_stop_ = false;
    global_traj_.points.clear();
    
    // Send stop command to robot
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
    
    response->success = true;
    response->message = "Trajectory cancelled and robot stopped";
    RCLCPP_INFO(get_logger(), "Trajectory cancelled via service");
  } else {
    response->success = true;
    response->message = "No active trajectory to cancel";
    RCLCPP_INFO(get_logger(), "Cancel requested but no active trajectory");
  }
}

void PDTrajectoryController::controlLoop()
{
  // === Safety first: if emergency stop is active, force robot to stop ===
  if (emergency_stop_) {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
    
    if (active_traj_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                           "Emergency stop active - trajectory paused");
    }
    return;
  }

  if (!active_traj_ || global_traj_.points.empty()) return;

  double elapsed = (now() - traj_start_time_).seconds();

  // === Get current robot pose from TF ===
  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    robot_tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
    return;
  }

  double xr = robot_tf.transform.translation.x;
  double yr = robot_tf.transform.translation.y;
  tf2::Quaternion q(
    robot_tf.transform.rotation.x,
    robot_tf.transform.rotation.y,
    robot_tf.transform.rotation.z,
    robot_tf.transform.rotation.w);
  double roll, pitch, theta_r;
  tf2::Matrix3x3(q).getRPY(roll, pitch, theta_r);

  // === Check if robot has reached the goal ===
  auto final = global_traj_.points.back();
  double dx_end = final.x - xr;
  double dy_end = final.y - yr;
  double dist_to_goal = std::sqrt(dx_end*dx_end + dy_end*dy_end);

  if (dist_to_goal < 0.1 || elapsed >= global_traj_.points.back().t) {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);

    active_traj_ = false;
    global_traj_.points.clear();
    RCLCPP_INFO(get_logger(), "Goal reached (dist=%.3fm), stopping robot.", dist_to_goal);
    return;
  }

  // === Get desired pose from trajectory interpolation ===
  auto desired_pose = getDesiredPose(elapsed);
  target_pub_->publish(desired_pose);

  double xd = desired_pose.pose.position.x;
  double yd = desired_pose.pose.position.y;

  // === Compute position + heading error ===
  double dx = xd - xr;
  double dy = yd - yr;
  double distance = std::sqrt(dx*dx + dy*dy); // distance error
  double theta_d = std::atan2(dy, dx);        // desired heading
  double heading_error = theta_d - theta_r;

  // Normalize heading error to [-pi, pi]
  while (heading_error > M_PI) heading_error -= 2*M_PI;
  while (heading_error < -M_PI) heading_error += 2*M_PI;

  double dt = (now() - last_cycle_time_).seconds();
  last_cycle_time_ = now();
  if (dt <= 0.0) dt = 0.1; // safety check

  // === PD Controller ===
  // Formula:
  //   u = Kp * error + Kd * (error - prev_error) / dt
  //
  // - Linear velocity v depends on distance error
  // - Angular velocity w depends on heading error
  double lin_error_deriv = (distance - prev_lin_error_) / dt;
  double ang_error_deriv = (heading_error - prev_ang_error_) / dt;

  double v = kp_ * distance + kd_ * lin_error_deriv;
  double w = kp_ * heading_error + kd_ * ang_error_deriv;

  // Save errors for derivative calculation next cycle
  prev_lin_error_ = distance;
  prev_ang_error_ = heading_error;

  // Clamp commands to max values
  v = std::clamp(v, 0.0, max_linear_velocity_);
  w = std::clamp(w, -max_angular_velocity_, max_angular_velocity_);

  // Publish velocity command
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = w;
  cmd_pub_->publish(cmd);
}

geometry_msgs::msg::PoseStamped PDTrajectoryController::getDesiredPose(double elapsed)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp = now();

  auto target = global_traj_.points.back();

  // Linear interpolation between trajectory points based on time
  for (size_t i = 0; i < global_traj_.points.size()-1; i++) {
    const auto & p0 = global_traj_.points[i];
    const auto & p1 = global_traj_.points[i+1];
    if (p0.t <= elapsed && elapsed <= p1.t) {
      double ratio = (elapsed - p0.t) / (p1.t - p0.t);
      target.x = p0.x + ratio * (p1.x - p0.x);
      target.y = p0.y + ratio * (p1.y - p0.y);
      break;
    }
  }

  pose.pose.position.x = target.x;
  pose.pose.position.y = target.y;
  pose.pose.orientation.w = 1.0; // I ignore yaw here, just keep neutral orientation
  return pose;
}

} // namespace bot_control

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_control::PDTrajectoryController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
