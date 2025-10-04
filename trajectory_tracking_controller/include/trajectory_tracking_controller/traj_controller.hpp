#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "bot_msgs/msg/trajectory.hpp"

namespace bot_control
{

class PDTrajectoryController : public rclcpp::Node
{
public:
  PDTrajectoryController(); // I build a node that follows a trajectory using a PD controller

private:
  // I subscribe to the trajectory and store it
  void trajectoryCallback(const bot_msgs::msg::Trajectory::SharedPtr traj);

  // I run my control loop at a fixed frequency
  void controlLoop();

  // I pick the desired pose from the trajectory at the current elapsed time
  geometry_msgs::msg::PoseStamped getDesiredPose(double elapsed);

  // ROS interfaces I use
  rclcpp::Subscription<bot_msgs::msg::Trajectory>::SharedPtr traj_sub_; // input trajectory
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;     // output velocity commands
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_; // for visualizing the current target
  rclcpp::TimerBase::SharedPtr control_loop_; // periodic control loop timer

  // TF utilities for robot pose lookup
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // === PD Controller parameters ===
  // Here I am explicitly using a Proportional-Derivative controller
  // - Proportional term (kp_) reduces the error directly
  // - Derivative term (kd_) damps oscillations by considering rate of change
  double kp_, kd_;

  double max_linear_velocity_;   // I cap forward velocity
  double max_angular_velocity_;  // I cap angular velocity

  // === Controller state ===
  bot_msgs::msg::Trajectory global_traj_; // active trajectory I’m following
  rclcpp::Time traj_start_time_;          // when I started executing trajectory
  double prev_lin_error_, prev_ang_error_; // I need previous errors for D term
  rclcpp::Time last_cycle_time_;          // last control cycle time (for dt)
  bool active_traj_;                      // flag if a trajectory is active

  // === Safety mechanisms ===
  bool emergency_stop_; // I stop immediately if this is true
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_; // listen to emergency stop topic
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;       // service to cancel trajectory

  // Safety callbacks
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg); // react to emergency stop
  void cancelTrajectoryCallback(                                        // handle cancel requests
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

} // namespace bot_control
