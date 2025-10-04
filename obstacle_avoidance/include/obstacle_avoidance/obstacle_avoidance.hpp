#pragma once

#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bot_msgs/srv/waypoints_path.hpp"
#include "bot_msgs/msg/trajectory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace bot_nav
{

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance();

private:
  // I process incoming scan data and decide if a reroute is needed
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // I trigger a reroute when the robot is blocked
  void triggerReroute();
  void triggerReroute(double min_left, double min_right);

  // I fetch the current robot position from TF
  geometry_msgs::msg::Point getCurrentRobotPosition();

  // I create detours (left/right sidesteps) to go around obstacles
  std::vector<geometry_msgs::msg::Point> pickDetours(
      const geometry_msgs::msg::Point &start,
      const geometry_msgs::msg::Point &goal,
      bool choose_left);

  // I check how much clearance the robot has to the left and right
  std::pair<double,double> leftRightClearance(const sensor_msgs::msg::LaserScan &scan,
                                              double sector_width_deg) const;

  // Parameters I tuned for avoidance logic
  double obstacle_distance_;      // threshold distance to trigger reroute (m)
  double sector_width_deg_;       // half-width of the forward sector to check (deg)
  double side_offset_;            // lateral sidestep distance (m)
  double forward_offset_;         // forward move during sidestep (m)
  bool   dynamic_side_;           // choose left/right dynamically
  double min_reroute_interval_s_; // minimum time gap between reroutes

  std::string global_frame_;      // usually "odom"
  std::string base_frame_;        // usually "base_footprint"

  geometry_msgs::msg::Point goal_;   // final navigation goal

  // State I maintain to avoid repeated rerouting
  bool rerouting_;
  rclcpp::Time last_reroute_time_;

  // ROS interfaces I use
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<bot_msgs::srv::WaypointsPath>::SharedPtr smooth_client_;

  // TF utilities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Safety extensions I added
  double critical_distance_;  // immediate stop if closer than this
  bool obstacle_detected_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;

  // Safety helpers
  void publishEmergencyStop(bool stop);
  void cancelCurrentTrajectory();

  // Track the robot’s current trajectory
  geometry_msgs::msg::Point current_goal_;
  bool has_active_goal_;
  rclcpp::Subscription<bot_msgs::msg::Trajectory>::SharedPtr traj_sub_;

  // I update internal state whenever a new trajectory is received
  void trajectoryCallback(const bot_msgs::msg::Trajectory::SharedPtr msg);
};

} // namespace bot_nav
