#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bot_msgs/srv/waypoints_path.hpp"

class PathSmoother : public rclcpp::Node
{
public:
  PathSmoother(); // I set up the smoother node and advertise service + publisher

private:
  void handle_service( // I handle requests for path smoothing
    const std::shared_ptr<bot_msgs::srv::WaypointsPath::Request> request,
    std::shared_ptr<bot_msgs::srv::WaypointsPath::Response> response);

  nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path & input); // I run the smoothing algorithm

  void update_orientations(nav_msgs::msg::Path & path); // I adjust poses so orientations follow path direction

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;     // I publish the smoothed path
  rclcpp::Service<bot_msgs::srv::WaypointsPath>::SharedPtr service_; // I expose a service for smoothing

  std::string frame_id_;  // frame I use for all published paths
  double w_data_;         // weight for keeping original data
  double w_smooth_;       // weight for smoothing strength
  double tolerance_;      // stop criteria for iterations
  int max_its_;           // max iterations for smoothing
};
