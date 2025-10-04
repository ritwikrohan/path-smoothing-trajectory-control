#include <algorithm>
#include <cmath>

#include "obstacle_avoidance/obstacle_avoidance.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace bot_nav
{

ObstacleAvoidance::ObstacleAvoidance()
: Node("obstacle_avoidance"),
  rerouting_(false),            // I start with no reroute in progress
  obstacle_detected_(false),    // nothing critical yet
  last_reroute_time_(this->now()), // mark init time for cooldown
  has_active_goal_(false)       // no goal received yet
{
  // parameters I declared for tuning obstacle behavior
  this->declare_parameter<double>("obstacle_distance", 0.6);     // reroute if closer than this
  this->declare_parameter<double>("critical_distance", 0.5);     // emergency stop threshold
  this->declare_parameter<double>("sector_width_deg", 30.0);     // forward cone width
  this->declare_parameter<double>("side_offset", 1.5);          // sidestep distance
  this->declare_parameter<double>("forward_offset", 0.5);        // forward offset while sidestepping
  this->declare_parameter<bool>("dynamic_side", true);           // auto choose left/right
  this->declare_parameter<double>("min_reroute_interval_s", 3.0);// cooldown for reroutes
  this->declare_parameter<std::string>("global_frame", "odom");  // frame I use for TF lookups
  this->declare_parameter<std::string>("base_frame", "base_footprint"); // robot base frame

  // now I load the parameter values into my variables
  obstacle_distance_      = this->get_parameter("obstacle_distance").as_double();
  critical_distance_      = this->get_parameter("critical_distance").as_double();
  sector_width_deg_       = this->get_parameter("sector_width_deg").as_double();
  side_offset_            = this->get_parameter("side_offset").as_double();
  forward_offset_         = this->get_parameter("forward_offset").as_double();
  dynamic_side_           = this->get_parameter("dynamic_side").as_bool();
  min_reroute_interval_s_ = this->get_parameter("min_reroute_interval_s").as_double();
  global_frame_           = this->get_parameter("global_frame").as_string();
  base_frame_             = this->get_parameter("base_frame").as_string();

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());          // TF buffer
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);     // TF listener

  smooth_client_ = this->create_client<bot_msgs::srv::WaypointsPath>("/smooth_path"); // smoother service
  cancel_client_ = this->create_client<std_srvs::srv::Trigger>("/cancel_trajectory"); // cancel service
  emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10); // publish stop flag

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>( // subscribe to lidar
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidance::scanCallback, this, std::placeholders::_1));

  traj_sub_ = this->create_subscription<bot_msgs::msg::Trajectory>( // subscribe to planned trajectory
      "/trajectory", 10,
      std::bind(&ObstacleAvoidance::trajectoryCallback, this, std::placeholders::_1));

  // log info so I know the node came up correctly
  RCLCPP_INFO(get_logger(), "========================================");
  RCLCPP_INFO(get_logger(), "ObstacleAvoidance Initialized");
  RCLCPP_INFO(get_logger(), "  Side offset: %.1fm", side_offset_);
  RCLCPP_INFO(get_logger(), "  Using smoother with sharp waypoint following");
  RCLCPP_INFO(get_logger(), "========================================");
}

void ObstacleAvoidance::trajectoryCallback(const bot_msgs::msg::Trajectory::SharedPtr msg)
{
  if (msg->points.empty()) return; // ignore if nothing came

  const auto& last_point = msg->points.back(); // final goal point
  current_goal_.x = last_point.x;              // save x
  current_goal_.y = last_point.y;              // save y
  current_goal_.z = 0.0;                       // keep z flat
  has_active_goal_ = true;                     // I mark goal active
  
  RCLCPP_INFO(get_logger(), "Trajectory goal: (%.2f, %.2f) [%zu points]", 
              current_goal_.x, current_goal_.y, msg->points.size());
}

void ObstacleAvoidance::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!has_active_goal_) return; // no point checking if no goal yet

  const auto [min_left, min_right] = leftRightClearance(*msg, sector_width_deg_); // clearance check
  const double min_front = std::min(min_left, min_right); // pick closest side

  if (std::isfinite(min_front) && min_front < critical_distance_) { // critical stop
    if (!obstacle_detected_) {
      RCLCPP_ERROR(get_logger(), "CRITICAL OBSTACLE %.2fm! STOPPING", min_front);
      publishEmergencyStop(true);   // stop robot
      cancelCurrentTrajectory();    // cancel active path
      obstacle_detected_ = true;    // mark as blocked
    }
    return; // don’t continue to reroute logic
  }

  if (std::isfinite(min_front) && min_front < obstacle_distance_) { // normal reroute trigger
    const double since_last = (this->now() - last_reroute_time_).seconds();
    
    if (!rerouting_ && since_last >= min_reroute_interval_s_) { // only if cooldown passed
      RCLCPP_WARN(get_logger(), "Obstacle %.2fm - SHARP REROUTE to (%.2f, %.2f)", 
                  min_front, current_goal_.x, current_goal_.y);
      cancelCurrentTrajectory();              // cancel path first
      triggerReroute(min_left, min_right);    // request new path
      obstacle_detected_ = true;              // mark as blocked
    }
  } else {
    if (obstacle_detected_) { // clear stop if obstacle gone
      publishEmergencyStop(false);
      obstacle_detected_ = false;
    }
  }
}

void ObstacleAvoidance::publishEmergencyStop(bool stop)
{
  std_msgs::msg::Bool msg;
  msg.data = stop;                 // just true/false flag
  emergency_stop_pub_->publish(msg); // publish to topic
}

void ObstacleAvoidance::cancelCurrentTrajectory()
{
  if (!cancel_client_->service_is_ready()) return; // don’t call if down
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); // empty trigger request
  cancel_client_->async_send_request(request); // fire async
}

std::pair<double,double> ObstacleAvoidance::leftRightClearance(
    const sensor_msgs::msg::LaserScan &scan,
    double sector_width_deg) const
{
  const double sector_width_rad = sector_width_deg * M_PI / 180.0; // convert to radians
  const double forward_angle = M_PI;  // I treat 180° as front
  
  const int n = static_cast<int>(scan.ranges.size());
  if (n <= 0) {
    return {std::numeric_limits<double>::infinity(), 
            std::numeric_limits<double>::infinity()}; // nothing in scan
  }

  const double min_ang = forward_angle - sector_width_rad; // left edge of sector
  const double max_ang = forward_angle + sector_width_rad; // right edge of sector

  double min_left  = std::numeric_limits<double>::infinity(); // keep closest left
  double min_right = std::numeric_limits<double>::infinity(); // keep closest right

  for (int i = 0; i < n; ++i) {
    const double ang = scan.angle_min + i * scan.angle_increment; // angle at this index
    
    if (ang >= min_ang && ang <= max_ang) { // only in forward sector
      const double r = scan.ranges[i];      // measured distance
      
      if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
        if (ang > forward_angle) {
          min_left = std::min(min_left, r);   // left side
        } else {
          min_right = std::min(min_right, r); // right side
        }
      }
    }
  }

  return {min_left, min_right}; // return both distances
}

geometry_msgs::msg::Point ObstacleAvoidance::getCurrentRobotPosition()
{
  geometry_msgs::msg::Point p;
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero); // TF lookup
    p.x = tf.transform.translation.x;  // set x
    p.y = tf.transform.translation.y;  // set y
    p.z = 0.0;                         // flat
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "TF error: %s", ex.what());
    p.x = 0.0; p.y = 0.0; p.z = 0.0;   // fallback to origin
  }
  return p;
}

std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::pickDetours(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &goal,
    bool choose_left)
{
  std::vector<geometry_msgs::msg::Point> detours;

  double dx = goal.x - start.x;  // goal direction vector
  double dy = goal.y - start.y;
  const double norm = std::hypot(dx, dy);
  
  if (norm < 1e-6) return detours; // ignore zero-length

  dx /= norm; dy /= norm; // normalize direction

  // perpendicular vectors
  const double left_x  = -dy;
  const double left_y  = +dx;
  const double right_x = +dy;
  const double right_y = -dx;

  const double sx = choose_left ? left_x : right_x; // pick side
  const double sy = choose_left ? left_y : right_y;

  double arc_length = side_offset_ * 1.5;  // I chose short arc, 1.5x offset
  int total_waypoints = 15;                // fixed number of waypoints

  for (int i = 1; i <= total_waypoints; i++) {
    double t = i / static_cast<double>(total_waypoints); // fraction along path
    geometry_msgs::msg::Point p;

    double side_factor; // smooth offset profile
    if (t < 0.25) {
      side_factor = std::sin(t / 0.25 * M_PI / 2.0); // ramp up
    } else if (t < 0.75) {
      side_factor = 1.0;                             // hold full offset
    } else {
      side_factor = std::cos((t - 0.75) / 0.25 * M_PI / 2.0); // ramp down
    }

    double forward_dist = arc_length * t; // forward progress

    // combine forward and side displacement
    p.x = start.x + dx * forward_dist + sx * side_offset_ * side_factor;
    p.y = start.y + dy * forward_dist + sy * side_offset_ * side_factor;
    p.z = 0.0;
    
    detours.push_back(p);
  }
  
  RCLCPP_WARN(get_logger(), "Generated %zu waypoints for SHORT arc (length=%.1fm, side=%.1fm, %s)", 
              detours.size(), arc_length, side_offset_, choose_left ? "LEFT" : "RIGHT");
  
  return detours;
}

void ObstacleAvoidance::triggerReroute(double min_left, double min_right)
{
  rerouting_ = true; // mark rerouting

  bool choose_left = dynamic_side_ ? (min_left < min_right) : true; // pick side

  if (!smooth_client_->wait_for_service(std::chrono::seconds(2))) { // smoother must be up
    RCLCPP_ERROR(get_logger(), "/smooth_path service not available");
    rerouting_ = false;
    return;
  }

  const auto start = getCurrentRobotPosition();                   // current pos
  const auto detours = pickDetours(start, current_goal_, choose_left); // detour path

  auto req = std::make_shared<bot_msgs::srv::WaypointsPath::Request>(); // path request
  req->waypoints.push_back(start);
  for (const auto &d : detours) req->waypoints.push_back(d);       // add detour waypoints
  req->waypoints.push_back(current_goal_);                         // rejoin goal

  RCLCPP_WARN(get_logger(), "Requesting sharp reroute via smoother (%zu waypoints)", 
              req->waypoints.size());

  auto fut = smooth_client_->async_send_request(
      req,
      [this](rclcpp::Client<bot_msgs::srv::WaypointsPath>::SharedFuture f) {
        try {
          f.get(); // wait for response
          this->publishEmergencyStop(false); // clear stop
          RCLCPP_INFO(this->get_logger(), "Sharp reroute complete");
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Reroute failed: %s", e.what());
        }
        this->last_reroute_time_ = this->now(); // update timestamp
        this->rerouting_ = false;               // reset flag
      });
}

} // namespace bot_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // start ROS
  rclcpp::spin(std::make_shared<bot_nav::ObstacleAvoidance>()); // spin node
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
