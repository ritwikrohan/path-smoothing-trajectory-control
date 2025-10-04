#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "bot_msgs/msg/trajectory.hpp"
#include "bot_msgs/msg/trajectory_point.hpp"
#include <cmath>

class TrajectoryGenerator : public rclcpp::Node
{
public:
  TrajectoryGenerator()
  : Node("trajectory_generator")
  {
    // Task 2: here I am changing the Path (nav_msgs/Path) into a Trajectory (bot_msgs/Trajectory)

    declare_parameter<double>("velocity", 0.2);  // I use a constant velocity [m/s]

    // I subscribe to the smoothed path from PathSmoother
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/smooth_plan", 10,
      std::bind(&TrajectoryGenerator::pathCallback, this, std::placeholders::_1));

    // I publish the generated trajectory
    traj_pub_ = create_publisher<bot_msgs::msg::Trajectory>("/trajectory", 10);

    RCLCPP_INFO(get_logger(), "TrajectoryGenerator ready, listening to /smooth_plan");
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    double v = get_parameter("velocity").as_double(); // fetch constant velocity parameter

    bot_msgs::msg::Trajectory traj;   // I build my custom trajectory message
    traj.header.stamp = now();
    traj.header.frame_id = path->header.frame_id; // keep frame consistent with input path

    double t = 0.0; // cumulative time along trajectory
    for (size_t i = 0; i < path->poses.size(); i++) {
      bot_msgs::msg::TrajectoryPoint pt; // each point in my trajectory
      pt.x = path->poses[i].pose.position.x;
      pt.y = path->poses[i].pose.position.y;

      if (i > 0) {
        // I calculate distance from the previous point
        double dx = pt.x - path->poses[i-1].pose.position.x;
        double dy = pt.y - path->poses[i-1].pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Since I assume constant velocity, time increment = distance / velocity
        t += dist / v;
      }
      pt.t = t; // assign cumulative time to this trajectory point

      traj.points.push_back(pt); // add point to trajectory
    }

    traj_pub_->publish(traj); // finally I publish the trajectory
    RCLCPP_INFO(get_logger(), "Published trajectory with %zu points", traj.points.size());
  }

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;    // input path subscriber
  rclcpp::Publisher<bot_msgs::msg::Trajectory>::SharedPtr traj_pub_; // output trajectory publisher
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>()); // spin node until shutdown
  rclcpp::shutdown();
  return 0;
}
