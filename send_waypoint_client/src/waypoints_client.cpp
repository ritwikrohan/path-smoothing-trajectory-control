#include "rclcpp/rclcpp.hpp"
#include "bot_msgs/srv/waypoints_path.hpp"
#include "geometry_msgs/msg/point.hpp"

class WaypointsClient : public rclcpp::Node
{
public:
  WaypointsClient() : Node("waypoints_client")
  {
    // I declare a parameter that holds waypoints as a flat list [x1, y1, x2, y2, ...]
    // this->declare_parameter<std::vector<double>>("waypoints", {});

    // Declare a selector param (string)
    std::string default_case = "waypoints_case1";
    this->declare_parameter<std::string>("case", default_case);

    auto case_id = this->get_parameter("case").as_string();

    // Declare the possible vectors
    this->declare_parameter<std::vector<double>>("waypoints_case1", {});
    this->declare_parameter<std::vector<double>>("waypoints_case2", {});
    this->declare_parameter<std::vector<double>>("waypoints_case3", {});

    // Pick the right one based on case_id
    auto flat_list = this->get_parameter(case_id).as_double_array();

    client_ = this->create_client<bot_msgs::srv::WaypointsPath>("/smooth_path"); // I create a client to call smoother service

    // I load the parameter values (flat list of doubles)
    // auto flat_list = this->get_parameter("waypoints").as_double_array();


    // Sanity check: must have even number of values (pairs of x,y)
    if (flat_list.empty() || flat_list.size() % 2 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Waypoints must be pairs of x,y values.");
      return;
    }

    // I fill the service request with proper Point messages
    request_ = std::make_shared<bot_msgs::srv::WaypointsPath::Request>();
    for (size_t i = 0; i < flat_list.size(); i += 2) {
      geometry_msgs::msg::Point pt;
      pt.x = flat_list[i];     // x coordinate
      pt.y = flat_list[i+1];   // y coordinate
      pt.z = 0.0;              // keep z flat
      request_->waypoints.push_back(pt);
    }

    // Wait until the /smooth_path service is ready
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /smooth_path service...");
    }

    // Send the request asynchronously
    auto future = client_->async_send_request(
      request_,
      std::bind(&WaypointsClient::responseCallback, this, std::placeholders::_1));
  }

private:
  void responseCallback(rclcpp::Client<bot_msgs::srv::WaypointsPath>::SharedFuture future)
  {
    try {
      auto response = future.get(); // I wait for response here
      RCLCPP_INFO(this->get_logger(), "Received smoothed path with %zu poses",
                  response->path.poses.size()); // log number of points returned
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what()); // handle error
    }
  }

  rclcpp::Client<bot_msgs::srv::WaypointsPath>::SharedPtr client_; // my service client
  bot_msgs::srv::WaypointsPath::Request::SharedPtr request_;       // cached request
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);                        // init ROS
  auto node = std::make_shared<WaypointsClient>(); // create my node
  rclcpp::spin(node);                              // spin until shutdown
  rclcpp::shutdown();                              // shutdown cleanly
  return 0;
}
