#include "path_smoother/path_smoother.hpp"
#include <cmath>

PathSmoother::PathSmoother()
: Node("path_smoother") // I start the node with name "path_smoother"
{
    frame_id_  = declare_parameter<std::string>("frame_id", "odom"); // frame for all paths
    w_data_    = declare_parameter<double>("w_data",   0.7);         // weight for original data term
    w_smooth_  = declare_parameter<double>("w_smooth", 0.3);         // weight for smoothness term
    tolerance_ = declare_parameter<double>("tolerance", 1e-6);       // convergence tolerance
    max_its_   = declare_parameter<int>("max_its", 1000);            // max iterations

    rclcpp::QoS qos(rclcpp::KeepLast(1)); // keep only the latest path
    qos.transient_local();                // new subscribers still get the last published path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_plan", qos);

    service_ = create_service<bot_msgs::srv::WaypointsPath>( // service to request smoothing
    "smooth_path",
    std::bind(&PathSmoother::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(get_logger(), "PathSmoother node started, service [/smooth_path]");
}

void PathSmoother::handle_service(
const std::shared_ptr<bot_msgs::srv::WaypointsPath::Request> request,
std::shared_ptr<bot_msgs::srv::WaypointsPath::Response> response)
{
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;

    // Convert waypoints (geometry_msgs/Point) into PoseStamped
    for (const auto & point : request->waypoints) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position = point;
        pose_stamped.pose.orientation.w = 1.0; // identity orientation
        path.poses.push_back(pose_stamped);
    }

    // Apply smoothing
    nav_msgs::msg::Path smoothed = smooth_path(path);

    response->path = smoothed;  // send back in response
    path_pub_->publish(smoothed); // also publish for RViz

    RCLCPP_INFO(get_logger(), "Smoothed path with %zu poses", smoothed.poses.size());
}

nav_msgs::msg::Path PathSmoother::smooth_path(const nav_msgs::msg::Path & input)
{
    // y = working path (updated iteratively)
    // x = original path (kept fixed to pull points back toward data)
    nav_msgs::msg::Path y = input;
    nav_msgs::msg::Path x = input;

    const size_t n = y.poses.size();
    if (n < 3) {
        return y; // no smoothing if too few points
    }

    // --- Algorithm: Gradient descent style update ---
    // Each point (except endpoints) is updated using:
    //   y[i] = y[i] + w_data * (x[i] - y[i]) + w_smooth * (y[i-1] + y[i+1] - 2*y[i])
    //
    // - First term: pulls y[i] toward the original data x[i] (data fidelity)
    // - Second term: enforces smoothness by minimizing curvature (discrete Laplacian)
    // - We repeat until changes are below tolerance or max iterations reached
    //
    // This is essentially iterative Laplacian smoothing with a data attachment term.

    double change = tolerance_;
    int its = 0;

    while (change >= tolerance_ && its < max_its_) {
        change = 0.0;

        for (size_t i = 1; i + 1 < n; ++i) { // skip endpoints, they must stay fixed
            // Smooth X coordinate
            {
                double yi = y.poses[i].pose.position.x;    // current value
                double yi_org = yi;                        // store original for change measurement
                double xim1 = y.poses[i - 1].pose.position.x; // neighbor left
                double xip1 = y.poses[i + 1].pose.position.x; // neighbor right
                double xi   = x.poses[i].pose.position.x;     // original data point

                // Update formula = gradient step
                yi = yi + w_data_ * (xi - yi)              // data term
                       + w_smooth_ * (xip1 + xim1 - 2.0*yi); // smoothness term

                y.poses[i].pose.position.x = yi;           // update position
                change += std::fabs(yi - yi_org);          // accumulate change
            }

            // Smooth Y coordinate (same formula)
            {
                double yi = y.poses[i].pose.position.y;
                double yi_org = yi;
                double yim1 = y.poses[i - 1].pose.position.y;
                double yip1 = y.poses[i + 1].pose.position.y;
                double xi   = x.poses[i].pose.position.y;

                yi = yi + w_data_ * (xi - yi)
                       + w_smooth_ * (yip1 + yim1 - 2.0*yi);

                y.poses[i].pose.position.y = yi;
                change += std::fabs(yi - yi_org);
            }
        }
        its++; // count iteration
    }

    // Once positions converge, I update orientations along the new path
    update_orientations(y);

    return y;
}

void PathSmoother::update_orientations(nav_msgs::msg::Path & path)
{
    size_t n = path.poses.size();
    if (n < 2) return;

    for (size_t i = 0; i < n; ++i) {
        // I compute yaw by looking at direction between neighbors
        size_t i0 = (i == 0) ? 0 : i - 1;         // previous index
        size_t i1 = (i == n - 1) ? n - 1 : i + 1; // next index
        const auto & p0 = path.poses[i0].pose.position;
        const auto & p1 = path.poses[i1].pose.position;

        double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x); // slope as heading

        // Convert yaw to quaternion (since ROS uses quaternions)
        path.poses[i].pose.orientation.x = 0.0;
        path.poses[i].pose.orientation.y = 0.0;
        path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
        path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSmoother>()); // spin until shutdown
    rclcpp::shutdown();
    return 0;
}
