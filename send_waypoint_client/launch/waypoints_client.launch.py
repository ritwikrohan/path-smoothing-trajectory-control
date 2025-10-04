import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = "send_waypoint_client"

    # Path YAML file with waypoints_case1/2/3
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        "params",
        "waypoints.yaml"
    )

    # Launch argument to choose which case to run
    case_arg = DeclareLaunchArgument(
        "case",
        default_value="waypoints_case1",
        description="Choose which test case to use (waypoints_case1/waypoints_case2/waypoints_case3)"
    )

    return LaunchDescription([
        case_arg,

        Node(
            package=pkg_name,
            executable="waypoints_client",
            name="waypoints_client",
            output="screen",
            parameters=[params_file,
                        {"case": LaunchConfiguration("case")}]
        )
    ])
