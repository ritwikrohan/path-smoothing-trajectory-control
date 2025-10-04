import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "bot_bringup"  # <-- put your bringup package name here

    # path to YAML file
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        "params",
        "assignment.yaml"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_description"),
            "launch",
            "gazebo.launch.py"
        ))
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_controller"),
            "launch",
            "controller.launch.py"
        ))
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        )),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    path_smoother_node = Node(
        package="path_smoother",
        executable="path_smoother_node",
        name="path_smoother",
        parameters=[params_file]
    )

    trajectory_generator_node = Node(
        package="trajectory_generator",
        executable="trajectory_generator_node",
        name="trajectory_generator",
        parameters=[params_file]
    )

    traj_controller_node = Node(
        package="trajectory_tracking_controller",
        executable="traj_controller",
        name="traj_controller",
        parameters=[params_file]
    )

    obstacle_avoidance_node = Node(
        package="obstacle_avoidance",
        executable="obstacle_avoidance_node",
        name="obstacle_avoidance",
        parameters=[params_file]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("bot_bringup"),
                "rviz",
                "my_rviz.rviz"
            )],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        joystick,
        path_smoother_node,
        trajectory_generator_node,
        traj_controller_node,
        obstacle_avoidance_node,
        rviz
    ])
