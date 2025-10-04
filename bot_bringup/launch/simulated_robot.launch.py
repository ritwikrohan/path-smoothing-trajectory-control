import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "bot_bringup" 

    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        "params",
        "assignment.yaml"
    )

    # Gazebo world + robot description
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_description"),
            "launch",
            "gazebo.launch.py"
        ))
    )

    # Controllers (diff drive, joint state, etc.)
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_controller"),
            "launch",
            "controller.launch.py"
        ))
    )

    # Joystick teleop
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        )),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    # -----------------------------
    # Task 1 – Path Smoothing
    # -----------------------------
    path_smoother_node = Node(
        package="path_smoother",
        executable="path_smoother_node",
        name="path_smoother",
        parameters=[params_file]
    )

    # -----------------------------
    # Task 2 – Trajectory Generation
    # -----------------------------
    trajectory_generator_node = Node(
        package="trajectory_generator",
        executable="trajectory_generator_node",
        name="trajectory_generator",
        parameters=[params_file]
    )

    # -----------------------------
    # Task 3 – Trajectory Tracking Controller
    # -----------------------------
    traj_controller_node = Node(
        package="trajectory_tracking_controller",
        executable="traj_controller",
        name="traj_controller",
        parameters=[params_file]
    )

    # -----------------------------
    # (Extra) Obstacle Avoidance Module
    # -----------------------------
    obstacle_avoidance_node = Node(
        package="obstacle_avoidance",
        executable="obstacle_avoidance_node",
        name="obstacle_avoidance",
        parameters=[params_file]
    )

    # Visualization
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
        path_smoother_node,         # Task 1
        trajectory_generator_node,  # Task 2
        traj_controller_node,       # Task 3
        obstacle_avoidance_node,    # Extra
        rviz
    ])
