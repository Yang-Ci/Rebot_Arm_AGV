"""Launch the Rebot Arm AGV laboratory world in Gazebo Harmonic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rebotarm_gazebo")
    mujoco_share = get_package_share_directory("rebotarm_mujoco_rs")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    world = os.path.join(package_share, "worlds", "rebotarm_lab.sdf")
    bridge = os.path.join(package_share, "config", "ros_gz_bridge.yaml")
    resource_path = os.pathsep.join(filter(None, [
        os.path.join(package_share, "models"),
        os.path.dirname(mujoco_share),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]))

    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        condition=IfCondition(gui),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        condition=UnlessCondition(gui),
        launch_arguments={
            "gz_args": ["-s -r -v 3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
        DeclareLaunchArgument("paused", default_value="false", description="Reserved for launch compatibility"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        gazebo_gui,
        gazebo_headless,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            output="screen",
            parameters=[{"config_file": bridge}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_scan_tf",
            arguments=[
                "--x", "0.185", "--y", "0", "--z", "0.175",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "base_scan",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0.09",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "imu_link",
            ],
        ),
    ])
