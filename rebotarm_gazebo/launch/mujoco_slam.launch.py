"""Launch the MuJoCo laboratory and SLAM Toolbox for online mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("rebotarm_gazebo")
    slam_share = get_package_share_directory("slam_toolbox")
    default_slam_params = os.path.join(share, "config", "slam_toolbox.yaml")
    default_rviz = os.path.join(share, "config", "nav2.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "arm_idle_position",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        ),
        DeclareLaunchArgument("gripper_idle_position", default_value="0.0"),
        DeclareLaunchArgument("arm_idle_lock", default_value="true"),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "mujoco_lab.launch.py")
            ),
            launch_arguments={
                "use_viewer": LaunchConfiguration("use_viewer"),
                "arm_idle_position": LaunchConfiguration("arm_idle_position"),
                "gripper_idle_position": LaunchConfiguration(
                    "gripper_idle_position"
                ),
                "arm_idle_lock": LaunchConfiguration("arm_idle_lock"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
