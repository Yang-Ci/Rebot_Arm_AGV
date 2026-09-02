"""Launch the full Rebot laboratory with MuJoCo as the physics backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mujoco_share = get_package_share_directory("rebotarm_mujoco_rs")

    return LaunchDescription([
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("simulation_mode", default_value="physics"),
        DeclareLaunchArgument("agv_drive_mode", default_value="velocity"),
        DeclareLaunchArgument("agv_max_linear_velocity", default_value="0.80"),
        DeclareLaunchArgument("agv_max_reverse_velocity", default_value="0.50"),
        DeclareLaunchArgument("agv_max_angular_velocity", default_value="1.80"),
        DeclareLaunchArgument("enable_task_tools", default_value="false"),
        DeclareLaunchArgument("enable_wrist_camera", default_value="false"),
        DeclareLaunchArgument("enable_rs_driver", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mujoco_share, "launch", "mujoco_rs.launch.py")
            ),
            launch_arguments={
                "use_viewer": LaunchConfiguration("use_viewer"),
                "simulation_mode": LaunchConfiguration("simulation_mode"),
                "agv_drive_mode": LaunchConfiguration("agv_drive_mode"),
                "agv_max_linear_velocity": LaunchConfiguration(
                    "agv_max_linear_velocity"
                ),
                "agv_max_reverse_velocity": LaunchConfiguration(
                    "agv_max_reverse_velocity"
                ),
                "agv_max_angular_velocity": LaunchConfiguration(
                    "agv_max_angular_velocity"
                ),
                "publish_clock": "true",
                "use_sim_time": "true",
                "enable_task_tools": LaunchConfiguration("enable_task_tools"),
                "enable_wrist_camera": LaunchConfiguration("enable_wrist_camera"),
                "enable_rs_driver": LaunchConfiguration("enable_rs_driver"),
            }.items(),
        ),
    ])
