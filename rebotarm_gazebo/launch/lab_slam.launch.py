"""Launch the Gazebo lab and online asynchronous SLAM Toolbox mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("rebotarm_gazebo")
    slam_share = get_package_share_directory("slam_toolbox")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "lab_world.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": os.path.join(share, "config", "slam_toolbox.yaml"),
            }.items(),
        ),
    ])
