"""Launch Gazebo and the matching Nav2 occupancy map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("rebotarm_gazebo")
    world_launch = os.path.join(share, "launch", "lab_world.launch.py")
    map_launch = os.path.join(share, "launch", "lab_map.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(map_launch)),
    ])
