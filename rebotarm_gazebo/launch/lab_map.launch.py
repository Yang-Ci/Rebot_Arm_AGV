"""Publish the Rebot laboratory occupancy map with Nav2 map_server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("rebotarm_gazebo")
    default_map = os.path.join(share, "maps", "rebotarm_lab.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": LaunchConfiguration("map"), "use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
    ])
