"""Launch the Nav2 map saver server to persist a SLAM-generated occupancy grid.

After launching, save the current map with:

    ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
      "{map_url: 'rebotarm_lab_slam'}"

Or use the CLI shortcut:

    ros2 run nav2_map_server map_saver_cli -f rebotarm_lab_slam
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory("rebotarm_gazebo")
    default_params = os.path.join(share, "config", "nav2_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="nav2_map_server",
            executable="map_saver_server",
            name="map_saver",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_saver",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time, "autostart": True, "node_names": ["map_saver"]},
            ],
        ),
    ])
