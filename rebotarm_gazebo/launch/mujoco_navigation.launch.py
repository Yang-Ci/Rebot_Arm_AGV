"""Launch MuJoCo, a saved map, AMCL, Nav2, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ROSTimer, SetUseSimTime
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory("rebotarm_gazebo")
    default_map = os.path.join(share, "maps", "rebotarm_lab.yaml")
    default_params = os.path.join(share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(share, "config", "nav2.rviz")

    use_viewer = LaunchConfiguration("use_viewer")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_parameters = [configured_params, {"use_sim_time": True}]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription([
        # The lifecycle manager is started by a ROS-time timer below.  This
        # makes AMCL activation wait until MuJoCo has published a stable clock.
        SetUseSimTime(True),
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "arm_idle_position",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        ),
        DeclareLaunchArgument("gripper_idle_position", default_value="0.0"),
        DeclareLaunchArgument("arm_idle_lock", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "mujoco_lab.launch.py")
            ),
            launch_arguments={
                "use_viewer": use_viewer,
                "arm_idle_position": LaunchConfiguration("arm_idle_position"),
                "gripper_idle_position": LaunchConfiguration(
                    "gripper_idle_position"
                ),
                "arm_idle_lock": LaunchConfiguration("arm_idle_lock"),
            }.items(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                configured_params,
                {"yaml_filename": map_file, "use_sim_time": True},
            ],
            remappings=remappings,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
        ),
        ROSTimer(
            period=0.5,
            actions=[Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "autostart": autostart,
                    "node_names": lifecycle_nodes,
                    "use_sim_time": True,
                }],
            )],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", default_rviz],
            parameters=[{"use_sim_time": True}],
        ),
    ])
