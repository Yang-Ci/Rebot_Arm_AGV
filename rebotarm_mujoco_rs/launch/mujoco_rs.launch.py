from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm_rs"),
            DeclareLaunchArgument("input_topic", default_value=""),
            DeclareLaunchArgument("output_topic", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_footprint_frame", default_value="base_footprint"),
            DeclareLaunchArgument("base_link_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_clock", default_value="true"),
            DeclareLaunchArgument("clock_publish_rate", default_value="100.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("laser_topic", default_value="/scan"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("laser_enabled", default_value="true"),
            DeclareLaunchArgument("laser_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("laser_samples", default_value="360"),
            DeclareLaunchArgument("laser_min_angle", default_value="-3.141592653589793"),
            DeclareLaunchArgument("laser_max_angle", default_value="3.141592653589793"),
            DeclareLaunchArgument("laser_min_range", default_value="0.08"),
            DeclareLaunchArgument("laser_max_range", default_value="12.0"),
            DeclareLaunchArgument("laser_range_noise_stddev", default_value="0.005"),
            DeclareLaunchArgument("sensor_noise_seed", default_value="7"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("imu_enabled", default_value="true"),
            DeclareLaunchArgument("imu_publish_rate", default_value="100.0"),
            DeclareLaunchArgument("odom_publish_rate", default_value="30.0"),
            DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
            DeclareLaunchArgument("update_rate", default_value="250.0"),
            DeclareLaunchArgument("viewer_sync_rate", default_value="30.0"),
            DeclareLaunchArgument("smoothing_alpha", default_value="1.0"),
            DeclareLaunchArgument("stale_timeout", default_value="1.0"),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument(
                "arm_kp", default_value="[80.0, 100.0, 100.0, 35.0, 25.0, 18.0]"
            ),
            DeclareLaunchArgument(
                "arm_kd", default_value="[8.0, 10.0, 10.0, 4.0, 3.0, 2.5]"
            ),
            DeclareLaunchArgument(
                "arm_tau_limit",
                default_value="[36.0, 36.0, 36.0, 14.0, 14.0, 14.0]",
            ),
            DeclareLaunchArgument("gripper_kp", default_value="1800.0"),
            DeclareLaunchArgument("gripper_kd", default_value="18.0"),
            DeclareLaunchArgument("gripper_tau_limit", default_value="64.0"),
            DeclareLaunchArgument("enable_agv_drive", default_value="true"),
            DeclareLaunchArgument("agv_drive_mode", default_value="velocity"),
            DeclareLaunchArgument("agv_kp", default_value="2.5"),
            DeclareLaunchArgument("agv_kd", default_value="0.12"),
            DeclareLaunchArgument("agv_tau_limit", default_value="8.0"),
            DeclareLaunchArgument("agv_max_linear_velocity", default_value="0.80"),
            DeclareLaunchArgument("agv_max_reverse_velocity", default_value="0.50"),
            DeclareLaunchArgument("agv_max_angular_velocity", default_value="1.80"),
            DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
            DeclareLaunchArgument("enable_task_tools", default_value="true"),
            DeclareLaunchArgument("enable_wrist_camera", default_value="true"),
            DeclareLaunchArgument("enable_rs_driver", default_value="true"),
            DeclareLaunchArgument("driver_joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("driver_max_joint_speed", default_value="1.0"),
            DeclareLaunchArgument("driver_max_gripper_speed", default_value="5.0"),
            DeclareLaunchArgument("task_joint_state_topic", default_value=""),
            DeclareLaunchArgument("position_settle_tolerance", default_value="0.035"),
            DeclareLaunchArgument("position_settle_timeout", default_value="2.0"),
            DeclareLaunchArgument("velocity_settle_tolerance", default_value="0.25"),
            Node(
                package="rebotarm_mujoco_rs",
                executable="mujoco_rs_sync",
                name="rebotarm_rs_mujoco",
                output="screen",
                parameters=[
                    {
                        # This node drives /clock from a wall-clock timer.  It
                        # must never subscribe to its own simulated clock.
                        "use_sim_time": False,
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_footprint_frame": LaunchConfiguration(
                            "base_footprint_frame"
                        ),
                        "base_link_frame": LaunchConfiguration("base_link_frame"),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_tf"), value_type=bool
                        ),
                        "publish_clock": ParameterValue(
                            LaunchConfiguration("publish_clock"), value_type=bool
                        ),
                        "clock_publish_rate": ParameterValue(
                            LaunchConfiguration("clock_publish_rate"), value_type=float
                        ),
                        "laser_topic": LaunchConfiguration("laser_topic"),
                        "laser_frame": LaunchConfiguration("laser_frame"),
                        "laser_enabled": ParameterValue(
                            LaunchConfiguration("laser_enabled"), value_type=bool
                        ),
                        "laser_publish_rate": ParameterValue(
                            LaunchConfiguration("laser_publish_rate"),
                            value_type=float,
                        ),
                        "laser_samples": ParameterValue(
                            LaunchConfiguration("laser_samples"), value_type=int
                        ),
                        "laser_min_angle": ParameterValue(
                            LaunchConfiguration("laser_min_angle"), value_type=float
                        ),
                        "laser_max_angle": ParameterValue(
                            LaunchConfiguration("laser_max_angle"), value_type=float
                        ),
                        "laser_min_range": ParameterValue(
                            LaunchConfiguration("laser_min_range"), value_type=float
                        ),
                        "laser_max_range": ParameterValue(
                            LaunchConfiguration("laser_max_range"), value_type=float
                        ),
                        "laser_range_noise_stddev": ParameterValue(
                            LaunchConfiguration("laser_range_noise_stddev"),
                            value_type=float,
                        ),
                        "sensor_noise_seed": ParameterValue(
                            LaunchConfiguration("sensor_noise_seed"), value_type=int
                        ),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "imu_frame": LaunchConfiguration("imu_frame"),
                        "imu_enabled": ParameterValue(
                            LaunchConfiguration("imu_enabled"), value_type=bool
                        ),
                        "imu_publish_rate": ParameterValue(
                            LaunchConfiguration("imu_publish_rate"), value_type=float
                        ),
                        "odom_publish_rate": ParameterValue(
                            LaunchConfiguration("odom_publish_rate"),
                            value_type=float,
                        ),
                        "simulation_mode": LaunchConfiguration("simulation_mode"),
                        "update_rate": ParameterValue(
                            LaunchConfiguration("update_rate"), value_type=float
                        ),
                        "viewer_sync_rate": ParameterValue(
                            LaunchConfiguration("viewer_sync_rate"),
                            value_type=float,
                        ),
                        "smoothing_alpha": ParameterValue(
                            LaunchConfiguration("smoothing_alpha"), value_type=float
                        ),
                        "stale_timeout": ParameterValue(
                            LaunchConfiguration("stale_timeout"), value_type=float
                        ),
                        "use_viewer": ParameterValue(
                            LaunchConfiguration("use_viewer"), value_type=bool
                        ),
                        "arm_kp": ParameterValue(
                            LaunchConfiguration("arm_kp"), value_type=List[float]
                        ),
                        "arm_kd": ParameterValue(
                            LaunchConfiguration("arm_kd"), value_type=List[float]
                        ),
                        "arm_tau_limit": ParameterValue(
                            LaunchConfiguration("arm_tau_limit"),
                            value_type=List[float],
                        ),
                        "gripper_kp": ParameterValue(
                            LaunchConfiguration("gripper_kp"), value_type=float
                        ),
                        "gripper_kd": ParameterValue(
                            LaunchConfiguration("gripper_kd"), value_type=float
                        ),
                        "gripper_tau_limit": ParameterValue(
                            LaunchConfiguration("gripper_tau_limit"), value_type=float
                        ),
                        "enable_agv_drive": ParameterValue(
                            LaunchConfiguration("enable_agv_drive"), value_type=bool
                        ),
                        "agv_drive_mode": LaunchConfiguration("agv_drive_mode"),
                        "agv_kp": ParameterValue(
                            LaunchConfiguration("agv_kp"), value_type=float
                        ),
                        "agv_kd": ParameterValue(
                            LaunchConfiguration("agv_kd"), value_type=float
                        ),
                        "agv_tau_limit": ParameterValue(
                            LaunchConfiguration("agv_tau_limit"), value_type=float
                        ),
                        "agv_max_linear_velocity": ParameterValue(
                            LaunchConfiguration("agv_max_linear_velocity"),
                            value_type=float,
                        ),
                        "agv_max_reverse_velocity": ParameterValue(
                            LaunchConfiguration("agv_max_reverse_velocity"),
                            value_type=float,
                        ),
                        "agv_max_angular_velocity": ParameterValue(
                            LaunchConfiguration("agv_max_angular_velocity"),
                            value_type=float,
                        ),
                        "cmd_vel_timeout": ParameterValue(
                            LaunchConfiguration("cmd_vel_timeout"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_task_server",
                name="rebotarm_rs_task_server",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_task_tools")),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "model_path": LaunchConfiguration("model_path"),
                        "joint_state_topic": LaunchConfiguration(
                            "task_joint_state_topic"
                        ),
                        "position_settle_tolerance": ParameterValue(
                            LaunchConfiguration("position_settle_tolerance"),
                            value_type=float,
                        ),
                        "position_settle_timeout": ParameterValue(
                            LaunchConfiguration("position_settle_timeout"),
                            value_type=float,
                        ),
                        "velocity_settle_tolerance": ParameterValue(
                            LaunchConfiguration("velocity_settle_tolerance"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_driver",
                name="rebotarm_rs_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_rs_driver")),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "joint_state_rate": ParameterValue(
                            LaunchConfiguration("driver_joint_state_rate"),
                            value_type=float,
                        ),
                        "max_joint_speed": ParameterValue(
                            LaunchConfiguration("driver_max_joint_speed"),
                            value_type=float,
                        ),
                        "max_gripper_speed": ParameterValue(
                            LaunchConfiguration("driver_max_gripper_speed"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_scene_camera",
                name="rebotarm_rs_scene_camera",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_task_tools")),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "model_path": LaunchConfiguration("model_path"),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_scene_camera",
                name="rebotarm_rs_wrist_camera",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_wrist_camera")),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "model_path": LaunchConfiguration("model_path"),
                        "camera_name": "wrist_rgb",
                        "frame_id": "wrist_rgb_frame",
                        "publish_hz": 12.0,
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_scene_detector",
                name="rebotarm_rs_scene_detector",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_task_tools")),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                    }
                ],
            ),
        ]
    )
