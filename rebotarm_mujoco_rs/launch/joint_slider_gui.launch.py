from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    publish_hz = LaunchConfiguration("publish_hz")
    gripper_open_position = LaunchConfiguration("gripper_open_position")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm_rs"),
            DeclareLaunchArgument("publish_hz", default_value="30.0"),
            DeclareLaunchArgument("gripper_open_position", default_value="5.0"),
            Node(
                package="rebotarm_mujoco_rs",
                executable="joint_slider_gui",
                name="rebotarm_rs_joint_slider_gui",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "publish_hz": ParameterValue(publish_hz, value_type=float),
                        "gripper_open_position": ParameterValue(
                            gripper_open_position, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
