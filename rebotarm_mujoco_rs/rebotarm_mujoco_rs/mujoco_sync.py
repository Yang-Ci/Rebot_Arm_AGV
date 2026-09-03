from __future__ import annotations

from contextlib import nullcontext
import fcntl
import json
import os
from pathlib import Path
import tempfile
import time

import mujoco
import numpy as np
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
_RS_ROS_VISUAL_OPEN_M = 0.045
_LEGACY_VISUAL_OPEN_M = 0.0285
_MUJOCO_GRIPPER_OPEN_M = 0.05
_DEFAULT_MODEL_NAME = "rs_agv_scene.xml"
_DEFAULT_OBJECTS = ("red_cube", "blue_block", "yellow_cylinder")
_WHEEL_JOINTS = ("front_left", "front_right", "rear_left", "rear_right")
_WHEEL_ROS_JOINTS = tuple(f"{name}_wheel_joint" for name in _WHEEL_JOINTS)
_LASER_GEOMGROUP = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
_TARGET_MATERIAL_RGBA = np.array([1.0, 0.55, 0.12, 0.72], dtype=np.float32)
_TARGET_SITE_RGBA = np.array([1.0, 0.55, 0.12, 0.35], dtype=np.float32)
_TARGET_HIDDEN_POS = np.array([0.0, 0.0, -10.0], dtype=np.float64)
_TARGET_HIDDEN_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def find_default_model() -> Path:
    try:
        installed_model = (
            Path(get_package_share_directory("rebotarm_mujoco_rs"))
            / "models"
            / _DEFAULT_MODEL_NAME
        )
        if installed_model.is_file():
            return installed_model
    except PackageNotFoundError:
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        default_model = parent / "models" / _DEFAULT_MODEL_NAME
        if default_model.is_file():
            return default_model
    raise FileNotFoundError(
        "Integrated B601-RS MuJoCo model was not found in rebotarm_mujoco_rs. "
        "Build the ROS workspace or pass model_path explicitly."
    )


class RsMujocoSync(Node):
    """Run RS MuJoCo dynamics and synchronize them with ROS joint targets."""

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_mujoco")

        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("base_pose_topic", "")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("clock_publish_rate", 100.0)
        self.declare_parameter("laser_topic", "/scan")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("laser_enabled", True)
        self.declare_parameter("laser_publish_rate", 10.0)
        self.declare_parameter("laser_samples", 360)
        self.declare_parameter("laser_min_angle", -np.pi)
        self.declare_parameter("laser_max_angle", np.pi)
        self.declare_parameter("laser_min_range", 0.08)
        self.declare_parameter("laser_max_range", 12.0)
        self.declare_parameter("laser_range_noise_stddev", 0.005)
        self.declare_parameter("sensor_noise_seed", 7)
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("imu_enabled", True)
        self.declare_parameter("imu_publish_rate", 100.0)
        self.declare_parameter("odom_publish_rate", 30.0)
        self.declare_parameter("target_visible_timeout", 0.7)
        self.declare_parameter("model_path", "")
        self.declare_parameter("simulation_mode", "kinematic")
        self.declare_parameter("update_rate", 250.0)
        self.declare_parameter("viewer_sync_rate", 30.0)
        self.declare_parameter("smoothing_alpha", 1.0)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("arm_idle_position", [0.0] * 6)
        self.declare_parameter("gripper_idle_position", 0.0)
        self.declare_parameter("arm_idle_lock", True)
        self.declare_parameter("use_viewer", False)
        self.declare_parameter("object_names", list(_DEFAULT_OBJECTS))
        self.declare_parameter("object_publish_rate", 30.0)
        self.declare_parameter("arm_kp", [80.0, 100.0, 100.0, 35.0, 25.0, 18.0])
        self.declare_parameter("arm_kd", [8.0, 10.0, 10.0, 4.0, 3.0, 2.5])
        self.declare_parameter(
            "arm_tau_limit", [36.0, 36.0, 36.0, 14.0, 14.0, 14.0]
        )
        # Match the validated DM finger dynamics.  RS drives one coupler that
        # is equality-linked to both fingers, so its force limit is the sum of
        # DM's two 32 N finger limits.
        self.declare_parameter("gripper_kp", 1800.0)
        self.declare_parameter("gripper_kd", 18.0)
        self.declare_parameter("gripper_tau_limit", 64.0)
        self.declare_parameter("enable_agv_drive", True)
        self.declare_parameter("agv_drive_mode", "velocity")
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_track", 0.46)
        self.declare_parameter("agv_kp", 2.5)
        self.declare_parameter("agv_kd", 0.12)
        self.declare_parameter("agv_tau_limit", 8.0)
        self.declare_parameter("agv_max_linear_velocity", 0.80)
        self.declare_parameter("agv_max_reverse_velocity", 0.50)
        self.declare_parameter("agv_max_angular_velocity", 1.80)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("base_pose_publish_rate", 30.0)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        input_topic = input_topic or f"/{namespace}/joint_states"
        output_topic = output_topic or f"/{namespace}/mujoco/joint_states"
        target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
        ).strip()
        target_pose_topic = target_pose_topic or f"/{namespace}/mujoco/target_pose"
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value).strip()
        cmd_vel_topic = cmd_vel_topic or "/cmd_vel"
        base_pose_topic = str(self.get_parameter("base_pose_topic").value).strip()
        base_pose_topic = base_pose_topic or f"/{namespace}/mujoco/base_pose"
        odom_topic = str(self.get_parameter("odom_topic").value).strip()
        odom_topic = odom_topic or "/odom"
        laser_topic = str(self.get_parameter("laser_topic").value).strip()
        laser_topic = laser_topic or "/scan"
        imu_topic = str(self.get_parameter("imu_topic").value).strip()
        imu_topic = imu_topic or "/imu"

        requested_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = (
            Path(requested_path).expanduser().resolve()
            if requested_path
            else find_default_model()
        )
        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")

        self.simulation_mode = str(
            self.get_parameter("simulation_mode").value
        ).strip().lower()
        if self.simulation_mode not in ("kinematic", "physics"):
            raise ValueError("simulation_mode must be 'kinematic' or 'physics'")

        self.update_rate = max(float(self.get_parameter("update_rate").value), 1.0)
        self.viewer_sync_period = 1.0 / max(
            float(self.get_parameter("viewer_sync_rate").value), 1.0
        )
        self.smoothing_alpha = float(
            np.clip(self.get_parameter("smoothing_alpha").value, 0.01, 1.0)
        )
        self.stale_timeout = max(
            float(self.get_parameter("stale_timeout").value), 0.0
        )
        self.arm_kp = self._vector_parameter("arm_kp")
        self.arm_kd = self._vector_parameter("arm_kd")
        self.arm_tau_limit = self._vector_parameter("arm_tau_limit")
        self.arm_idle_position = self._vector_parameter("arm_idle_position")
        self.gripper_idle_position = float(
            self.get_parameter("gripper_idle_position").value
        )
        self.arm_idle_lock = bool(self.get_parameter("arm_idle_lock").value)
        if np.any(self.arm_kp < 0.0) or np.any(self.arm_kd < 0.0):
            raise ValueError("arm_kp and arm_kd values must be non-negative")
        if np.any(self.arm_tau_limit < 0.0):
            raise ValueError("arm_tau_limit values must be non-negative")
        self.gripper_kp = float(self.get_parameter("gripper_kp").value)
        self.gripper_kd = float(self.get_parameter("gripper_kd").value)
        self.gripper_tau_limit = float(self.get_parameter("gripper_tau_limit").value)
        self.enable_agv_drive = bool(
            self.get_parameter("enable_agv_drive").value
        )
        self.agv_drive_mode = str(
            self.get_parameter("agv_drive_mode").value
        ).strip().lower()
        if self.agv_drive_mode not in ("velocity", "wheel"):
            raise ValueError("agv_drive_mode must be 'velocity' or 'wheel'")
        self.wheel_radius = max(
            float(self.get_parameter("wheel_radius").value), 1e-4
        )
        self.wheel_track = max(float(self.get_parameter("wheel_track").value), 1e-3)
        self.agv_kp = max(float(self.get_parameter("agv_kp").value), 0.0)
        self.agv_kd = max(float(self.get_parameter("agv_kd").value), 0.0)
        self.agv_tau_limit = max(
            float(self.get_parameter("agv_tau_limit").value), 0.0
        )
        self.agv_max_linear_velocity = max(
            float(self.get_parameter("agv_max_linear_velocity").value), 0.0
        )
        self.agv_max_reverse_velocity = max(
            float(self.get_parameter("agv_max_reverse_velocity").value), 0.0
        )
        self.agv_max_angular_velocity = max(
            float(self.get_parameter("agv_max_angular_velocity").value), 0.0
        )
        self.cmd_vel_timeout = max(
            float(self.get_parameter("cmd_vel_timeout").value), 0.0
        )
        self.base_pose_period = 1.0 / max(
            float(self.get_parameter("base_pose_publish_rate").value), 1.0
        )
        self.odom_topic = odom_topic
        self.laser_topic = laser_topic
        self.odom_frame = (
            str(self.get_parameter("odom_frame").value).strip() or "odom"
        )
        self.base_footprint_frame = (
            str(self.get_parameter("base_footprint_frame").value).strip()
            or "base_footprint"
        )
        self.base_link_frame = (
            str(self.get_parameter("base_link_frame").value).strip() or "base_link"
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_clock = bool(self.get_parameter("publish_clock").value)
        self.clock_period = 1.0 / max(
            float(self.get_parameter("clock_publish_rate").value), 1.0
        )
        self.laser_frame = (
            str(self.get_parameter("laser_frame").value).strip() or "laser"
        )
        self.laser_enabled = bool(self.get_parameter("laser_enabled").value)
        self.laser_period = 1.0 / max(
            float(self.get_parameter("laser_publish_rate").value), 1.0
        )
        self.laser_samples = max(
            int(self.get_parameter("laser_samples").value), 2
        )
        self.laser_min_angle = float(
            self.get_parameter("laser_min_angle").value
        )
        self.laser_max_angle = float(
            self.get_parameter("laser_max_angle").value
        )
        if self.laser_max_angle <= self.laser_min_angle:
            raise ValueError("laser_max_angle must be greater than laser_min_angle")
        self.laser_min_range = max(
            float(self.get_parameter("laser_min_range").value), 0.0
        )
        self.laser_max_range = max(
            float(self.get_parameter("laser_max_range").value),
            self.laser_min_range + 1e-6,
        )
        self.laser_range_noise_stddev = max(
            float(self.get_parameter("laser_range_noise_stddev").value), 0.0
        )
        self._sensor_rng = np.random.default_rng(
            int(self.get_parameter("sensor_noise_seed").value)
        )
        self.laser_angles = np.linspace(
            self.laser_min_angle,
            self.laser_max_angle,
            self.laser_samples,
            endpoint=False,
        )
        self.odom_period = 1.0 / max(
            float(self.get_parameter("odom_publish_rate").value), 1.0
        )
        self.imu_topic = imu_topic
        self.imu_frame = (
            str(self.get_parameter("imu_frame").value).strip() or "imu_link"
        )
        self.imu_enabled = bool(self.get_parameter("imu_enabled").value)
        self.imu_period = 1.0 / max(
            float(self.get_parameter("imu_publish_rate").value), 1.0
        )
        self.target_visible_timeout = max(
            float(self.get_parameter("target_visible_timeout").value),
            0.0,
        )

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        # MuJoCo resets data.time to zero.  Keep a process-local epoch so ROS
        # time and every stamped sensor message remain monotonic across reset.
        self._sim_time_epoch = 0.0
        mujoco.mj_forward(self.model, self.data)
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "ik_target"
        )
        if target_body_id >= 0:
            self.target_mocap_id = int(self.model.body_mocapid[target_body_id])
            if self.target_mocap_id < 0:
                self.get_logger().warn(
                    "ik_target is not a mocap body; target pose visualization disabled"
                )
        else:
            self.target_mocap_id = -1
            self.get_logger().warn(
                "ik_target body not found; target pose visualization disabled"
            )
        self.target_mat_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_MATERIAL, "target_mat"
        )
        self.target_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "ik_target_sphere"
        )
        self.target_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ik_target_site"
        )
        self.arm_joint_ids = np.array(
            [self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in _ARM_JOINTS]
        )
        self.arm_qpos_addrs = self.model.jnt_qposadr[self.arm_joint_ids]
        self.arm_dof_addrs = self.model.jnt_dofadr[self.arm_joint_ids]
        self.arm_actuator_ids = np.array(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor")
                for name in _ARM_JOINTS
            ]
        )
        self.gripper_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint7"
        )
        self.gripper_qpos_addr = int(self.model.jnt_qposadr[self.gripper_joint_id])
        self.gripper_dof_addr = int(self.model.jnt_dofadr[self.gripper_joint_id])
        self.gripper_actuator_id = self._required_id(
            mujoco.mjtObj.mjOBJ_ACTUATOR, "joint7_motor"
        )
        self.left_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint_left"
        )
        self.right_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint_right"
        )
        self.left_qpos_addr = int(self.model.jnt_qposadr[self.left_joint_id])
        self.right_qpos_addr = int(self.model.jnt_qposadr[self.right_joint_id])
        self.left_dof_addr = int(self.model.jnt_dofadr[self.left_joint_id])
        self.right_dof_addr = int(self.model.jnt_dofadr[self.right_joint_id])
        self.base_body_id = self._required_id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "agv_freejoint"
        )
        self.base_qpos_addr = int(self.model.jnt_qposadr[base_joint_id])
        self.base_dof_addr = int(self.model.jnt_dofadr[base_joint_id])
        self.laser_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, self.laser_frame
        )
        if self.laser_body_id < 0:
            if self.laser_enabled:
                self.get_logger().warn(
                    f"MuJoCo body '{self.laser_frame}' not found; laser disabled"
                )
            self.laser_enabled = False
            self._base_to_laser_pos = np.zeros(3, dtype=np.float64)
            self._base_to_laser_quat = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )
        else:
            base_position = self.data.xpos[self.base_body_id]
            base_rotation = self.data.xmat[self.base_body_id].reshape(3, 3)
            laser_position = self.data.xpos[self.laser_body_id]
            laser_rotation = self.data.xmat[self.laser_body_id].reshape(3, 3)
            relative_rotation = base_rotation.T @ laser_rotation
            relative_quat = np.zeros(4, dtype=np.float64)
            mujoco.mju_mat2Quat(relative_quat, relative_rotation.flatten())
            self._base_to_laser_pos = base_rotation.T @ (
                laser_position - base_position
            )
            self._base_to_laser_quat = relative_quat
        self.imu_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "imu"
        )
        if self.imu_body_id < 0:
            if self.imu_enabled:
                self.get_logger().warn("MuJoCo body 'imu' not found; IMU disabled")
            self.imu_enabled = False
            self._base_to_imu_pos = np.zeros(3, dtype=np.float64)
            self._base_to_imu_quat = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )
        else:
            base_position = self.data.xpos[self.base_body_id]
            base_rotation = self.data.xmat[self.base_body_id].reshape(3, 3)
            imu_position = self.data.xpos[self.imu_body_id]
            imu_rotation = self.data.xmat[self.imu_body_id].reshape(3, 3)
            relative_rotation = base_rotation.T @ imu_rotation
            relative_quat = np.zeros(4, dtype=np.float64)
            mujoco.mju_mat2Quat(relative_quat, relative_rotation.flatten())
            self._base_to_imu_pos = base_rotation.T @ (
                imu_position - base_position
            )
            self._base_to_imu_quat = relative_quat
        self._ray_geom_id = np.zeros(1, dtype=np.int32)
        self.wheel_joint_ids = np.array(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_JOINT, f"wheel_{name}_joint")
                for name in _WHEEL_JOINTS
            ]
        )
        self.wheel_qpos_addrs = self.model.jnt_qposadr[self.wheel_joint_ids]
        self.wheel_dof_addrs = self.model.jnt_dofadr[self.wheel_joint_ids]
        self.wheel_actuator_ids = np.array(
            [
                self._required_id(
                    mujoco.mjtObj.mjOBJ_ACTUATOR, f"wheel_{name}_motor"
                )
                for name in _WHEEL_JOINTS
            ]
        )
        self.object_names = [
            str(name) for name in self.get_parameter("object_names").value
        ]
        self.object_body_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in self.object_names
        }
        self.object_body_ids = {
            name: body_id
            for name, body_id in self.object_body_ids.items()
            if body_id >= 0
        }
        self.object_publish_period = 1.0 / max(
            float(self.get_parameter("object_publish_rate").value), 1.0
        )

        self._target_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._target_pose_monotonic: float | None = None
        self._target_visible = True
        self._set_target_visible_locked(False)

        self.target_arm = self.arm_idle_position.copy()
        self.target_gripper = self.gripper_idle_position
        self.last_input_time: float | None = None
        self.last_publish_time = 0.0
        self.last_object_publish_time = 0.0
        self.last_base_pose_publish_time = 0.0
        self.last_odom_publish_time = 0.0
        self.last_tf_publish_time = 0.0
        self.last_laser_publish_time = 0.0
        self.last_clock_publish_sim_time = -np.inf
        self.last_imu_publish_sim_time = -np.inf
        self._reset_odom_reference_locked()
        self._reset_imu_reference_locked()
        self._cmd_vel = np.zeros(2, dtype=np.float64)
        self._cmd_vel_time: float | None = None
        self._physics_step_accumulator = 0.0
        self._last_update_time = time.monotonic()
        self._last_viewer_sync_time = -np.inf

        self.publisher = self.create_publisher(
            JointState,
            output_topic,
            qos_profile_sensor_data,
        )
        self.object_publisher = self.create_publisher(
            String,
            f"/{namespace}/mujoco/object_states",
            10,
        )
        self.base_pose_publisher = self.create_publisher(
            PoseStamped,
            base_pose_topic,
            10,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            odom_topic,
            10,
        )
        self.laser_publisher = self.create_publisher(
            LaserScan,
            laser_topic,
            qos_profile_sensor_data,
        )
        self._clock_lock_fd: int | None = None
        if self.publish_clock:
            self._acquire_clock_source_lock()
        self.clock_publisher = self.create_publisher(
            Clock, "/clock", qos_profile_sensor_data
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            imu_topic,
            qos_profile_sensor_data,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )
        self.reset_service = self.create_service(
            Trigger,
            f"/{namespace}/mujoco/reset",
            self._reset,
        )

        self.viewer = None
        if bool(self.get_parameter("use_viewer").value):
            from mujoco import viewer as mujoco_viewer

            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(1.0 / self.update_rate, self._update)
        self.get_logger().info(
            f"RS MuJoCo ready: mode={self.simulation_mode}, "
            f"agv_drive={self.agv_drive_mode}, input={input_topic}, "
            f"output={output_topic}, target_pose={target_pose_topic}, "
            f"cmd_vel={cmd_vel_topic}, base_pose={base_pose_topic}, "
            f"odom={odom_topic}, scan={laser_topic}, imu={imu_topic}, "
            f"clock={'on' if self.publish_clock else 'off'}, "
            f"model={self.model_path}"
        )

    def _base_planar_pose_locked(self) -> tuple[np.ndarray, np.ndarray, float]:
        position = self.data.xpos[self.base_body_id]
        rotation = self.data.xmat[self.base_body_id].reshape(3, 3)
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
        return position.copy(), rotation, yaw

    def _reset_odom_reference_locked(self) -> None:
        position, _, yaw = self._base_planar_pose_locked()
        self._odom_origin_position = position[:2].copy()
        self._odom_origin_yaw = yaw
        self._last_odom_position = position[:2].copy()
        self._last_odom_yaw = 0.0
        self._last_odom_time = float(self.data.time)

    def _reset_imu_reference_locked(self) -> None:
        position, _, yaw = self._base_planar_pose_locked()
        self._imu_last_position = position.copy()
        self._imu_last_velocity = np.zeros(3, dtype=np.float64)
        self._imu_last_yaw = yaw
        self._imu_last_sim_time = float(self.data.time)

    def _simulation_time(self) -> float:
        return max(self._sim_time_epoch + float(self.data.time), 0.0)

    def _sim_stamp(self) -> TimeMsg:
        seconds = self._simulation_time()
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return TimeMsg(sec=sec, nanosec=nanosec)

    def _acquire_clock_source_lock(self) -> None:
        """Prevent two MuJoCo clock sources in the same ROS domain."""
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        safe_domain = "".join(
            character if character.isalnum() else "_" for character in domain
        )
        lock_path = Path(tempfile.gettempdir()) / (
            f"rebotarm_mujoco_clock_{os.getuid()}_domain_{safe_domain}.lock"
        )
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            os.close(lock_fd)
            raise RuntimeError(
                "Another RebotArm MuJoCo /clock source is already running "
                f"in ROS domain {domain}. Stop it or use a different "
                "ROS_DOMAIN_ID before starting another simulation."
            ) from error
        self._clock_lock_fd = lock_fd

    @staticmethod
    def _advance_schedule(
        now: float, last_deadline: float, period: float
    ) -> tuple[bool, float]:
        """Keep periodic publishers on their requested average simulation rate."""
        if not np.isfinite(last_deadline):
            return True, now
        elapsed = now - last_deadline
        if elapsed + 1e-12 < period:
            return False, last_deadline
        periods = max(1, int(np.floor((elapsed + 1e-12) / period)))
        return True, last_deadline + periods * period

    def _odom_pose_locked(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        position, rotation, yaw = self._base_planar_pose_locked()
        delta = position[:2] - self._odom_origin_position
        yaw_delta = yaw - self._odom_origin_yaw
        yaw_delta = float(
            np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta))
        )
        odom_position = np.array(
            [
                np.cos(self._odom_origin_yaw) * delta[0]
                + np.sin(self._odom_origin_yaw) * delta[1],
                -np.sin(self._odom_origin_yaw) * delta[0]
                + np.cos(self._odom_origin_yaw) * delta[1],
            ],
            dtype=np.float64,
        )
        return odom_position, position, rotation, yaw_delta

    @staticmethod
    def _yaw_quaternion(yaw: float) -> np.ndarray:
        return np.array(
            [np.cos(0.5 * yaw), 0.0, 0.0, np.sin(0.5 * yaw)],
            dtype=np.float64,
        )

    def _vector_parameter(self, name: str) -> np.ndarray:
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if values.shape != (6,):
            raise ValueError(f"{name} must contain 6 values")
        return values

    def _required_id(self, object_type, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"required MuJoCo object not found: {name}")
        return int(object_id)

    def _joint_state_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        for index, name in enumerate(_ARM_JOINTS):
            if name in values and np.isfinite(values[name]):
                self.target_arm[index] = float(values[name])

        if "gripper_joint1" in values:
            self.target_gripper = self._visual_to_mujoco_gripper(
                values["gripper_joint1"], _RS_ROS_VISUAL_OPEN_M
            )
        elif "finger_left" in values:
            self.target_gripper = self._visual_to_mujoco_gripper(
                values["finger_left"], _LEGACY_VISUAL_OPEN_M
            )
        self.last_input_time = time.monotonic()

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        pos = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=np.float64,
        )
        quat = np.array(
            [
                float(msg.pose.orientation.w),
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quat))
        quat = _TARGET_HIDDEN_QUAT.copy() if norm < 1e-9 else quat / norm
        self._target_pose = (pos, quat)
        self._target_pose_monotonic = time.monotonic()

    def _cmd_vel_callback(self, msg: Twist) -> None:
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if not (np.isfinite(linear) and np.isfinite(angular)):
            return
        self._cmd_vel = np.array(
            [
                np.clip(
                    linear,
                    -self.agv_max_reverse_velocity,
                    self.agv_max_linear_velocity,
                ),
                np.clip(angular, -self.agv_max_angular_velocity, self.agv_max_angular_velocity),
            ],
            dtype=np.float64,
        )
        self._cmd_vel_time = time.monotonic()

    def _active_cmd_vel(self) -> np.ndarray:
        if not self.enable_agv_drive or self._cmd_vel_time is None:
            return np.zeros(2, dtype=np.float64)
        if (
            self.cmd_vel_timeout > 0.0
            and time.monotonic() - self._cmd_vel_time > self.cmd_vel_timeout
        ):
            return np.zeros(2, dtype=np.float64)
        return self._cmd_vel

    @staticmethod
    def _quat_to_matrix(quat: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(quat))
        if norm < 1e-9:
            return np.eye(3, dtype=np.float64)
        w, x, y, z = quat / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
        lw, lx, ly, lz = left
        rw, rx, ry, rz = right
        return np.array(
            [
                lw * rw - lx * rx - ly * ry - lz * rz,
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
            ],
            dtype=np.float64,
        )

    def _wheel_velocity_targets(self, cmd_vel: np.ndarray) -> np.ndarray:
        linear, angular = cmd_vel
        left_linear = linear - 0.5 * self.wheel_track * angular
        right_linear = linear + 0.5 * self.wheel_track * angular
        return np.array(
            [left_linear, right_linear, left_linear, right_linear],
            dtype=np.float64,
        ) / self.wheel_radius

    @staticmethod
    def _visual_to_mujoco_gripper(position: float, visual_open: float) -> float:
        ratio = np.clip(float(position) / visual_open, 0.0, 1.0)
        return float(ratio * _MUJOCO_GRIPPER_OPEN_M)

    def _lock_arm_idle(self) -> None:
        """Parking-brake-style lock for navigation without arm commands."""
        self.data.qpos[self.arm_qpos_addrs] = self.arm_idle_position
        self.data.qvel[self.arm_dof_addrs] = 0.0
        self.data.qpos[self.gripper_qpos_addr] = self.gripper_idle_position
        self.data.qpos[self.left_qpos_addr] = self.gripper_idle_position
        self.data.qpos[self.right_qpos_addr] = self.gripper_idle_position
        self.data.qvel[
            [self.gripper_dof_addr, self.left_dof_addr, self.right_dof_addr]
        ] = 0.0

    def _update(self) -> None:
        now = time.monotonic()
        # Viewer synchronization and occasional ROS scheduling delays can make
        # one callback late. Advance physics by elapsed wall time so simulation
        # does not fall into slow motion, while limiting a single catch-up step.
        dt = min(max(now - self._last_update_time, 0.0), 0.05)
        self._last_update_time = now

        arm_command_active = self.last_input_time is not None and (
            self.stale_timeout <= 0.0
            or now - self.last_input_time <= self.stale_timeout
        )
        if not arm_command_active:
            # Without a fresh arm command, keep actively driving to the parked
            # pose instead of adopting whatever pose inertia has disturbed.
            self.target_arm = self.arm_idle_position.copy()
            self.target_gripper = self.gripper_idle_position

        viewer_lock = self.viewer.lock() if self.viewer is not None else nullcontext()
        with viewer_lock:
            if self.simulation_mode == "kinematic":
                self._update_kinematic(dt)
            else:
                self._update_physics(dt, arm_command_active)
            if self._apply_target_pose_locked():
                mujoco.mj_forward(self.model, self.data)
            self._publish_clock_locked()
            self._publish_state()
            self._publish_object_states()
            self._publish_base_pose()
            self._publish_odom_locked()
            self._publish_laser_locked()
            self._publish_imu_locked()
            self._publish_tf_locked()
        if self.viewer is not None:
            if self.viewer.is_running():
                if now - self._last_viewer_sync_time >= self.viewer_sync_period:
                    self.viewer.sync()
                    self._last_viewer_sync_time = now
            else:
                self.viewer.close()
                self.viewer = None

    def _update_kinematic(self, dt: float) -> None:
        current = self.data.qpos[self.arm_qpos_addrs]
        next_arm = current + self.smoothing_alpha * (self.target_arm - current)
        self.data.qpos[self.arm_qpos_addrs] = next_arm
        self.data.qvel[self.arm_dof_addrs] = 0.0

        current_gripper = float(self.data.qpos[self.gripper_qpos_addr])
        next_gripper = current_gripper + self.smoothing_alpha * (
            self.target_gripper - current_gripper
        )
        self.data.qpos[self.gripper_qpos_addr] = next_gripper
        self.data.qpos[self.left_qpos_addr] = next_gripper
        self.data.qpos[self.right_qpos_addr] = next_gripper
        self.data.qvel[
            [self.gripper_dof_addr, self.left_dof_addr, self.right_dof_addr]
        ] = 0.0

        cmd_vel = self._active_cmd_vel()
        wheel_velocity = self._wheel_velocity_targets(cmd_vel)
        if dt > 0.0:
            self._integrate_base_pose_locked(cmd_vel, dt)
            self.data.qpos[self.wheel_qpos_addrs] += wheel_velocity * dt
            self.data.time += dt
        self.data.qvel[self.wheel_dof_addrs] = wheel_velocity
        mujoco.mj_forward(self.model, self.data)

    def _integrate_base_pose_locked(
        self, cmd_vel: np.ndarray, dt: float
    ) -> None:
        linear, angular = cmd_vel
        position = self.data.qpos[
            self.base_qpos_addr : self.base_qpos_addr + 3
        ]
        quat = self.data.qpos[
            self.base_qpos_addr + 3 : self.base_qpos_addr + 7
        ].copy()
        quat /= max(float(np.linalg.norm(quat)), 1e-9)

        rotation = self._quat_to_matrix(quat)
        position += rotation[:, 0] * linear * dt
        angle = angular * dt
        yaw_delta = np.array(
            [np.cos(0.5 * angle), 0.0, 0.0, np.sin(0.5 * angle)],
            dtype=np.float64,
        )
        quat = self._quat_multiply(quat, yaw_delta)
        quat /= max(float(np.linalg.norm(quat)), 1e-9)

        self.data.qpos[self.base_qpos_addr : self.base_qpos_addr + 3] = position
        self.data.qpos[
            self.base_qpos_addr + 3 : self.base_qpos_addr + 7
        ] = quat

    def _apply_target_pose_locked(self) -> bool:
        if self.target_mocap_id < 0:
            return False
        if self._target_pose is None:
            return self._set_target_visible_locked(False)

        now = time.monotonic()
        visible = (
            self._target_pose_monotonic is not None
            and now - self._target_pose_monotonic <= self.target_visible_timeout
        )
        visual_changed = self._set_target_visible_locked(visible)
        if not visible:
            if not np.allclose(
                self.data.mocap_pos[self.target_mocap_id], _TARGET_HIDDEN_POS
            ):
                self.data.mocap_pos[self.target_mocap_id] = _TARGET_HIDDEN_POS
                self.data.mocap_quat[self.target_mocap_id] = _TARGET_HIDDEN_QUAT
                return True
            return visual_changed

        pos, quat = self._target_pose
        changed = not (
            np.allclose(self.data.mocap_pos[self.target_mocap_id], pos)
            and np.allclose(self.data.mocap_quat[self.target_mocap_id], quat)
        )
        self.data.mocap_pos[self.target_mocap_id] = pos
        self.data.mocap_quat[self.target_mocap_id] = quat
        return bool(changed or visual_changed)

    def _set_target_visible_locked(self, visible: bool) -> bool:
        if self._target_visible == visible:
            return False
        self._target_visible = visible
        alpha = 1.0 if visible else 0.0
        if self.target_mat_id >= 0:
            rgba = _TARGET_MATERIAL_RGBA.copy()
            rgba[3] *= alpha
            self.model.mat_rgba[self.target_mat_id] = rgba
        if self.target_geom_id >= 0:
            rgba = _TARGET_MATERIAL_RGBA.copy()
            rgba[3] *= alpha
            self.model.geom_rgba[self.target_geom_id] = rgba
        if self.target_site_id >= 0:
            rgba = _TARGET_SITE_RGBA.copy()
            rgba[3] *= alpha
            self.model.site_rgba[self.target_site_id] = rgba
        return True

    def _update_physics(self, dt: float, arm_command_active: bool) -> None:
        timestep = float(self.model.opt.timestep)
        self._physics_step_accumulator += dt
        steps = int(self._physics_step_accumulator / timestep)
        if steps < 1:
            return
        self._physics_step_accumulator -= steps * timestep
        lock_idle_arm = self.arm_idle_lock and not arm_command_active
        for _ in range(steps):
            q = self.data.qpos[self.arm_qpos_addrs]
            qd = self.data.qvel[self.arm_dof_addrs]
            dynamics_bias = self.data.qfrc_bias[self.arm_dof_addrs]
            # MuJoCo uses M*qacc + qfrc_bias = applied forces, so holding a
            # pose requires the bias force plus damping.
            tau = dynamics_bias + self.arm_kp * (self.target_arm - q) - self.arm_kd * qd
            self.data.ctrl[self.arm_actuator_ids] = np.clip(
                tau, -self.arm_tau_limit, self.arm_tau_limit
            )

            gripper_q = float(self.data.qpos[self.gripper_qpos_addr])
            gripper_qd = float(self.data.qvel[self.gripper_dof_addr])
            gripper_tau = self.gripper_kp * (
                self.target_gripper - gripper_q
            ) - self.gripper_kd * gripper_qd
            self.data.ctrl[self.gripper_actuator_id] = float(
                np.clip(gripper_tau, -self.gripper_tau_limit, self.gripper_tau_limit)
            )
            self._apply_wheel_physics_control()
            if self.agv_drive_mode == "velocity":
                self._apply_base_velocity_control()
            mujoco.mj_step(self.model, self.data)
            if lock_idle_arm:
                self._lock_arm_idle()
        if lock_idle_arm:
            mujoco.mj_forward(self.model, self.data)

    def _apply_wheel_physics_control(self) -> None:
        wheel_velocity_target = self._wheel_velocity_targets(self._active_cmd_vel())
        wheel_velocity = self.data.qvel[self.wheel_dof_addrs]
        wheel_tau = self.agv_kp * (
            wheel_velocity_target - wheel_velocity
        ) - self.agv_kd * wheel_velocity
        self.data.ctrl[self.wheel_actuator_ids] = np.clip(
            wheel_tau, -self.agv_tau_limit, self.agv_tau_limit
        )

    def _apply_base_velocity_control(self) -> None:
        """Track planar velocity like Gazebo's model velocity controller."""
        linear, angular = self._active_cmd_vel()
        rotation = self.data.xmat[self.base_body_id].reshape(3, 3)
        forward = rotation[:, 0]
        base_velocity = self.data.qvel[
            self.base_dof_addr : self.base_dof_addr + 6
        ]
        base_velocity[0] = float(forward[0] * linear)
        base_velocity[1] = float(forward[1] * linear)
        base_velocity[3] = 0.0
        base_velocity[4] = 0.0
        base_velocity[5] = float(angular)

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self._sim_stamp()
        msg.name = [
            *_ARM_JOINTS,
            "gripper_joint1",
            "gripper_joint2",
            *_WHEEL_ROS_JOINTS,
        ]
        msg.position = [
            *[float(value) for value in self.data.qpos[self.arm_qpos_addrs]],
            float(self.data.qpos[self.left_qpos_addr]),
            float(self.data.qpos[self.right_qpos_addr]),
            *[float(value) for value in self.data.qpos[self.wheel_qpos_addrs]],
        ]
        msg.velocity = [
            *[float(value) for value in self.data.qvel[self.arm_dof_addrs]],
            float(self.data.qvel[self.left_dof_addr]),
            float(self.data.qvel[self.right_dof_addr]),
            *[float(value) for value in self.data.qvel[self.wheel_dof_addrs]],
        ]
        msg.effort = [
            *[float(value) for value in self.data.qfrc_actuator[self.arm_dof_addrs]],
            0.0,
            0.0,
            *[
                float(value)
                for value in self.data.qfrc_actuator[self.wheel_dof_addrs]
            ],
        ]
        self.publisher.publish(msg)
        self.last_publish_time = time.monotonic()

    def _publish_object_states(self) -> None:
        now = float(self.data.time)
        if now - self.last_object_publish_time < self.object_publish_period:
            return
        objects = []
        for name, body_id in self.object_body_ids.items():
            quat_wxyz = [float(value) for value in self.data.xquat[body_id]]
            objects.append(
                {
                    "name": name,
                    "position": [float(value) for value in self.data.xpos[body_id]],
                    "quaternion": quat_wxyz,
                    "quat_wxyz": quat_wxyz,
                }
            )
        msg = String()
        msg.data = json.dumps(
            {"objects": objects, "simulation_mode": self.simulation_mode},
            separators=(",", ":"),
        )
        self.object_publisher.publish(msg)
        self.last_object_publish_time = now

    def _publish_base_pose(self) -> None:
        now = float(self.data.time)
        if now - self.last_base_pose_publish_time < self.base_pose_period:
            return
        msg = PoseStamped()
        msg.header.stamp = self._sim_stamp()
        msg.header.frame_id = "world"
        position = self.data.xpos[self.base_body_id]
        quaternion = self.data.xquat[self.base_body_id]
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = float(quaternion[0])
        msg.pose.orientation.x = float(quaternion[1])
        msg.pose.orientation.y = float(quaternion[2])
        msg.pose.orientation.z = float(quaternion[3])
        self.base_pose_publisher.publish(msg)
        self.last_base_pose_publish_time = now

    def _publish_odom_locked(self) -> None:
        now = float(self.data.time)
        if now - self.last_odom_publish_time < self.odom_period:
            return

        odom_position, world_position, rotation, yaw = self._odom_pose_locked()
        elapsed = max(now - self._last_odom_time, 1e-6)
        world_delta = world_position[:2] - self._last_odom_position
        linear_velocity = float(
            np.dot(rotation[:, 0], np.append(world_delta, 0.0)) / elapsed
        )
        yaw_delta = yaw - self._last_odom_yaw
        yaw_delta = float(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta)))
        angular_velocity = yaw_delta / elapsed

        msg = Odometry()
        msg.header.stamp = self._sim_stamp()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_footprint_frame
        msg.pose.pose.position.x = float(odom_position[0])
        msg.pose.pose.position.y = float(odom_position[1])
        odom_quat = self._yaw_quaternion(yaw)
        msg.pose.pose.orientation.w = float(odom_quat[0])
        msg.pose.pose.orientation.x = float(odom_quat[1])
        msg.pose.pose.orientation.y = float(odom_quat[2])
        msg.pose.pose.orientation.z = float(odom_quat[3])
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(msg)

        self._last_odom_position = world_position[:2].copy()
        self._last_odom_yaw = yaw
        self._last_odom_time = now
        self.last_odom_publish_time = now

    def _publish_laser_locked(self) -> None:
        if not self.laser_enabled:
            return
        now = float(self.data.time)
        if now - self.last_laser_publish_time < self.laser_period:
            return

        origin = self.data.xpos[self.laser_body_id]
        rotation = self.data.xmat[self.laser_body_id].reshape(3, 3)
        forward = rotation[:, 0]
        left = rotation[:, 1]
        ranges = np.full(self.laser_samples, np.inf, dtype=np.float64)
        for index, angle in enumerate(self.laser_angles):
            direction = forward * np.cos(angle) + left * np.sin(angle)
            distance = mujoco.mj_ray(
                self.model,
                self.data,
                origin,
                direction,
                _LASER_GEOMGROUP,
                True,
                self.base_body_id,
                self._ray_geom_id,
            )
            if (
                distance > 0.0
                and self.laser_min_range <= distance <= self.laser_max_range
            ):
                if self.laser_range_noise_stddev > 0.0:
                    distance += float(
                        self._sensor_rng.normal(0.0, self.laser_range_noise_stddev)
                    )
                    distance = float(
                        np.clip(distance, self.laser_min_range, self.laser_max_range)
                    )
                ranges[index] = distance

        msg = LaserScan()
        msg.header.stamp = self._sim_stamp()
        msg.header.frame_id = self.laser_frame
        msg.angle_min = float(self.laser_angles[0])
        msg.angle_max = float(self.laser_angles[-1])
        msg.angle_increment = float(
            (self.laser_max_angle - self.laser_min_angle) / self.laser_samples
        )
        msg.time_increment = 0.0
        msg.scan_time = float(self.laser_period)
        msg.range_min = float(self.laser_min_range)
        msg.range_max = float(self.laser_max_range)
        msg.ranges = [float(value) for value in ranges]
        self.laser_publisher.publish(msg)
        self.last_laser_publish_time = now

    def _publish_clock_locked(self) -> None:
        if not self.publish_clock:
            return
        now = float(self.data.time)
        due, next_deadline = self._advance_schedule(
            now, self.last_clock_publish_sim_time, self.clock_period
        )
        if not due:
            return
        msg = Clock()
        msg.clock = self._sim_stamp()
        self.clock_publisher.publish(msg)
        self.last_clock_publish_sim_time = next_deadline

    def _publish_imu_locked(self) -> None:
        if not self.imu_enabled:
            return
        now = float(self.data.time)
        due, next_deadline = self._advance_schedule(
            now, self.last_imu_publish_sim_time, self.imu_period
        )
        if not due:
            return

        position, rotation, yaw = self._base_planar_pose_locked()
        elapsed = now - self._imu_last_sim_time
        if elapsed > 1e-9:
            world_velocity = (position - self._imu_last_position) / elapsed
            world_acceleration = (
                world_velocity - self._imu_last_velocity
            ) / elapsed
            yaw_delta = float(
                np.arctan2(
                    np.sin(yaw - self._imu_last_yaw),
                    np.cos(yaw - self._imu_last_yaw),
                )
            )
            angular_velocity = yaw_delta / elapsed
        else:
            world_velocity = np.zeros(3, dtype=np.float64)
            world_acceleration = np.zeros(3, dtype=np.float64)
            angular_velocity = 0.0

        # An accelerometer at rest measures +g along its local Z axis.
        specific_force = rotation.T @ (
            world_acceleration + np.array([0.0, 0.0, 9.81])
        )
        quaternion = self.data.xquat[self.base_body_id]
        msg = Imu()
        msg.header.stamp = self._sim_stamp()
        msg.header.frame_id = self.imu_frame
        msg.orientation.w = float(quaternion[0])
        msg.orientation.x = float(quaternion[1])
        msg.orientation.y = float(quaternion[2])
        msg.orientation.z = float(quaternion[3])
        msg.angular_velocity.z = float(angular_velocity)
        msg.linear_acceleration.x = float(specific_force[0])
        msg.linear_acceleration.y = float(specific_force[1])
        msg.linear_acceleration.z = float(specific_force[2])
        msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025]
        msg.angular_velocity_covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001
        ]
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02
        ]
        self.imu_publisher.publish(msg)

        self._imu_last_position = position
        self._imu_last_velocity = world_velocity
        self._imu_last_yaw = yaw
        self._imu_last_sim_time = now
        self.last_imu_publish_sim_time = next_deadline

    def _publish_tf_locked(self) -> None:
        if not self.publish_tf:
            return
        now = float(self.data.time)
        if now - self.last_tf_publish_time < self.odom_period:
            return

        odom_position, world_position, _, yaw = self._odom_pose_locked()
        stamp = self._sim_stamp()
        odom_to_footprint = TransformStamped()
        odom_to_footprint.header.stamp = stamp
        odom_to_footprint.header.frame_id = self.odom_frame
        odom_to_footprint.child_frame_id = self.base_footprint_frame
        odom_to_footprint.transform.translation.x = float(odom_position[0])
        odom_to_footprint.transform.translation.y = float(odom_position[1])
        odom_quat = self._yaw_quaternion(yaw)
        odom_to_footprint.transform.rotation.w = float(odom_quat[0])
        odom_to_footprint.transform.rotation.x = float(odom_quat[1])
        odom_to_footprint.transform.rotation.y = float(odom_quat[2])
        odom_to_footprint.transform.rotation.z = float(odom_quat[3])

        footprint_to_base = TransformStamped()
        footprint_to_base.header.stamp = stamp
        footprint_to_base.header.frame_id = self.base_footprint_frame
        footprint_to_base.child_frame_id = self.base_link_frame
        footprint_to_base.transform.translation.z = float(world_position[2])
        # odom -> base_footprint already carries the planar yaw.  The fixed
        # height transform must not apply yaw a second time.
        footprint_to_base.transform.rotation.w = 1.0

        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = stamp
        base_to_laser.header.frame_id = self.base_link_frame
        base_to_laser.child_frame_id = self.laser_frame
        base_to_laser.transform.translation.x = float(self._base_to_laser_pos[0])
        base_to_laser.transform.translation.y = float(self._base_to_laser_pos[1])
        base_to_laser.transform.translation.z = float(self._base_to_laser_pos[2])
        base_to_laser.transform.rotation.w = float(self._base_to_laser_quat[0])
        base_to_laser.transform.rotation.x = float(self._base_to_laser_quat[1])
        base_to_laser.transform.rotation.y = float(self._base_to_laser_quat[2])
        base_to_laser.transform.rotation.z = float(self._base_to_laser_quat[3])

        base_to_imu = TransformStamped()
        base_to_imu.header.stamp = stamp
        base_to_imu.header.frame_id = self.base_link_frame
        base_to_imu.child_frame_id = self.imu_frame
        base_to_imu.transform.translation.x = float(self._base_to_imu_pos[0])
        base_to_imu.transform.translation.y = float(self._base_to_imu_pos[1])
        base_to_imu.transform.translation.z = float(self._base_to_imu_pos[2])
        base_to_imu.transform.rotation.w = float(self._base_to_imu_quat[0])
        base_to_imu.transform.rotation.x = float(self._base_to_imu_quat[1])
        base_to_imu.transform.rotation.y = float(self._base_to_imu_quat[2])
        base_to_imu.transform.rotation.z = float(self._base_to_imu_quat[3])

        self.tf_broadcaster.sendTransform(
            [odom_to_footprint, footprint_to_base, base_to_laser, base_to_imu]
        )
        self.last_tf_publish_time = now

    def _reset(self, _request, response):
        reset_epoch = self._simulation_time()
        mujoco.mj_resetData(self.model, self.data)
        self._sim_time_epoch = reset_epoch
        self.target_arm = self.arm_idle_position.copy()
        self.target_gripper = self.gripper_idle_position
        self._cmd_vel.fill(0.0)
        self._cmd_vel_time = None
        self._last_update_time = time.monotonic()
        mujoco.mj_forward(self.model, self.data)
        self._reset_odom_reference_locked()
        self._reset_imu_reference_locked()
        self.last_odom_publish_time = 0.0
        self.last_tf_publish_time = 0.0
        self.last_laser_publish_time = 0.0
        self.last_object_publish_time = 0.0
        self.last_base_pose_publish_time = 0.0
        self.last_clock_publish_sim_time = -np.inf
        self.last_imu_publish_sim_time = -np.inf
        self._physics_step_accumulator = 0.0
        self._last_viewer_sync_time = -np.inf
        response.success = True
        response.message = "RS MuJoCo reset"
        return response

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        result = super().destroy_node()
        if self._clock_lock_fd is not None:
            fcntl.flock(self._clock_lock_fd, fcntl.LOCK_UN)
            os.close(self._clock_lock_fd)
            self._clock_lock_fd = None
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsMujocoSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            if rclpy.ok():
                raise
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
