from __future__ import annotations

import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rebotarm_msgs.msg import ArmStatus, JointMotorState, JointPosVelCmd
from rebotarm_msgs.srv import GripperCommand, SetGripper
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
_JOINT_LIMITS = (
    (-2.8, 2.8),
    (0.0, 3.14),
    (0.0, 3.14),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-3.14, 3.14),
)
_GRIPPER_MOTOR_OPEN_RAD = 5.0
_GRIPPER_VISUAL_OPEN_M = 0.045


class RsDriver(Node):
    """Provide the RS command/feedback interface for the local MuJoCo stack.

    The task server emits JointPosVelCmd messages. This node converts those
    commands into smooth /<namespace>/joint_states feedback, which the MuJoCo
    bridge consumes as its arm target.
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_driver")
        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("max_joint_speed", 1.0)
        self.declare_parameter("max_gripper_speed", 5.0)
        self.declare_parameter("gripper_open_position", _GRIPPER_MOTOR_OPEN_RAD)
        self.declare_parameter("start_enabled", True)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        rate = max(float(self.get_parameter("joint_state_rate").value), 1.0)
        self.max_joint_speed = max(
            0.01, float(self.get_parameter("max_joint_speed").value)
        )
        self.max_gripper_speed = max(
            0.01, float(self.get_parameter("max_gripper_speed").value)
        )
        self.gripper_open_position = max(
            0.01, float(self.get_parameter("gripper_open_position").value)
        )
        self.enabled = bool(self.get_parameter("start_enabled").value)

        self.positions = [0.0] * len(_ARM_JOINTS)
        self.targets = [0.0] * len(_ARM_JOINTS)
        self.velocities = [0.0] * len(_ARM_JOINTS)
        self.gripper_position = 0.0
        self.gripper_target = 0.0
        self.gripper_velocity = 0.0
        self.last_time = self.get_clock().now()
        self._lock = threading.RLock()

        self.joint_state_pub = self.create_publisher(
            JointState,
            f"/{namespace}/joint_states",
            qos_profile_sensor_data,
        )
        self.joint_motor_pubs = [
            self.create_publisher(
                JointMotorState,
                f"/{namespace}/joints/{name}/state",
                qos_profile_sensor_data,
            )
            for name in _ARM_JOINTS
        ]
        self.gripper_state_pub = self.create_publisher(
            JointMotorState,
            f"/{namespace}/gripper/state",
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            ArmStatus,
            f"/{namespace}/arm_status",
            status_qos,
        )

        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for index, name in enumerate(_ARM_JOINTS):
            self.create_subscription(
                JointPosVelCmd,
                f"/{namespace}/joints/{name}/cmd/pos_vel",
                self._joint_command_callback(index),
                command_qos,
            )
        self.create_subscription(
            JointPosVelCmd,
            f"/{namespace}/gripper/cmd/pos_vel",
            self._gripper_command_callback,
            command_qos,
        )

        self.create_service(
            Trigger,
            f"/{namespace}/enable",
            self._enable,
        )
        self.create_service(
            Trigger,
            f"/{namespace}/disable",
            self._disable,
        )
        self.create_service(
            Trigger,
            f"/{namespace}/safe_home",
            self._safe_home,
        )
        self.create_service(
            SetGripper,
            f"/{namespace}/gripper/set",
            self._set_gripper,
        )
        self.create_service(
            GripperCommand,
            f"/{namespace}/gripper/open",
            self._open_gripper,
        )
        self.create_service(
            GripperCommand,
            f"/{namespace}/gripper/close",
            self._close_gripper,
        )

        self.create_timer(1.0 / rate, self._tick)
        self._publish_status()
        self.get_logger().info(
            f"RS driver ready: namespace=/{namespace}, feedback={rate:g} Hz"
        )

    def _joint_command_callback(self, index: int):
        def callback(msg: JointPosVelCmd) -> None:
            with self._lock:
                if not self.enabled:
                    return
                lower, upper = _JOINT_LIMITS[index]
                self.targets[index] = self._clamp(float(msg.pos), lower, upper)

        return callback

    def _gripper_command_callback(self, msg: JointPosVelCmd) -> None:
        with self._lock:
            if self.enabled:
                self.gripper_target = self._clamp(
                    float(msg.pos), 0.0, self.gripper_open_position
                )

    def _enable(self, _request, response):
        self.enabled = True
        self._publish_status()
        response.success = True
        response.message = "RS driver enabled"
        return response

    def _disable(self, _request, response):
        self.enabled = False
        self._publish_status()
        response.success = True
        response.message = "RS driver disabled"
        return response

    def _safe_home(self, _request, response):
        with self._lock:
            self.targets = [0.0] * len(_ARM_JOINTS)
            self.gripper_target = 0.0
        response.success = True
        response.message = "RS driver safe-home accepted"
        return response

    def _set_gripper(self, request, response):
        with self._lock:
            if self.enabled:
                self.gripper_target = self._clamp(
                    float(request.position), 0.0, self.gripper_open_position
                )
        response.success = self.enabled
        response.reached_position = float(self.gripper_target)
        return response

    def _open_gripper(self, request, response):
        target = (
            float(request.position)
            if request.position != 0.0
            else self.gripper_open_position
        )
        with self._lock:
            if self.enabled:
                self.gripper_target = self._clamp(
                    target, 0.0, self.gripper_open_position
                )
        response.success = self.enabled
        response.reached_position = float(self.gripper_target)
        response.message = "RS driver gripper open accepted"
        return response

    def _close_gripper(self, request, response):
        with self._lock:
            if self.enabled:
                self.gripper_target = self._clamp(
                    float(request.position), 0.0, self.gripper_open_position
                )
        response.success = self.enabled
        response.reached_position = float(self.gripper_target)
        response.message = "RS driver gripper close accepted"
        return response

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 0.001)
        self.last_time = now
        with self._lock:
            for index in range(len(_ARM_JOINTS)):
                before = self.positions[index]
                self.positions[index] = self._step_towards(
                    before,
                    self.targets[index],
                    self.max_joint_speed * dt,
                )
                self.velocities[index] = (self.positions[index] - before) / dt

            before_gripper = self.gripper_position
            self.gripper_position = self._step_towards(
                before_gripper,
                self.gripper_target,
                self.max_gripper_speed * dt,
            )
            self.gripper_velocity = (self.gripper_position - before_gripper) / dt
            self._publish_states(now)

    def _publish_states(self, now) -> None:
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(_ARM_JOINTS)
        msg.position = list(self.positions)
        msg.velocity = list(self.velocities)
        msg.effort = [0.0] * len(_ARM_JOINTS)

        ratio = self.gripper_position / self.gripper_open_position
        visual_position = ratio * _GRIPPER_VISUAL_OPEN_M
        visual_velocity = (
            self.gripper_velocity
            / self.gripper_open_position
            * _GRIPPER_VISUAL_OPEN_M
        )
        msg.name.extend(["gripper_joint1", "gripper_joint2"])
        msg.position.extend([visual_position, visual_position])
        msg.velocity.extend([visual_velocity, visual_velocity])
        msg.effort.extend([0.0, 0.0])
        self.joint_state_pub.publish(msg)

        for index, name in enumerate(_ARM_JOINTS):
            state = JointMotorState()
            state.header = msg.header
            state.joint_name = name
            state.position = float(self.positions[index])
            state.velocity = float(self.velocities[index])
            self.joint_motor_pubs[index].publish(state)

        gripper_state = JointMotorState()
        gripper_state.header = msg.header
        gripper_state.joint_name = "gripper"
        gripper_state.position = float(self.gripper_position)
        gripper_state.velocity = float(self.gripper_velocity)
        self.gripper_state_pub.publish(gripper_state)

    def _publish_status(self) -> None:
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = "rs_mujoco_driver"
        msg.enabled = self.enabled
        msg.control_loop_active = self.enabled
        msg.state_machine = "LOWLEVEL_STREAMING" if self.enabled else "DISABLED"
        msg.joint_names = list(_ARM_JOINTS)
        msg.per_joint_status_code = [0] * len(_ARM_JOINTS)
        msg.error_codes = []
        self.status_pub.publish(msg)

    @staticmethod
    def _step_towards(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if math.isclose(delta, 0.0, abs_tol=1e-9):
            return target
        return current + RsDriver._clamp(delta, -max_step, max_step)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # rclpy Jazzy can surface RCLError instead of
        # ExternalShutdownException while a launch-wide shutdown invalidates
        # the context.  Preserve real runtime failures while treating an
        # already-shutdown context as a normal exit.
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
