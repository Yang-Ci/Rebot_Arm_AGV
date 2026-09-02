from __future__ import annotations

from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointPosVelCmd
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class SliderSpec:
    name: str
    label: str
    lower: float
    upper: float
    default: float
    resolution: float
    unit: str


SLIDERS = [
    SliderSpec("joint1", "joint1 / base rotation", -2.8, 2.8, 0.0, 0.001, "rad"),
    SliderSpec("joint2", "joint2 / shoulder pitch", -0.2, 3.14, 0.0, 0.001, "rad"),
    SliderSpec("joint3", "joint3 / elbow pitch", -0.2, 3.14, 0.0, 0.001, "rad"),
    SliderSpec("joint4", "joint4 / wrist roll", -1.57, 1.57, 0.0, 0.001, "rad"),
    SliderSpec("joint5", "joint5 / wrist pitch", -1.57, 1.57, 0.0, 0.001, "rad"),
    SliderSpec("joint6", "joint6 / wrist yaw", -3.14, 3.14, 0.0, 0.001, "rad"),
    SliderSpec("gripper_joint1", "gripper open/close", 0.0, 0.045, 0.0, 0.0005, "m"),
]

_ARM_SLIDERS = [s for s in SLIDERS if s.name.startswith("joint")]
_GRIPPER_SLIDER = SLIDERS[-1]

# visual-to-raw conversion for the fake driver's gripper command
_VISUAL_OPEN_M = 0.045


class JointSliderGui(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_rs_joint_slider_gui")

        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("gripper_open_position", 5.0)

        self.arm_namespace = str(
            self.get_parameter("arm_namespace").value or "rebotarm_rs"
        ).strip("/")
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.gripper_open_position = max(
            0.01, float(self.get_parameter("gripper_open_position").value)
        )

        # publish JointPosVelCmd to each joint command topic
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.joint_cmd_pubs: list = []
        for spec in _ARM_SLIDERS:
            topic = f"/{self.arm_namespace}/joints/{spec.name}/cmd/pos_vel"
            self.joint_cmd_pubs.append(
                (spec, self.create_publisher(JointPosVelCmd, topic, cmd_qos))
            )
        self.gripper_cmd_pub = self.create_publisher(
            JointPosVelCmd,
            f"/{self.arm_namespace}/gripper/cmd/pos_vel",
            cmd_qos,
        )

        # subscribe to joint_states for feedback display
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            JointState,
            f"/{self.arm_namespace}/joint_states",
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            ArmStatus,
            f"/{self.arm_namespace}/arm_status",
            self._arm_status_callback,
            latched_qos,
        )

        # gravity comp service clients
        self.gc_start_client = self.create_client(
            Trigger, f"/{self.arm_namespace}/gravity_compensation/start"
        )
        self.gc_stop_client = self.create_client(
            Trigger, f"/{self.arm_namespace}/gravity_compensation/stop"
        )
        self.gc_status_client = self.create_client(
            Trigger, f"/{self.arm_namespace}/gravity_compensation/status"
        )

        self.feedback_positions: dict[str, float] = {}

        self.root = tk.Tk()
        self.root.title("reBotArm RS joint slider")
        self.root.geometry("800x620")
        self.root.minsize(720, 560)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.values: dict[str, tk.DoubleVar] = {}
        self.value_labels: dict[str, ttk.Label] = {}
        self.feedback_labels: dict[str, ttk.Label] = {}
        self.continuous_publish = tk.BooleanVar(value=True)
        self.status_text = tk.StringVar()
        self.gc_status_text = tk.StringVar(value="gravity comp: not queried")

        self._build_ui()
        self._publish_once()
        self._schedule_publish()
        self._schedule_ros_spin()

        self.get_logger().info(
            f"RS joint slider GUI: namespace=/{self.arm_namespace}, "
            f"publishing {len(_ARM_SLIDERS)} joint cmds + gripper at "
            f"{self.publish_hz:.1f} Hz"
        )

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(16, 12, 16, 8))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        title = ttk.Label(header, text="reBotArm RS Joint Slider", font=("", 16, "bold"))
        title.grid(row=0, column=0, sticky="w")
        subtitle = ttk.Label(
            header,
            text=f"namespace: /{self.arm_namespace}    rate: {self.publish_hz:.1f} Hz",
        )
        subtitle.grid(row=1, column=0, sticky="w", pady=(4, 0))

        slider_frame = ttk.Frame(self.root, padding=(16, 0, 16, 8))
        slider_frame.grid(row=1, column=0, sticky="nsew")
        slider_frame.columnconfigure(1, weight=1)

        for row, spec in enumerate(SLIDERS):
            value = tk.DoubleVar(value=spec.default)
            self.values[spec.name] = value

            label = ttk.Label(slider_frame, text=spec.label, width=24)
            label.grid(row=row, column=0, sticky="w", padx=(0, 10), pady=5)

            scale = ttk.Scale(
                slider_frame,
                from_=spec.lower,
                to=spec.upper,
                variable=value,
                command=lambda _u, s=spec: self._on_slider_change(s),
            )
            scale.grid(row=row, column=1, sticky="ew", pady=5)

            value_label = ttk.Label(slider_frame, width=13)
            value_label.grid(row=row, column=2, sticky="e", padx=(8, 0), pady=5)
            self.value_labels[spec.name] = value_label
            self._refresh_value_label(spec)

            fb_label = ttk.Label(slider_frame, width=14, foreground="#0066aa")
            fb_label.grid(row=row, column=3, sticky="e", padx=(8, 0), pady=5)
            self.feedback_labels[spec.name] = fb_label

            range_label = ttk.Label(
                slider_frame,
                text=f"[{spec.lower:g}, {spec.upper:g}]",
                width=14,
            )
            range_label.grid(row=row, column=4, sticky="e", padx=(8, 0), pady=5)

        controls = ttk.Frame(self.root, padding=(16, 4, 16, 8))
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(8, weight=1)

        ttk.Button(controls, text="Reset", command=self.reset_all).grid(
            row=0, column=0, padx=(0, 8)
        )
        ttk.Button(controls, text="Grip Close", command=lambda: self.set_gripper(0.0)).grid(
            row=0, column=1, padx=8
        )
        ttk.Button(controls, text="Grip Half", command=lambda: self.set_gripper(0.0225)).grid(
            row=0, column=2, padx=8
        )
        ttk.Button(controls, text="Grip Open", command=lambda: self.set_gripper(0.045)).grid(
            row=0, column=3, padx=8
        )
        ttk.Button(controls, text="Send Once", command=self._publish_once).grid(
            row=0, column=4, padx=8
        )
        ttk.Checkbutton(
            controls,
            text="Continuous",
            variable=self.continuous_publish,
        ).grid(row=0, column=5, padx=8)

        gravity = ttk.LabelFrame(self.root, text="Gravity Compensation", padding=(16, 8, 16, 10))
        gravity.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 8))
        gravity.columnconfigure(3, weight=1)

        ttk.Button(gravity, text="Start", command=self.start_gravity_compensation).grid(
            row=0, column=0, padx=(0, 8)
        )
        ttk.Button(gravity, text="Stop", command=self.stop_gravity_compensation).grid(
            row=0, column=1, padx=8
        )
        ttk.Button(gravity, text="Status", command=self.query_gravity_compensation).grid(
            row=0, column=2, padx=8
        )
        ttk.Label(gravity, textvariable=self.gc_status_text).grid(
            row=0, column=3, sticky="w", padx=(14, 0)
        )

        footer = ttk.Frame(self.root, padding=(16, 0, 16, 14))
        footer.grid(row=4, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        self.status_text.set("ready")
        ttk.Label(footer, textvariable=self.status_text).grid(row=0, column=0, sticky="w")

    def _on_slider_change(self, spec: SliderSpec) -> None:
        self._refresh_value_label(spec)
        if not self.continuous_publish.get():
            self.status_text.set("slider changed, click Send Once to send pose")

    def _refresh_value_label(self, spec: SliderSpec) -> None:
        value = self.values[spec.name].get()
        self.value_labels[spec.name].configure(text=f"{value:.4f} {spec.unit}")

    def _schedule_publish(self) -> None:
        if rclpy.ok():
            if self.continuous_publish.get():
                self._publish_once()
            delay_ms = max(int(1000.0 / self.publish_hz), 1)
            self.root.after(delay_ms, self._schedule_publish)

    def _schedule_ros_spin(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(30, self._schedule_ros_spin)

    def _visual_to_raw_gripper(self, visual: float) -> float:
        return float(visual) / _VISUAL_OPEN_M * self.gripper_open_position

    def _publish_once(self) -> None:
        now = self.get_clock().now().to_msg()
        for spec, pub in self.joint_cmd_pubs:
            msg = JointPosVelCmd()
            msg.stamp = now
            msg.pos = float(self.values[spec.name].get())
            msg.vlim = 0.0
            pub.publish(msg)

        gripper_msg = JointPosVelCmd()
        gripper_msg.stamp = now
        gripper_visual = float(self.values["gripper_joint1"].get())
        gripper_msg.pos = self._visual_to_raw_gripper(gripper_visual)
        gripper_msg.vlim = 0.0
        self.gripper_cmd_pub.publish(gripper_msg)

        self.status_text.set(
            f"sent {len(_ARM_SLIDERS)} joint cmds + gripper to /{self.arm_namespace}, "
            f"gripper={gripper_visual:.4f} m"
        )

    def reset_all(self) -> None:
        for spec in SLIDERS:
            self.values[spec.name].set(spec.default)
            self._refresh_value_label(spec)
        self._publish_once()

    def set_gripper(self, value: float) -> None:
        self.values["gripper_joint1"].set(value)
        self._refresh_value_label(_GRIPPER_SLIDER)
        self._publish_once()

    def _joint_state_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        for name, val in values.items():
            if name in self.feedback_labels:
                self.feedback_positions[name] = float(val)
                unit = "rad" if name.startswith("joint") else "m"
                self.feedback_labels[name].configure(
                    text=f"fb: {float(val):.4f} {unit}"
                )

    def start_gravity_compensation(self) -> None:
        self._call_gravity_service(self.gc_start_client, "start gravity comp")

    def stop_gravity_compensation(self) -> None:
        self._call_gravity_service(self.gc_stop_client, "stop gravity comp")

    def query_gravity_compensation(self) -> None:
        self._call_gravity_service(self.gc_status_client, "query gravity comp")

    def _call_gravity_service(self, client, label: str) -> None:
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.05)
        if not client.service_is_ready():
            self.gc_status_text.set(f"{label}: service unavailable")
            return
        self.gc_status_text.set(f"{label}: requesting")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, lbl=label: self._handle_gravity_response(f, lbl)
        )

    def _handle_gravity_response(self, future, label: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.gc_status_text.set(f"{label} failed: {exc}")
            return
        active = (
            False if label.startswith("stop") and response.success
            else bool(response.success)
        )
        state = "running" if active else "inactive"
        self.gc_status_text.set(f"gravity comp: {state} / {response.message or label}")

    def _arm_status_callback(self, msg: ArmStatus) -> None:
        active = msg.state_machine == "GRAVITY_COMP"
        state = "running" if active else "inactive"
        detail = msg.state_machine or "UNKNOWN"
        self.gc_status_text.set(f"gravity comp: {state} / {detail}")

    def close(self) -> None:
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JointSliderGui()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
