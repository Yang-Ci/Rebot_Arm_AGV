from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


_OBJECTS = {
    "red_cube": {"color": "red", "width_m": 0.045, "height_m": 0.045},
    "blue_block": {"color": "blue", "width_m": 0.052, "height_m": 0.032},
    "yellow_cylinder": {"color": "yellow", "width_m": 0.040, "height_m": 0.040},
}


class RsSceneDetector(Node):
    """Expose MuJoCo task-object states through the DM Agent detection API."""

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_scene_detector")
        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("object_states_topic", "")
        self.declare_parameter("detections_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("poses_topic", "")
        self.declare_parameter("target_color", "red")
        self.declare_parameter("publish_hz", 10.0)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.object_states_topic = str(
            self.get_parameter("object_states_topic").value
            or f"/{namespace}/mujoco/object_states"
        )
        self.detections_topic = str(
            self.get_parameter("detections_topic").value
            or f"/{namespace}/vision/color_blocks/detections"
        )
        self.target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
            or f"/{namespace}/vision/color_blocks/target_pose"
        )
        self.poses_topic = str(
            self.get_parameter("poses_topic").value
            or f"/{namespace}/vision/color_blocks/poses"
        )
        self.target_color = str(self.get_parameter("target_color").value).lower()
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)

        self._latest_objects: list[dict[str, Any]] = []
        self._latest_stamp = 0.0
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.target_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.poses_pub = self.create_publisher(PoseArray, self.poses_topic, 10)
        self.create_subscription(String, self.object_states_topic, self._objects_callback, 10)
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            "RS scene detector ready: "
            f"objects={self.object_states_topic}, detections={self.detections_topic}"
        )

    def _objects_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        objects = payload.get("objects")
        if isinstance(objects, list):
            self._latest_objects = [item for item in objects if isinstance(item, dict)]
            self._latest_stamp = time.monotonic()

    def _publish(self) -> None:
        if not self._latest_objects:
            return

        detections = []
        for item in self._latest_objects:
            name = str(item.get("name", ""))
            spec = _OBJECTS.get(name)
            position = item.get("position")
            quaternion = item.get("quaternion")
            if spec is None or not isinstance(position, list) or len(position) < 3:
                continue
            try:
                x, y, z = (float(position[index]) for index in range(3))
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (x, y, z)) or z < 0.04:
                continue

            yaw = self._yaw_from_wxyz(quaternion)
            width = float(spec["width_m"])
            height = float(spec["height_m"])
            detections.append(
                {
                    "name": name,
                    "color": spec["color"],
                    "x": round(x, 5),
                    "y": round(y, 5),
                    "z": round(z, 5),
                    "width_m": width,
                    "height_m": height,
                    "longest_m": max(width, height),
                    "shortest_m": min(width, height),
                    "grasp_yaw_rad": round(yaw, 5),
                    "source": "mujoco_ground_truth",
                }
            )

        target = next(
            (item for item in detections if item["color"] == self.target_color),
            detections[0] if detections else None,
        )
        stamp = self.get_clock().now().to_msg()
        payload = {
            "target_color": self.target_color,
            "count": len(detections),
            "target": target,
            "detections": detections,
            "source": "rs_mujoco_object_states",
            "age_sec": round(max(time.monotonic() - self._latest_stamp, 0.0), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(msg)

        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = "world"
        poses.poses = [self._pose(item) for item in detections]
        self.poses_pub.publish(poses)
        if target is not None:
            target_msg = PoseStamped()
            target_msg.header = poses.header
            target_msg.pose = self._pose(target)
            self.target_pub.publish(target_msg)

    @staticmethod
    def _pose(item: dict[str, Any]) -> Pose:
        pose = Pose()
        pose.position.x = float(item["x"])
        pose.position.y = float(item["y"])
        pose.position.z = float(item["z"])
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _yaw_from_wxyz(value: Any) -> float:
        if not isinstance(value, list) or len(value) < 4:
            return 0.0
        try:
            w, x, y, z = (float(value[index]) for index in range(4))
        except (TypeError, ValueError):
            return 0.0
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsSceneDetector()
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
