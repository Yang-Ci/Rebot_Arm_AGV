"""Generate a Gazebo Sim SDF model from the local RS URDF and AGV meshes."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "RS" / "urdf" / "ReBot_Arm_RS.urdf"
OUTPUT = Path(__file__).resolve().parents[1] / "models" / "rebotarm_rs_agv" / "model.sdf"
MESH_ROOT = "model://rebotarm_mujoco_rs/models/meshes"


def add(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, **attributes)
    if text is not None:
        node.text = str(text)
    return node


def material(parent, rgba):
    node = add(parent, "material")
    add(node, "ambient", rgba)
    add(node, "diffuse", rgba)


def inertia(parent, mass, ixx, iyy, izz, pose="0 0 0 0 0 0"):
    node = add(parent, "inertial")
    add(node, "pose", pose)
    add(node, "mass", mass)
    tensor = add(node, "inertia")
    add(tensor, "ixx", ixx)
    add(tensor, "iyy", iyy)
    add(tensor, "izz", izz)
    add(tensor, "ixy", 0)
    add(tensor, "ixz", 0)
    add(tensor, "iyz", 0)


def mesh_visual(link, name, filename, pose, rgba, scale="0.001 0.001 0.001"):
    visual = add(link, "visual", name=name)
    add(visual, "pose", pose)
    geometry = add(visual, "geometry")
    mesh = add(geometry, "mesh")
    add(mesh, "uri", f"{MESH_ROOT}/agv/{filename}")
    add(mesh, "scale", scale)
    material(visual, rgba)


def add_chassis(model):
    chassis = ET.Element("link", name="base_link")
    inertia(chassis, 8.0, 0.1296, 0.1664, 0.272, "0 0 0.075 0 0 0")
    collision = add(chassis, "collision", name="chassis_collision")
    add(collision, "pose", "0 0 0.075 0 0 0")
    geometry = add(collision, "geometry")
    box = add(geometry, "box")
    add(box, "size", "0.48 0.42 0.08")
    surface = add(collision, "surface")
    friction = add(surface, "friction")
    ode = add(friction, "ode")
    add(ode, "mu", 1.0)
    add(ode, "mu2", 1.0)
    mesh_visual(chassis, "bottom_tray", "agv_bottom_tray_v1.stl", "0 0 0 0 0 0", "0.055 0.065 0.075 1")
    mesh_visual(chassis, "top_cover", "agv_top_cover_v1.stl", "0 0 0.120 0 0 0", "0.075 0.085 0.095 1")
    mesh_visual(chassis, "adapter_plate", "arm_adapter_plate_v1.stl", "0 0 0.126 0 0 0", "0.38 0.41 0.45 1")
    mesh_visual(chassis, "cable_grommet", "cable_grommet_v1.stl", "0 -0.115 0.136 0 0 0", "0.18 0.20 0.22 1")
    mesh_visual(chassis, "latch_left", "latch_mount_v1_print_x2.stl", "-0.088 0 0.136 0 0 0", "0.38 0.41 0.45 1")
    mesh_visual(chassis, "latch_right", "latch_mount_v1_print_x2.stl", "0.088 0 0.136 0 0 0", "0.38 0.41 0.45 1")
    imu = add(chassis, "sensor", name="base_imu", type="imu")
    add(imu, "pose", "0 0 0.09 0 0 0")
    add(imu, "always_on", "true")
    add(imu, "update_rate", 100)
    add(imu, "topic", "/imu")
    add(imu, "gz_frame_id", "imu_link")
    model.insert(0, chassis)


def add_wheels(model):
    positions = {
        "front_left": (0.15, 0.23),
        "front_right": (0.15, -0.23),
        "rear_left": (-0.15, 0.23),
        "rear_right": (-0.15, -0.23),
    }
    insertion = 1
    for name, (x, y) in positions.items():
        link_name = f"wheel_{name}"
        link = ET.Element("link", name=link_name)
        inertia(link, 0.4, 0.000366, 0.0005, 0.000366, f"{x} {y} 0.055 -1.57079632679 0 0")
        mesh_visual(link, "wheel_visual", "wheel_placeholder_v1_print_x4.stl", f"{x} {y} 0.055 0 0 0", "0.012 0.014 0.018 1")
        collision = add(link, "collision", name="wheel_collision")
        add(collision, "pose", f"{x} {y} 0.055 -1.57079632679 0 0")
        geometry = add(collision, "geometry")
        cylinder = add(geometry, "cylinder")
        add(cylinder, "radius", 0.05)
        add(cylinder, "length", 0.04)
        surface = add(collision, "surface")
        friction = add(surface, "friction")
        ode = add(friction, "ode")
        add(ode, "mu", 1.2)
        add(ode, "mu2", 1.2)
        model.insert(insertion, link)
        insertion += 1
        joint = ET.Element("joint", name=f"{link_name}_joint", type="revolute")
        add(joint, "pose", f"{x} {y} 0.055 -1.57079632679 0 0")
        add(joint, "parent", "base_link")
        add(joint, "child", link_name)
        axis = add(joint, "axis")
        add(axis, "xyz", "0 0 1")
        limit = add(axis, "limit")
        add(limit, "effort", 40)
        add(limit, "velocity", 30)
        dynamics = add(axis, "dynamics")
        add(dynamics, "damping", 0.04)
        model.insert(insertion, joint)
        insertion += 1


def add_lidar(model):
    link = ET.Element("link", name="base_scan")
    add(link, "pose", "0.185 0 0.175 0 0 0", relative_to="base_link")
    inertia(link, 0.12, 0.00005, 0.00005, 0.00005)
    collision = add(link, "collision", name="lidar_collision")
    geometry = add(collision, "geometry")
    cylinder = add(geometry, "cylinder")
    add(cylinder, "radius", 0.03)
    add(cylinder, "length", 0.05)
    visual = add(link, "visual", name="lidar_visual")
    geometry = add(visual, "geometry")
    cylinder = add(geometry, "cylinder")
    add(cylinder, "radius", 0.03)
    add(cylinder, "length", 0.05)
    material(visual, "0.07 0.38 0.52 1")

    sensor = add(link, "sensor", name="planar_lidar", type="gpu_lidar")
    add(sensor, "always_on", "true")
    add(sensor, "visualize", "true")
    add(sensor, "update_rate", 10)
    add(sensor, "topic", "/scan")
    add(sensor, "gz_frame_id", "base_scan")
    ray = add(sensor, "ray")
    scan = add(ray, "scan")
    horizontal = add(scan, "horizontal")
    add(horizontal, "samples", 720)
    add(horizontal, "resolution", 1)
    add(horizontal, "min_angle", -3.14159265)
    add(horizontal, "max_angle", 3.14159265)
    distance = add(ray, "range")
    add(distance, "min", 0.12)
    add(distance, "max", 12.0)
    add(distance, "resolution", 0.01)
    noise = add(ray, "noise")
    add(noise, "type", "gaussian")
    add(noise, "mean", 0)
    add(noise, "stddev", 0.006)
    model.append(link)

    joint = add(model, "joint", name="base_scan_joint", type="fixed")
    add(joint, "parent", "base_link")
    add(joint, "child", "base_scan")


def add_plugins(model):
    drive = add(model, "plugin", filename="gz-sim-velocity-control-system", name="gz::sim::systems::VelocityControl")
    add(drive, "topic", "/cmd_vel")

    odometry = add(model, "plugin", filename="gz-sim-odometry-publisher-system", name="gz::sim::systems::OdometryPublisher")
    add(odometry, "dimensions", 2)
    add(odometry, "odom_frame", "odom")
    add(odometry, "robot_base_frame", "base_link")
    add(odometry, "child_frame_id", "base_link")
    add(odometry, "odom_publish_frequency", 30)
    add(odometry, "odom_topic", "/odom")
    add(odometry, "tf_topic", "/tf")

    joint_states = add(model, "plugin", filename="gz-sim-joint-state-publisher-system", name="gz::sim::systems::JointStatePublisher")
    for jn in (
        "wheel_front_left_joint", "wheel_front_right_joint",
        "wheel_rear_left_joint", "wheel_rear_right_joint",
        "joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6",
        "gripper_joint1", "gripper_joint2",
    ):
        add(joint_states, "joint_name", jn)
    add(joint_states, "update_rate", 30)

    for name, effort in (
        ("joint1", 36), ("joint2", 36), ("joint3", 36),
        ("joint4", 14), ("joint5", 14), ("joint6", 14),
        ("gripper_joint1", 80), ("gripper_joint2", 80),
    ):
        controller = add(model, "plugin", filename="gz-sim-joint-position-controller-system", name="gz::sim::systems::JointPositionController")
        add(controller, "joint_name", name)
        add(controller, "p_gain", 80 if name.startswith("gripper") else 160)
        add(controller, "i_gain", 0)
        add(controller, "d_gain", 8)
        add(controller, "cmd_max", effort)
        add(controller, "cmd_min", -effort)


def main():
    converted = subprocess.run(
        ["gz", "sdf", "-p", str(URDF)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(converted)
    model = root.find("model")
    if model is None:
        raise RuntimeError("URDF conversion did not produce an SDF model")
    model.set("name", "rebotarm_rs_agv")

    # The URDF arm root becomes a child of the mobile-base chassis.
    for node in model.iter():
        if node.text and node.text.strip() == "base_link":
            node.text = node.text.replace("base_link", "arm_base_link")
        for key, value in tuple(node.attrib.items()):
            if value == "base_link":
                node.set(key, "arm_base_link")
    arm_root = model.find("link[@name='arm_base_link']")
    if arm_root is None:
        raise RuntimeError("Converted arm is missing base_link")
    pose_el = ET.SubElement(arm_root, "pose")
    pose_el.text = "0 0 0.136 0 0 0"
    arm_root.remove(pose_el)
    arm_root.insert(0, pose_el)

    available_meshes = REPO / "rebotarm_mujoco_rs" / "models" / "meshes"
    for link in model.findall("link"):
        for collision in tuple(link.findall("collision")):
            uri = collision.find("geometry/mesh/uri")
            if uri is not None and not (available_meshes / Path(uri.text).name).is_file():
                link.remove(collision)

    for uri in model.findall(".//mesh/uri"):
        uri.text = f"{MESH_ROOT}/{Path(uri.text).name}"
    model.set("canonical_link", "base_link")

    for joint in model.findall("joint"):
        if joint.get("type") not in {"revolute", "prismatic"}:
            continue
        dynamics = joint.find("axis/dynamics")
        if dynamics is not None and dynamics.find("damping") is None:
            add(dynamics, "damping", 0.5 if joint.get("type") == "revolute" else 2.0)

    add_chassis(model)
    add_wheels(model)
    add_lidar(model)
    arm_mount = add(model, "joint", name="arm_mount_joint", type="fixed")
    add(arm_mount, "pose", "0 0 0.136 0 0 0", relative_to="base_link")
    add(arm_mount, "parent", "base_link")
    add(arm_mount, "child", "arm_base_link")
    add_plugins(model)

    ET.indent(root, space="  ")
    OUTPUT.write_text("<?xml version=\"1.0\"?>\n" + ET.tostring(root, encoding="unicode") + "\n")
    print(f"generated {OUTPUT}")


if __name__ == "__main__":
    main()
