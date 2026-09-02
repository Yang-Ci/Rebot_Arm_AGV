#!/usr/bin/env python3
"""Build the local B601-RS + AGV v1 combined MuJoCo scene.

The RS model is copied from the local rebotarm_mujoco_rs package and modified
only in this output scene. The AGV CAD meshes are millimetre-scale, so their
MJCF mesh assets use a 0.001 scale.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


AGV_ASSETS = {
    "agv_bottom_tray": "agv_bottom_tray_v1.stl",
    "agv_top_cover": "agv_top_cover_v1.stl",
    "agv_adapter": "arm_adapter_plate_v1.stl",
    "agv_grommet": "cable_grommet_v1.stl",
    "agv_latch": "latch_mount_v1_print_x2.stl",
    "agv_wheel": "wheel_placeholder_v1_print_x4.stl",
}

WHEELS = {
    "front_left": (0.150, 0.230),
    "front_right": (0.150, -0.230),
    "rear_left": (-0.150, 0.230),
    "rear_right": (-0.150, -0.230),
}

# The original task scene assumes a compact fixed base. The 0.48 m AGV front
# edge reaches x=0.24, while the table starts at x=0.13. Move the task area by
# only the minimum clearance needed; a larger offset pushes the red cube out of
# the RS's reliable top-down grasp workspace.
TASK_SCENE_X_OFFSET = 0.115
AGV_STL_SOURCE = Path(__file__).resolve().parents[1] / "mobile_base" / "cad" / "agv_v1" / "stl"


def parse_xml(path: Path) -> ET.ElementTree:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def element(tag: str, **attributes: str) -> ET.Element:
    return ET.Element(tag, {key: str(value) for key, value in attributes.items()})


def add_agv_assets(asset: ET.Element) -> None:
    for name, filename in AGV_ASSETS.items():
        asset.append(
            element(
                "mesh",
                name=name,
                file=f"agv/{filename}",
                scale="0.001 0.001 0.001",
            )
        )
    asset.append(element("material", name="agv_body_mat", rgba="0.10 0.11 0.12 1"))
    asset.append(element("material", name="agv_adapter_mat", rgba="0.28 0.30 0.32 1"))
    asset.append(element("material", name="agv_wheel_mat", rgba="0.015 0.015 0.018 1"))


def add_chassis(base_body: ET.Element) -> None:
    chassis = element("body", name="agv_chassis", pos="0 0 -0.136")
    chassis.append(
        element(
            "inertial",
            pos="0 0 0.06",
            mass="8.0",
            diaginertia="0.1296 0.1664 0.2720",
        )
    )

    chassis_parts = (
        ("agv_bottom_tray", "agv_body_mat", "0 0 0"),
        ("agv_top_cover", "agv_body_mat", "0 0 0.120"),
        ("agv_adapter", "agv_adapter_mat", "0 0 0.126"),
        ("agv_grommet", "agv_adapter_mat", "0 -0.115 0.136"),
    )
    for mesh_name, material, position in chassis_parts:
        chassis.append(
            element(
                "geom",
                name=f"{mesh_name}_visual",
                type="mesh",
                mesh=mesh_name,
                material=material,
                pos=position,
                group="1",
                contype="0",
                conaffinity="0",
            )
        )

    for sign_x in (-1.0, 1.0):
        chassis.append(
            element(
                "geom",
                name=f"agv_latch_{'left' if sign_x < 0 else 'right'}_visual",
                type="mesh",
                mesh="agv_latch",
                material="agv_adapter_mat",
                pos=f"{0.088 * sign_x:.3f} 0 0.136",
                group="1",
                contype="0",
                conaffinity="0",
            )
        )

    chassis.append(
        element(
            "geom",
            name="agv_chassis_collision",
            type="box",
            size="0.240 0.210 0.055",
            pos="0 0 0.065",
            group="3",
            rgba="0 0 0 0",
            friction="1.0 0.02 0.002",
        )
    )

    laser = element("body", name="laser", pos="0.185 0 0.150")
    laser.append(
        element(
            "geom",
            name="laser_visual",
            type="cylinder",
            size="0.030 0.025",
            material="agv_adapter_mat",
            group="1",
            contype="0",
            conaffinity="0",
        )
    )
    chassis.append(laser)

    for wheel_name, (x, y) in WHEELS.items():
        wheel = element("body", name=f"wheel_{wheel_name}", pos=f"{x:.3f} {y:.3f} 0.055")
        wheel.append(
            element(
                "joint",
                name=f"wheel_{wheel_name}_joint",
                type="hinge",
                axis="0 1 0",
                damping="0.04",
            )
        )
        wheel.append(
            element(
                "inertial",
                pos="0 0 0",
                mass="0.4",
                diaginertia="0.000366 0.000500 0.000366",
            )
        )
        wheel.append(
            element(
                "geom",
                name=f"wheel_{wheel_name}_visual",
                type="mesh",
                mesh="agv_wheel",
                material="agv_wheel_mat",
                group="1",
                contype="0",
                conaffinity="0",
            )
        )
        wheel.append(
            element(
                "geom",
                name=f"wheel_{wheel_name}_collision",
                type="cylinder",
                size="0.050 0.020",
                quat="0.7071055 0.7071081 0 0",
                group="3",
                rgba="0 0 0 0",
                friction="1.2 0.04 0.004",
            )
        )
        chassis.append(wheel)

    first_arm_child = next(child for child in base_body if child.tag == "body")
    base_body.insert(list(base_body).index(first_arm_child), chassis)


def offset_task_scene(worldbody: ET.Element) -> None:
    movable_tags = {"body", "camera", "geom", "light"}
    fixed_names = {"floor"}
    for child in worldbody:
        if child.tag not in movable_tags or child.get("name") in fixed_names:
            continue
        position = child.get("pos")
        if not position:
            continue
        values = position.split()
        values[0] = f"{float(values[0]) + TASK_SCENE_X_OFFSET:.6f}"
        child.set("pos", " ".join(values))


def add_navigation_walls(worldbody: ET.Element) -> None:
    walls = (
        ("north_wall", "0.000 1.000 0.255", "0.985 0.015 0.250"),
        ("south_wall", "0.000 -1.000 0.255", "0.985 0.015 0.250"),
        ("east_wall", "1.000 0.000 0.255", "0.015 0.985 0.250"),
        ("west_wall", "-1.000 0.000 0.255", "0.015 0.985 0.250"),
    )
    for name, position, size in walls:
        worldbody.append(
            element(
                "geom",
                name=name,
                type="box",
                pos=position,
                size=size,
                material="rs_table",
            )
        )


def build_model(package_dir: Path, output: Path) -> None:
    agv_mesh_dir = package_dir / "models" / "meshes" / "agv"
    agv_mesh_dir.mkdir(parents=True, exist_ok=True)
    for filename in AGV_ASSETS.values():
        source = AGV_STL_SOURCE / filename
        if not source.is_file():
            raise FileNotFoundError(f"AGV mesh not found: {source}")
        shutil.copy2(source, agv_mesh_dir / filename)

    arm_tree = parse_xml(package_dir / "models" / "rs_arm.xml")
    scene_tree = parse_xml(package_dir / "models" / "rs_grasp_scene.xml")
    arm_root = arm_tree.getroot()
    scene_root = scene_tree.getroot()

    arm_root.set("model", "rebot_rs_agv_v1")

    arm_asset = arm_root.find("asset")
    scene_asset = scene_root.find("asset")
    if arm_asset is None or scene_asset is None:
        raise ValueError("RS model or grasp scene is missing its asset section")
    for child in scene_asset:
        arm_asset.append(deepcopy(child))
    add_agv_assets(arm_asset)

    arm_worldbody = arm_root.find("worldbody")
    scene_worldbody = scene_root.find("worldbody")
    if arm_worldbody is None or scene_worldbody is None:
        raise ValueError("RS model or grasp scene is missing its worldbody")
    offset_task_scene(scene_worldbody)
    for child in reversed(scene_worldbody):
        if child.tag == "geom" and child.get("name") == "floor":
            child.set("pos", "0 0 0.005")
        if child.tag not in (ET.Comment, ET.ProcessingInstruction):
            arm_worldbody.insert(0, deepcopy(child))
    add_navigation_walls(arm_worldbody)

    base_body = next(
        (
            child
            for child in arm_worldbody
            if child.tag == "body" and child.get("name") == "base_link"
        ),
        None,
    )
    if base_body is None:
        raise ValueError("Expected the RS base_link body at worldbody root")
    base_body.set("pos", "0 0 0.136")
    base_body.insert(0, element("freejoint", name="agv_freejoint"))
    add_chassis(base_body)

    arm_actuator = arm_root.find("actuator")
    if arm_actuator is None:
        raise ValueError("RS model is missing its actuator section")
    for wheel_name in WHEELS:
        arm_actuator.append(
            element(
                "motor",
                name=f"wheel_{wheel_name}_motor",
                joint=f"wheel_{wheel_name}_joint",
                ctrlrange="-2 2",
                gear="1",
            )
        )

    arm_contact = arm_root.find("contact")
    scene_contact = scene_root.find("contact")
    if arm_contact is None or scene_contact is None:
        raise ValueError("RS model or grasp scene is missing its contact section")
    for child in scene_contact:
        arm_contact.append(deepcopy(child))
    arm_contact.append(element("exclude", body1="base_link", body2="agv_chassis"))
    for wheel_name in WHEELS:
        arm_contact.append(
            element("exclude", body1="agv_chassis", body2=f"wheel_{wheel_name}")
        )

    for tag in ("option", "statistic", "visual"):
        scene_section = scene_root.find(tag)
        if scene_section is not None:
            arm_root.append(deepcopy(scene_section))

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(arm_tree, space="  ")
    arm_tree.write(output, encoding="utf-8", xml_declaration=True)


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_package = script_dir.parent / "rebotarm_mujoco_rs"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, default=default_package)
    parser.add_argument(
        "--output",
        type=Path,
        default=default_package / "models" / "rs_agv_scene.xml",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    build_model(args.package_dir.resolve(), args.output.resolve())
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
