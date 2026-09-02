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

# Match the Gazebo laboratory coordinates.  The AGV starts at (-2.45, -0.55)
# and docks at x=2.50 in front of the manipulation table centred at x=3.00.
# The source grasp scene table is centred at x=0.38, hence the 2.615 m offset.
TASK_SCENE_X_OFFSET = 2.615
AGV_START = (-2.45, -0.55)
AGV_STL_SOURCE = Path(__file__).resolve().parents[1] / "mobile_base" / "cad" / "agv_v1" / "stl"

FONT_5X7 = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
}


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
    asset.append(element("material", name="lab_wall_mat", rgba="0.82 0.84 0.86 1"))
    asset.append(element("material", name="lab_rack_mat", rgba="0.25 0.30 0.36 1"))
    asset.append(element("material", name="lab_pallet_mat", rgba="0.62 0.32 0.10 1"))
    asset.append(element("material", name="lab_charge_mat", rgba="0.04 0.52 0.42 0.32"))
    asset.append(element("material", name="lab_delivery_mat", rgba="0.08 0.42 0.70 0.32"))
    asset.append(element("material", name="lab_dynamic_mat", rgba="0.78 0.56 0.05 0.18"))
    asset.append(element("material", name="lab_lane_mat", rgba="0.96 0.82 0.10 0.8"))


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

    imu = element("body", name="imu", pos="0 0 0.090")
    imu.append(
        element(
            "site",
            name="imu_site",
            type="box",
            size="0.012 0.009 0.004",
            rgba="0.15 0.65 0.95 0.8",
            group="2",
        )
    )
    chassis.append(imu)

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


def add_lab_box(
    worldbody: ET.Element,
    name: str,
    position: str,
    size: str,
    material: str = "lab_wall_mat",
    *,
    collision: bool = True,
) -> None:
    attributes = {
        "name": name,
        "type": "box",
        "pos": position,
        "size": size,
        "material": material,
    }
    if not collision:
        attributes.update({"contype": "0", "conaffinity": "0", "group": "1"})
    worldbody.append(element("geom", **attributes))


def add_flat_site(
    worldbody: ET.Element,
    name: str,
    position: str,
    size: str,
    rgba: str,
) -> None:
    """Add a purely visual, sub-millimetre ground marker."""
    worldbody.append(
        element(
            "site",
            name=name,
            type="box",
            pos=position,
            size=size,
            rgba=rgba,
            group="1",
        )
    )


def add_ground_label(
    worldbody: ET.Element,
    name: str,
    text: str,
    x: float,
    y: float,
    half_width: float,
    half_height: float,
) -> None:
    """Draw a compact 5x7 label from non-colliding visual sites."""
    columns = len(text) * 5 + max(len(text) - 1, 0)
    step = min(0.03, 1.6 * half_width / columns, 0.65 * half_height / 7)
    start_x = x - 0.5 * (columns - 1) * step
    for character_index, character in enumerate(text):
        glyph = FONT_5X7[character]
        for row, pixels in enumerate(glyph):
            column = 0
            while column < 5:
                if pixels[column] == "0":
                    column += 1
                    continue
                run_start = column
                while column < 5 and pixels[column] == "1":
                    column += 1
                run_end = column - 1
                run_cells = run_end - run_start + 1
                center_column = character_index * 6 + 0.5 * (
                    run_start + run_end
                )
                add_flat_site(
                    worldbody,
                    f"{name}_text_{character_index}_{row}_{run_start}",
                    f"{start_x + center_column * step:.6f} "
                    f"{y + (3 - row) * step:.6f} 0.00045",
                    f"{0.42 * run_cells * step:.6f} {0.42 * step:.6f} 0.00005",
                    "0.96 0.97 0.98 0.95",
                )


def add_lab_scene(worldbody: ET.Element) -> None:
    # MJCF box sizes are half-extents.  These values match rebotarm_lab.sdf and
    # generate_lab_map.py so the same saved occupancy map works in both engines.
    walls = (
        ("west_boundary", "-4.000 0.000 0.240", "0.075 3.075 0.240"),
        ("east_boundary", "4.000 0.000 0.240", "0.075 3.075 0.240"),
        ("south_boundary", "0.000 -3.000 0.240", "4.075 0.075 0.240"),
        ("north_boundary", "0.000 3.000 0.240", "4.075 0.075 0.240"),
        ("storage_partition", "-1.300 1.800 0.240", "0.060 0.900 0.240"),
        ("north_partition", "0.700 1.000 0.240", "1.000 0.060 0.240"),
        ("south_partition", "-0.800 -1.600 0.240", "1.200 0.060 0.240"),
        ("delivery_partition", "1.400 -1.800 0.240", "0.060 0.900 0.240"),
    )
    for name, position, size in walls:
        add_lab_box(worldbody, name, position, size)

    add_lab_box(
        worldbody,
        "storage_rack",
        "-3.000 1.750 0.550",
        "0.600 0.550 0.550",
        "lab_rack_mat",
    )

    pallet = element("body", name="dynamic_pallet", pos="0 1.72 0.20")
    pallet.append(element("freejoint", name="dynamic_pallet_freejoint"))
    pallet.append(
        element(
            "geom",
            name="dynamic_pallet_geom",
            type="box",
            size="0.350 0.225 0.200",
            material="lab_pallet_mat",
            mass="12",
            friction="1.2 0.02 0.002",
        )
    )
    worldbody.append(pallet)

    zones = (
        ("charging_zone", "CHARGE", -3.05, -2.10, 0.70, 0.55, "0.04 0.52 0.42 0.38"),
        ("delivery_zone", "DELIVERY", 3.05, -2.10, 0.70, 0.55, "0.08 0.42 0.70 0.38"),
        ("dynamic_zone", "DYNAMIC", 0.0, 1.73, 0.75, 0.625, "0.78 0.56 0.05 0.30"),
        ("workcell_dock", "DOCK", 2.50, 0.0, 0.24, 0.25, "0.08 0.42 0.70 0.45"),
    )
    for name, label, x, y, half_width, half_height, rgba in zones:
        add_flat_site(
            worldbody,
            name,
            f"{x:.3f} {y:.3f} 0.0002",
            f"{half_width:.3f} {half_height:.3f} 0.00005",
            rgba,
        )
        add_ground_label(
            worldbody, name, label, x, y, half_width, half_height
        )

    add_flat_site(
        worldbody,
        "lane_left",
        "0 -0.62 0.0002",
        "3.60 0.0125 0.00005",
        "0.96 0.82 0.10 0.8",
    )
    add_flat_site(
        worldbody,
        "lane_right",
        "0 0.62 0.0002",
        "3.60 0.0125 0.00005",
        "0.96 0.82 0.10 0.8",
    )

    worldbody.append(
        element(
            "light",
            name="lab_sun",
            pos="0 0 8",
            dir="-0.45 0.2 -0.87",
            directional="true",
        )
    )
    worldbody.append(
        element(
            "camera",
            name="lab_overview",
            mode="fixed",
            pos="0 -7.5 7.0",
            xyaxes="1 0 0 0 0.682 0.731",
            fovy="55",
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
    floor = next(
        (
            child
            for child in arm_worldbody
            if child.tag == "geom" and child.get("name") == "floor"
        ),
        None,
    )
    if floor is not None:
        floor.set("size", "4.2 3.2 0.04")
        floor.set("pos", "0 0 0")
    add_lab_scene(arm_worldbody)

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
    base_body.set("pos", f"{AGV_START[0]} {AGV_START[1]} 0.136")
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
                ctrlrange="-8 8",
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

    statistic = arm_root.find("statistic")
    if statistic is not None:
        statistic.set("center", "0 0 0.3")
        statistic.set("extent", "5.2")

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
