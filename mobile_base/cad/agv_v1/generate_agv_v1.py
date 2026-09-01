#!/usr/bin/env python3
"""Generate the first reBotArm AGV chassis CAD/STL package.

The model is intentionally parameterized so measurements from the physical
robot can be applied without redrawing the chassis.  Dimensions are in mm.

CadQuery is a generation-time dependency only; the generated STEP/STL files
are committed beside this script and do not require CadQuery at runtime.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import struct

import cadquery as cq
from cadquery import exporters
import numpy as np


@dataclass(frozen=True)
class AgvDimensions:
    # Chassis envelope.
    chassis_length: float = 480.0
    chassis_width: float = 420.0
    chassis_height: float = 120.0
    chassis_wall: float = 8.0
    chassis_floor: float = 6.0
    chassis_corner_radius: float = 18.0
    top_cover_thickness: float = 6.0

    # Repository base_link.STL envelope is 140 x 200 x 75 mm.
    arm_base_width: float = 140.0
    arm_base_length: float = 200.0
    arm_base_height: float = 75.0
    arm_slot_x: float = 45.0
    arm_slot_y: float = 90.0

    # Replaceable arm adapter plate.
    adapter_width: float = 210.0
    adapter_length: float = 280.0
    adapter_thickness: float = 10.0
    adapter_corner_radius: float = 10.0
    arm_thread_pilot_diameter: float = 6.8  # M8 tapping pilot in aluminium.
    adapter_mount_clearance_diameter: float = 6.6  # M6 clearance.

    # Cable routing: capsule/oval cutout, long axis along chassis X.
    cable_port_length: float = 70.0
    cable_port_width: float = 38.0
    cable_port_y: float = -115.0
    grommet_wall: float = 4.0
    grommet_height: float = 6.0

    # Latch mounting pad.  The toggle latch itself should be a rated metal part.
    latch_mount_width: float = 30.0
    latch_mount_length: float = 52.0
    latch_mount_base_thickness: float = 8.0
    latch_mount_upright_height: float = 24.0
    latch_mount_hole_diameter: float = 5.5
    latch_mount_x: float = 88.0
    latch_mount_hole_y: float = 16.0

    # Wheel placeholders for assembly/interference checking.
    wheel_diameter: float = 100.0
    wheel_width: float = 40.0
    wheel_axle_diameter: float = 16.0
    wheel_center_x: float = 150.0
    wheel_center_z: float = 55.0


D = AgvDimensions()


def rounded_prism(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """Create a Z-up prism with rounded XY corners and its bottom at Z=0."""

    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def capsule(length: float, width: float, height: float) -> cq.Workplane:
    """Create an extruded capsule with its long axis along X."""

    if length <= width:
        raise ValueError("capsule length must be greater than width")
    radius = width / 2.0
    half_center_distance = (length - width) / 2.0
    center = cq.Workplane("XY").box(
        length - width, width, height, centered=(True, True, False)
    )
    left = (
        cq.Workplane("XY")
        .center(-half_center_distance, 0)
        .circle(radius)
        .extrude(height)
    )
    right = (
        cq.Workplane("XY")
        .center(half_center_distance, 0)
        .circle(radius)
        .extrude(height)
    )
    return center.union(left).union(right)


def cylinder_z(diameter: float, height: float, x: float, y: float, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(diameter / 2.0)
        .extrude(height)
        .translate((0, 0, z))
    )


def cover_fastener_points() -> list[tuple[float, float]]:
    return [
        (-220.0, -190.0),
        (220.0, -190.0),
        (-220.0, 190.0),
        (220.0, 190.0),
        (0.0, -206.0),
        (0.0, 206.0),
        (-236.0, 0.0),
        (236.0, 0.0),
    ]


def adapter_mount_points() -> list[tuple[float, float]]:
    return [(-90.0, -120.0), (90.0, -120.0), (-90.0, 120.0), (90.0, 120.0)]


def arm_slot_fastener_points() -> list[tuple[float, float]]:
    return [
        (-D.arm_slot_x, -D.arm_slot_y),
        (D.arm_slot_x, -D.arm_slot_y),
        (-D.arm_slot_x, D.arm_slot_y),
        (D.arm_slot_x, D.arm_slot_y),
    ]


def latch_fastener_points() -> list[tuple[float, float]]:
    return [
        (-D.latch_mount_x, -D.latch_mount_hole_y),
        (-D.latch_mount_x, D.latch_mount_hole_y),
        (D.latch_mount_x, -D.latch_mount_hole_y),
        (D.latch_mount_x, D.latch_mount_hole_y),
    ]


def build_bottom_tray() -> cq.Workplane:
    outer = rounded_prism(
        D.chassis_length,
        D.chassis_width,
        D.chassis_height,
        D.chassis_corner_radius,
    )
    inner = rounded_prism(
        D.chassis_length - 2.0 * D.chassis_wall,
        D.chassis_width - 2.0 * D.chassis_wall,
        D.chassis_height - D.chassis_floor + 2.0,
        max(D.chassis_corner_radius - D.chassis_wall, 2.0),
    ).translate((0, 0, D.chassis_floor))
    tray = outer.cut(inner)

    # Four axle clearance holes through the left/right walls.
    for x in (-D.wheel_center_x, D.wheel_center_x):
        axle_cut = (
            cq.Workplane("XZ")
            .center(x, D.wheel_center_z)
            .circle(D.wheel_axle_diameter / 2.0 + 1.0)
            .extrude(D.chassis_width + 20.0, both=True)
        )
        tray = tray.cut(axle_cut)

    # Pilot holes in the top rim for M5 cover fasteners.
    for x, y in cover_fastener_points():
        tray = tray.cut(cylinder_z(4.2, 22.0, x, y, D.chassis_height - 20.0))
    return tray


def cable_cut(height: float) -> cq.Workplane:
    return capsule(D.cable_port_length, D.cable_port_width, height).translate(
        (0, D.cable_port_y, 0)
    )


def build_top_cover() -> cq.Workplane:
    cover = rounded_prism(
        D.chassis_length,
        D.chassis_width,
        D.top_cover_thickness,
        D.chassis_corner_radius,
    )
    cover = cover.cut(cable_cut(D.top_cover_thickness + 2.0).translate((0, 0, -1.0)))
    cover = (
        cover.faces(">Z")
        .workplane()
        .pushPoints(cover_fastener_points())
        .hole(5.5)
    )
    cover = (
        cover.faces(">Z")
        .workplane()
        .pushPoints(adapter_mount_points())
        .hole(D.adapter_mount_clearance_diameter)
    )
    return cover


def build_arm_adapter() -> cq.Workplane:
    adapter = rounded_prism(
        D.adapter_width,
        D.adapter_length,
        D.adapter_thickness,
        D.adapter_corner_radius,
    )
    adapter = adapter.cut(
        cable_cut(D.adapter_thickness + 2.0).translate((0, 0, -1.0))
    )
    # Four M8 tapping pilots aligned with the existing U-shaped slots in
    # repository base_link.STL.  Use threaded inserts when printing in polymer.
    adapter = (
        adapter.faces(">Z")
        .workplane()
        .pushPoints(arm_slot_fastener_points())
        .hole(D.arm_thread_pilot_diameter)
    )
    adapter = (
        adapter.faces(">Z")
        .workplane()
        .pushPoints(adapter_mount_points())
        .hole(D.adapter_mount_clearance_diameter)
    )
    adapter = (
        adapter.faces(">Z")
        .workplane()
        .pushPoints(latch_fastener_points())
        .hole(D.latch_mount_hole_diameter)
    )
    return adapter


def build_latch_mount() -> cq.Workplane:
    base = rounded_prism(
        D.latch_mount_width,
        D.latch_mount_length,
        D.latch_mount_base_thickness,
        3.0,
    )
    base = (
        base.faces(">Z")
        .workplane()
        .pushPoints([(0.0, -D.latch_mount_hole_y), (0.0, D.latch_mount_hole_y)])
        .hole(D.latch_mount_hole_diameter)
    )
    upright = (
        cq.Workplane("XY")
        .box(
            6.0,
            D.latch_mount_length,
            D.latch_mount_upright_height,
            centered=(True, True, False),
        )
        .translate((D.latch_mount_width / 2.0 - 3.0, 0, D.latch_mount_base_thickness))
    )
    return base.union(upright)


def build_cable_grommet() -> cq.Workplane:
    outer = capsule(
        D.cable_port_length + 2.0 * D.grommet_wall,
        D.cable_port_width + 2.0 * D.grommet_wall,
        D.grommet_height,
    )
    inner = capsule(
        D.cable_port_length,
        D.cable_port_width,
        D.grommet_height + 2.0,
    ).translate((0, 0, -1.0))
    return outer.cut(inner)


def build_wheel_placeholder() -> cq.Workplane:
    origin = cq.Vector(0, -D.wheel_width / 2.0, 0)
    axis = cq.Vector(0, 1, 0)
    wheel = cq.Solid.makeCylinder(D.wheel_diameter / 2.0, D.wheel_width, origin, axis)
    axle = cq.Solid.makeCylinder(
        D.wheel_axle_diameter / 2.0,
        D.wheel_width + 2.0,
        cq.Vector(0, -D.wheel_width / 2.0 - 1.0, 0),
        axis,
    )
    return cq.Workplane(obj=wheel.cut(axle))


def build_fastener_placeholder() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(4.0).extrude(18.0)
    washer = cylinder_z(20.0, 2.0, 0, 0, 16.0).cut(cylinder_z(8.5, 2.0, 0, 0, 16.0))
    head = cylinder_z(15.0, 6.0, 0, 0, 18.0)
    return shaft.union(washer).union(head)


def build_assembly(
    tray: cq.Workplane,
    cover: cq.Workplane,
    adapter: cq.Workplane,
    latch_mount: cq.Workplane,
    grommet: cq.Workplane,
    wheel: cq.Workplane,
    fastener: cq.Workplane,
    *,
    include_fasteners: bool,
) -> cq.Compound:
    cover_z = D.chassis_height
    adapter_z = cover_z + D.top_cover_thickness
    arm_base_z = adapter_z + D.adapter_thickness
    shapes: list[cq.Shape] = [
        tray.val(),
        cover.translate((0, 0, cover_z)).val(),
        adapter.translate((0, 0, adapter_z)).val(),
        grommet.translate((0, D.cable_port_y, arm_base_z)).val(),
    ]

    left_mount = latch_mount.translate(
        (-D.latch_mount_x, 0, arm_base_z)
    )
    right_mount = latch_mount.mirror("YZ").translate(
        (D.latch_mount_x, 0, arm_base_z)
    )
    shapes.extend([left_mount.val(), right_mount.val()])

    wheel_y = D.chassis_width / 2.0 + D.wheel_width / 2.0
    for x in (-D.wheel_center_x, D.wheel_center_x):
        shapes.append(wheel.translate((x, -wheel_y, D.wheel_center_z)).val())
        shapes.append(wheel.translate((x, wheel_y, D.wheel_center_z)).val())

    if include_fasteners:
        # The screw shaft begins in the adapter; head/washer sit on the
        # repository base plate when it is added to the reference assembly.
        screw_z = adapter_z
        for x, y in arm_slot_fastener_points():
            shapes.append(fastener.translate((x, y, screw_z)).val())
    return cq.Compound.makeCompound(shapes)


_STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ],
    align=False,
)


def read_binary_stl(path: Path) -> tuple[bytes, np.ndarray]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is too short: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if len(data) != expected:
        raise ValueError(f"Expected binary STL ({expected} bytes), got {len(data)}: {path}")
    records = np.frombuffer(data, dtype=_STL_DTYPE, offset=84, count=count).copy()
    return data[:80], records


def write_binary_stl(path: Path, header: bytes, records: np.ndarray) -> None:
    clean_header = header[:80].ljust(80, b" ")
    with path.open("wb") as stream:
        stream.write(clean_header)
        stream.write(struct.pack("<I", len(records)))
        stream.write(records.astype(_STL_DTYPE, copy=False).tobytes())


def add_repository_base_to_assembly(
    assembly_stl: Path,
    repository_base_stl: Path,
    output_stl: Path,
) -> None:
    header, assembly_records = read_binary_stl(assembly_stl)
    _, base_records = read_binary_stl(repository_base_stl)
    # Repository STL uses metres; generated CadQuery geometry uses millimetres.
    base_records["vertices"] *= 1000.0
    base_records["vertices"][:, :, 2] += (
        D.chassis_height + D.top_cover_thickness + D.adapter_thickness
    )
    merged = np.concatenate((assembly_records, base_records))
    write_binary_stl(output_stl, header, merged)


def stl_metrics(path: Path) -> dict[str, object]:
    _, records = read_binary_stl(path)
    vertices = records["vertices"].reshape(-1, 3)
    minimum = vertices.min(axis=0)
    maximum = vertices.max(axis=0)
    return {
        "triangles": int(len(records)),
        "min_mm": [round(float(value), 4) for value in minimum],
        "max_mm": [round(float(value), 4) for value in maximum],
        "size_mm": [round(float(value), 4) for value in maximum - minimum],
        "finite": bool(np.isfinite(vertices).all()),
    }


def export_part(shape: cq.Workplane | cq.Shape, stl_path: Path, step_path: Path) -> None:
    exporters.export(
        shape,
        str(stl_path),
        tolerance=0.08,
        angularTolerance=0.08,
        opt={"ascii": False},
    )
    exporters.export(shape, str(step_path))


def resolve_repository_base(repo_root: Path) -> Path:
    return repo_root / "rebotarm_ros2_RS/src/rebotarm_mujoco_rs/models/meshes/base_link.STL"


def generate(output_dir: Path, repository_base: Path) -> dict[str, object]:
    stl_dir = output_dir / "stl"
    step_dir = output_dir / "step"
    stl_dir.mkdir(parents=True, exist_ok=True)
    step_dir.mkdir(parents=True, exist_ok=True)

    tray = build_bottom_tray()
    cover = build_top_cover()
    adapter = build_arm_adapter()
    latch_mount = build_latch_mount()
    grommet = build_cable_grommet()
    wheel = build_wheel_placeholder()
    fastener = build_fastener_placeholder()

    parts: dict[str, cq.Workplane | cq.Shape] = {
        "agv_bottom_tray_v1": tray,
        "agv_top_cover_v1": cover,
        "arm_adapter_plate_v1": adapter,
        "latch_mount_v1_print_x2": latch_mount,
        "cable_grommet_v1": grommet,
        "wheel_placeholder_v1_print_x4": wheel,
        "fastener_placeholder_v1_print_x4": fastener,
    }
    for name, shape in parts.items():
        export_part(shape, stl_dir / f"{name}.stl", step_dir / f"{name}.step")

    assembly = build_assembly(
        tray,
        cover,
        adapter,
        latch_mount,
        grommet,
        wheel,
        fastener,
        include_fasteners=False,
    )
    reference_assembly = build_assembly(
        tray,
        cover,
        adapter,
        latch_mount,
        grommet,
        wheel,
        fastener,
        include_fasteners=True,
    )
    assembly_stl = stl_dir / "agv_chassis_assembly_v1.stl"
    exporters.export(
        assembly,
        str(assembly_stl),
        tolerance=0.1,
        angularTolerance=0.1,
        opt={"ascii": False},
    )
    exporters.export(assembly, str(step_dir / "agv_chassis_assembly_v1.step"))

    reference_stl = stl_dir / "agv_chassis_with_repository_base_v1.stl"
    temporary_reference = stl_dir / ".agv_reference_without_base.stl"
    exporters.export(
        reference_assembly,
        str(temporary_reference),
        tolerance=0.1,
        angularTolerance=0.1,
        opt={"ascii": False},
    )
    add_repository_base_to_assembly(temporary_reference, repository_base, reference_stl)
    temporary_reference.unlink()

    metrics = {
        path.name: stl_metrics(path)
        for path in sorted(stl_dir.glob("*.stl"))
    }
    try:
        source_base = str(repository_base.relative_to(output_dir.parents[1]))
    except ValueError:
        source_base = str(repository_base)
    manifest: dict[str, object] = {
        "model": "reBotArm AGV chassis v1",
        "units": "mm",
        "source_base_stl": source_base,
        "dimensions": asdict(D),
        "stl_metrics": metrics,
        "cadquery_shape_validity": {
            "bottom_tray": bool(tray.val().isValid()),
            "top_cover": bool(cover.val().isValid()),
            "arm_adapter": bool(adapter.val().isValid()),
            "latch_mount": bool(latch_mount.val().isValid()),
            "cable_grommet": bool(grommet.val().isValid()),
            "wheel_placeholder": bool(wheel.val().isValid()),
            "fastener_placeholder": bool(fastener.val().isValid()),
            "assembly_compound": bool(assembly.isValid()),
        },
        "notes": [
            "The four arm mount pilots align to the existing open U-slots at x=+-45, y=+-90 mm.",
            "Use M8 threaded inserts for polymer adapters or tap M8 after drilling 6.8 mm in aluminium.",
            "Latch mounts are geometry placeholders; use rated metal over-centre clamps for hardware.",
            "Wheel files are interference-check placeholders, not tyre tread manufacturing models.",
            "Assembly STL files are multi-body layout references; manufacture from the individual part files.",
        ],
    }
    (output_dir / "model_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=script_dir,
        help="Directory that will receive stl/, step/, and model_manifest.json",
    )
    parser.add_argument(
        "--repository-base",
        type=Path,
        default=resolve_repository_base(repo_root),
        help="Path to repository base_link.STL (metre units)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    repository_base = args.repository_base.resolve()
    if not repository_base.is_file():
        raise FileNotFoundError(repository_base)
    manifest = generate(output_dir, repository_base)
    print(f"Generated reBotArm AGV v1 under: {output_dir}")
    for name, metrics in manifest["stl_metrics"].items():
        print(
            f"  {name}: triangles={metrics['triangles']}, "
            f"size_mm={metrics['size_mm']}, finite={metrics['finite']}"
        )


if __name__ == "__main__":
    main()
