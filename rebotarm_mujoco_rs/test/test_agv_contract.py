from pathlib import Path

import mujoco
import numpy as np

from rebotarm_mujoco_rs.mujoco_sync import RsMujocoSync


MODEL_PATH = (
    Path(__file__).resolve().parents[1] / "models" / "rs_agv_scene.xml"
)


def test_agv_model_has_navigation_contract() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    required = {
        mujoco.mjtObj.mjOBJ_BODY: (
            "base_link",
            "agv_chassis",
            "laser",
            "imu",
            "dynamic_pallet",
        ),
        mujoco.mjtObj.mjOBJ_JOINT: (
            "agv_freejoint",
            "wheel_front_left_joint",
            "wheel_front_right_joint",
            "wheel_rear_left_joint",
            "wheel_rear_right_joint",
        ),
        mujoco.mjtObj.mjOBJ_ACTUATOR: (
            "wheel_front_left_motor",
            "wheel_front_right_motor",
            "wheel_rear_left_motor",
            "wheel_rear_right_motor",
        ),
    }
    for object_type, names in required.items():
        for name in names:
            assert mujoco.mj_name2id(model, object_type, name) >= 0, name


def test_agv_model_matches_lab_navigation_coordinates() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    freejoint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "agv_freejoint"
    )
    qpos_address = model.jnt_qposadr[freejoint_id]
    assert np.allclose(
        data.qpos[qpos_address : qpos_address + 3],
        [-2.45, -0.55, 0.136],
        atol=1e-9,
    )

    for geom_name in (
        "north_boundary",
        "south_boundary",
        "east_boundary",
        "west_boundary",
        "north_partition",
        "south_partition",
        "storage_rack",
    ):
        assert mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
        ) >= 0, geom_name


def test_periodic_schedule_preserves_average_rate() -> None:
    deadline = -np.inf
    publications = []
    for step in range(1, 251):
        now = step * 0.004
        due, deadline = RsMujocoSync._advance_schedule(now, deadline, 0.01)
        if due:
            publications.append(now)

    # The physics step cannot represent 10 ms exactly, so intervals alternate
    # between 8 ms and 12 ms while preserving a 100 Hz average.
    assert len(publications) == 100
    assert set(np.round(np.diff(publications), 3)) == {0.008, 0.012}


def test_colored_ground_zones_are_visual_only_sites() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    for name in (
        "charging_zone",
        "delivery_zone",
        "dynamic_zone",
        "workcell_dock",
        "lane_left",
        "lane_right",
    ):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        assert site_id >= 0, name
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) < 0
        assert model.site_size[site_id, 2] <= 0.00005


def test_quaternion_multiply_composes_yaw() -> None:
    quarter_turn = np.array(
        [np.cos(np.pi / 4.0), 0.0, 0.0, np.sin(np.pi / 4.0)]
    )

    result = RsMujocoSync._quat_multiply(quarter_turn, quarter_turn)

    assert np.allclose(result, [0.0, 0.0, 0.0, 1.0], atol=1e-12)
