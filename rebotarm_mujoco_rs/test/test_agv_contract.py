from pathlib import Path
import time

import mujoco
import numpy as np
import rclpy
from std_srvs.srv import Trigger

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


def test_agv_wheel_contacts_are_stable() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for name in (
        "wheel_front_left",
        "wheel_front_right",
        "wheel_rear_left",
        "wheel_rear_right",
    ):
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"{name}_collision"
        )
        assert np.allclose(
            model.geom_solref[geom_id], [0.02, 1.0], atol=1e-12
        )
        assert np.allclose(
            model.geom_solimp[geom_id],
            [0.9, 0.95, 0.001, 0.5, 2.0],
            atol=1e-12,
        )


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


def test_reset_preserves_monotonic_ros_time() -> None:
    rclpy.init()
    node = RsMujocoSync()
    try:
        node.data.time = 12.5
        before = node._simulation_time()

        response = node._reset(None, Trigger.Response())

        assert response.success
        assert node.data.time == 0.0
        assert node._simulation_time() == before
        node.data.time = 0.01
        assert node._simulation_time() > before
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_idle_arm_holds_zero_while_agv_moves() -> None:
    rclpy.init()
    node = RsMujocoSync()
    try:
        node.last_input_time = None

        # Kinematic mode must return an already-disturbed arm to its idle pose.
        node.data.qpos[node.arm_qpos_addrs] = np.full(6, 0.2)
        node._last_update_time = time.monotonic() - 0.004
        node._update()
        assert np.allclose(node.data.qpos[node.arm_qpos_addrs], 0.0)

        # With no fresh joint command, physics must keep commanding zero while
        # the AGV is driven, rather than adopting the drifting current pose.
        node.simulation_mode = "physics"
        node._cmd_vel = np.array([0.8, 0.0])
        for _ in range(500):
            node._cmd_vel_time = time.monotonic()
            node._last_update_time = time.monotonic() - 0.004
            node._update()

        assert np.allclose(node.target_arm, 0.0)
        assert np.allclose(node.data.qpos[node.arm_qpos_addrs], 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
