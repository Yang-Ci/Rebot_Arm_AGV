from pathlib import Path

import mujoco
import numpy as np

from rebotarm_mujoco_rs.mujoco_sync import RsMujocoSync


MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "rs_agv_scene.xml"


def test_agv_model_has_navigation_contract() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    required = {
        mujoco.mjtObj.mjOBJ_BODY: ("base_link", "agv_chassis", "laser"),
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


def test_quaternion_multiply_composes_yaw() -> None:
    quarter_turn = np.array(
        [np.cos(np.pi / 4.0), 0.0, 0.0, np.sin(np.pi / 4.0)]
    )

    result = RsMujocoSync._quat_multiply(quarter_turn, quarter_turn)

    assert np.allclose(result, [0.0, 0.0, 0.0, 1.0], atol=1e-12)
