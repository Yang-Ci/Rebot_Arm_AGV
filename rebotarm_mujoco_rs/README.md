# rebotarm_mujoco_rs

ROS 2 integration for the B601-RS MuJoCo model. The required model XML and STL
meshes are tracked directly in `models/`, based on
`LAN-GER/reBot-B601-RS-for-mujoco_sim` revision
`1249cb6efdf393ba636056fc41df30dc6ba389aa` with local gripper, material, and
Seeed-badge updates. The upstream repository had no LICENSE file at that
revision.

Only model assets are used. This package does not import the upstream
`rebot_b601_rs_sim` algorithms, QP solver, examples, or tests. ROS integration,
scene detection, IK, and task execution are implemented in this package.

This local package is copied from `/home/robot/reBot_Arm_Mujoco-RS` and then
extended in this repository; the reference repository remains read-only. The
default model is `models/rs_agv_scene.xml`, which mounts the B601-RS on the AGV
v1 chassis and adds four grounded wheel joints.

Build it with the sibling `rebotarm_msgs` package so the workspace is
self-contained.

Regenerate the combined model after changing either RS scene or AGV CAD:

```bash
python3 tools/build_rs_agv_model.py
```

The generator copies the required AGV STL files from
`mobile_base/cad/agv_v1/stl` into `models/meshes/agv/`, so a clean rebuild has
all runtime assets.

The wrapper does not open SocketCAN or send hardware commands. It subscribes to
ROS `JointState`, so the real arm continues to have a single owner: the
`reBotArmController` node.

The RS package now also includes a physics grasp environment with red, blue,
and yellow objects, overhead and D405-style wrist ROS cameras, object detections, Cartesian IK,
trajectory actions, and task recording services used by `rebotarm_agent`.

Modes:

- `kinematic`: directly synchronizes ROS joint state into MuJoCo.
- `physics`: tracks ROS targets with conservative PD plus MuJoCo bias forces.

Both modes accept differential-drive commands on `/cmd_vel` by default. The
kinematic mode integrates the chassis pose directly; the physics mode drives the
four wheel actuators with a velocity PD controller. The synchronized base pose
is published on `<namespace>/mujoco/base_pose` and consumed by the scene camera
and task server. Set `cmd_vel_topic:=/<namespace>/cmd_vel` at launch time if a
namespaced velocity input is required.

The combined scene also provides the standard ground-robot interfaces used by
SLAM Toolbox and Nav2. A front 2D lidar placeholder is mounted at
`x=0.185 m, z=0.150 m` in the `laser` frame, and a 2 m x 2 m perimeter wall
makes the simulated scan observable. The bridge publishes perfect odometry and
TF from MuJoCo, and computes `/scan` with MuJoCo ray casting. These topic and
frame conventions follow the
[linorobot2](https://github.com/linorobot/linorobot2) reference project; no
linorobot2 source is included.

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:=rebotarm_rs simulation_mode:=physics use_viewer:=true
```

Important topics:

- `/cmd_vel`
- `/odom`
- `/scan`
- `/tf`
- `/rebotarm_rs/mujoco/base_pose`
- `/rebotarm_rs/mujoco/object_states`
- `/rebotarm_rs/mujoco/overhead_rgb/image_raw`
- `/rebotarm_rs/mujoco/wrist_rgb/image_raw`
- `/rebotarm_rs/mujoco/wrist_rgb/camera_info`
- `/rebotarm_rs/vision/color_blocks/detections`

Task endpoints:

- `/rebotarm_rs/move_to_pose_ik`
- `/rebotarm_rs/move_to_pose`
- `/rebotarm_rs/follow_joint_trajectory`
