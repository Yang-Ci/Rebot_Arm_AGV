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

If no fresh joint-state target arrives within `stale_timeout`, the bridge
returns to `arm_idle_position` (all-zero by default) and keeps driving toward
that pose. By default `arm_idle_lock` additionally applies a parking-brake
style lock during navigation, which prevents wheel-ground contact vibration
from exciting the arm. `gripper_idle_position` defaults to closed.

Both modes accept differential-drive commands on `/cmd_vel` by default. The
kinematic mode integrates the chassis pose directly. Physics mode defaults to
Gazebo-like planar velocity tracking for repeatable navigation; set
`agv_drive_mode:=wheel` to use four-wheel traction and slip instead. The synchronized base pose
is published on `<namespace>/mujoco/base_pose` and consumed by the scene camera
and task server. Set `cmd_vel_topic:=/<namespace>/cmd_vel` at launch time if a
namespaced velocity input is required. The default limits are `0.80 m/s`
forward, `0.50 m/s` reverse, and `1.80 rad/s` yaw; override them with
`agv_max_linear_velocity`, `agv_max_reverse_velocity`, and
`agv_max_angular_velocity` when needed.

The combined scene also provides the standard ground-robot interfaces used by
SLAM Toolbox and Nav2. It reproduces the Gazebo package's 8 x 6 m laboratory,
including boundary walls, interior partitions, workcell, storage rack, task
zones, and a movable pallet. The robot starts at `(-2.45, -0.55)`, matching the
pre-built map and AMCL configuration.

A front 2D lidar is mounted at `x=0.185 m, z=0.150 m` in the `laser` frame, and
an IMU is mounted in the `imu_link` frame. The bridge publishes simulation
clock, odometry, IMU, TF, and ray-cast laser scans. Optional range noise can be
enabled with `laser_range_noise_stddev`; its random sequence is reproducible
through `sensor_noise_seed`. These topic and frame conventions follow the
[linorobot2](https://github.com/linorobot/linorobot2) reference project; no
linorobot2 source is included.

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:=rebotarm_rs simulation_mode:=physics use_viewer:=true
```

For mapping and navigation, use the launch files in `rebotarm_gazebo`, which
share the same map and Nav2 parameters with Gazebo:

```bash
# Online mapping
ros2 launch rebotarm_gazebo mujoco_slam.launch.py

# Online mapping and navigation together
ros2 launch rebotarm_gazebo mujoco_slam_nav.launch.py

# Localization and navigation on the checked-in map
ros2 launch rebotarm_gazebo mujoco_navigation.launch.py
```

Add `use_viewer:=false use_rviz:=false` for a headless run. The MuJoCo bridge
must keep wall time internally because it is the `/clock` source; all other ROS
nodes use simulation time. Viewer rendering is synchronized at 30 FPS by
default, independently of the 250 Hz physics update; override it with
`viewer_sync_rate` if needed.

Important topics:

- `/cmd_vel`
- `/clock`
- `/odom`
- `/scan`
- `/imu`
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
