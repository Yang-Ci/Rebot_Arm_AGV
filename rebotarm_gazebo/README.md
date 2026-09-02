# Rebot Arm AGV Gazebo laboratory

This package provides the first Gazebo Harmonic test environment for the combined AGV and arm project. The 8 x 6 m laboratory uses a TurtleBot3-style SLAM layout with low walls, interior partitions and doorways, while retaining the MuJoCo-equivalent manipulation workcell, storage rack, charging zone, delivery zone, a clear 1.2 m navigation corridor, and a dynamic-obstacle test area.

The package also provides a complete Nav2 autonomous navigation stack: online SLAM mapping, AMCL localization with a pre-built map, MPPI trajectory control, Navfn global planning, costmaps, and recovery behaviors.

The world includes the local RS arm on the four-wheel AGV chassis. Its Gazebo interface publishes `/scan`, `/imu`, `/odom`, `/tf` and `/joint_states`, and accepts `geometry_msgs/msg/Twist` on `/cmd_vel`. The default navigation profile uses Gazebo model velocity control for repeatable SLAM tests; the four wheel links and their collision geometry remain in the model for a later traction-calibration profile.

## Build

```bash
cd /home/robot/Rebot_Arm_AGV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rebotarm_gazebo
source install/setup.bash
```

## Launch

### Basic world + map (`lab_demo`)

Gazebo world with the pre-built occupancy map published on `/map`. No navigation stack:

```bash
ros2 launch rebotarm_gazebo lab_demo.launch.py
```

### Online SLAM mapping (`lab_slam`)

Gazebo with SLAM Toolbox for exploring and building a new map. Drive the robot with teleop:

```bash
ros2 launch rebotarm_gazebo lab_slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After exploring the complete area, save the generated map in another terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f rebotarm_lab_slam
```

### SLAM + Nav2 (`lab_slam_nav`)

Gazebo, SLAM Toolbox, and the full Nav2 stack simultaneously: build a map and navigate at the same time. Useful for first-time exploration where no pre-built map exists. RViz launches with the Nav2 config:

```bash
ros2 launch rebotarm_gazebo lab_slam_nav.launch.py
```

Send a 2D Nav Goal from RViz or via CLI:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Save the resulting map afterwards:

```bash
ros2 launch rebotarm_gazebo lab_map_saver.launch.py
ros2 run nav2_map_server map_saver_cli -f rebotarm_lab_slam
```

### Autonomous navigation (`lab_navigation`)

Gazebo, the pre-built map, AMCL localization, and the full Nav2 stack. Use this after a map already exists:

```bash
ros2 launch rebotarm_gazebo lab_navigation.launch.py
```

### Map saver (`lab_map_saver`)

Launch the Nav2 map saver server standalone (assumes SLAM is running):

```bash
ros2 launch rebotarm_gazebo lab_map_saver.launch.py
ros2 run nav2_map_server map_saver_cli -f rebotarm_lab_slam
```

### Common launch arguments

All Gazebo-based launches accept:

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `true` | Start Gazebo GUI (`false` for headless) |
| `use_rviz` | `true` | Start RViz with the Nav2 config (`lab_slam_nav`, `lab_navigation` only) |
| `params_file` | `config/nav2_params.yaml` | Override Nav2 parameters |
| `map` | `maps/rebotarm_lab.yaml` | Override the map file (`lab_navigation` only) |
| `autostart` | `true` | Auto-activate Nav2 lifecycle nodes |

For a server-only smoke test:

```bash
ros2 launch rebotarm_gazebo lab_slam_nav.launch.py gui:=false use_rviz:=false
```

The static map is published on `/map`, Gazebo simulation time and robot topics are bridged to ROS 2, and `config/semantic_zones.yaml` records the task polygons and approach poses.

## MuJoCo SLAM and navigation

The same laboratory geometry, map coordinates, SLAM Toolbox configuration,
Nav2 parameters, and RViz layout are available with MuJoCo. MuJoCo publishes
`/clock`, `/scan`, `/imu`, `/odom`, and the navigation TF tree directly, so no
Gazebo bridge is started.

```bash
# MuJoCo laboratory and robot interfaces only
ros2 launch rebotarm_gazebo mujoco_lab.launch.py

# Online SLAM mapping
ros2 launch rebotarm_gazebo mujoco_slam.launch.py

# Build a map and navigate at the same time
ros2 launch rebotarm_gazebo mujoco_slam_nav.launch.py

# AMCL localization and navigation with maps/rebotarm_lab.yaml
ros2 launch rebotarm_gazebo mujoco_navigation.launch.py
```

All MuJoCo launches default to physics mode. They accept `use_viewer:=false`
for headless simulation; launches containing RViz also accept
`use_rviz:=false`. `mujoco_navigation.launch.py` additionally accepts `map`,
`params_file`, and `autostart`; `mujoco_slam_nav.launch.py` accepts
`params_file`, `slam_params_file`, and `autostart`. The default limits are
`0.80 m/s` forward, `0.50 m/s` reverse, and `1.80 rad/s` yaw. All four launches
accept `agv_max_linear_velocity`, `agv_max_reverse_velocity`, and
`agv_max_angular_velocity` overrides.

To drive manually while mapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f rebotarm_lab_mujoco
```

## Configuration

- `config/nav2_params.yaml` -- All Nav2 node parameters: AMCL localization (initial pose at robot spawn `-2.45, -0.55`), MPPI controller (DiffDrive, `vx_max=0.5`, `wz_max=1.0`), Navfn planner, local/global costmaps with the AGV footprint, recovery behaviors, smoother, and waypoint follower. Also contains a SLAM Toolbox section for `lab_slam_nav`.
- `config/slam_toolbox.yaml` -- Standalone SLAM Toolbox parameters for `lab_slam`.
- `config/nav2.rviz` -- RViz config with Grid, TF, LaserScan, Map, Global/Local Costmap, Path, AMCL Particles, and the Nav2 control panel.
- `config/ros_gz_bridge.yaml` -- Gazebo-to-ROS topic bridge definitions (`/scan`, `/imu`, `/odom`, `/tf`, `/joint_states`, `/cmd_vel`, `/clock`).
- `config/semantic_zones.yaml` -- Task zone polygons and approach poses.

Regenerate generated assets after changing static geometry or the RS URDF with:

```bash
python3 rebotarm_gazebo/scripts/generate_lab_map.py
python3 rebotarm_gazebo/scripts/generate_rs_agv_sdf.py
```

The workcell docking poses assume the current AGV footprint (480 x 500 mm including wheels): pre-dock `(2.15, 0, 0)` and dock `(2.50, 0, 0)`.

## TF tree

```
map -> odom -> base_footprint -> base_link -> laser
                                      \-> imu_link
```

- `map -> odom`: published by AMCL (`lab_navigation`) or SLAM Toolbox (`lab_slam_nav`)
- `odom -> base_footprint -> base_link`: published by Gazebo or the MuJoCo bridge
- Sensor transforms: published by Gazebo static transforms or the MuJoCo bridge
