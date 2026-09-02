# Rebot Arm AGV

这个仓库归档 B601 机械臂的两套结构、第一版 AGV 底盘结构和轮式方案调研，并提供一个本地复制后的 RS MuJoCo/ROS 2 仿真包。参考仓库 `/home/robot/reBot_Arm_Mujoco-RS` 不被修改。

## 目录

```text
Rebot_Arm_AGV/
├── RS/                      # B601-RS URDF、视觉/碰撞/MuJoCo 网格
├── DM/                      # B601-DM URDF、视觉/碰撞/MuJoCo 网格
├── rebotarm_msgs/           # 本地 ROS 2 消息/服务/action 定义
├── rebotarm_mujoco_rs/      # 本地 RS + AGV MuJoCo/ROS 2 仿真包
├── rebotarm_gazebo/         # Gazebo/MuJoCo 共用的实验室、SLAM 与 Nav2 启动包
├── mobile_base/
│   ├── cad/agv_v1/          # 第一版 AGV 底盘 STEP/STL、生成脚本和校验图
│   └── docs/                # 底盘概念图和轮子方案调研
├── tools/                   # RS + AGV 组合模型生成脚本
└── docs/
    └── arm_description.md   # RS/DM 复用说明、网格分类和同步规则
```

## 快速查看

- RS 主模型：`RS/urdf/ReBot_Arm_RS.urdf`
- DM 主模型：`DM/urdf/ReBot_Arm_DM.urdf`
- AGV v1 装配参考：`mobile_base/cad/agv_v1/stl/agv_chassis_with_repository_base_v1.stl`
- RS + AGV MuJoCo 组合模型：`rebotarm_mujoco_rs/models/rs_agv_scene.xml`
- 组合模型预览：`rebotarm_mujoco_rs/models/rs_agv_scene_preview.png`
- AGV 可编辑装配：`mobile_base/cad/agv_v1/step/agv_chassis_assembly_v1.step`
- 轮子方案：`mobile_base/docs/wheel_options.md`

两个机械臂 URDF 都使用 `../meshes/...` 相对路径。复制或加载时必须保留 `urdf/` 和 `meshes/` 的相对层级，Linux 下也不要改动 `.STL` 与 `.stl` 的大小写。

## 当前状态

- RS/DM 来自 `/home/robot/桌面/Rebot_Arm_description`，保留全部 URDF 和 STL。
- `mobile_base/cad/agv_v1/` 来自新生成的 AGV v1 底盘模型，含参数化生成脚本。
- 轮子文件目前是 `Ø100 × 40 mm` 干涉检查占位件，不是可直接生产的轮胎。
- `rebotarm_mujoco_rs/` 从 `/home/robot/reBot_Arm_Mujoco-RS` 复制后修改，默认加载 `rs_agv_scene.xml`，支持 `cmd_vel` 驱动 AGV，并发布 `/clock`、`/odom`、`/scan`、`/imu` 和导航 TF。
- MuJoCo 与 Gazebo 使用同一套 8 x 6 m 实验室坐标、静态地图、SLAM Toolbox 和 Nav2 参数，可切换仿真后端而无需改导航接口。

## 本地仿真

```bash
python3 tools/build_rs_agv_model.py
colcon build --symlink-install --packages-select \
  rebotarm_msgs rebotarm_mujoco_rs rebotarm_gazebo
source install/setup.bash
ros2 launch rebotarm_gazebo mujoco_slam_nav.launch.py
```

MuJoCo 已支持三种完整导航流程：

```bash
ros2 launch rebotarm_gazebo mujoco_slam.launch.py        # 在线建图
ros2 launch rebotarm_gazebo mujoco_slam_nav.launch.py    # 边建图边导航
ros2 launch rebotarm_gazebo mujoco_navigation.launch.py  # 已有地图 + AMCL + Nav2
```

组合场景带前置 2D 激光雷达、IMU 和完整实验室障碍物。仿真桥发布
`/clock`、`/odom`、`/scan`、`/imu` 及
`odom -> base_footprint -> base_link -> laser/imu_link` TF，接口约定参考
[linorobot2](https://github.com/linorobot/linorobot2)。linorobot2 仅作为只读参考，源码没有复制进本仓库。

默认速度入口采用 linorobot2/Nav2 约定的 `/cmd_vel`。无界面运行时给启动命令
增加 `use_viewer:=false use_rviz:=false`；需要保留旧命名空间话题时，可在核心
MuJoCo 启动命令中增加 `cmd_vel_topic:=/rebotarm_rs/cmd_vel`。
