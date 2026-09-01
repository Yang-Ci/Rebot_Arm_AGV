# AGV 轮子方案调研

目标是给 `480 × 420 × 120 mm` 的 AGV v1 底盘选轮。以下方案按驱动方式比较，轮径先统一从 80-125 mm 档位里选，避免改变现有底盘离地高度太多。

## 推荐顺序

| 优先级 | 方案 | 适合场景 | 主要代价 |
| --- | --- | --- | --- |
| 1 | 4 麦克纳姆轮 | 室内机械臂移动、需要横移和原地转向 | 对地面、编码器和标定要求高 |
| 2 | 2 差速驱动轮 + 2 万向脚轮 | 低成本、结构简单、验证底盘速度 | 不能横移，转向占空间 |
| 3 | 4 普通全向轮 | 低速室内全向、比麦轮便宜 | 承载和越障弱，振动明显 |
| 4 | 4 轮滑移转向 | 承载和稳定性更好、结构紧凑 | 转向轮胎磨损，里程计误差大 |
| 5 | 转向轮 / 阿克曼 | 直线性能好，适合较高速移动 | 机械和控制复杂度最高 |

## 方案细节

### 1. 四麦克纳姆轮

- 建议规格：`Ø100 × 40 mm` 左右，配合带编码器的直流减速电机或无刷轮毂电机。
- 速度建议：轮径 100 mm 时，150-300 RPM 约对应 `0.8-1.6 m/s`。
- 优点：前后、横移、斜线和原地旋转都可控，机械臂换工位时最方便。
- 注意：需要平整地面、准确轮距/轴距标定和四轮独立闭环；地毯、灰尘和轮子磨损会明显打滑。
- 参考：Husarion ROSbot XL 的麦轮版使用 `0.05 m` 轮半径、`0.05 m` 轮宽，轮距 `0.270 m`，轴距 `0.170 m`，并提供 ROS 2 驱动控制器。

参考资料：

- [Husarion ROSbot XL ROS 2](https://github.com/husarion/rosbot_xl_ros)
- [ROSbot XL wheel.urdf.xacro](https://github.com/husarion/rosbot_xl_ros/blob/master/rosbot_xl_description/urdf/wheel.urdf.xacro)
- [Linorobot2 Mecanum Drive](https://github.com/linorobot/linorobot2)

### 2. 两差速轮 + 万向脚轮

- 建议规格：两个 `Ø100 × 40 mm` 驱动轮，前后各一个或两个 `Ø80-100 mm` 万向脚轮。
- 优点：电机、驱动器、控制和里程计最简单，成本低，可靠性高。
- 注意：不能横移；机械臂伸出后重心变化会加大转向阻力，脚轮需要有足够承载。
- 参考：TurtleBot3 Burger 使用两驱动轮加后部支撑，轮半径 `0.033 m`，轮宽 `0.018 m`，左右轮距 `0.160 m`。

参考资料：

- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [TurtleBot3 Burger URDF](https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_description/urdf/turtlebot3_burger.urdf)
- [Linorobot2 2WD](https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_description/urdf/2wd_properties.urdf.xacro)

### 3. 四普通全向轮

- 建议规格：`Ø80-100 mm`、轮宽 `25-40 mm`，轮轴按矩形布置。
- 优点：也能实现全向移动，结构通常比麦轮便宜，滚轮更换方便。
- 注意：单排滚轮连续通过接缝会有振动；承载能力低于同尺寸实心轮，不适合粗糙地面。
- 参考：Linorobot2 提供全向/麦轮控制配置，便于对比 ROS 2 控制。

参考资料：

- [Linorobot2 omni_drive_controller](https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_description/urdf/controllers/omni_drive.urdf.xacro)
- [ROS 2 omni wheel examples](https://github.com/emre-tasocak/omni3_ws)

### 4. 四轮滑移转向

- 建议规格：`Ø100-125 mm` 充气轮、橡胶实心轮或高强度免充气轮，四轮独立驱动。
- 优点：结构紧凑、承载和稳定性好，底盘高度容易控制。
- 注意：原地转向时轮胎侧滑，会磨损轮面并引入里程计误差；需要 IMU 或视觉定位辅助。
- 参考：Leo Rover 常规四轮底盘轮半径为 `0.0625 m`，并可作为四轮移动平台参考。

参考资料：

- [Leo Rover ROS 2 packages](https://github.com/LeoRover/leo_common-ros2)
- [Leo Rover description](https://github.com/LeoRover/leo_common-ros2/blob/ros2/leo_description/urdf/macros.xacro)

### 5. 转向轮 / 阿克曼

- 建议规格：两个驱动轮加两个转向轮，或四轮独立转向；轮径 `Ø100-125 mm`。
- 优点：直线行驶效率高，轮胎磨损小，适合较长距离和较高速的仓库路线。
- 注意：机械结构、转向零位标定和控制都比前四种复杂，不能简单原地旋转。

参考资料：

- [ROS 2 Gazebo Ackermann steering vehicle](https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle)
- [Ackermann steering ROS 2 examples](https://github.com/samuko-things/robo_car)

## 可借鉴的开源轮模型

- [ROSbot XL mecanum meshes](https://github.com/husarion/rosbot_xl_ros/tree/master/rosbot_xl_description/meshes)
- [Linorobot2 wheel and mecanum wheel xacro](https://github.com/linorobot/linorobot2/tree/jazzy/linorobot2_description/urdf/mech)
- [TurtleBot3 wheel STL](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_description/meshes/wheels)
- [Leo Rover wheel models](https://github.com/LeoRover/leo_common-ros2/tree/ros2/leo_description/models)
- [Highly Configurable Wheel](https://github.com/alexfranke/Highly-Configurable-Wheel)

## 下一步建议

1. 先按现有 `Ø100 × 40 mm` 占位轮做四麦轮 URDF/MJCF，因为它最能保留 AG v1 的全向移动能力。
2. 硬件选型前测量整机满载重量、电池位置和机械臂最大伸出力矩，再确定每轮额定载荷。
3. 如果预算优先，先做两差速轮加脚轮的最小可行版本，验证底盘、电源和导航后再换麦轮。

