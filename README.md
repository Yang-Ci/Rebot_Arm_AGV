# Rebot Arm AGV

这个仓库先归档 B601 机械臂的两套结构，以及第一版 AGV 底盘结构和轮式方案调研。它是一个模型与结构资料库，不包含机械臂仿真器或运行时代码。

## 目录

```text
Rebot_Arm_AGV/
├── RS/                      # B601-RS URDF、视觉/碰撞/MuJoCo 网格
├── DM/                      # B601-DM URDF、视觉/碰撞/MuJoCo 网格
├── mobile_base/
│   ├── cad/agv_v1/          # 第一版 AGV 底盘 STEP/STL、生成脚本和校验图
│   └── docs/                # 底盘概念图和轮子方案调研
└── docs/
    └── arm_description.md   # RS/DM 复用说明、网格分类和同步规则
```

## 快速查看

- RS 主模型：`RS/urdf/ReBot_Arm_RS.urdf`
- DM 主模型：`DM/urdf/ReBot_Arm_DM.urdf`
- AGV v1 装配参考：`mobile_base/cad/agv_v1/stl/agv_chassis_with_repository_base_v1.stl`
- AGV 可编辑装配：`mobile_base/cad/agv_v1/step/agv_chassis_assembly_v1.step`
- 轮子方案：`mobile_base/docs/wheel_options.md`

两个机械臂 URDF 都使用 `../meshes/...` 相对路径。复制或加载时必须保留 `urdf/` 和 `meshes/` 的相对层级，Linux 下也不要改动 `.STL` 与 `.stl` 的大小写。

## 当前状态

- RS/DM 来自 `/home/robot/桌面/Rebot_Arm_description`，保留全部 URDF 和 STL。
- `mobile_base/cad/agv_v1/` 来自新生成的 AGV v1 底盘模型，含参数化生成脚本。
- 轮子文件目前是 `Ø100 × 40 mm` 干涉检查占位件，不是可直接生产的轮胎。

