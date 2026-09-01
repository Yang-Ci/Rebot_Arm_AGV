# reBotArm AGV 底盘 v1

这是基于仓库真实 `base_link.STL` 制作的第一版参数化 AGV 底盘 CAD。模型单位统一为
毫米，机械臂底座参考外包络为 `140 × 200 × 75 mm`。

## 首选查看文件

- `stl/agv_chassis_with_repository_base_v1.stl`：AGV、转接结构、四个螺丝占位件和仓库真实
  机械臂底座的装配参考；适合查看布局或导入仿真，不是一体打印件。
- `step/agv_chassis_assembly_v1.step`：不含机械臂底座的可编辑装配参考。
- `agv_v1_validation.png`：三个视角的装配检查图。

## 可制造分件

| 文件 | 数量 | 用途 |
|---|---:|---|
| `agv_bottom_tray_v1` | 1 | 480 × 420 × 120 mm 底盘框架，含轮轴避让和上盖固定孔 |
| `agv_top_cover_v1` | 1 | 6 mm 上盖，含转接板安装孔和贯通理线口 |
| `arm_adapter_plate_v1` | 1 | 210 × 280 × 10 mm 可更换机械臂转接板 |
| `latch_mount_v1_print_x2` | 2 | 金属快拆卡扣的左右安装座；卡扣本体应购买额定载荷金属件 |
| `cable_grommet_v1` | 1 | 70 × 38 mm 理线口护圈模型 |
| `fastener_placeholder_v1_print_x4` | 4 | 装配检查用螺丝/垫圈占位件，不含真实螺纹 |
| `wheel_placeholder_v1_print_x4` | 4 | 100 × 40 mm 轮子干涉检查占位件，不是轮胎生产模型 |

每个分件同时提供 `.stl` 和 `.step`。装配 STL 包含多个相互接触的封闭实体，只应用于
查看；切片、CNC 或加工时使用上表中的独立分件。

## 机械臂安装

仓库底座的四个开口 U 形槽中心按 STL 测得约为：

```text
x = ±45 mm
y = ±90 mm
```

转接板在这四个位置预留 `6.8 mm` 孔，默认设计意图是在铝板上攻 M8 螺纹。若使用聚合物
打印件，应改用足够强度的金属螺纹嵌件或贯穿螺栓和锁紧螺母。实物装配前必须重新测量槽宽、
底板厚度、垫圈直径和螺丝长度。

理线口中心位于转接板中心后方 `115 mm`，长轴沿底盘 X 方向。它与机械臂侧面接口窗口
对应，并在上盖和转接板中同时贯通。

## 主要尺寸

```text
底盘主体             480 × 420 × 120 mm
含轮总宽             约 500 mm
上盖厚度             6 mm
机械臂转接板         210 × 280 × 10 mm
轮子占位尺寸         Ø100 × 40 mm
完整参考装配高度     211 mm（包含原始底座，不包含机械臂其余连杆）
```

## 重新生成

生成脚本使用 CadQuery。建议使用独立临时环境，不要安装进机械臂运行用的 ROS venv：

```bash
python3 -m venv /tmp/rebot-agv-cad
/tmp/rebot-agv-cad/bin/pip install cadquery
/tmp/rebot-agv-cad/bin/python cad/agv_v1/generate_agv_v1.py
```

需要修改尺寸时，编辑 `generate_agv_v1.py` 中的 `AgvDimensions`，然后重新运行。导出结果
会更新 `stl/`、`step/` 和 `model_manifest.json`。

## 校核状态

- 所有独立分件均通过 CadQuery B-Rep 有效性检查；
- 所有独立 STL 均已验证为封闭网格且三角面绕序一致；
- 输出顶点均为有限值，尺寸记录见 `model_manifest.json`；
- 已在 MuJoCo 中加载真实 `base_link.STL` 做三视角装配检查。

这仍是结构验证 v1，不是已完成承载认证的生产设计。实际承载约 6.1 kg 机械臂前，应至少
完成转接板材料选择、底盘重心/抗倾覆计算、螺纹抗拔、急停工况和实体静载测试。建议转接板
优先使用 6061 铝合金或同等级金属材料，不要把打印塑料卡扣当作最终安全约束件。
