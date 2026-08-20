# V2_FLU 文件夹 STL 文件说明

> 目录: `Tools/simulation/gz/models/lingyun01/meshes/V2_FLU/`
> 用途: 灵云01号飞艇 Gazebo Harmonic 仿真模型的全部网格文件（FLU坐标系）
> 配套脚本: `../convert_stl_to_flu.py`（SW → FLU 转换）

---

## 1. 概述

本文件夹包含 **11 个 STL 文件**，全部为 **FLU 坐标系（X前 / Y左 / Z上）**、单位 **米**，中心位于 `(0, 0, 0)`（= base_link 原点 = 艇身包围盒中心）。

这些文件由 SolidWorks 导出的**原始 SW 坐标系（毫米）**STL（已归档至 `../V2_source_SW/`）转换而来，供 `model.sdf` 直接加载。

### 1.1 文件夹定位（重要）

本文件夹是**飞艇仿真模型正式使用的网格文件**，`model.sdf` 直接加载这里的 STL。与同级其他文件夹的关系：

| 文件夹 | 定位 | 是否使用 |
|--------|------|---------|
| **V2_FLU/** | ✅ **正式使用**（model.sdf 加载的唯一网格来源） | 是 |
| V2_source_SW/ | V2 原始 SolidWorks 源文件（可重新生成 V2_FLU） | 备份 |
| V1/ | V1 旧版模型（历史版本） | 否 |
| origin_centered_backup/ | 原点居中处理的备份 | 备份 |
| source_backup/ | V1 原始 SolidWorks 备份 | 备份 |

**维护原则**：改模型只改 `V2_source_SW/` 源文件，重新运行 [convert_stl_to_flu.py](../convert_stl_to_flu.py) 生成到本文件夹；**直接手改本文件夹的 STL 会丢失转换关系**。

---

## 2. 坐标系与变换方法

### 2.1 原始 SW 坐标系（毫米）

| 轴 | 方向 | 说明 |
|----|------|------|
| SW_X | 左右 | X 小 = 左，X 大 = 右 |
| SW_Y | 前后 | Y 大 = 前，Y 小 = 后 |
| SW_Z | 上下 | Z 大 = 上 |

### 2.2 转换公式（convert_stl_to_flu.py）

```
FLU_X =  SW_Y / 1000        (前为正)
FLU_Y = -SW_X / 1000        (左为正)
FLU_Z =  SW_Z / 1000        (上为正)
最后整体平移 -艇身中心FLU，使 hull 中心落在原点 (0,0,0)
```

### 2.3 平移基准

平移基准取自 `总体无螺旋桨.STL`（原始SW）的包围盒中心：

| 项 | SW 坐标 (mm) | FLU 坐标 (m) |
|----|-------------|-------------|
| 艇身中心 | X=11553, Y=17062, Z=5484 | (17.062, -11.553, 5.484) |
| 平移向量 translate | — | **(-17.062, +11.553, -5.484)** |

转换后所有文件的包围盒中心均落在 FLU 原点 `(0, 0, 0)`。

---

## 3. 文件清单总表

| # | V2_FLU 文件 | 原始 SW 文件 | 用途（model.sdf） | 旋转轴/推力 |
|---|------------|-------------|------------------|------------|
| 1 | hull_all.STL | 总体无螺旋桨.STL | base_link 外观 + 碰撞体 | — |
| 2 | lift_up_lf.STL | 上升螺旋桨左前.STL | rotor_0 上升左前 | Z轴 / 向上 |
| 3 | lift_up_rf.STL | 上升螺旋桨右前.STL | rotor_1 上升右前 | Z轴 / 向上 |
| 4 | lift_up_lb.STL | 上升螺旋桨左后.STL | rotor_2 上升左后 | Z轴 / 向上 |
| 5 | lift_up_rb.STL | 上升螺旋桨右后.STL | rotor_3 上升右后 | Z轴 / 向上 |
| 6 | lift_dn_f.STL | 下降螺旋桨前.STL | rotor_4 下降前 | Z轴 / 向下 |
| 7 | lift_dn_b.STL | 下降螺旋桨后.STL | rotor_5 下降后 | Z轴 / 向下 |
| 8 | thrust_lf.STL | 推进螺旋桨左前.STL | rotor_6 推进左前 | X轴 / 向前 |
| 9 | thrust_rf.STL | 推进螺旋桨右前.STL | rotor_7 推进右前 | X轴 / 向前 |
| 10 | thrust_lb.STL | 推进螺旋桨左后.STL | rotor_8 推进左后 | X轴 / 向前 |
| 11 | thrust_rb.STL | 推进螺旋桨右后.STL | rotor_9 推进右后 | X轴 / 向前 |

---

## 4. 各文件详细说明

### 4.1 hull_all.STL — 飞艇主体（四囊体）

- **原始文件**: `../总体无螺旋桨.STL`（SW毫米，经转换生成）
- **模型坐标范围**: X[-16.772, +16.772], Y[-11.251, +11.251], Z[-3.658, +3.658]
- **尺寸**: 33.543 × 22.503 × 7.315 m
- **三角形数**: 404,080
- **用途**:
  - [model.sdf#L69](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L69`) `hull_visual`（外观，奶白色材质）
  - [model.sdf#L84](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L84`) `hull_collision`（碰撞体）
- **说明**: 4 个椭球气囊拼接的整体外形（长轴沿 X、短轴沿 Y），含尾椎与吊舱结构。法线已验证 100% 归一化、左右完全对称。

### 4.2 上升电机螺旋桨（4 个，推力沿 +Z 向上）

| V2_FLU 文件 | 对应 link | 安装位置 FLU (X,Y,Z) | 旋向 | model.sdf 引用 |
|------------|-----------|----------------------|------|---------------|
| lift_up_lf.STL | rotor_0 上升左前 | (7.557, +6.338, -0.584) | CCW | [L187](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L187`) |
| lift_up_rf.STL | rotor_1 上升右前 | (7.557, -6.337, -0.584) | CW | [L240](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L240`) |
| lift_up_lb.STL | rotor_2 上升左后 | (-9.843, +5.814, -0.584) | CW | [L293](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L293`) |
| lift_up_rb.STL | rotor_3 上升右后 | (-9.843, -5.812, -0.584) | CCW | [L344](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L344`) |

- **尺寸**: 约 0.872 × 0.106 × 0.027 m（水平圆盘桨）
- **推力**: motorConstant>0 → 沿局部 +Z（向上，产生升力）
- **旋转方向平衡**: 对角同向、反扭矩抵消

### 4.3 下降电机螺旋桨（2 个，推力沿 -Z 向下）

| V2_FLU 文件 | 对应 link | 安装位置 FLU (X,Y,Z) | 旋向 | model.sdf 引用 |
|------------|-----------|----------------------|------|---------------|
| lift_dn_f.STL | rotor_4 下降前 | (6.957, +0.017, +0.505) | CW | [L407](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L407`) |
| lift_dn_b.STL | rotor_5 下降后 | (-11.443, +0.017, +0.505) | CCW | [L458](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L458`) |

- **尺寸**: 约 0.872 × 0.106 × 0.027 m（水平圆盘桨，与上升桨同型号）
- **推力**: motorConstant<0 → 沿局部 -Z（向下，产生下降力）
- **位置**: 位于艇身 X 中轴线上（Y≈0），前近后远

### 4.4 推进电机螺旋桨（4 个，推力沿 +X 向前）

| V2_FLU 文件 | 对应 link | 安装位置 FLU (X,Y,Z) | 旋向 | model.sdf 引用 |
|------------|-----------|----------------------|------|---------------|
| thrust_lf.STL | rotor_6 推进左前 | (2.230, +5.888, -2.378) | CW | [L526](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L526`) |
| thrust_rf.STL | rotor_7 推进右前 | (2.230, -5.887, -2.378) | CCW | [L579](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L579`) |
| thrust_lb.STL | rotor_8 推进左后 | (-2.971, +5.888, -2.378) | CW | [L632](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L632`) |
| thrust_rb.STL | rotor_9 推进右后 | (-2.972, -5.887, -2.378) | CCW | [L685](`file:///home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf#L685`) |

- **尺寸**: 约 0.026 × 0.15 × 1.62 m（竖直细长桨）
- **安装姿态**: link 绕 Y 轴 +90°，局部 Z 轴指向 FLU +X（向前推进）
- **推力**: 沿 +X（水平推进 + 左右差动偏航）
- **位置**: 位于重心下方（Z=-2.378），水平前后对称

---

## 5. 渲染材质汇总

各文件在 model.sdf 中的材质定义：

| 文件 | 材质 | 颜色 | 备注 |
|------|------|------|------|
| hull_all.STL | ambient 0.93 0.93 0.95 1.0 + diffuse 0.93 0.93 0.95 1.0 | 奶白色 | **不透明**（alpha=1.0，修复过半透明问题） |
| lift_up_*.STL | Gazebo/DarkGrey | 深灰 | 螺旋桨 |
| lift_dn_*.STL | Gazebo/DarkGrey | 深灰 | 螺旋桨 |
| thrust_*.STL | Gazebo/DarkGrey | 深灰 | 螺旋桨 |

> 注：风机/阀门为 model.sdf 内建的绿色圆柱 / 黄色圆盘（可视化占位），不在本文件夹。

---

## 6. 模型结构与文件关系

`model.sdf` 中各 link 与本文件夹 mesh 的加载关系：

```
model.sdf
├── base_link  (原点=艇身中心, 重心(0,0,-1.5))
│   ├── visual/collision ── hull_all.STL           ← 主体外观+碰撞
│   ├── rotor_0~3 (上升, 推力+Z) ── lift_up_lf/rf/lb/rb.STL
│   ├── rotor_4~5 (下降, 推力-Z) ── lift_dn_f/b.STL
│   ├── rotor_6~9 (推进, 推力+X) ── thrust_lf/rf/lb/rb.STL
│   ├── blower/valve ×4 (风机/阀门可视化) ── model.sdf 内建几何
│   └── com_visual (重心红色球) ── model.sdf 内建几何
└── 电机插件 (rotor 动力学) ── 读取同位置 link
```

**坐标一致性要求**：mesh 文件的几何位置（本文件夹 FLU 坐标）必须与 model.sdf 中对应 link 的 `<pose>`、以及 PX4 控制分配 `CA_ROTOR*`（FRD 相对重心）三者保持一致。三者转换关系：

```
FLU(本文件夹)  ↔  model.sdf link pose（直接相等）  ↔  FRD 控制分配（需转换）
```

---

## 7. 命名规则

| 前缀 | 含义 | 后缀 | 含义 |
|------|------|------|------|
| `hull_all` | 主体四囊整体 | — | — |
| `lift_up_` | 上升电机（推力向上） | `lf/rf/lb/rb` | 左前/右前/左后/右后 |
| `lift_dn_` | 下降电机（推力向下） | `f/b` | 前/后 |
| `thrust_` | 推进电机（推力向前） | `lf/rf/lb/rb` | 左前/右前/左后/右后 |

对应关系：`lf`=LeftFront(左前), `rf`=RightFront(右前), `lb`=LeftBack(左后), `rb`=RightBack(右后)。

---

## 8. 转换脚本用法

```bash
cd meshes/
python3 convert_stl_to_flu.py V2_source_SW/  # 默认输出到 V2_FLU/
# 或指定输出目录
python3 convert_stl_to_flu.py V2_source_SW/ V2_FLU/
```

脚本会自动：
1. 扫描指定目录所有 `*.STL`（原始 SW 毫米文件，归档在 `V2_source_SW/`）
2. 以 `总体无螺旋桨.STL` 包围盒中心为平移基准
3. 旋转缩放平移 → 输出到 `V2_FLU/`

---

## 9. 维护与更新流程

当需要修改飞艇模型时，请按以下流程操作（**不要直接改 V2_FLU 下的 STL**）：

```
1. 修改 SolidWorks 源模型
   ↓
2. 导出对应中文命名 STL（毫米, SW坐标系）
   ↓
3. 覆盖到 V2_source_SW/ 对应文件
   ↓
4. 运行: python3 convert_stl_to_flu.py V2_source_SW/ V2_FLU/
   ↓
5. 若新增/删除部件, 同步更新 model.sdf 中 link/visual/collision
   ↓
6. 若电机位置变化, 同步更新 CA_ROTOR* (FRD) 控制分配参数
   ↓
7. 重新编译/启动仿真验证
```

**常见操作**：
- **只改外观**（不改物理）：直接改 V2_source_SW → 重新转换覆盖 hull_all.STL 即可
- **改电机位置**：改源文件 + 重新转换 + 同步 model.sdf link pose 和 CA_ROTOR 参数
- **换材质**：无需动 STL，直接改 model.sdf 的 `<material>` 块

---

## 10. 注意事项

1. **坐标系**: 本文件夹全部为 FLU（X前/Y左/Z上）、米、中心在原点；PX4 控制分配使用 FRD（X前/Y右/Z下）时需另行转换。
2. **中心定义**: 所有文件中心 = 艇身包围盒中心，**不是重心（CG）**。当前 model.sdf 重心定义为 `(0, 0, -1.5)`（艇身中心下方 1.5m）。
3. **材质**: hull 材质为奶白色（ambient/diffuse alpha=1.0，不透明）；螺旋桨为 `Gazebo/DarkGrey`。
4. **螺旋桨类型**: 所有螺旋桨 STL 均为**静止桨叶形态**（非旋转圆盘），Gazebo 的 rotor 插件会驱动 link 旋转，显示为旋转效果。
5. **风机/阀门占位**: 风机阀门为 model.sdf 内建的圆柱/圆盘（非本文件夹文件），仅作可视化，无独立 STL。
6. **文件版本**: 这些文件对应 V2 固件配置（airframe 2058_gz_lingyun01），与 V1 不兼容。
