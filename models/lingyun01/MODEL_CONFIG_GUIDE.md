# 灵云01飞艇 - Gazebo模型配置详解

**文件路径**: `/home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf`

**最后更新**: 2026年8月18日

**版本**: v2.4

---

## 📁 文档概述

本文档详细介绍灵云01飞艇在Gazebo中的仿真模型配置（model.sdf），包括：
- 模型整体结构
- Link和Joint的定义
- 坐标系和参考框架
- 各组件的功能说明

---
## 🔗 Link和Joint的定义

### 1️⃣ Link组件定义

Gazebo 中一个完整的 link 由以下部分组成：

```
link（连杆）
├── <pose>          ← ⭐ Link相对于模型帧的位置和朝向
├── <inertial>      ← 惯性参数（质量、惯性张量、质心）
│   ├── <mass>      ← 质量 (kg)
│   ├── <inertia>   ← 惯性张量 (ixx, iyy, izz 等)
│   └── <pose>      ← 质心相对于link原点的偏移
├── <visual>        ← 可视化模型（仅用于渲染显示）
│   ├── <geometry>  ← 几何形状（mesh/box/cylinder等）
│   ├── <pose>     ← visual相对于link原点的偏移
│   └── <material>  ← 材质颜色
└── <collision>     ← 碰撞模型（用于物理碰撞检测）
    ├── <geometry>  ← 碰撞体几何形状
    └── <pose>     ← collision相对于link原点的偏移
```

#### 📍 **Link的 <pose> 含义（核心）**

根据 SDFormat 官方规范，**`<link><pose>` 定义的是该 Link 相对于模型帧（Model Frame）的位置和朝向**。

> 参考：[SDFormat Pose Frame Semantics](http://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/)
> "all link poses are specified relative to the model frame"

```xml
<!-- 以灵云01为例 (V2: 上升电机0, 左前) -->
<link name='rotor_0'>
  <pose>7.557 6.338 -0.584 0 0 0</pose>
</link>
```

这意味着：**rotor_0 的坐标系原点位于模型帧 (7.557, 6.338, -0.584) 处**。

**注意**：由于灵云01的模型帧（base_link原点）恰好在世界原点 (0,0,0)，所以 Link pose 的值**看起来**等于世界坐标，但本质上是相对于模型帧的偏移。

#### 🎨 **Visual / Collision 的 <pose> 含义**

`<visual>` 和 `<collision>` 的 `<pose>` 是**相对于其所属 Link 坐标系的偏移**。

```xml
<link name='rotor_6'>
  <pose>2.230 5.888 -2.378 0 1.5708 0</pose>       ← 相对于模型帧

  <visual name='rotor_6_visual'>
    <pose>-2.378 -5.888 -2.230 0 -1.5708 0</pose>  ← 相对于Link坐标系
  </visual>
</link>
```

**STL模型的最终渲染位置 = 模型帧 + Link pose + Visual pose**

#### 📐 **坐标变换链**

```
模型帧 (Model Frame / __model__)
    │
    │  Link.pose = (X, Y, Z, R, P, Y)
    │  → 确定Link坐标系在模型帧中的位置
    │
    └── Link坐标系
            │
            │  Visual.pose = (x, y, z, r, p, y)
            │  或 Collision.pose = (x, y, z, r, p, y)
            │  → 确定视觉/碰撞体在Link中的相对位置
            │
            └── STL模型实际渲染/碰撞位置
```

---

### 2️⃣ Joint组件定义

Joint 用于连接两个 Link，定义它们之间的运动约束关系：

```
joint（关节）
├── <parent>              ← 父Link名称
├── <child>               ← 子Link名称
├── <pose>                ← ⭐ Joint的位置（可选）
├── <type>                ← 关节类型
├── <axis>                ← 运动轴定义
│   ├── <xyz>             ← 旋转/滑动轴方向向量
│   ├── <limit>           ← 运动范围限制
│   ├── <dynamics>        ← 动力学参数（摩擦、阻尼）
│   └── <use_parent_model_frame> ← axis坐标系参考系
└── （其他可选参数...）
```

#### 🔧 **Joint类型**

| 类型 | 说明 | 自由度 | 典型用途 |
|------|------|--------|---------|
| **revolute** | 旋转关节 | 1（绕轴旋转） | 电机旋转、倾斜 |
| **prismatic** | 滑动关节 | 1（沿轴平移） | 导轨运动 |
| **fixed** | 固定关节 | 0 | 刚性连接 |
| **ball** | 球形关节 | 3 | 万向节 |

#### 📍 **Joint的 <pose> 含义**

根据 SDFormat 官方规范，**`<joint><pose>` 定义的是该 Joint 相对于子 Link 帧的位置和朝向**。

> 参考：[SDFormat Pose Frame Semantics](http://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/)
> "joint frames relative to their child link frames"

##### ✅ **当 Joint 没有 `<pose>` 时（隐式定义）**

```xml
<joint name='rotor_0_joint' type='revolute'>
  <!-- 没有 <pose> 元素 -->
  <parent>base_link</parent>
  <child>rotor_0</child>
  ...
</joint>
```

**SDFormat 的处理方式**：Joint 位于子 Link 的坐标系原点处。
即 `rotor_0_joint` 的位置 = `rotor_0` 的坐标系原点 = 模型帧 + rotor_0 的 pose = `(7.557, 6.338, -0.584)`。

##### ✅ **当 Joint 有显式 `<pose>` 时**

```xml
<joint name='example_joint' type='revolute'>
  <pose>1.0 2.0 3.0 0 0 0</pose>
  <parent>base_link</parent>
  <child>some_link</child>
</joint>
```

**Joint 位于子 Link 坐标系中 `(1.0, 2.0, 3.0)` 处**（相对于子Link帧的偏移）。

#### 🔄 **use_parent_model_frame 参数**

```xml
<axis>
  <xyz>0 -1 0</xyz>
  <use_parent_model_frame>1</use_parent_model_frame>
</axis>
```

此参数控制**旋转轴方向向量的参考坐标系**：

| 值 | axis 参考系 | 说明 |
|----|------------|------|
| **0**（默认） | 父 Link 的局部坐标系 | 轴方向随父 Link 旋转 |
| **1** | **模型世界坐标系**（base_link） | 轴方向始终固定不变 |

灵云01中所有 Joint 都使用 `use_parent_model_frame=1`，确保旋转轴基于 base_link 坐标系定义，不随父 Link 姿态变化。

---

### 3️⃣ Link 与 Joint 的协作关系

以上升电机 rotor_0 为例，完整展示 Link 和 Joint 如何配合工作（**V2 新方案：单 Link 结构，无 tilt 舵机，10电机布局**）：

```
┌─────────────────────────────────────────────────────────────┐
│                    模型世界坐标系                              │
│                                                             │
│  base_link: pose=(0, 0, 0)                                  │
│      │                                                      │
│      │  rotor_0_joint (无pose)                               │
│      │  位置 = 子Link(rotor_0)的坐标系原点                    │
│      │  轴: Z轴(0,0,1), 无限制                               │
│      │                                                      │
│      └── rotor_0: pose=(7.557, 6.338, -0.584)               │
│              │  相对于模型帧 = (7.557, 6.338, -0.584) OK     │
│              │  推力方向: motorConstant=+1.608e-03 (正值, 向上)│
│              │                                              │
│              └── visual: pose=(-7.557, -6.338, 0.584)       │
│                 STL最终位置 = link_pose + visual_pose        │
│                   + STL顶点 = STL顶点(相对主体中心) OK        │
└─────────────────────────────────────────────────────────────┘
```

**关键结论**：
- `rotor_0` 的 `<pose>` 值为 (7.557, 6.338, -0.584)，相对于模型帧
- 由于模型帧在世界原点，rotor_0 的世界坐标即 (7.557, 6.338, -0.584)
- Joint 无显式 pose，位于子 Link 的坐标系原点
- rotor_0 绕自身 Z 轴旋转产生升力，**推力方向由 motorConstant 符号控制**（正值向上，负值向下）
- **推力施加点**在 rotor_0 的 link 帧原点，即电机安装位置，力臂正确

**STL 相对主体中心补偿原理（V2）**：

V2 中所有电机 STL 均由 SW 坐标系（毫米）预转换到 FLU 米制（见 `meshes/convert_stl_to_flu.py`），再平移到相对主体中心（`meshes/V2_FLU/`）。因此 STL 顶点坐标本身就是相对主体中心的安装位置，link pose 设为电机安装位置后，通过 visual/collision 的 pose 做反向补偿让 STL 回到正确位置：

```
视觉最终位置 = link_pose + visual_pose + STL顶点坐标
            = (7.557,6.338,-0.584) + (-7.557,-6.338,0.584) + STL顶点
            = STL顶点(相对主体中心的安装位置)  --> 视觉位置正确
```

| 元素 | pose值 | 说明 |
|------|--------|------|
| rotor_0 link | (7.557, 6.338, -0.584) | 物理推力施加点，电机安装位置 |
| rotor_0 visual | (-7.557, -6.338, 0.584) | 补偿link pose偏移，使STL回到相对主体中心位置 |
| rotor_0 collision | (-7.557, -6.338, 0.584) | 同visual，碰撞体也在正确位置 |

---

## 🏗️ 模型整体结构

### 📊 Link组件列表

灵云01飞艇模型（V2, 10电机）由**19个Link**组成：1个主体 + 10个电机（4上升+2下降+4推进）+ 8个风机/阀门可视化占位（新方案：无 tilt_motor 支架，全部直接连接 base_link）：

| Link名称 | 类型 | 功能 | 质量(kg) |
|---------|------|------|---------|
| **base_link** | 主体 | 飞艇气囊主体 | 2206 (仿真占位值) |
| **rotor_0~3** | 上升螺旋桨 | 推力向上 (motorConstant=+1.608e-03) | 0.001 |
| **rotor_4~5** | 下降螺旋桨 | 推力向下 (motorConstant=-1.608e-03) | 0.001 |
| **rotor_6~9** | 推进螺旋桨 | 水平推力向前 | 0.001 |
| **blower_main_left/right, blower_aux_left/right** | 充气风机 | 四囊同步充气 (可视化占位) | 0.001 |
| **valve_main_left/right, valve_aux_left/right** | 排气阀门 | 四囊同步排气 (可视化占位) | 0.001 |

### 🔗 Link父子关系树状图

```
base_link (飞艇主体, 质量: 2206kg 仿真占位值, 重心=(0,0,-1.5))
│
├── rotor_0 (上升左前, 质量: 0.001kg, motorConstant=+1.608e-03, 推力向上, ccw)
│   └── lift_up_lf.STL
├── rotor_1 (上升右前, 质量: 0.001kg, motorConstant=+1.608e-03, 推力向上, cw)
│   └── lift_up_rf.STL
├── rotor_2 (上升左后, 质量: 0.001kg, motorConstant=+1.608e-03, 推力向上, cw)
│   └── lift_up_lb.STL
├── rotor_3 (上升右后, 质量: 0.001kg, motorConstant=+1.608e-03, 推力向上, ccw)
│   └── lift_up_rb.STL
├── rotor_4 (下降前, 质量: 0.001kg, motorConstant=-1.608e-03, 推力向下, cw)
│   └── lift_dn_f.STL
├── rotor_5 (下降后, 质量: 0.001kg, motorConstant=-1.608e-03, 推力向下, ccw)
│   └── lift_dn_b.STL
├── rotor_6 (推进左前, 质量: 0.001kg, motorConstant=+8.677e-03, 推力向前, cw)
│   └── thrust_lf.STL
├── rotor_7 (推进右前, 质量: 0.001kg, motorConstant=+8.677e-03, 推力向前, ccw)
│   └── thrust_rf.STL
├── rotor_8 (推进左后, 质量: 0.001kg, motorConstant=+8.677e-03, 推力向前, cw)
│   └── thrust_lb.STL
├── rotor_9 (推进右后, 质量: 0.001kg, motorConstant=+8.677e-03, 推力向前, ccw)
│   └── thrust_rb.STL
├── blower_main_left / blower_main_right (主囊充气风机, Y=±3.3)
├── blower_aux_left / blower_aux_right (副囊充气风机, Y=±8.6)
├── valve_main_left / valve_main_right (主囊排气阀门, Y=±3.3)
└── valve_aux_left / valve_aux_right (副囊排气阀门, Y=±8.6)
```

---

## 📐 坐标系说明

### 🌍 参考框架标准

本模型使用 **PX4 FLU标准**：

| 轴 | 方向 | 说明 |
|----|------|------|
| **X轴** | 前（Forward） | 飞艇前进方向 |
| **Y轴** | 左（Left） | 飞艇左侧方向 |
| **Z轴** | 上（Up） | 垂直向上 |

**坐标系类型**: 右手定则

### 📍 Base_link原点

- **位置**: (0, 0, 0)
- **含义**: base_link的坐标系原点，也是整个模型的参考原点

### 🎯 飞艇参考中心点 C

灵云01飞艇定义了一个**参考中心点 C**，作为质心、浮心和 CA_ROTOR 参数的统一参考基准。

**C 点坐标（V2）**: `(0, 0, 0)` — base_link原点 = 艇身包围盒中心 (FLU)
**重心坐标（V2）**: `(0, 0, -1.5)` FLU (base_link/inertial/pose)

#### C 点的计算来源（V2）

V2 中 C 点 = base_link 原点 = 艇身包围盒中心 (0,0,0)。重心相对 C 点下沉 1.5m，即 (0,0,-1.5)。

> **V1历史，已废弃**: V1 设计使用旧重心 `(-0.012, -2.894, -0.009)` 作为参考中心 C（旧 hull_all.STL 边界框中心，Y轴因 SolidWorks 画图原点偏离而不对称）。V2 已重新定义主体中心在原点，重心改为 (0,0,-1.5)。

#### C 点在项目中的使用位置（V2）

| 文件 | 配置项 | 值 | 说明 |
|------|--------|-----|------|
| `model.sdf` | `base_link/inertial/pose` | `0 0 -1.5` | 质心位置（Gazebo物理引擎在此计算动力学） |
| `model.sdf` | `com_visual/pose` | `0 0 -1.5` | 重心可视化球体位置 |
| `model.sdf` | `buoyancy_offset` | `0 0 -1.5` | 浮力偏移（AirshipDynamics插件用作comOffset） |
| `model.sdf` | `buoyancy_center` | `0 0 -1.5` | 浮力中心（与质心重合，消除初始力矩） |
| `2058_gz_lingyun01` | CA_ROTOR0~9 PX/PY/PZ | `FRD坐标, 相对重心(0,0,-1.5)` | 电机位置相对于重心的偏移（PX4 FRD） |
| `2058_gz_lingyun01` | `CA_AS_PZ_PROP` 等 | `0.878` | 推进电机到重心垂直距离 (m) |

#### 为什么质心和浮心都设为 C 点下方同一位置？

将质心（inertial pose）和浮心（buoyancy_center）都设为 (0,0,-1.5)，确保：
1. **浮力与重力共线** → 不产生初始力矩 → 飞艇不会在启动时旋转
2. **CA_ROTOR 参数基于重心** → 电机位置 = FLU坐标 - 重心，再转FRD
3. **控制分配器参数一致** → CA_AS_PZ_PROP 等常量与 CA_ROTOR 基于同一参考点（重心）

> **物理精度说明**: V2 重心 (0,0,-1.5) 为暂定设计值，浮力中心与质心重合，对中性浮力飞艇无初始力矩，浮力恢复力矩会自动修正微小偏差。

### 🔗 坐标系统一性原理

灵云01模型的核心设计原则：**FLU坐标系 = base_link坐标系 = 模型帧**，三者统一。

#### 为什么必须统一？

只有 base_link 原点 = 主体中心（即 base_link pose = (0,0,0)），电机的 link pose 才能直接使用 FLU 相对主体中心坐标，安装到飞艇外皮上设计的位置。推力施加点也在那个位置。

```
FLU坐标系 = base_link坐标系 = 模型帧
    |
    |  电机link pose = FLU相对主体中心坐标 (V2_FLU预转换)
    |  → 安装位置正确
    |  → 推力施加点 = 电机安装位置
    |
    |  STL相对主体中心 + visual反向pose = 视觉归位
    |  → 渲染位置正确
```

> **V1历史，已废弃**: V1 使用 SW 绝对坐标 STL（SW坐标系=base_link坐标系=模型帧）。V2 已通过 `meshes/convert_stl_to_flu.py` 将 SW 毫米坐标预转换为 FLU 米制相对主体中心坐标（`meshes/V2_FLU/`），STL 顶点坐标即安装位置。

#### inertial pose（重心）不改变坐标系

base_link 的 `inertial pose = (0, 0, -1.5)` 只是告诉 Gazebo 物理引擎"重心在哪"，**不改变坐标系**。所有 link pose 仍然基于模型帧（= FLU坐标系）计算。

```
坐标系: base_link原点 = 主体中心 = (0,0,0)        ← 不变
重心:   inertial_pose = (0, 0, -1.5)              ← 只是物理属性
电机0:  link_pose = (7.557, 6.338, -0.584)        ← 基于模型帧(FLU坐标系)
```

Gazebo 计算力矩时：`力矩 = (推力作用点 - 重心) × 推力方向`，自动使用 inertial_pose 作为参考点。

#### visual/collision 反向补偿的完整公式

由于电机 STL 使用 FLU 相对主体中心坐标，而 rotor 的 link pose 必须设为安装位置（物理正确），需要通过 visual/collision 的 pose 做反向补偿：

```
视觉最终位置 = 模型帧 + link_pose + visual_pose + STL顶点坐标
            = (0,0,0) + (7.557,6.338,-0.584) + (-7.557,-6.338,0.584) + STL顶点
            = STL顶点(FLU相对主体中心)
            → 视觉归位正确

物理推力点 = 模型帧 + link_pose
           = (0,0,0) + (7.557,6.338,-0.584)
           = (7.557,6.338,-0.584)
           → 推力在电机安装位置正确
```

**visual_pose 只影响视觉渲染，不影响物理计算。** 物理计算只看 link_pose，不看 visual/collision 的 pose。

#### PX4控制分配与Gazebo物理引擎的分工

飞艇的力矩计算由两套独立系统分别处理，它们必须一致：

| 系统 | 定义位置 | 参数 | 作用 |
|------|---------|------|------|
| PX4控制分配 | 2058_gz_lingyun01 | CA_ROTOR0~9 PX/PY/PZ | 告诉PX4: 电机相对重心(0,0,-1.5)的位置 (FRD) |
| PX4控制分配 | ActuatorEffectivenessCustom.cpp | CA_AS_K_LUP/LDN/PROP, CA_AS_PZ_PROP等 | 告诉PX4: 推力如何分配到各电机 |
| Gazebo物理 | model.sdf | rotor link pose | 告诉Gazebo: 电机在世界中的位置 |
| Gazebo物理 | model.sdf | inertial pose | 告诉Gazebo: 重心位置 |

- **PX4控制分配**计算"期望"：根据力矩需求，计算每个电机应该输出多少推力
- **Gazebo物理引擎**计算"实际"：根据电机推力和安装位置，计算真实的力和力矩
- **实际物理影响由Gazebo决定**，PX4只告诉电机"转多快"

如果两者不一致（如之前 rotor link pose=(0,0,0) 的 bug），PX4 以为力矩方向正确，但 Gazebo 中实际力矩方向错误，导致飞艇失控。

---

## 🔍 Link pose与SDFormat规范

### ⚠️ 重要：Link pose相对于模型帧

根据 SDFormat 官方规范（[Pose Frame Semantics](http://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/)）：

- **`<link><pose>` 是相对于模型帧（Model Frame）的偏移**
- **`<joint><pose>` 是相对于子 Link 帧的偏移**
- 模型帧可通过 `__model__` 保留名引用

### 📐 各元素pose参考帧汇总

| 元素 | `<pose>` 相对于 | 说明 |
|------|----------------|------|
| **`//model/pose`** | 世界帧 | 模型在世界中的位置 |
| **`//model/link/pose`** | **模型帧** | Link相对于模型帧的偏移 |
| **`//model/joint/pose`** | **子Link帧** | Joint相对于子Link帧的偏移 |
| **`//link/visual/pose`** | 所属Link帧 | Visual相对于Link的偏移 |
| **`//link/collision/pose`** | 所属Link帧 | Collision相对于Link的偏移 |

### ✅ 验证测试

**测试场景**：将`rotor_0`的link pose从`(7.557, 6.338, -0.584)`改为`(0, 0, 0)`（不修改visual pose）

**分析**：
- rotor_0的link pose相对于模型帧 -> 改为(0,0,0)意味着rotor_0的link帧原点在模型帧原点
- MulticopterMotorModel在rotor_0的link帧原点施加推力 -> 推力施加点偏离电机安装位置约9.9m
- 推力对重心产生巨大力矩，前部电机(X>0)和后部电机(X<0)力矩方向相反

**实际结果**：
- 推力施加在模型原点(0,0,0)，对重心(0,0,-1.5)产生巨大俯仰力矩
- 前部电机(rotor_0/1)与后部电机(rotor_2/3)力矩方向相反，飞艇姿态失控
- 飞艇"到处乱飞，绕大圈转"

**结论**：rotor的link pose必须设为电机安装位置，确保推力施加点在电机物理位置处，力臂正确！

### 📂 参考：标准多旋翼电机模型

标准多旋翼（如 iris、typhoon_h480）使用相同的配置模式：

```xml
<!-- rotor_0 (螺旋桨) -->
<link name='rotor_0'>
  <pose>0.35 -0.35 0.045 0 0 0</pose>  ← 相对于模型帧，电机安装位置
</link>
```

**关键特点**：
- rotor_0的link pose直接设为电机安装位置（相对于模型帧）
- Joint无显式pose，位于子Link坐标系原点
- 推力施加点 = link pose = 电机安装位置
- 符合SDFormat规范

**这证明了灵云01的配置是正确的！** ✅

---

## �� Link配置详解

### 1️⃣ Base_link（飞艇主体）

```xml
<link name='base_link'>
  <pose>0 0 0 0 0 0</pose>
  <inertial>
    <pose>0 0 -1.5 0 0 0</pose>
    <mass>2206</mass>
    <inertia>
      <ixx>44700.0</ixx><ixy>0</ixy><ixz>0</ixz>
      <iyy>112200.0</iyy><iyz>0</iyz>
      <izz>145500.0</izz>
    </inertia>
  </inertial>
  <gravity>true</gravity>
  <visual name='hull_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://.../meshes/V2_FLU/hull_all.STL</uri>
      </mesh>
    </geometry>
  </visual>
</link>
```

**参数说明**:

| 参数 | 值 | 说明 |
|------|-----|------|
| pose | (0,0,0,0,0,0) | Link在父坐标系中的位置和朝向 |
| inertial/pose | (0,0,-1.5) | 质心（重心）位置，Gazebo物理引擎在此计算动力学 |
| gravity | true | 受重力（重力与浮力由AirshipDynamics插件共同作用，净浮力为0） |
| mass | 2206 kg | 飞艇总质量 (仿真占位值) |
| inertia | (44700,112200,145500) | 惯性张量（IXX,IYY,IZZ, SolidWorks实测） |

---

### 2️⃣ 上升/下降电机配置（V2, 以rotor_0为例）

V2 中升力电机分两组：**上升电机（rotor_0~3，推力向上）+ 下降电机（rotor_4~5，推力向下）**。均采用**单Link结构**（新方案：无 tilt 舵机，直接连接 base_link），**推力方向由 motorConstant 符号控制**：
- motorConstant > 0：推力沿 link 局部 +Z 轴（向上，上升电机）
- motorConstant < 0：推力沿 link 局部 -Z 轴（向下，下降电机）

#### 2.1 螺旋桨Link（含 motorConstant 配置）

```xml
<!-- 上升电机 rotor_0 (左前) 示例 -->
<link name='rotor_0'>
  <pose>7.557 6.338 -0.584 0 0 0</pose>       <!-- 电机安装位置, 推力施加点 -->
  <gravity>false</gravity>
  <mass>1e-8</mass>
  <inertia>...</inertia>
  <visual name='rotor_0_visual'>
    <pose>-7.557 -6.338 0.584 0 0 0</pose>    <!-- 补偿link pose, 使STL回到相对主体中心位置 -->
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://.../meshes/V2_FLU/lift_up_lf.STL</uri>
      </mesh>
    </geometry>
  </visual>
  <collision name='rotor_0_collision'>
    <pose>-7.557 -6.338 0.584 0 0 0</pose>    <!-- 同visual, 碰撞体也在正确位置 -->
    <geometry>
      <cylinder>
        <radius>0.8</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
  </collision>
</link>
```

#### 2.2 升力电机 motorConstant 配置（V2: 上升0-3 + 下降4-5）

新方案通过 motorConstant 符号控制推力方向，无需 tilt 舵机翻转（T-MOTOR A10, 最大推力 222.5N）：

| 电机编号 | Link名称 | motorConstant | 推力方向 | 功能 |
|---------|---------|---------------|---------|------|
| 0 | rotor_0 (左前) | **+1.608e-03** | +Z（向上） | 上升电机 (ccw) |
| 1 | rotor_1 (右前) | **+1.608e-03** | +Z（向上） | 上升电机 (cw) |
| 2 | rotor_2 (左后) | **+1.608e-03** | +Z（向上） | 上升电机 (cw) |
| 3 | rotor_3 (右后) | **+1.608e-03** | +Z（向上） | 上升电机 (ccw) |
| 4 | rotor_4 (前) | **-1.608e-03** | -Z（向下） | 下降电机 (cw) |
| 5 | rotor_5 (后) | **-1.608e-03** | -Z（向下） | 下降电机 (ccw) |

**特点**:
- **Link pose**: (7.557, 6.338, -0.584) - **电机安装位置**，物理推力施加点（上升电机左前示例）
- **Visual/Collision pose**: (-7.557, -6.338, 0.584) - **反向补偿link pose**，使 STL 回到相对主体中心位置
- **质量**: 1e-8 kg（仅螺旋桨，虚拟质量）
- **碰撞体**: 简化为圆柱体（radius=0.8m, length=0.05m）
- **推力方向**: 由 motorConstant 符号控制，正值向上（上升0-3），负值向下（下降4-5）
- **maxRotVelocity**: 372.0（A10, 与 SIM_GZ_EC 的 DIS=372 对应）

**重要：为什么rotor的link pose必须设为电机安装位置？**

MulticopterMotorModel插件在rotor的link帧原点施加推力。如果rotor的link pose为(0,0,0)（模型原点），推力施加点偏离电机安装位置约9.9m，会产生巨大的俯仰力矩：
- 前部电机(X>0)：推力对重心产生抬头/低头力矩（取决于推力方向）
- 后部电机(X<0)：推力对重心产生相反方向的俯仰力矩

修复后rotor link pose = 电机安装位置，推力施加点与电机位置重合，力臂正确，俯仰力矩由控制分配器精确计算。

**抬头力矩抑制机制（新方案）**：

推进电机在重心下方0.878m，推力向前会产生抬头力矩。新方案通过**升力电机推力差动分配**抑制抬头力矩（不再使用 tilt 舵机）：
- 抬头时：前组上升电机(0,1)增强，后组上升电机(2,3)减弱（下降电机前后差动协同）
- 低头时：后组增强，前组减弱
- 控制分配器（ActuatorEffectivenessCustom）自动计算各电机推力分配

---

### 3️⃣ 推进电机配置（V2, 以rotor_6为例）

推进电机（rotor_6~9, 四角布局）采用**单Link结构**，STL文件使用 V2_FLU 预转换的相对主体中心坐标（FLU米制）

**与升力电机同样的问题**：推进电机rotor的link pose也必须设为安装位置（物理推力施加点），否则推力施加在模型原点(0,0,0)，会产生巨大的偏航和俯仰力矩，导致飞艇乱跑。

**推进电机推力方向原理**（方式A: rc_cessna风格）：

MulticopterMotorModel插件始终沿link的局部**+Z轴**施加推力。要使推进电机推力沿+X方向（向前），需要让rotor link的局部Z轴指向base_link的+X方向。方法是在link pose中绕Y轴旋转+90度。

```
base_link坐标系:        rotor_6 link坐标系(绕Y轴+90度后):
  Z(上)                    X(前, 原Z方向)
  |                        |
  |                        |
  +--- X(前)         Z(后, 原-X方向)
 /
Y(左)

推力方向 = rotor link的+Z轴 = base_link的+X方向(向前)
旋转轴 = rotor joint的axis(0,0,1) = rotor link的+Z轴 = base_link的+X方向(绕X轴旋转)
```

```xml
<link name='rotor_6'>
  <pose>2.230 5.888 -2.378 0 1.5708 0</pose>    <!-- 绕Y轴+90度, 使link Z轴指向前方(+X) -->
  <gravity>false</gravity>
  <mass>1e-8</mass>
  <inertial>...</inertial>
  <visual name='rotor_6_visual'>
    <pose>-2.378 -5.888 -2.230 0 -1.5708 0</pose>  <!-- 补偿link旋转, 使mesh位置正确 -->
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://.../meshes/V2_FLU/thrust_lf.STL</uri>
      </mesh>
    </geometry>
  </visual>
  <collision name='rotor_6_collision'>
    <pose>-2.378 -5.888 -2.230 0 -1.5708 0</pose>  <!-- 同visual -->
    <geometry>
      <cylinder>
        <radius>1.0</radius>
        <length>0.08</length>
      </cylinder>
    </geometry>
  </collision>
</link>
<joint name='rotor_6_joint' type='revolute'>
  <!-- 无pose旋转, 旋转已放在link pose中 -->
  <parent>base_link</parent>
  <child>rotor_6</child>
  <axis>
    <xyz>0 0 1</xyz>                             <!-- link局部Z轴, 经旋转后=base_link X轴 -->
    <limit>
      <lower>-1e+16</lower>
      <upper>1e+16</upper>
    </limit>
    <dynamics>
      <spring_reference>0</spring_reference>
      <spring_stiffness>0</spring_stiffness>      <!-- 必须为0! 之前5000导致电机卡死 -->
    </dynamics>
  </axis>
</joint>
```

**特点**:
- **Link pose**: (2.230, 5.888, -2.378, 0, 1.5708, 0) - 安装位置 + 绕Y轴+90度旋转，使link的Z轴指向+X方向
- **Visual/Collision pose**: (-2.378, -5.888, -2.230, 0, -1.5708, 0) - 补偿link旋转，使STL mesh在世界坐标系中位置正确
- **Joint pose**: 无旋转（旋转已放在link pose中）
- **Axis**: (0, 0, 1) - link局部Z轴，由于link旋转90度，实际旋转轴在base_link中是+X方向
- **spring_stiffness**: 必须为0，之前遗留值5000会导致电机被弹簧力卡死
- **motorConstant**: +8.677e-03（HOBBYWING P65M, 最大推力 1352.4N），maxRotVelocity=394.8
- **旋转方向**: 左侧(6,8) CW, 右侧(7,9) CCW（左右对转, 滚转反扭矩抵消）

**Visual/Collision pose补偿计算**：

当link pose添加了绕Y轴+90度旋转后，visual/collision的pose需要相应调整，确保STL mesh在世界坐标系中的位置不变。

```
补偿公式: visual_pose = R_y(-pi/2) * [原visual平移] + R_y(-pi/2)

以rotor_6为例:
  原visual平移 = [-2.230, -5.888, 2.378]
  R_y(-pi/2) * [-2.230, -5.888, 2.378] = [-2.378, -5.888, -2.230]
  再加上反向旋转: pitch = -1.5708
  最终: (-2.378, -5.888, -2.230, 0, -1.5708, 0)
```

**重要：为什么推进电机的link pose也必须设为安装位置？**

与升力电机同理，MulticopterMotorModel插件在rotor的link帧原点施加推力。如果link pose为(0,0,0)（模型原点），推力施加点偏离电机安装位置约6.7m，会产生巨大的偏航和俯仰力矩：
- 推力在模型原点施加，对重心产生偏航力矩（左右电机不对称）
- 推力在模型原点施加，对重心产生俯仰力矩（推进电机在重心下方0.878m）
- 结果：飞艇乱跑，绕大圈转

修复后rotor link pose = 安装位置，推力施加点与电机位置重合，力臂正确。

**为什么不用joint pose旋转或use_parent_model_frame？**

之前尝试过两种方案，均有问题：
1. **joint pose旋转**：SDF中joint的axis默认在child link frame中解析，joint pose旋转child link后axis方向也会改变，导致旋转轴方向不符合预期
2. **use_parent_model_frame**：Gazebo Harmonic对此属性支持不完善（SDF解析时会报警告"XML Element not defined in SDF"），实际行为可能不符合预期

方式A（rc_cessna风格）是PX4中最常用的推进电机配置方式，将旋转放在link pose中，axis用`0 0 1`，不依赖任何特殊属性，最可靠。

---

## 🔄 Joint配置详解

### 1️⃣ Joint通用参数

```xml
<joint name='xxx_joint' type='revolute'>
  <parent>父Link名称</parent>
  <child>子Link名称</child>
  <axis>
    <xyz>旋转轴方向</xyz>
    <limit>
      <lower>最小角度</lower>
      <upper>最大角度</upper>
    </limit>
    <dynamics>
      <friction>摩擦力</friction>
      <damping>阻尼系数</damping>
    </dynamics>
  </axis>
</joint>
```

**参数说明**:

| 参数 | 说明 | 典型值 |
|------|------|--------|
| name | Joint名称 | rotor_0_joint, rotor_6_joint |
| type | Joint类型 | revolute(旋转), prismatic(滑动), fixed(固定) |
| parent | 父Link | base_link |
| child | 子Link | rotor_0, rotor_6 |
| axis/xyz | 旋转轴方向 | (0,0,1)绕Z轴, (1,0,0)绕X轴 |
| limit/lower | 最小角度限制 | 弧度值 |
| limit/upper | 最大角度限制 | 弧度值 |
| dynamics/friction | 摩擦力 | N·m/(rad/s)ⁿ |
| dynamics/damping | 阻尼系数 | N·m·s/rad |

---

### 2️⃣ 上升/下降电机Joint（单关节结构）

新方案下上升/下降电机采用**单关节结构**：rotor_0~5 直接通过 rotor_joint 连接 base_link，**无 tilt 倾斜关节**。推力方向由 motorConstant 符号控制，无需机械翻转。

#### 2.1 旋转关节（rotor_0_joint）

```xml
<!-- 螺旋桨旋转关节: rotor_0 直接连接 base_link -->
<joint name='rotor_0_joint' type='revolute'>
  <parent>base_link</parent>
  <child>rotor_0</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1e+16</lower>
      <upper>1e+16</upper>
    </limit>
    <dynamics>
      <spring_reference>0</spring_reference>
      <spring_stiffness>0</spring_stiffness>
    </dynamics>
  </axis>
</joint>
```

**功能说明**:

| 参数 | 值 | 说明 |
|------|-----|------|
| **旋转轴** | (0,0,1) | 绕Z轴旋转（产生升力） |
| **角度范围** | 无限制 | 电机可以自由旋转 |
| **父Link** | base_link | 螺旋桨直接连接飞艇主体 |
| **推力方向** | motorConstant 符号控制 | 正值向上（上升），负值向下（下降） |

**新方案 vs 旧方案对比**：

| 特性 | 旧方案（tilt 舵机） | 新方案（motorConstant 符号） |
|------|---------------------|---------------------------|
| 关节数量 | 2个（tilt_joint + rotor_joint） | 1个（rotor_joint） |
| 推力方向控制 | tilt 舵机机械翻转（0°~180°） | motorConstant 符号（+/-） |
| 父子链 | base -> tilt_motor -> rotor | base -> rotor |
| 抬头力矩抑制 | tilt 舵机翻转推力方向 | 升力电机推力差动分配 |
| 机械复杂度 | 高（需舵机控制） | 低（固定推力方向） |

---

### 3️⃣ 推进电机Joint（单关节结构）

```xml
<!-- 推进电机旋转关节 -->
<joint name='rotor_6_joint' type='revolute'>
  <!-- 无pose旋转, 旋转已放在link pose中 -->
  <parent>base_link</parent>
  <child>rotor_6</child>
  <axis>
    <xyz>0 0 1</xyz>                             <!-- link局部Z轴, 经旋转后=base_link X轴 -->
    <limit>
      <lower>-1e+16</lower>
      <upper>1e+16</upper>
    </limit>
    <dynamics>
      <spring_reference>0</spring_reference>
      <spring_stiffness>0</spring_stiffness>
    </dynamics>
  </axis>
</joint>
```

**功能说明**:

| 参数 | 值 | 说明 |
|------|-----|------|
| **旋转轴** | (0,0,1) | link局部Z轴, 由于link绕Y轴旋转90度, 实际旋转轴=base_link X轴 |
| **角度范围** | 无限制 | 电机可以自由旋转 |
| **父Link** | base_link | 直接连接飞艇主体 |
| **Joint pose** | 无 | 旋转已放在link pose中, 不需要joint pose |

---

## 📊 上升/下降/推进电机 对比（V2）

| 特性 | 上升电机 (rotor_0~3) | 下降电机 (rotor_4~5) | 推进电机 (rotor_6~9) |
|------|---------------------|---------------------|---------------------|
| **Link数量** | 1个（仅螺旋桨） | 1个（仅螺旋桨） | 1个（仅螺旋桨） |
| **父子链** | base -> rotor | base -> rotor | base -> rotor |
| **控制功能** | 上升 + 俯仰差动 + 横滚差动 | 下降 + 俯仰差动 | 水平推进 + 偏航差动 |
| **倾斜关节** | 无（V2已移除 tilt 舵机） | 无 | 无 |
| **旋转轴** | rotor_joint绕Z轴(局部) | rotor_joint绕Z轴(局部) | rotor_joint绕Z轴(局部), link旋转后=base_link X轴 |
| **推力方向** | motorConstant符号（+向上） | motorConstant符号（-向下） | +X(向前, link旋转后) |
| **Link pose旋转** | 无 | 无 | 绕Y轴+90度 |
| **推力方向控制** | motorConstant +1.608e-03 | motorConstant -1.608e-03 | 固定（link pose旋转90度）+8.677e-03 |
| **典型应用** | 垂直上升、俯仰/横滚姿态控制 | 垂直下降、俯仰控制 | 水平推进、偏航控制 |

---

## 📋 电机布局位置表（V2, FLU坐标）

### 上升电机（四角布局, 推力向上）

| 电机编号 | Link名称 | motorConstant | FLU坐标中心(X,Y,Z) | 位置描述 |
|---------|---------|---------------|----------------------|---------|
| 0 | rotor_0 | +1.608e-03（向上） | (+7.557, +6.338, -0.584) | 左前 (ccw) |
| 1 | rotor_1 | +1.608e-03（向上） | (+7.557, -6.337, -0.584) | 右前 (cw) |
| 2 | rotor_2 | +1.608e-03（向上） | (-9.843, +5.814, -0.584) | 左后 (cw) |
| 3 | rotor_3 | +1.608e-03（向上） | (-9.843, -5.812, -0.584) | 右后 (ccw) |

**特点**:
- 四角布局: 前组(0,1) X=+7.557, 后组(2,3) X=-9.843
- 左右对称: 左组(0,2) Y=+6.3m, 右组(1,3) Y=-6.3m, 横滚力臂 PY≈6.34m
- **推力方向**: motorConstant 正值, 推力向上（上升）
- **横滚控制**: 左组(0,2) vs 右组(1,3) 差动产生横滚力矩
- STL使用FLU相对主体中心坐标，rotor link pose = 电机安装位置（物理正确），visual/collision pose = 负偏移

### 下降电机（中轴前后, 推力向下）

| 电机编号 | Link名称 | motorConstant | FLU坐标中心(X,Y,Z) | 位置描述 |
|---------|---------|---------------|----------------------|---------|
| 4 | rotor_4 | -1.608e-03（向下） | (+6.957, +0.017, +0.505) | 前 (cw) |
| 5 | rotor_5 | -1.608e-03（向下） | (-11.443, +0.017, +0.505) | 后 (ccw) |

**特点**:
- 中轴前后: 前(4) X=+6.957, 后(5) X=-11.443, Y≈0
- **推力方向**: motorConstant 负值, 推力向下（下降）
- 下降电机参与俯仰差动（前降低头/后降抬头）

### 推进电机（四角布局, 推力向前）

| 电机编号 | Link名称 | Link pose (x y z roll pitch yaw) | visual/collision pose | 位置描述 |
|---------|---------|--------------------------------|----------------------|---------|
| 6 | rotor_6 | (2.230, 5.888, -2.378, 0, 1.5708, 0) | (-2.378, -5.888, -2.230, 0, -1.5708, 0) | 左前 (cw) |
| 7 | rotor_7 | (2.230, -5.887, -2.378, 0, 1.5708, 0) | (-2.378, 5.887, -2.230, 0, -1.5708, 0) | 右前 (ccw) |
| 8 | rotor_8 | (-2.971, 5.888, -2.378, 0, 1.5708, 0) | (-2.378, -5.888, 2.971, 0, -1.5708, 0) | 左后 (cw) |
| 9 | rotor_9 | (-2.972, -5.887, -2.378, 0, 1.5708, 0) | (-2.378, 5.887, 2.972, 0, -1.5708, 0) | 右后 (ccw) |

**特点**:
- Z坐标统一: 全部在Z=-2.378m（重心(0,0,-1.5)下方约0.878m）
- 左右对称: 左(Y=+5.888m)右(Y=-5.887m), 左右对转反扭矩抵消
- 前后: 前(X=+2.230m)后(X=-2.971/-2.972m)
- Link pose统一绕Y轴+90度: 使link局部Z轴指向base_link +X方向(推力向前)
- visual/collision pose补偿link旋转: 使STL mesh在世界坐标系中位置正确
- 推力方向: 始终沿+X方向（向前），固定不变

---

## 🎯 Joint位置定义规则

### 📐 规则说明

在Gazebo SDF中，**Joint的位置由子Link的pose决定**：

```xml
<joint name='rotor_0_joint'>
  <!-- 没有<pose>元素 -->
  <parent>base_link</parent>
  <child>rotor_0</child>
</joint>

<link name='rotor_0'>
  <pose>7.557 6.338 -0.584 0 0 0</pose>
</link>

<!-- Joint位置 = 子Link的pose前3个参数 -->
<!-- Joint位置 = (7.557, 6.338, -0.584) -->
```

### ✅ 隐式定义（当前配置）

```xml
<joint>
  <parent>base_link</parent>
  <child>rotor_0</child>
</joint>
```

- Joint位置由子Link的pose决定
- 简洁，推荐使用

### ✅ 显式定义

```xml
<joint>
  <pose>7.557 6.338 -0.584 0 0 0</pose>
  <parent>base_link</parent>
  <child>rotor_0</child>
</joint>
```

- Joint和Link位置独立定义
- 可以不同，但需要保持一致性

---

## 🔧 配置示例：推进电机

### 当前配置（方式A: rc_cessna风格, V2 推进电机）

推进电机（rotor_6~9）STL文件使用 V2_FLU 预转换的相对主体中心坐标（FLU米制），link pose必须设为安装位置：

```xml
<link name='rotor_6'>
  <pose>2.230 5.888 -2.378 0 1.5708 0</pose>    <!-- 绕Y轴+90度, 使link Z轴指向前方(+X) -->
  <visual name='rotor_6_visual'>
    <pose>-2.378 -5.888 -2.230 0 -1.5708 0</pose>  <!-- 补偿link旋转, 使mesh位置正确 -->
  </visual>
  <collision name='rotor_6_collision'>
    <pose>-2.378 -5.888 -2.230 0 -1.5708 0</pose>
  </collision>
</link>

<joint name='rotor_6_joint' type='revolute'>
  <!-- 无pose旋转, 旋转已放在link pose中 -->
  <parent>base_link</parent>
  <child>rotor_6</child>
  <axis>
    <xyz>0 0 1</xyz>                             <!-- link局部Z轴, 经旋转后=base_link X轴 -->
    <dynamics>
      <spring_stiffness>0</spring_stiffness>      <!-- 必须为0! -->
    </dynamics>
  </axis>
</joint>
```

**配置说明**:
- Link pose = 安装位置 + 绕Y轴+90度旋转，确保推力沿+X方向（向前）
- Visual/Collision pose = 补偿link旋转，使STL mesh在世界坐标系中位置正确
- Joint无pose旋转（旋转已放在link pose中）
- Axis = (0,0,1)，link局部Z轴，经旋转后实际旋转轴在base_link中是+X方向
- spring_stiffness必须为0，之前遗留值5000导致电机被弹簧力卡死

---

## 📝 备注

### 坐标系说明

- 本模型使用PX4 FLU标准坐标系
- X轴指向前方，Y轴指向左侧，Z轴指向上方
- 使用右手定则

### 质量分布

- base_link质量很大(2206kg, 仿真占位值;设计起飞重量仍在核算中, 暂以此值保持中性浮力)，代表飞艇气囊
- rotor质量很小(1e-8kg)，仅代表螺旋桨（V2方案：无 tilt_motor 支架）

### 引力设置

- **base_link 的 gravity=true**：受重力作用，由 AirshipDynamics 浮力插件施加浮力与之平衡（净浮力=0）
- **所有电机/风机/阀门 Link 的 gravity=false**：仅可视化占位，不参与重力计算
- 飞艇依靠气囊浮力悬浮，而非螺旋桨升力平衡重力

### STL文件说明

- **V2 所有 STL 均使用 FLU 相对主体中心坐标**（`meshes/V2_FLU/`，由 SW 毫米坐标经 `convert_stl_to_flu.py` 预转换，主体中心在原点）
- **上升电机(rotor_0~3)**：link pose = 电机安装位置（物理正确），visual/collision pose = 负偏移（补偿STL相对主体中心），推力方向由 motorConstant 正号控制（+1.608e-03，向上）
- **下降电机(rotor_4~5)**：同上升电机，motorConstant 负号控制（-1.608e-03，向下）
- **推进电机(rotor_6~9)**：link pose = 安装位置（物理推力点，绕Y轴+90度），visual/collision pose = 负偏移（补偿link旋转）
- **所有电机的rotor link pose都必须设为物理安装位置**，否则推力施加在模型原点(0,0,0)，产生巨大的错误力矩

### 推力方向与力矩分析

V2 方案下升力类电机推力方向**固定**，由 motorConstant 符号控制（无 tilt 舵机翻转）：

| 电机类型 | 推力方向 | 俯仰力矩 | 偏航力矩 | 推力方向控制方式 |
|---------|---------|---------|---------|-------------------|
| 上升电机 0-3 (motorConstant>0) | +Z（向上，上升） | 前组(0,1)增大->抬头, 后组(2,3)增大->低头 | 无（对角反扭矩抵消） | motorConstant 正值，固定向上 |
| 下降电机 4-5 (motorConstant<0) | -Z（向下，下降） | 前(4)增大->低头, 后(5)增大->抬头 | 无（前后反扭矩抵消） | motorConstant 负值，固定向下 |
| 推进电机 6-9 | +X(向前) | 向前推->抬头(电机在重心下方0.878m) | 左侧推->左转, 右侧推->右转 | link pose绕Y轴+90度，固定向前 |

**关键结论**：
- 上升/下降电机推力方向**固定**，由 motorConstant 符号控制（正值向上，负值向下），无需 tilt 舵机翻转
- 俯仰控制通过升力类电机推力差动分配实现：前组(0,1)增强抬头，后组(2,3)增强低头（下降电机4-5前后差动协同）
- 横滚控制通过上升电机左右差动实现：左组(0,2) vs 右组(1,3)，力臂 PY≈6.34m（气囊不参与横滚）
- 推进电机推力方向始终水平向前，推进-俯仰耦合方向固定不变（由 ActuatorEffectivenessCustom 补偿）
- 抬头力矩抑制通过升力电机推力差动分配（不再使用 tilt 舵机）

---

## 🎈 统一浮力调节系统 (四气囊同步充放气)

### 架构变更说明

灵云01飞艇已**取消四气囊横滚控制**。4个空气囊不再做左右差动横滚调节,而是**完全同步充放气**,用于调节飞艇高度:
- 充气 = 4气囊同步增重 = 下沉
- 放气 = 4气囊同步减重 = 上升

横滚控制相关的 BALLOON_R_* / TRIM_BALLOON_* 参数、ballast_setpoint 的 roll_moment_demand 字段均已删除。

### 原理

四气囊总空气质量作为可变载荷影响垂直浮力,从而调节高度:

```
F_net = buoyancy - (m_base + m_ballast_total) × g
```

- 浮力(向上)固定, 由氦气囊体积决定
- 4气囊总质量 m_ballast_total 增大 → 净浮力减小 → 下沉
- 4气囊总质量 m_ballast_total 减小 → 净浮力增大 → 上升

### 四气囊布局（V2）

| 索引 | 名称 | Y坐标(m) | 相对重心Y(m) | 位置 |
|------|------|---------|-------------|------|
| 0 | LI (左内主囊) | +3.3 | +3.3 | 左侧内侧 |
| 1 | LO (左外副囊) | +8.6 | +8.6 | 左侧外侧 |
| 2 | RI (右内主囊) | -3.3 | -3.3 | 右侧内侧 |
| 3 | RO (右外副囊) | -8.6 | -8.6 | 右侧外侧 |

**重心Y坐标**: 0m (inertial pose (0,0,-1.5), V2)

> 注: 四气囊不再按左右差动分配质量,而是完全同步充放气(共用同一质量)。上表仅为物理安装位置参考。
> V1历史: 旧气囊布局(基于旧重心Y=-2.894)的数值已废弃。

### 空气囊参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 单囊体积 | 100 m³ | 空气囊体积 |
| 最大表压 | 5 kPa | 结构极限 |
| 最大空气质量 | 128.5 kg | 5kPa表压下 |
| 风机流量 | 0.102 kg/s | 充气/抽气同款 |
| 执行器 | 4囊同步 | 每囊1风机+1阀门, 共4风机+4阀门 |

### 数据流架构

ballast_control 订阅 airship_altitude_setpoint 目标高度,经统一浮力PID驱动四气囊同步充放气:

```
airship_att_control (发布目标高度)
    │
    │  airship_altitude_setpoint (uORB)
    │
    ▼
ballast_control (统一浮力PID: 高度误差 -> 净浮力需求 -> 充放气命令)
    │
    │  ballast_setpoint (uORB, 唯一发布者)
    │  包含: net_buoyancy, ballast_mass[4], 四气囊同步执行器命令
    │
    ▼
GZMixingInterfaceBallast (gz_bridge)
    │
    │  /model/lingyun01/ballast_cmd      (x=net_buoyancy, y 已废弃)
    │  /model/lingyun01/ballast_actuator (x=索引, y=状态位图, z=质量)
    │
    ▼
AirshipDynamics (Gazebo插件)
    │
    │  四气囊总质量作为可变载荷: buoyancy += total_ballast_mass × gravity (向下)
    │  (已删除横滚力矩计算)
    │
    ▼
Gazebo物理引擎 (应用垂直力到base_link)
```

### 执行器状态位图

每个空气囊的执行器状态编码为2位位图（V2: 每囊1风机+1阀门）,4囊完全同步动作:

| Bit | 执行器 | 功能 |
|-----|--------|------|
| 0 | blower | 充气风机 (增重, 配合阀门鼓风) |
| 1 | valve | 排气阀门 (开=自然排气减重) |

**互锁保护**: 充气和排气禁止同时进行 (代码实现)
**同步策略**: 4个气囊执行同一套充放气命令(完全相同), 不做左右差动

### AirshipDynamics 插件 SDF 参数

AirshipDynamics 插件支持以下浮力调节相关 SDF 参数 (可在 model.sdf 中配置):

| SDF元素 | 默认值 | 说明 |
|---------|--------|------|
| `net_buoyancy` | 0 | 额外浮力微调 (N) |
| `ballast_mass_max` | 128.5 | 单气囊最大质量 (kg) |

> 注: 旧横滚参数 roll_control_enabled / arm_inner / arm_outer 已随横滚控制移除。

### PX4 参数配置

**ballast_control 模块 (BALLOON_前缀, 统一浮力PID)**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| BALLOON_AST_EN | 1 | 启用浮力控制 |
| BALLOON_DEADZONE | 0.5 | 高度死区 (m) |
| BALLOON_THRSHLD | 2.0 | 鼓风机/阀门切换阈值 (m) |
| BALLOON_P_GAIN | 0.8 | 高度PID比例增益 |
| BALLOON_I_GAIN | 0.05 | 高度PID积分增益 |
| BALLOON_D_GAIN | 0.2 | 高度PID微分增益 |
| BALLOON_I_MAX | 50.0 | 高度PID积分限幅 |
| BALLOON_RATE_MAX | 20.0 | 浮力调节最大速率 |
| BALLOON_M_MAX | 128.5 | 单气囊最大空气质量 (kg) |
| BLWR_FLOW | 0.102 | 风机空气质量流量 (kg/s) |
| BLOWER_TAU | 10.0 | 鼓风机时间常数 (s) |
| VALVE_OPEN_DELAY | 0.5 | 阀门开启延迟 (s) |

> 注: 横滚控制参数(BALLOON_R_*)与配平参数(TRIM_BALLOON_*)已删除。

### 质量分配策略

四气囊**完全同步**充放气,不做左右差动分配:

```
充气(下沉): Δm_LI = Δm_LO = Δm_RI = Δm_RO = +Δm   (4囊同步增重)
放气(上升): Δm_LI = Δm_LO = Δm_RI = Δm_RO = -Δm   (4囊同步减重)
```

4囊总质量变化 = 4 × Δm, 作为垂直浮力调节的可变载荷。

---

## 🔗 相关文档

- [灵云01主体STL分析报告](./meshes/source_backup/原始STL备份.md)
- [Gazebo SDF规范文档](http://sdformat.org/spec)

---

**维护者**: 灵云01项目组

**版本历史**:
- v2.4 (2026-08-18):
  - **清理文档中V1旧数值, 全部更新为V2 10电机新布局**
  - 重心从旧值(-0.012,-2.894,-0.009)更新为V2的(0,0,-1.5)
  - 电机布局从V1(升力0-3 X轴线性 + 推进4-7)更新为V2(上升0-3四角 + 下降4-5中轴 + 推进6-9四角)
  - 电机坐标/motorConstant更新为V2实际值(+1.608e-03/-1.608e-03/+8.677e-03)
  - Link数量从9个更新为19个(V2: 含下降电机与风机/阀门占位)
  - STL说明更新为 V2_FLU 预转换(FLU相对主体中心), 替换旧的SW绝对坐标描述
  - 推进电机示例从rotor_4更新为rotor_6, 耦合力臂从1.5m更新为0.878m(CA_AS_PZ_PROP)
  - 四气囊布局更新为V2(主囊±3.3, 副囊±8.6), 执行器位图更新为每囊1风机1阀门
  - 保留全部V2演进说明(取消四气囊横滚、移除tilt舵机)
- v2.3 (2026-08-12):
  - **重大架构变更: 取消四气囊横滚控制, 改为统一浮力调节**
  - 四气囊不再做左右差动横滚调节, 改为完全同步充放气用于调节高度
  - 充气=4囊同步增重=下沉; 放气=4囊同步减重=上升
  - 删除横滚力矩计算公式与 roll_disturbance_cmd 测试接口描述
  - 数据流变更: airship_att_control 发布目标高度(airship_altitude_setpoint) -> ballast_control 统一浮力PID -> 四气囊同步充放气
  - AirshipDynamics 插件删除横滚力矩计算, 改为四气囊总质量影响垂直浮力
  - 删除横滚相关参数 (BALLOON_R_*, TRIM_BALLOON_*, AS_ROLL_*, roll_control_enabled, arm_inner, arm_outer)
  - 执行器状态位图更新为四气囊同步充放气描述
- v2.2 (2026-07-14):
  - 横滚控制数据流架构变更: ballast_control 独立级联PID, 不再订阅 att_control 的 roll_moment_demand
  - att_control 删除 publishBallastSetpoint, torque_x 置0 (横滚不通过电机控制)
  - 修复自订阅回环问题: 两个模块同时发布 ballast_setpoint 导致 _roll_moment_demand 不稳定
  - 数据流变更: vehicle_attitude/angular_velocity -> ballast_control(独立PID) -> gz_bridge -> AirshipDynamics
  - 仿真测试验证: 200N*m 扰动下横滚角从6.7度修正到1.1度, rolDmd 非零(-0.040~-0.020)
- v2.1 (2026-07-13):
  - 新增"横滚控制系统 (四气囊空气囊)"章节
  - 描述四气囊空气质量差产生横滚力矩的原理
  - 四气囊布局和力臂参数表
  - 数据流架构: att_control → ballast_control → gz_bridge → AirshipDynamics
  - 执行器状态位图编码说明
  - AirshipDynamics 插件 SDF 参数 (roll_control_enabled, arm_inner, arm_outer, ballast_mass_max)
  - PX4 参数配置 (AS_ROLL_*, BALLOON_R_*, BLWR_FLOW, VALVE_HYST, VALVE_MIN_T)
  - 质量分配策略 (外囊70%, 内囊30%, 反对称分配)
- v2.0 (2026-06-25):
  - **重大更新：移除 tilt 舵机，改为无舵机新方案**
  - 升力电机从双Link结构（tilt_motor支架 + rotor螺旋桨）改为单Link结构（仅rotor螺旋桨）
  - 移除所有 tilt_motor_0~3、tilt_0_joint~tilt_3_joint、tilt_angle 相关配置
  - 推力方向控制从"tilt 舵机机械翻转（0°~180°）"改为"motorConstant 符号控制"
  - M0/M3: motorConstant=+1.416e-03（正值，推力向上，上升，20kg级）
  - M1/M2: motorConstant=-2.832e-03（负值，推力向下，下降，40kg级）
  - 抬头力矩抑制从"tilt 舵机翻转推力方向"改为"升力电机推力差动分配"
  - rotor_0~3 直接连接 base_link，父子链从 base->tilt->rotor 简化为 base->rotor
  - Link数量从10个减少为9个（移除4个 tilt_motor 支架）
  - 更新所有 tilt_motor/tilt_joint 示例为 rotor/rotor_joint 示例
  - 更新升力电机vs推进电机对比表为新方案
  - 更新推力方向与力矩分析表为固定推力方向
  - 更新验证测试为推力施加点偏移示例
  - 更新参考模型从 tiltrotor 改为标准多旋翼电机模型
- v1.6 (2026-06-08):
  - 推进电机配置改为方式A(rc_cessna风格): 旋转放在link pose中, axis用0 0 1
  - 修复推进电机旋转轴问题: 之前joint pose旋转+use_parent_model_frame方案在Gazebo Harmonic中不可靠
  - 更新visual/collision pose补偿计算: link旋转后需要R_y(-pi/2)变换
  - 新增"推进电机推力方向原理"说明和坐标系变换图
  - 新增"为什么不用joint pose旋转或use_parent_model_frame"说明
- v1.5 (2026-06-05):
  - 修复推进电机rotor_4-7的link pose: 从(0,0,0)改为安装位置+visual负偏移补偿
  - 修复rotor_4_joint的spring_stiffness: 从5000改为0（之前导致电机卡死）
  - 新增推力方向与力矩分析表：升力电机翻转后俯仰力矩反转，推进电机方向固定
  - 更新推进电机配置示例和位置表
- v1.4 (2026-06-05):
  - 修复tilt_2/3翻转根因：rotor link pose必须与tilt_motor相同，确保推力施加点力臂=0
  - 更新升力电机配置：rotor visual/collision pose添加负偏移补偿STL绝对坐标
  - 更新协作关系图：添加推力施加点和STL补偿说明
  - 更新验证测试：rotor pose=(0,0,0)导致后部电机翻转的实测结果
  - 更新STL文件说明：区分升力电机和推进电机的pose配置差异
- v1.2 (2026-06-03):
  - 修正pose理解：Link pose相对于模型帧，Joint pose相对于子Link帧
  - 新增"各元素pose参考帧汇总"表格
  - 引用SDFormat官方规范作为依据
  - 修正"Link pose与Gazebo行为"为"Link pose与SDFormat规范"
  - 修正协作关系图中的描述
- v1.1 (2026-06-03):
  - 新增"Link pose与Gazebo行为"章节
  - 添加Link pose是世界坐标的重要发现
  - 添加验证测试说明（rotor_0 pose改为0,0,0后掉下来）
  - 添加tiltrotor矢量电机模型参考
  - 修复标题格式问题
- v1.0 (2024-06-02): 初始版本
