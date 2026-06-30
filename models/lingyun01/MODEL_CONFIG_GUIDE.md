# 灵云01飞艇 - Gazebo模型配置详解

**文件路径**: `/home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/model.sdf`

**最后更新**: 2026年6月25日

**版本**: v2.0

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
<!-- 以灵云01为例 -->
<link name='rotor_0'>
  <pose>11.225 -2.894 0.331 0 0 0</pose>
</link>
```

这意味着：**rotor_0 的坐标系原点位于模型帧 (11.225, -2.894, 0.331) 处**。

**注意**：由于灵云01的模型帧（base_link原点）恰好在世界原点 (0,0,0)，所以 Link pose 的值**看起来**等于世界坐标，但本质上是相对于模型帧的偏移。

#### 🎨 **Visual / Collision 的 <pose> 含义**

`<visual>` 和 `<collision>` 的 `<pose>` 是**相对于其所属 Link 坐标系的偏移**。

```xml
<link name='rotor_4'>
  <pose>2.547 2.387 -1.512 0 1.5708 0</pose>       ← 相对于模型帧

  <visual name='rotor_4_visual'>
    <pose>-1.512 -2.387 -2.547 0 -1.5708 0</pose>  ← 相对于Link坐标系
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
即 `rotor_0_joint` 的位置 = `rotor_0` 的坐标系原点 = 模型帧 + rotor_0 的 pose = `(11.225, -2.894, 0.331)`。

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

以升力电机 rotor_0 为例，完整展示 Link 和 Joint 如何配合工作（**新方案：单 Link 结构，无 tilt 舵机**）：

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
│      └── rotor_0: pose=(11.225, -2.894, 0.331)               │
│              │  相对于模型帧 = (11.225, -2.894, 0.331) OK     │
│              │  推力方向: motorConstant=1.416e-03 (正值, 向上)│
│              │                                              │
│              └── visual: pose=(-11.225, 2.894, -0.331)       │
│                 STL最终位置 = link_pose + visual_pose        │
│                   + STL顶点 = STL顶点(绝对坐标) OK           │
└─────────────────────────────────────────────────────────────┘
```

**关键结论**：
- `rotor_0` 的 `<pose>` 值为 (11.225, -2.894, 0.331)，相对于模型帧
- 由于模型帧在世界原点，rotor_0 的世界坐标即 (11.225, -2.894, 0.331)
- Joint 无显式 pose，位于子 Link 的坐标系原点
- rotor_0 绕自身 Z 轴旋转产生升力，**推力方向由 motorConstant 符号控制**（正值向上，负值向下）
- **推力施加点**在 rotor_0 的 link 帧原点，即电机安装位置，力臂正确

**STL绝对坐标补偿原理**：

由于升力电机 STL 使用 SolidWorks 绝对坐标，而 rotor_0 的 link pose 必须设为电机安装位置（物理正确），需要通过 visual/collision 的 pose 做反向补偿：

```
视觉最终位置 = link_pose + visual_pose + STL顶点坐标
            = (11.225,-2.894,0.331) + (-11.225,2.894,-0.331) + STL顶点
            = STL顶点(绝对坐标)  --> 视觉位置正确
```

| 元素 | pose值 | 说明 |
|------|--------|------|
| rotor_0 link | (11.225, -2.894, 0.331) | 物理推力施加点，电机安装位置 |
| rotor_0 visual | (-11.225, 2.894, -0.331) | 补偿link pose偏移，使STL回到绝对坐标位置 |
| rotor_0 collision | (-11.225, 2.894, -0.331) | 同visual，碰撞体也在正确位置 |

---

## 🏗️ 模型整体结构

### 📊 Link组件列表

灵云01飞艇模型由**9个Link**组成（新方案：无 tilt_motor 支架，rotor_0~3 直接连接 base_link）：

| Link名称 | 类型 | 功能 | 质量(kg) |
|---------|------|------|---------|
| **base_link** | 主体 | 飞艇气囊主体 | 2206 (仿真占位值) |
| **rotor_0~3** | 升力螺旋桨 | 产生升力（推力方向由 motorConstant 符号控制） | 0.001 |
| **rotor_4~7** | 推进螺旋桨 | 产生推力 | 0.001 |

### 🔗 Link父子关系树状图

```
base_link (飞艇主体, 质量: 2206kg 仿真占位值)
│
├── rotor_0 (升力螺旋桨0, 质量: 0.001kg, motorConstant=+1.416e-03, 推力向上)
│   └── lingyun01_lift_motor_front1.STL
│
├── rotor_1 (升力螺旋桨1, 质量: 0.001kg, motorConstant=-2.832e-03, 推力向下)
│   └── lingyun01_lift_motor_front2.STL
│
├── rotor_2 (升力螺旋桨2, 质量: 0.001kg, motorConstant=-2.832e-03, 推力向下)
│   └── lingyun01_lift_motor_back1.STL
│
├── rotor_3 (升力螺旋桨3, 质量: 0.001kg, motorConstant=+1.416e-03, 推力向上)
│   └── lingyun01_lift_motor_back2.STL
│
├── rotor_4 (推进螺旋桨0, 质量: 0.001kg)
│   └── lingyun01_thrust_motor_LF.STL
│
├── rotor_5 (推进螺旋桨1, 质量: 0.001kg)
│   └── lingyun01_thrust_motor_LB.STL
│
├── rotor_6 (推进螺旋桨2, 质量: 0.001kg)
│   └── lingyun01_thrust_motor_RF.STL
│
└── rotor_7 (推进螺旋桨3, 质量: 0.001kg)
    └── lingyun01_thrust_motor_RB.STL
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

**C 点坐标**: `(-0.012, -2.894, -0.009)`

#### C 点的计算来源

C 点是 hull_all.STL（飞艇主体）的**边界框中心**：

```
C = ((X_min + X_max) / 2,  (Y_min + Y_max) / 2,  (Z_min + Z_max) / 2)
```

| 轴 | STL最小值 | STL最大值 | 中心点 |
|----|----------|----------|--------|
| X | -16.998 | +16.767 | **-0.116** (约 -0.012) |
| Y | -13.110 | +7.310 | **-2.900** (约 -2.894) |
| Z | -3.737 | +4.140 | **+0.202** (约 -0.009) |

> **注意**: Y轴严重不对称（-13.1 到 +7.3），因为结构工程师在SolidWorks中画图时，原点不在飞艇几何中心，而是偏向Y正方向（左侧）。

#### C 点在项目中的使用位置

| 文件 | 配置项 | 值 | 说明 |
|------|--------|-----|------|
| `model.sdf` | `base_link/inertial/pose` | `-0.012 -2.894 -0.009` | 质心位置（Gazebo物理引擎在此计算动力学） |
| `model.sdf` | `com_visual/pose` | `-0.012 -2.894 -0.009` | 重心可视化球体位置 |
| `model.sdf` | `buoyancy_offset` | `-0.012 -2.894 -0.009` | 浮力偏移（AirshipDynamics插件用作comOffset） |
| `model.sdf` | `buoyancy_center` | `-0.012 -2.894 -0.009` | 浮力中心（与质心重合，消除初始力矩） |
| `2058_gz_lingyun01` | CA_ROTOR0~7 PX/PY/PZ | `STL绝对坐标 - C` | 电机位置相对于C点的偏移 |
| `ActuatorEffectivenessCustom.cpp` | `PZ_PROP` 等 | 基于 C 计算 | 推进-俯仰耦合补偿常量 |

#### 为什么质心和浮心都设为 C 点？

将质心（inertial pose）和浮心（buoyancy_center）都设为 C 点，确保：
1. **浮力与重力共线** → 不产生初始力矩 → 飞艇不会在启动时旋转
2. **CA_ROTOR 参数不需要额外偏移** → 电机位置 = STL绝对坐标 - C
3. **控制分配器参数一致** → PZ_PROP 等常量与 CA_ROTOR 基于同一参考点

> **物理精度说明**: C 点是边界框中心而非真正的体积中心（COV ≈ (-0.989, -2.884, -0.379)），但对中性浮力飞艇影响极小，浮力恢复力矩会自动修正微小偏差。

### 🔗 坐标系统一性原理

灵云01模型的核心设计原则：**SW坐标系 = base_link坐标系 = 模型帧**，三者统一。

#### 为什么必须统一？

只有 base_link 原点 = SW 原点（即 base_link pose = (0,0,0)），电机的 link pose 才能直接使用 SW 绝对坐标，安装到飞艇外皮上 SW 设计的安装位置。推力施加点也在那个位置。

```
SW坐标系 = base_link坐标系 = 模型帧
    |
    |  电机link pose = SW绝对坐标
    |  → 安装位置正确
    |  → 推力施加点 = 电机安装位置
    |
    |  STL绝对坐标 + visual反向pose = 视觉归位
    |  → 渲染位置正确
```

#### inertial pose（重心）不改变坐标系

base_link 的 `inertial pose = (-0.012, -2.894, -0.009)` 只是告诉 Gazebo 物理引擎"重心在哪"，**不改变坐标系**。所有 link pose 仍然基于模型帧（= SW坐标系）计算。

```
坐标系: base_link原点 = SW原点 = (0,0,0)        ← 不变
重心:   inertial_pose = (-0.012, -2.894, -0.009) ← 只是物理属性
电机0:  link_pose = (11.225, -2.894, 0.331)      ← 基于模型帧(SW坐标系)
```

Gazebo 计算力矩时：`力矩 = (推力作用点 - 重心) × 推力方向`，自动使用 inertial_pose 作为参考点。

#### visual/collision 反向补偿的完整公式

由于电机 STL 使用 SW 绝对坐标，而 rotor 的 link pose 必须设为安装位置（物理正确），需要通过 visual/collision 的 pose 做反向补偿：

```
视觉最终位置 = 模型帧 + link_pose + visual_pose + STL顶点坐标
            = (0,0,0) + (11.225,-2.894,0.331) + (-11.225,2.894,-0.331) + STL顶点
            = STL顶点(SW绝对坐标)
            → 视觉归位正确

物理推力点 = 模型帧 + link_pose
           = (0,0,0) + (11.225,-2.894,0.331)
           = (11.225,-2.894,0.331)
           → 推力在电机安装位置正确
```

**visual_pose 只影响视觉渲染，不影响物理计算。** 物理计算只看 link_pose，不看 visual/collision 的 pose。

#### PX4控制分配与Gazebo物理引擎的分工

飞艇的力矩计算由两套独立系统分别处理，它们必须一致：

| 系统 | 定义位置 | 参数 | 作用 |
|------|---------|------|------|
| PX4控制分配 | 2058_gz_lingyun01 | CA_ROTOR0~7 PX/PY/PZ | 告诉PX4: 电机相对重心C的位置 |
| PX4控制分配 | ActuatorEffectivenessCustom.cpp | K_LIFT, PZ_PROP等 | 告诉PX4: 推力如何分配到各电机 |
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

**测试场景**：将`rotor_0`的link pose从`(11.225, -2.894, 0.331)`改为`(0, 0, 0)`（不修改visual pose）

**分析**：
- rotor_0的link pose相对于模型帧 -> 改为(0,0,0)意味着rotor_0的link帧原点在模型帧原点
- MulticopterMotorModel在rotor_0的link帧原点施加推力 -> 推力施加点偏离电机安装位置约11.5m
- 推力对重心产生巨大力矩，前部电机(X>0)和后部电机(X<0)力矩方向相反

**实际结果**：
- 推力施加在模型原点(0,0,0)，对重心C(-0.012,-2.894,-0.009)产生巨大俯仰力矩
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
  <gravity>false</gravity>
  <mass>2206</mass>
  <inertia>
    <ixx>1000</ixx><ixy>0</ixy><ixz>0</ixz>
    <iyy>1000</iyy><iyz>0</iyz>
    <izz>2000</izz>
  </inertia>
  <visual name='base_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://.../lingyun01_hull_all.STL</uri>
      </mesh>
    </geometry>
  </visual>
</link>
```

**参数说明**:

| 参数 | 值 | 说明 |
|------|-----|------|
| pose | (0,0,0,0,0,0) | Link在父坐标系中的位置和朝向 |
| gravity | false | 不受重力影响（飞艇靠气囊浮力） |
| mass | 2206 kg | 飞艇总质量 (仿真占位值) |
| inertia | (1000,1000,2000) | 惯性张量（IXX,IYY,IZZ） |

---

### 2️⃣ 升力电机配置（以rotor_0为例）

升力电机采用**单Link结构**（新方案：无 tilt 舵机，rotor_0~3 直接连接 base_link），**推力方向由 motorConstant 符号控制**：
- motorConstant > 0：推力沿 link 局部 +Z 轴（向上，上升）
- motorConstant < 0：推力沿 link 局部 -Z 轴（向下，下降）

#### 2.1 螺旋桨Link（含 motorConstant 配置）

```xml
<link name='rotor_0'>
  <pose>11.225 -2.894 0.331 0 0 0</pose>       <!-- 电机安装位置, 推力施加点 -->
  <gravity>false</gravity>
  <mass>1e-8</mass>
  <inertia>...</inertia>
  <visual name='rotor_0_visual'>
    <pose>-11.225 2.894 -0.331 0 0 0</pose>    <!-- 补偿link pose, 使STL回到绝对坐标位置 -->
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://.../lingyun01_lift_motor_front1.STL</uri>
      </mesh>
    </geometry>
  </visual>
  <collision name='rotor_0_collision'>
    <pose>-11.225 2.894 -0.331 0 0 0</pose>    <!-- 同visual, 碰撞体也在正确位置 -->
    <geometry>
      <cylinder>
        <radius>0.8</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
  </collision>
</link>
```

#### 2.2 升力电机 motorConstant 配置（推力方向控制）

新方案通过 motorConstant 符号控制推力方向，无需 tilt 舵机翻转：

| 电机编号 | Link名称 | motorConstant | 推力方向 | 功能 |
|---------|---------|---------------|---------|------|
| 0 | rotor_0 | **+1.416e-03** | +Z（向上） | 上升（20kg级） |
| 1 | rotor_1 | **-2.832e-03** | -Z（向下） | 下降（40kg级） |
| 2 | rotor_2 | **-2.832e-03** | -Z（向下） | 下降（40kg级） |
| 3 | rotor_3 | **+1.416e-03** | +Z（向上） | 上升（20kg级） |

**特点**:
- **Link pose**: (11.225, -2.894, 0.331) - **电机安装位置**，物理推力施加点
- **Visual/Collision pose**: (-11.225, 2.894, -0.331) - **反向补偿link pose**，使STL绝对坐标顶点回到正确位置
- **质量**: 1e-8 kg（仅螺旋桨，虚拟质量）
- **碰撞体**: 简化为圆柱体（radius=0.8m, length=0.05m）
- **推力方向**: 由 motorConstant 符号控制，正值向上（上升），负值向下（下降）

**重要：为什么rotor的link pose必须设为电机安装位置？**

MulticopterMotorModel插件在rotor的link帧原点施加推力。如果rotor的link pose为(0,0,0)（模型原点），推力施加点偏离电机安装位置约11.5m，会产生巨大的俯仰力矩：
- 前部电机(X>0)：推力对重心产生抬头/低头力矩（取决于推力方向）
- 后部电机(X<0)：推力对重心产生相反方向的俯仰力矩

修复后rotor link pose = 电机安装位置，推力施加点与电机位置重合，力臂正确，俯仰力矩由控制分配器精确计算。

**抬头力矩抑制机制（新方案）**：

推进电机在重心下方1.5m，推力向前会产生抬头力矩。新方案通过**升力电机推力差动分配**抑制抬头力矩（不再使用 tilt 舵机）：
- 抬头时：前组上升电机(M0)/下降电机(M1)增强，后组下降电机(M2)/上升电机(M3)减弱
- 低头时：后组增强，前组减弱
- 控制分配器（ActuatorEffectivenessCustom）自动计算各电机推力分配

---

### 3️⃣ 推进电机配置（以rotor_4为例）

推进电机采用**单Link结构**，STL文件使用SW导出的绝对坐标（与飞艇主体同一原点）

**与升力电机同样的问题**：推进电机rotor的link pose也必须设为安装位置（物理推力施加点），否则推力施加在模型原点(0,0,0)，会产生巨大的偏航和俯仰力矩，导致飞艇乱跑。

**推进电机推力方向原理**（方式A: rc_cessna风格）：

MulticopterMotorModel插件始终沿link的局部**+Z轴**施加推力。要使推进电机推力沿+X方向（向前），需要让rotor link的局部Z轴指向base_link的+X方向。方法是在link pose中绕Y轴旋转+90度。

```
base_link坐标系:        rotor_4 link坐标系(绕Y轴+90度后):
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
<link name='rotor_4'>
  <pose>2.55 2.39 -1.51 0 1.5708 0</pose>       <!-- 绕Y轴+90度, 使link Z轴指向前方(+X) -->
  <gravity>false</gravity>
  <mass>1e-8</mass>
  <inertial>...</inertial>
  <visual name='rotor_4_visual'>
    <pose>-1.51 -2.39 -2.55 0 -1.5708 0</pose>  <!-- 补偿link旋转, 使mesh位置正确 -->
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://.../lingyun01_thrust_motor_LF.STL</uri>
      </mesh>
    </geometry>
  </visual>
  <collision name='rotor_4_collision'>
    <pose>-1.51 -2.39 -2.55 0 -1.5708 0</pose>  <!-- 同visual -->
    <geometry>
      <cylinder>
        <radius>1.0</radius>
        <length>0.08</length>
      </cylinder>
    </geometry>
  </collision>
</link>
<joint name='rotor_4_joint' type='revolute'>
  <!-- 无pose旋转, 旋转已放在link pose中 -->
  <parent>base_link</parent>
  <child>rotor_4</child>
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
- **Link pose**: (2.55, 2.39, -1.51, 0, 1.5708, 0) - 安装位置 + 绕Y轴+90度旋转，使link的Z轴指向+X方向
- **Visual/Collision pose**: (-1.51, -2.39, -2.55, 0, -1.5708, 0) - 补偿link旋转，使STL mesh在世界坐标系中位置正确
- **Joint pose**: 无旋转（旋转已放在link pose中）
- **Axis**: (0, 0, 1) - link局部Z轴，由于link旋转90度，实际旋转轴在base_link中是+X方向
- **spring_stiffness**: 必须为0，之前遗留值5000会导致电机被弹簧力卡死

**Visual/Collision pose补偿计算**：

当link pose添加了绕Y轴+90度旋转后，visual/collision的pose需要相应调整，确保STL mesh在世界坐标系中的位置不变。

```
补偿公式: visual_pose = R_y(-pi/2) * [原visual平移] + R_y(-pi/2)

以rotor_4为例:
  原visual平移 = [-2.55, -2.39, 1.51]
  R_y(-pi/2) * [-2.55, -2.39, 1.51] = [-1.51, -2.39, -2.55]
  再加上反向旋转: pitch = -1.5708
  最终: (-1.51, -2.39, -2.55, 0, -1.5708, 0)
```

**重要：为什么推进电机的link pose也必须设为安装位置？**

与升力电机同理，MulticopterMotorModel插件在rotor的link帧原点施加推力。如果link pose为(0,0,0)（模型原点），推力施加点偏离电机安装位置约2.5m，会产生巨大的偏航和俯仰力矩：
- 推力在模型原点施加，对重心产生偏航力矩（左右电机不对称）
- 推力在模型原点施加，对重心产生俯仰力矩（推进电机在重心下方1.5m）
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
| name | Joint名称 | rotor_0_joint, rotor_4_joint |
| type | Joint类型 | revolute(旋转), prismatic(滑动), fixed(固定) |
| parent | 父Link | base_link |
| child | 子Link | rotor_0, rotor_4 |
| axis/xyz | 旋转轴方向 | (0,0,1)绕Z轴, (1,0,0)绕X轴 |
| limit/lower | 最小角度限制 | 弧度值 |
| limit/upper | 最大角度限制 | 弧度值 |
| dynamics/friction | 摩擦力 | N·m/(rad/s)ⁿ |
| dynamics/damping | 阻尼系数 | N·m·s/rad |

---

### 2️⃣ 升力电机Joint（单关节结构）

新方案下升力电机采用**单关节结构**：rotor_0~3 直接通过 rotor_joint 连接 base_link，**无 tilt 倾斜关节**。推力方向由 motorConstant 符号控制，无需机械翻转。

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
<joint name='rotor_4_joint' type='revolute'>
  <!-- 无pose旋转, 旋转已放在link pose中 -->
  <parent>base_link</parent>
  <child>rotor_4</child>
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

## 📊 升力电机 vs 推进电机 对比

| 特性 | 升力电机 (rotor_0~3) | 推进电机 (rotor_4~7) |
|------|---------------------|---------------------|
| **Link数量** | 1个（仅螺旋桨） | 1个（仅螺旋桨） |
| **父子链** | base -> rotor | base -> rotor |
| **控制功能** | 升力（上升/下降）+ 俯仰差动 | 推力（水平前进）+ 偏航差动 |
| **倾斜关节** | 无（新方案已移除 tilt 舵机） | 无 |
| **旋转轴** | rotor_joint绕Z轴(局部) | rotor_joint绕Z轴(局部), link旋转后=base_link X轴 |
| **推力方向** | motorConstant符号控制（+向上/-向下） | +X(向前, link旋转后) |
| **Link pose旋转** | 无 | 绕Y轴+90度 |
| **推力方向控制** | motorConstant 符号（+1.416e-03/-2.832e-03） | 固定（link pose旋转90度） |
| **典型应用** | 垂直升力、俯仰姿态控制、抬头力矩抑制 | 水平推进、偏航控制 |

---

## 📋 电机布局位置表

### 升力电机（X轴线性排列）

| 电机编号 | Link名称 | motorConstant | STL绝对坐标中心(X,Y,Z) | 位置描述 |
|---------|---------|---------------|----------------------|---------|
| 0 | rotor_0 | +1.416e-03（向上） | (+11.225, -2.894, +0.331) | 前部电机1 (最前) |
| 1 | rotor_1 | -2.832e-03（向下） | (+9.215, -2.894, +0.331) | 前部电机2 |
| 2 | rotor_2 | -2.832e-03（向下） | (-13.535, -2.894, +0.331) | 后部电机1 |
| 3 | rotor_3 | +1.416e-03（向上） | (-15.525, -2.894, +0.331) | 后部电机2 (最后) |

**特点**:
- Y坐标统一: 全部在Y=-2.894m
- Z坐标统一: 全部在Z=+0.331m
- X轴分布: 前部电机(+9~+11m)，后部电机(-15~-14m)
- **推力方向**: 由 motorConstant 符号控制，M0/M3 向上（上升），M1/M2 向下（下降）
- STL文件使用绝对坐标，rotor link pose = 电机安装位置（物理正确），visual/collision pose = 负偏移（补偿STL绝对坐标）

### 推进电机（X-Y平面分布）

| 电机编号 | Link名称 | Link pose (x y z roll pitch yaw) | visual/collision pose | 位置描述 |
|---------|---------|--------------------------------|----------------------|---------|
| 4 | rotor_4 | (2.55, 2.39, -1.51, 0, 1.5708, 0) | (-1.51, -2.39, -2.55, 0, -1.5708, 0) | 左前 |
| 5 | rotor_5 | (-2.55, 2.39, -1.51, 0, 1.5708, 0) | (-1.51, -2.39, 2.55, 0, -1.5708, 0) | 左后 |
| 6 | rotor_6 | (2.55, -8.19, -1.51, 0, 1.5708, 0) | (-1.51, 8.19, -2.55, 0, -1.5708, 0) | 右前 |
| 7 | rotor_7 | (-2.55, -8.19, -1.51, 0, 1.5708, 0) | (-1.51, 8.19, 2.55, 0, -1.5708, 0) | 右后 |

**特点**:
- Z坐标统一: 全部在Z=-1.51m（重心下方约1.5m）
- 左右对称: 左(Y=+2.39m)右(Y=-8.19m)
- 前后对称: 前(X=+2.55m)后(X=-2.55m)
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
  <pose>11.225 -2.894 0.331 0 0 0</pose>
</link>

<!-- Joint位置 = 子Link的pose前3个参数 -->
<!-- Joint位置 = (11.225, -2.894, 0.331) -->
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
  <pose>11.225 -2.894 0.331 0 0 0</pose>
  <parent>base_link</parent>
  <child>rotor_0</child>
</joint>
```

- Joint和Link位置独立定义
- 可以不同，但需要保持一致性

---

## 🔧 配置示例：推进电机

### 当前配置（方式A: rc_cessna风格）

推进电机STL文件使用SW导出的绝对坐标，与升力电机相同，link pose必须设为安装位置：

```xml
<link name='rotor_4'>
  <pose>2.55 2.39 -1.51 0 1.5708 0</pose>       <!-- 绕Y轴+90度, 使link Z轴指向前方(+X) -->
  <visual name='rotor_4_visual'>
    <pose>-1.51 -2.39 -2.55 0 -1.5708 0</pose>  <!-- 补偿link旋转, 使mesh位置正确 -->
  </visual>
  <collision name='rotor_4_collision'>
    <pose>-1.51 -2.39 -2.55 0 -1.5708 0</pose>
  </collision>
</link>

<joint name='rotor_4_joint' type='revolute'>
  <!-- 无pose旋转, 旋转已放在link pose中 -->
  <parent>base_link</parent>
  <child>rotor_4</child>
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
- rotor质量很小(0.001kg)，仅代表螺旋桨（新方案：无 tilt_motor 支架）

### 引力设置

- 所有Link的gravity设置为false
- 飞艇依靠气囊浮力悬浮，而非螺旋桨升力平衡重力

### STL文件说明

- **所有STL文件均使用SW导出的绝对坐标**（与飞艇主体同一原点）
- 升力电机STL已从source_backup恢复为原始绝对坐标版本
- 推进电机STL本身就是绝对坐标
- **升力电机(rotor_0~3)**：link pose = 电机安装位置（物理正确），visual/collision pose = 负偏移（补偿STL绝对坐标），推力方向由 motorConstant 符号控制
- **推进电机(rotor_4~7)**：link pose = 安装位置（物理推力点），visual/collision pose = 负偏移（补偿STL绝对坐标）
- **所有电机的rotor link pose都必须设为物理安装位置**，否则推力施加在模型原点(0,0,0)，产生巨大的错误力矩

### 推力方向与力矩分析

新方案下升力电机推力方向**固定**，由 motorConstant 符号控制（无 tilt 舵机翻转）：

| 电机类型 | 推力方向 | 俯仰力矩 | 偏航力矩 | 推力方向控制方式 |
|---------|---------|---------|---------|-------------------|
| 升力电机 M0/M3 (motorConstant>0) | +Z（向上，上升） | 前部M0增大->抬头, 后部M3增大->低头 | 无（反扭矩抵消） | motorConstant 正值，固定向上 |
| 升力电机 M1/M2 (motorConstant<0) | -Z（向下，下降） | 前部M1增大->低头, 后部M2增大->抬头 | 无（反扭矩抵消） | motorConstant 负值，固定向下 |
| 推进电机(4-7) | +X(向前) | 向前推->抬头(电机在重心下方1.5m) | 左侧推->左转, 右侧推->右转 | link pose绕Y轴+90度，固定向前 |

**关键结论**：
- 升力电机推力方向**固定**，由 motorConstant 符号控制（正值向上，负值向下），无需 tilt 舵机翻转
- 俯仰控制通过升力电机推力差动分配实现：前组(M0/M1)增强抬头，后组(M2/M3)增强低头
- 推进电机推力方向始终水平向前，推进-俯仰耦合方向固定不变
- 抬头力矩抑制通过升力电机推力差动分配（不再使用 tilt 舵机）

---

## 🔗 相关文档

- [灵云01主体STL分析报告](./meshes/source_backup/原始STL备份.md)
- [Gazebo SDF规范文档](http://sdformat.org/spec)

---

**维护者**: 灵云01项目组

**版本历史**:
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
