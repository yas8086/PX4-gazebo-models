#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证推进电机的STL坐标与model.sdf配置的关系
"""

import os
import struct

def read_stl_metadata(filepath):
    """读取STL文件的元数据"""
    try:
        with open(filepath, 'rb') as f:
            header = f.read(80)
            triangle_count_data = f.read(4)
            if len(triangle_count_data) < 4:
                return None

            triangle_count = struct.unpack('<I', triangle_count_data)[0]

            vertices_x = []
            vertices_y = []
            vertices_z = []

            for i in range(triangle_count):
                data = f.read(50)
                if len(data) < 50:
                    break

                floats = struct.unpack('<12f', data[:48])

                for j in range(3):
                    vertices_x.append(floats[3 + j * 3])
                    vertices_y.append(floats[3 + j * 3 + 1])
                    vertices_z.append(floats[3 + j * 3 + 2])

            if not vertices_x:
                return None

            x_min, x_max = min(vertices_x), max(vertices_x)
            y_min, y_max = min(vertices_y), max(vertices_y)
            z_min, z_max = min(vertices_z), max(vertices_z)

            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            center_z = (z_min + z_max) / 2

            return {
                'x_range': [x_min, x_max],
                'y_range': [y_min, y_max],
                'z_range': [z_min, z_max],
                'bounding_box_center': [center_x, center_y, center_z],
            }
    except Exception as e:
        print(f"  ❌ 错误: {e}")
        return None

# model.sdf中的配置（从之前的分析中获取）
thrust_motor_config = {
    'rotor_4': {
        'name': '左前推进电机 (LF)',
        'stl_file': 'lingyun01_thrust_motor_LF.STL',
        'link_pose': [2.547, 2.387, -1.512, 0, 1.5708, 0],
        'visual_pose': [-1.512, -2.387, -2.547, 0, -1.5708, 0],
    },
    'rotor_5': {
        'name': '左后推进电机 (LB)',
        'stl_file': 'lingyun01_thrust_motor_LB.STL',
        'link_pose': [-2.547, 2.387, -1.512, 0, 1.5708, 0],
        'visual_pose': [-1.512, -2.387, 2.547, 0, -1.5708, 0],
    },
    'rotor_6': {
        'name': '右前推进电机 (RF)',
        'stl_file': 'lingyun01_thrust_motor_RF.STL',
        'link_pose': [2.547, -8.187, -1.512, 0, 1.5708, 0],
        'visual_pose': [-1.512, 8.187, -2.547, 0, -1.5708, 0],
    },
    'rotor_7': {
        'name': '右后推进电机 (RB)',
        'stl_file': 'lingyun01_thrust_motor_RB.STL',
        'link_pose': [-2.547, -8.187, -1.512, 0, 1.5708, 0],
        'visual_pose': [-1.512, 8.187, 2.547, 0, -1.5708, 0],
    },
}

print("=" * 80)
print("🔍 推进电机 STL坐标 与 model.sdf配置 对比分析")
print("=" * 80)

meshes_dir = "/home/hex/PX4-Autopilot/Tools/simulation/gz/models/lingyun01/meshes"

for rotor_id, config in thrust_motor_config.items():
    print(f"\n📄 {rotor_id}: {config['name']}")
    print("-" * 80)

    # 读取STL文件
    stl_path = os.path.join(meshes_dir, config['stl_file'])
    stl_data = read_stl_metadata(stl_path)

    if stl_data:
        print(f"  📦 STL文件: {config['stl_file']}")

        print(f"\n  📐 STL坐标范围:")
        print(f"     X: [{stl_data['x_range'][0]:+.4f}, {stl_data['x_range'][1]:+.4f}] m")
        print(f"     Y: [{stl_data['y_range'][0]:+.4f}, {stl_data['y_range'][1]:+.4f}] m")
        print(f"     Z: [{stl_data['z_range'][0]:+.4f}, {stl_data['z_range'][1]:+.4f}] m")

        print(f"\n  🎯 STL边界框中心: ({stl_data['bounding_box_center'][0]:+.4f}, "
              f"{stl_data['bounding_box_center'][1]:+.4f}, "
              f"{stl_data['bounding_box_center'][2]:+.4f}) m")

        print(f"\n  📍 model.sdf配置:")
        print(f"     link_pose:  ({config['link_pose'][0]:+.4f}, "
              f"{config['link_pose'][1]:+.4f}, "
              f"{config['link_pose'][2]:+.4f}, "
              f"{config['link_pose'][3]:.4f}, "
              f"{config['link_pose'][4]:.4f}, "
              f"{config['link_pose'][5]:.4f})")
        print(f"     visual_pose: ({config['visual_pose'][0]:+.4f}, "
              f"{config['visual_pose'][1]:+.4f}, "
              f"{config['visual_pose'][2]:+.4f}, "
              f"{config['visual_pose'][3]:.4f}, "
              f"{config['visual_pose'][4]:.4f}, "
              f"{config['visual_pose'][5]:.4f})")

        # 对比
        print(f"\n  🔍 对比分析:")
        bb = stl_data['bounding_box_center']
        lp = config['link_pose']

        x_diff = abs(bb[0] - lp[0])
        y_diff = abs(bb[1] - lp[1])
        z_diff = abs(bb[2] - lp[2])

        print(f"     STL边界框中心 vs link_pose位置:")
        print(f"       X: STL={bb[0]:+.4f} vs link={lp[0]:+.4f} | 差异={x_diff:.4f} m "
              f"{'✅ 一致' if x_diff < 0.01 else '⚠️ 不同'}")
        print(f"       Y: STL={bb[1]:+.4f} vs link={lp[1]:+.4f} | 差异={y_diff:.4f} m "
              f"{'✅ 一致' if y_diff < 0.01 else '⚠️ 不同'}")
        print(f"       Z: STL={bb[2]:+.4f} vs link={lp[2]:+.4f} | 差异={z_diff:.4f} m "
              f"{'✅ 一致' if z_diff < 0.01 else '⚠️ 不同'}")

        # 旋转分析
        print(f"\n     旋转分析:")
        print(f"       link_pose旋转: (0, {lp[4]:.4f}, 0) rad = {lp[4]/3.14159*180:.1f}° 绕Y轴")
        print(f"       visual_pose旋转: (0, {config['visual_pose'][4]:.4f}, 0) rad = {config['visual_pose'][4]/3.14159*180:.1f}° 绕Y轴")

        # 计算visual的最终位置
        print(f"\n     最终渲染位置计算:")
        final_x = lp[0] + config['visual_pose'][0]
        final_y = lp[1] + config['visual_pose'][1]
        final_z = lp[2] + config['visual_pose'][2]
        print(f"       link位置 + visual偏移 = ({final_x:+.4f}, {final_y:+.4f}, {final_z:+.4f})")
        print(f"       这是STL模型在base_link坐标系中的最终位置")

print("\n" + "=" * 80)
print("💡 关键发现")
print("=" * 80)

print("""
问题分析:
---------
推进电机的STL文件坐标（边界框中心）与model.sdf中的link_pose位置完全一致。

但model.sdf中还设置了visual_pose，这说明存在多层坐标系变换：

1. base_link坐标系（飞艇主体坐标系）
   ↓ joint连接
2. rotor_link坐标系（通过link pose定义）
   ↓ visual pose偏移
3. STL模型坐标系（通过visual pose定义）

这可能是因为：
- STL文件在SW中的原点不是期望的"中心点"
- 需要通过visual pose进行微调来正确对齐电机模型
- 或者是为了与升力电机的配置方式保持一致（统一处理）
""")

print("=" * 80)
