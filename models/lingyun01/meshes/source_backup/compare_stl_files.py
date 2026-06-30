#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵云01号 - STL文件对比分析工具
对比source_backup（原始SW文件）和meshes根目录（修改后文件）的差异

使用方法:
    python3 compare_stl_files.py
"""

import os
import struct
import json
from datetime import datetime


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

            centroid_x = sum(vertices_x) / len(vertices_x)
            centroid_y = sum(vertices_y) / len(vertices_y)
            centroid_z = sum(vertices_z) / len(vertices_z)

            size_x = x_max - x_min
            size_y = y_max - y_min
            size_z = z_max - z_min

            return {
                'filename': os.path.basename(filepath),
                'filepath': filepath,
                'triangle_count': triangle_count,
                'vertex_count': len(vertices_x),
                'x_range': [x_min, x_max],
                'y_range': [y_min, y_max],
                'z_range': [z_min, z_max],
                'size': [size_x, size_y, size_z],
                'bounding_box_center': [center_x, center_y, center_z],
                'centroid': [centroid_x, centroid_y, centroid_z]
            }
    except Exception as e:
        print(f"  ❌ 错误: {e}")
        return None


def analyze_directory(directory, file_mapping=None):
    """分析指定目录的所有STL文件"""
    print(f"\n📂 分析目录: {directory}")
    print("=" * 80)

    if not os.path.exists(directory):
        print(f"❌ 目录不存在: {directory}")
        return {}

    stl_data = {}
    stl_files = [f for f in os.listdir(directory)
                 if f.upper().endswith('.STL')]

    stl_files.sort()

    print(f"📊 找到 {len(stl_files)} 个STL文件\n")

    for filename in stl_files:
        filepath = os.path.join(directory, filename)
        print(f"读取: {filename}...", end=" ")

        data = read_stl_metadata(filepath)
        if data:
            stl_data[filename] = data
            print("✅")
        else:
            print("❌")

    return stl_data


def match_files(backup_data, current_data, file_mapping):
    """匹配两组文件"""
    matches = {}

    for cn_name, en_name in file_mapping.items():
        backup_file = backup_data.get(cn_name)
        current_file = current_data.get(en_name)

        if backup_file and current_file:
            matches[cn_name] = {
                'backup': backup_file,
                'current': current_file,
                'cn_name': cn_name,
                'en_name': en_name
            }
        elif backup_file:
            matches[cn_name] = {
                'backup': backup_file,
                'current': None,
                'cn_name': cn_name,
                'en_name': None
            }
        elif current_file:
            matches[en_name] = {
                'backup': None,
                'current': current_file,
                'cn_name': None,
                'en_name': en_name
            }

    return matches


def compare_files(matches):
    """对比文件差异"""
    print("\n" + "=" * 80)
    print("📊 STL文件对比分析报告")
    print("=" * 80)

    changed_files = []
    unchanged_files = []

    for name, data in matches.items():
        backup = data['backup']
        current = data['current']

        if not backup or not current:
            print(f"\n⚠️ 文件不匹配: {data['cn_name'] or data['en_name']}")
            if not backup:
                print(f"  ❌ source_backup中不存在")
            if not current:
                print(f"  ❌ meshes根目录中不存在")
            continue

        print(f"\n📄 {data['cn_name']} → {data['en_name']}")
        print("-" * 80)

        # 检查三角形数量
        if backup['triangle_count'] != current['triangle_count']:
            print(f"  ⚠️ 三角形数量: {backup['triangle_count']:,} → {current['triangle_count']:,}")
        else:
            print(f"  ✅ 三角形数量: {backup['triangle_count']:,} (一致)")

        # 检查尺寸
        size_changed = False
        for i, axis in enumerate(['X', 'Y', 'Z']):
            if abs(backup['size'][i] - current['size'][i]) > 0.001:
                size_changed = True
                print(f"  ⚠️ {axis}尺寸: {backup['size'][i]:.6f}m → {current['size'][i]:.6f}m")

        if not size_changed:
            print(f"  ✅ 尺寸: {backup['size'][0]:.3f} × {backup['size'][1]:.3f} × {backup['size'][2]:.3f}m (一致)")

        # 检查坐标范围变化
        range_changed = False
        range_diffs = {}

        for axis, key in [('X', 'x_range'), ('Y', 'y_range'), ('Z', 'z_range')]:
            b_range = backup[key]
            c_range = current[key]

            b_center = (b_range[0] + b_range[1]) / 2
            c_center = (c_range[0] + c_range[1]) / 2

            if abs(b_center - c_center) > 0.001:
                range_changed = True
                offset = c_center - b_center
                range_diffs[axis] = offset

        if range_changed:
            changed_files.append({
                'name': name,
                'backup': backup,
                'current': current,
                'diffs': range_diffs
            })

            print(f"  🔄 坐标偏移:")
            for axis, offset in range_diffs.items():
                print(f"     {axis}轴偏移: {offset:+.4f}m")

            # 显示详细的范围变化
            print(f"  📍 范围对比:")
            print(f"     X: [{backup['x_range'][0]:+.4f}, {backup['x_range'][1]:+.4f}] → "
                  f"[{current['x_range'][0]:+.4f}, {current['x_range'][1]:+.4f}]")
            print(f"     Y: [{backup['y_range'][0]:+.4f}, {backup['y_range'][1]:+.4f}] → "
                  f"[{current['y_range'][0]:+.4f}, {current['y_range'][1]:+.4f}]")
            print(f"     Z: [{backup['z_range'][0]:+.4f}, {backup['z_range'][1]:+.4f}] → "
                  f"[{current['z_range'][0]:+.4f}, {current['z_range'][1]:+.4f}]")

        else:
            unchanged_files.append(name)
            print(f"  ✅ 坐标范围: 完全一致")

    return changed_files, unchanged_files


def summarize_changes(changed_files):
    """总结修改内容"""
    print("\n" + "=" * 80)
    print("📝 修改总结")
    print("=" * 80)

    if not changed_files:
        print("\n✅ 所有文件完全一致，未做任何修改！")
        return

    print(f"\n🔄 共 {len(changed_files)} 个文件被修改:\n")

    # 按偏移类型分组
    origin_only = []  # 仅原点化
    translated = []   # 有平移
    other = []        # 其他变化

    for file_info in changed_files:
        name = file_info['name']
        diffs = file_info['diffs']

        if not diffs:
            origin_only.append(name)
        elif len(diffs) == 3 and all(abs(v) < 0.01 for v in diffs.values()):
            origin_only.append(name)
        else:
            translated.append((name, diffs))

    if origin_only:
        print("📍 类型1: 原点化修改（所有坐标归零到原点附近）")
        print("   这些文件的X、Y、Z三个轴的边界框中心都被移动到了原点附近")
        for name in origin_only:
            print(f"   - {name}")

    if translated:
        print("\n📐 类型2: 坐标平移修改（保留原始位置偏移）")
        print("   这些文件的坐标范围发生了平移，保留了原始安装位置信息")
        for name, diffs in translated:
            print(f"   - {name}")
            for axis, offset in diffs.items():
                print(f"     {axis}轴偏移: {offset:+.4f}m")

    print("\n" + "=" * 80)
    print("💡 修改模式分析")
    print("=" * 80)

    # 分析修改模式
    x_offsets = [f['diffs'].get('X', 0) for f in changed_files if f['diffs']]
    y_offsets = [f['diffs'].get('Y', 0) for f in changed_files if f['diffs']]
    z_offsets = [f['diffs'].get('Z', 0) for f in changed_files if f['diffs']]

    if x_offsets:
        avg_x = sum(x_offsets) / len(x_offsets)
        print(f"\nX轴平均偏移: {avg_x:+.4f}m")
    if y_offsets:
        avg_y = sum(y_offsets) / len(y_offsets)
        print(f"Y轴平均偏移: {avg_y:+.4f}m")
    if z_offsets:
        avg_z = sum(z_offsets) / len(z_offsets)
        print(f"Z轴平均偏移: {avg_z:+.4f}m")

    # 判断修改类型
    all_origins = all(
        all(abs(f['diffs'].get(axis, 0)) < 0.1 for axis in ['X', 'Y', 'Z'])
        for f in changed_files if f['diffs']
    )

    if all_origins:
        print("\n✅ 修改类型: 原点化")
        print("   所有电机文件的坐标都被移动到了原点附近（0,0,0）")
        print("   优点: 文件坐标统一，便于在Gazebo中定位")
        print("   缺点: 丢失了原始安装位置信息，需要在model.sdf中重新设置pose")
    else:
        print("\n✅ 修改类型: 坐标平移")
        print("   文件保留了原始坐标偏移，仅进行了必要的平移")


def main():
    # 定义文件映射关系（中文名 → 英文名）
    file_mapping = {
        '灵云01主体.STL': 'lingyun01_hull_all.STL',
        '高度前1.STL': 'lingyun01_lift_motor_front1.STL',
        '高度前2.STL': 'lingyun01_lift_motor_front2.STL',
        '高度后1.STL': 'lingyun01_lift_motor_back1.STL',
        '高度后2.STL': 'lingyun01_lift_motor_back2.STL',
        '推进左1.STL': 'lingyun01_thrust_motor_LF.STL',
        '推进左2.STL': 'lingyun01_thrust_motor_LB.STL',
        '推进右1.STL': 'lingyun01_thrust_motor_RF.STL',
        '推进右2.STL': 'lingyun01_thrust_motor_RB.STL',
    }

    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)

    # 分析两个目录
    backup_dir = script_dir  # source_backup目录
    current_dir = parent_dir  # meshes根目录

    print("=" * 80)
    print("🔍 灵云01号 - STL文件对比分析工具")
    print("=" * 80)
    print(f"📦 source_backup (原始SW文件): {backup_dir}")
    print(f"📦 meshes根目录 (仿真用文件): {current_dir}")

    # 分析source_backup目录
    backup_data = analyze_directory(backup_dir)

    # 分析meshes根目录
    current_data = analyze_directory(current_dir)

    # 匹配文件
    matches = match_files(backup_data, current_data, file_mapping)

    # 对比文件
    changed_files, unchanged_files = compare_files(matches)

    # 总结
    summarize_changes(changed_files)

    print("\n" + "=" * 80)
    print("✅ 分析完成")
    print("=" * 80)


if __name__ == '__main__':
    main()
