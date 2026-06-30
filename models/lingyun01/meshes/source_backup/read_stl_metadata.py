#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵云01号 - STL文件元数据读取工具
用于读取source_backup文件夹中所有STL文件的坐标信息

使用方法:
    python3 read_stl_metadata.py
"""

import os
import struct
import json
from datetime import datetime

def read_stl_metadata(filepath):
    """
    读取STL文件的元数据

    返回:
        dict: 包含文件信息的字典
    """
    try:
        with open(filepath, 'rb') as f:
            # 读取80字节头部
            header = f.read(80)

            # 读取三角形数量
            triangle_count_data = f.read(4)
            if len(triangle_count_data) < 4:
                return None

            triangle_count = struct.unpack('<I', triangle_count_data)[0]

            # 读取所有顶点
            vertices_x = []
            vertices_y = []
            vertices_z = []

            for i in range(triangle_count):
                # 读取50字节（12个float法向量 + 3个顶点×3个坐标）
                data = f.read(50)
                if len(data) < 50:
                    break

                floats = struct.unpack('<12f', data[:48])

                # 提取顶点坐标（跳过法向量）
                for j in range(3):
                    vertices_x.append(floats[3 + j * 3])
                    vertices_y.append(floats[3 + j * 3 + 1])
                    vertices_z.append(floats[3 + j * 3 + 2])

            if not vertices_x:
                return None

            # 计算统计信息
            x_min, x_max = min(vertices_x), max(vertices_x)
            y_min, y_max = min(vertices_y), max(vertices_y)
            z_min, z_max = min(vertices_z), max(vertices_z)

            # 计算边界框中心（几何中心）
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            center_z = (z_min + z_max) / 2

            # 计算质心（所有顶点的平均值）
            centroid_x = sum(vertices_x) / len(vertices_x)
            centroid_y = sum(vertices_y) / len(vertices_y)
            centroid_z = sum(vertices_z) / len(vertices_z)

            # 计算尺寸
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


def print_stl_info(stl_data, index=None):
    """打印STL文件的详细信息"""
    if not stl_data:
        return

    prefix = f"[{index}] " if index is not None else "📄 "

    print(f"{prefix}{stl_data['filename']}")
    print(f"   三角形数量: {stl_data['triangle_count']:,}")
    print(f"   顶点数量: {stl_data['vertex_count']:,}")

    print(f"   X范围: [{stl_data['x_range'][0]:+.6f}, {stl_data['x_range'][1]:+.6f}]")
    print(f"        尺寸: {stl_data['size'][0]:.6f} m")

    print(f"   Y范围: [{stl_data['y_range'][0]:+.6f}, {stl_data['y_range'][1]:+.6f}]")
    print(f"        尺寸: {stl_data['size'][1]:.6f} m")

    print(f"   Z范围: [{stl_data['z_range'][0]:+.6f}, {stl_data['z_range'][1]:+.6f}]")
    print(f"        尺寸: {stl_data['size'][2]:.6f} m")

    print(f"   边界框中心: ({stl_data['bounding_box_center'][0]:+.6f}, "
          f"{stl_data['bounding_box_center'][1]:+.6f}, "
          f"{stl_data['bounding_box_center'][2]:+.6f})")

    print(f"   质心: ({stl_data['centroid'][0]:+.6f}, "
          f"{stl_data['centroid'][1]:+.6f}, "
          f"{stl_data['centroid'][2]:+.6f})")
    print()


def save_to_json(all_data, output_file):
    """保存数据到JSON文件"""
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(all_data, f, indent=2, ensure_ascii=False)
    print(f"✅ 数据已保存到: {output_file}")


def main():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 80)
    print("📦 灵云01号 - STL文件元数据读取工具")
    print("=" * 80)
    print(f"📂 目录: {script_dir}")
    print()

    # 查找所有STL文件
    stl_files = []
    for file in os.listdir(script_dir):
        if file.upper().endswith('.STL'):
            filepath = os.path.join(script_dir, file)
            stl_files.append((file, filepath))

    # 按文件名排序
    stl_files.sort(key=lambda x: x[0])

    if not stl_files:
        print("❌ 未找到STL文件")
        return

    print(f"📊 找到 {len(stl_files)} 个STL文件\n")
    print("-" * 80)

    # 读取所有STL文件
    all_data = []
    for index, (filename, filepath) in enumerate(stl_files, 1):
        print(f"正在读取 [{index}/{len(stl_files)}]: {filename}")
        stl_data = read_stl_metadata(filepath)

        if stl_data:
            print_stl_info(stl_data, index)
            all_data.append(stl_data)
        else:
            print(f"  ❌ 无法读取文件\n")

    # 保存到JSON
    json_file = os.path.join(script_dir, 'stl_metadata.json')
    save_to_json(all_data, json_file)

    print("=" * 80)
    print(f"✅ 处理完成: {len(all_data)}/{len(stl_files)} 个文件成功读取")
    print("=" * 80)


if __name__ == '__main__':
    main()
