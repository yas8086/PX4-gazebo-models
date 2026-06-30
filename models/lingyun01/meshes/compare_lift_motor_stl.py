#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵云01号 - 升力电机STL文件深度对比分析工具
对比source_backup（SW原始导出）和meshes根目录（原点化后）的升力电机STL文件

重点分析：
1. 读取所有三角形顶点
2. 计算每个STL的尺寸（X/Y/Z范围）
3. 检查形状一致性（通过平移回备份坐标后，比较顶点位置差异）
4. 特别关注：原点化过程中是否只做了平移，还是也做了旋转
5. 输出详细的对比结果

使用方法:
    python3 compare_lift_motor_stl.py
"""

import struct
import os
import numpy as np


# ============================================================
# 文件映射：备份文件名 → 当前文件名
# ============================================================
FILE_MAPPING = {
    '高度前1.STL': 'lingyun01_lift_motor_front1.STL',
    '高度前2.STL': 'lingyun01_lift_motor_front2.STL',
    '高度后1.STL': 'lingyun01_lift_motor_back1.STL',
    '高度后2.STL': 'lingyun01_lift_motor_back2.STL',
}


def read_stl_triangles(filepath):
    """读取STL文件的所有三角形，返回法向量和顶点数组"""
    triangles = []  # 每个元素: (normal, v0, v1, v2)

    with open(filepath, 'rb') as f:
        header = f.read(80)
        tri_count_data = f.read(4)
        if len(tri_count_data) < 4:
            return []

        tri_count = struct.unpack('<I', tri_count_data)[0]

        for _ in range(tri_count):
            data = f.read(50)
            if len(data) < 50:
                break
            floats = struct.unpack('<12f', data[:48])
            # floats: nx,ny,nz, v0x,v0y,v0z, v1x,v1y,v1z, v2x,v2y,v2z
            normal = np.array(floats[0:3])
            v0 = np.array(floats[3:6])
            v1 = np.array(floats[6:9])
            v2 = np.array(floats[9:12])
            triangles.append((normal, v0, v1, v2))

    return triangles


def get_all_vertices(triangles):
    """从三角形列表中提取所有顶点（含重复），返回Nx3数组"""
    verts = []
    for _, v0, v1, v2 in triangles:
        verts.append(v0)
        verts.append(v1)
        verts.append(v2)
    return np.array(verts)


def compute_bbox(vertices):
    """计算顶点的包围盒"""
    vmin = vertices.min(axis=0)
    vmax = vertices.max(axis=0)
    center = (vmin + vmax) / 2
    size = vmax - vmin
    return vmin, vmax, center, size


def compute_centroid(vertices):
    """计算质心"""
    return vertices.mean(axis=0)


def check_pure_translation(backup_verts, current_verts):
    """
    检查两组顶点之间是否只存在平移关系（无旋转/缩放）

    方法：
    1. 将两组顶点都中心化（减去各自的质心）
    2. 如果只有平移，中心化后顶点应该完全一致
    3. 如果有旋转，中心化后顶点会有差异
    4. 用SVD求解最优旋转矩阵，检查残差
    """
    # 中心化
    backup_centered = backup_verts - backup_verts.mean(axis=0)
    current_centered = current_verts - current_verts.mean(axis=0)

    # 方法1: 直接比较中心化后的顶点差异
    # 先按坐标排序，因为三角形顺序可能不同
    backup_sorted = np.sort(backup_centered.view([('', backup_centered.dtype)] * 3),
                            axis=0).view(backup_centered.dtype)
    current_sorted = np.sort(current_centered.view([('', current_centered.dtype)] * 3),
                             axis=0).view(current_centered.dtype)

    # 方法2: 用SVD求解最优旋转矩阵 R，使得 R @ backup_centered.T ≈ current_centered.T
    # H = backup_centered.T @ current_centered
    H = backup_centered.T @ current_centered
    U, S, Vt = np.linalg.svd(H)

    # 最优旋转矩阵
    R = Vt.T @ U.T

    # 确保是正交矩阵（行列式=1，不是反射）
    det = np.linalg.det(R)
    if det < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
        det = np.linalg.det(R)

    # 计算旋转后的残差
    rotated_backup = (R @ backup_centered.T).T
    residuals = np.linalg.norm(rotated_backup - current_centered, axis=1)
    mean_residual = residuals.mean()
    max_residual = residuals.max()

    # 计算平移向量
    translation = current_verts.mean(axis=0) - R @ backup_verts.mean(axis=0)

    # 检查R是否接近单位矩阵
    identity = np.eye(3)
    rotation_diff = R - identity
    rotation_angle_rad = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    rotation_angle_deg = np.degrees(rotation_angle_rad)

    return {
        'R': R,
        'det_R': det,
        'rotation_angle_deg': rotation_angle_deg,
        'translation': translation,
        'mean_residual': mean_residual,
        'max_residual': max_residual,
        'is_pure_translation': rotation_angle_deg < 0.01 and mean_residual < 0.001,
    }


def check_rotation_axes(R, rotation_angle_deg):
    """分析旋转矩阵，判断绕哪个轴旋转"""
    if rotation_angle_deg < 0.01:
        return "无旋转（纯平移）"

    # 旋转轴 = R的特征值为1对应的特征向量
    eigenvalues, eigenvectors = np.linalg.eig(R)
    # 找最接近1的特征值
    idx = np.argmin(np.abs(eigenvalues - 1.0))
    axis = eigenvectors[:, idx].real
    axis = axis / np.linalg.norm(axis)

    # 判断主旋转轴
    abs_axis = np.abs(axis)
    dominant = ['X', 'Y', 'Z'][np.argmax(abs_axis)]

    return f"绕{dominant}轴旋转 {rotation_angle_deg:.4f} 度 (旋转轴: [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}])"


def analyze_single_stl(filepath, label):
    """分析单个STL文件，返回详细信息"""
    triangles = read_stl_triangles(filepath)
    vertices = get_all_vertices(triangles)
    vmin, vmax, center, size = compute_bbox(vertices)
    centroid = compute_centroid(vertices)

    return {
        'label': label,
        'filepath': filepath,
        'triangle_count': len(triangles),
        'vertex_count': len(vertices),
        'bbox_min': vmin,
        'bbox_max': vmax,
        'bbox_center': center,
        'bbox_size': size,
        'centroid': centroid,
        'vertices': vertices,
        'triangles': triangles,
    }


def compare_pair(backup_info, current_info):
    """对比一对文件"""
    cn_name = backup_info['label']
    en_name = current_info['label']

    print(f"\n{'=' * 80}")
    print(f"对比: {cn_name}  <--->  {en_name}")
    print(f"{'=' * 80}")

    # 1. 三角形数量
    print(f"\n--- 1. 三角形数量 ---")
    b_tri = backup_info['triangle_count']
    c_tri = current_info['triangle_count']
    if b_tri == c_tri:
        print(f"  一致: {b_tri}")
    else:
        print(f"  不一致: 备份={b_tri}, 当前={c_tri}")

    # 2. 包围盒尺寸
    print(f"\n--- 2. 包围盒尺寸 ---")
    b_size = backup_info['bbox_size']
    c_size = current_info['bbox_size']
    for i, axis in enumerate(['X', 'Y', 'Z']):
        diff = abs(b_size[i] - c_size[i])
        match = "一致" if diff < 0.001 else f"差异 {diff:.6f}"
        print(f"  {axis}: 备份={b_size[i]:.6f}m, 当前={c_size[i]:.6f}m  ({match})")

    # 3. 包围盒中心偏移
    print(f"\n--- 3. 包围盒中心偏移 ---")
    b_center = backup_info['bbox_center']
    c_center = current_info['bbox_center']
    offset = c_center - b_center
    print(f"  备份中心: [{b_center[0]:+.6f}, {b_center[1]:+.6f}, {b_center[2]:+.6f}]")
    print(f"  当前中心: [{c_center[0]:+.6f}, {c_center[1]:+.6f}, {c_center[2]:+.6f}]")
    print(f"  偏移量:   [{offset[0]:+.6f}, {offset[1]:+.6f}, {offset[2]:+.6f}]")

    # 4. 质心偏移
    print(f"\n--- 4. 质心偏移 ---")
    b_centroid = backup_info['centroid']
    c_centroid = current_info['centroid']
    centroid_offset = c_centroid - b_centroid
    print(f"  备份质心: [{b_centroid[0]:+.6f}, {b_centroid[1]:+.6f}, {b_centroid[2]:+.6f}]")
    print(f"  当前质心: [{c_centroid[0]:+.6f}, {c_centroid[1]:+.6f}, {c_centroid[2]:+.6f}]")
    print(f"  偏移量:   [{centroid_offset[0]:+.6f}, {centroid_offset[1]:+.6f}, {centroid_offset[2]:+.6f}]")

    # 5. 旋转检测（核心分析）
    print(f"\n--- 5. 旋转/平移检测（SVD分析） ---")
    result = check_pure_translation(backup_info['vertices'], current_info['vertices'])

    R = result['R']
    print(f"  旋转矩阵 R:")
    for i in range(3):
        print(f"    [{R[i, 0]:+.8f}, {R[i, 1]:+.8f}, {R[i, 2]:+.8f}]")
    print(f"  行列式 det(R): {result['det_R']:.8f}")
    print(f"  旋转角度: {result['rotation_angle_deg']:.6f} 度")
    print(f"  旋转轴分析: {check_rotation_axes(R, result['rotation_angle_deg'])}")
    print(f"  平移向量: [{result['translation'][0]:+.6f}, {result['translation'][1]:+.6f}, {result['translation'][2]:+.6f}]")
    print(f"  SVD拟合残差: 平均={result['mean_residual']:.8f}m, 最大={result['max_residual']:.8f}m")

    if result['is_pure_translation']:
        print(f"  结论: 纯平移，无旋转/缩放")
    else:
        print(f"  结论: 存在旋转或缩放变换（非纯平移）")

    # 6. 顶点级精确对比
    print(f"\n--- 6. 顶点级精确对比 ---")
    backup_verts = backup_info['vertices']
    current_verts = current_info['vertices']

    # 用平移补偿后比较
    compensated = current_verts - centroid_offset
    diff = compensated - backup_verts
    diff_norms = np.linalg.norm(diff, axis=1)

    print(f"  用质心偏移补偿后:")
    print(f"    顶点差异: 平均={diff_norms.mean():.8f}m, 最大={diff_norms.max():.8f}m, 最小={diff_norms.min():.8f}m")

    # 用SVD求出的旋转+平移补偿后比较
    R = result['R']
    t = result['translation']
    transformed = (R @ backup_verts.T).T + t
    diff2 = transformed - current_verts
    diff2_norms = np.linalg.norm(diff2, axis=1)

    print(f"  用SVD求出的R+t补偿后:")
    print(f"    顶点差异: 平均={diff2_norms.mean():.8f}m, 最大={diff2_norms.max():.8f}m, 最小={diff2_norms.min():.8f}m")

    # 7. 包围盒范围详细对比
    print(f"\n--- 7. 包围盒范围详细对比 ---")
    b_min = backup_info['bbox_min']
    b_max = backup_info['bbox_max']
    c_min = current_info['bbox_min']
    c_max = current_info['bbox_max']
    print(f"  备份 X: [{b_min[0]:+.6f}, {b_max[0]:+.6f}]  当前 X: [{c_min[0]:+.6f}, {c_max[0]:+.6f}]")
    print(f"  备份 Y: [{b_min[1]:+.6f}, {b_max[1]:+.6f}]  当前 Y: [{c_min[1]:+.6f}, {c_max[1]:+.6f}]")
    print(f"  备份 Z: [{b_min[2]:+.6f}, {b_max[2]:+.6f}]  当前 Z: [{c_min[2]:+.6f}, {c_max[2]:+.6f}]")

    # 检查尺寸是否在X/Y/Z轴之间发生了交换（旋转的标志）
    print(f"\n--- 8. 轴交换检测 ---")
    b_sorted = sorted(b_size)
    c_sorted = sorted(c_size)
    axis_swapped = False
    for i in range(3):
        if abs(b_sorted[i] - c_sorted[i]) > 0.001:
            axis_swapped = True
            break

    if not axis_swapped:
        print(f"  排序后尺寸一致: 备份={[f'{s:.6f}' for s in b_sorted]}, 当前={[f'{s:.6f}' for s in c_sorted]}")
        print(f"  无轴交换发生")
    else:
        print(f"  排序后尺寸不一致: 备份={[f'{s:.6f}' for s in b_sorted]}, 当前={[f'{s:.6f}' for s in c_sorted]}")
        print(f"  可能存在轴交换（旋转90度类变换）")

    return result


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    backup_dir = os.path.join(script_dir, 'source_backup')

    print("=" * 80)
    print("灵云01号 - 升力电机STL文件深度对比分析")
    print("=" * 80)
    print(f"备份目录(SW原始): {backup_dir}")
    print(f"当前目录(原点化): {script_dir}")
    print(f"对比文件数: {len(FILE_MAPPING)}")

    results = {}

    for cn_name, en_name in FILE_MAPPING.items():
        backup_path = os.path.join(backup_dir, cn_name)
        current_path = os.path.join(script_dir, en_name)

        if not os.path.exists(backup_path):
            print(f"\n[错误] 备份文件不存在: {backup_path}")
            continue
        if not os.path.exists(current_path):
            print(f"\n[错误] 当前文件不存在: {current_path}")
            continue

        backup_info = analyze_single_stl(backup_path, cn_name)
        current_info = analyze_single_stl(current_path, en_name)

        result = compare_pair(backup_info, current_info)
        results[cn_name] = result

    # ============================================================
    # 总结
    # ============================================================
    print(f"\n\n{'=' * 80}")
    print("总结")
    print(f"{'=' * 80}")

    pure_translation_count = 0
    has_rotation_count = 0

    for cn_name, result in results.items():
        en_name = FILE_MAPPING[cn_name]
        if result['is_pure_translation']:
            pure_translation_count += 1
            print(f"  {cn_name} -> {en_name}: 纯平移 (旋转角度={result['rotation_angle_deg']:.6f}度)")
        else:
            has_rotation_count += 1
            print(f"  {cn_name} -> {en_name}: 存在旋转 (旋转角度={result['rotation_angle_deg']:.6f}度)")

    print(f"\n纯平移: {pure_translation_count}个, 存在旋转: {has_rotation_count}个")

    if has_rotation_count == 0:
        print("\n结论: 所有升力电机STL文件在原点化过程中只做了平移，没有旋转或缩放。")
    else:
        print("\n结论: 部分升力电机STL文件在原点化过程中存在旋转变换，需要进一步检查。")

    # 打印各文件的平移量汇总
    print(f"\n各文件平移量汇总:")
    print(f"  {'文件':<20} {'X偏移':>12} {'Y偏移':>12} {'Z偏移':>12} {'旋转角度(度)':>14}")
    print(f"  {'-' * 70}")
    for cn_name, result in results.items():
        en_name = FILE_MAPPING[cn_name]
        t = result['translation']
        print(f"  {en_name:<20} {t[0]:>+12.6f} {t[1]:>+12.6f} {t[2]:>+12.6f} {result['rotation_angle_deg']:>14.6f}")

    print(f"\n{'=' * 80}")
    print("分析完成")
    print(f"{'=' * 80}")


if __name__ == '__main__':
    main()
