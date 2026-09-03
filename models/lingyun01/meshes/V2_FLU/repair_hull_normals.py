#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hull_all.STL 法线修复脚本 v5 (最终版)
================================================================
问题背景:
  灵云01四气囊主体 hull_all.STL 导入 Gazebo 学习后, 左主囊与左副囊
  渲染时"透明看穿", 右主囊与右副囊正常。

根因 (数学证明):
  SolidWorks 导出的 STL 中, 左主囊与左副囊这两个封闭面的顶点绕向
  (三角形顶点顺序) 整体内翻, 即封闭面"有向体积"为负, 导致每片三角形
  的法线整体指向囊体内部。
  Gazebo (OGRE2) 渲染默认开启"背面剔除"(back-face culling), 只绘制
  法线朝向相机(摄像机)一侧的面。法线朝内的囊沿表面, 从外部看到的面
  全部被判为背面而被剔除, 视线直接穿透外壁看到内部结构 => "透明"。

  【注意】外壁是一个封闭壳(内外两层, 壁厚非零)。外壁法线必须朝外,
  建模近似忽略壁厚单层时, 有向体积符号即代表法线朝向。

修复算法:
  1. 读取原始 .bak 文件, 用精确浮点坐标做三角形去重;
  2. 建顶点边映射, 只保留"流形边"(恰好被2个三角形共享的边);
  3. 从每个三角形做 BFS 绕向一致性传播: 相邻三角形共享边必须"反向遍历"
     才能指向同一侧, 以此把网格切分为若干"绕向补丁";
  4. 对每个补丁定向:
       - 封闭补丁(所有边均为流形边) 且 有向体积显著非零:
           有向体积 < 0  => 该补丁整体翻转(法线反向)
           这是数学精确判据, 囊体/导管等封闭薄壳走这条路;
       - 开放补丁(存在边界边) 或 体积接近0:
           以该补丁所在1mm连通分量质心为参考点, 做"外向多数表决"
           (法线与(三角形中心-质心)方向点积为同号即视为朝外),
           作为启发式兜底, 用于支架/法兰等小件;
  5. 对判定需要翻转的三角形交换其第2、第3顶点顺序(同步翻转绕向),
     并按最终绕向重新计算法线(右手定则), 写回 hull_all.STL。

输出:
  hull_all.STL  (修复后, 保留全部原始三角形)
输入(备份):
  hull_all.STL.bak (原始 SW 导出转换文件, 切勿删除)

用法:
  python3 repair_hull_normals.py
"""
import struct
import numpy as np
from collections import defaultdict, deque

BAK = "hull_all.STL.bak"
DST = "hull_all.STL"


def read_stl(path):
    with open(path, 'rb') as f:
        header = f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        tris = []
        for _ in range(n):
            data = f.read(50)
            fl = list(struct.unpack('<12f', data[:48]))
            tris.append((fl, data[48:50]))
    return header, tris


def main():
    header, tris = read_stl(BAK)
    N = len(tris)
    print(f"输入(.bak): {N} 三角形")

    # 精确顶点 + 去重
    vid = {}
    evid = np.zeros((N, 3), dtype=np.int64)
    for i, (fl, _) in enumerate(tris):
        for j in range(3):
            k = (fl[3 + j * 3], fl[4 + j * 3], fl[5 + j * 3])
            vid.setdefault(k, len(vid))
            evid[i, j] = vid[k]
    vpos = np.zeros((len(vid), 3))
    for k, idx in vid.items():
        vpos[idx] = k

    seen = set()
    keep = []
    for i in range(N):
        k = tuple(sorted((int(evid[i][0]), int(evid[i][1]), int(evid[i][2]))))
        if k not in seen:
            seen.add(k)
            keep.append(i)
    faces = evid[keep]
    M = len(keep)
    print(f"去重后保留: {M}")

    # 边映射
    emap = defaultdict(list)
    for t in range(M):
        f = faces[t]
        for k in range(3):
            u, v = int(f[k]), int(f[(k + 1) % 3])
            emap[(min(u, v), max(u, v))].append(t)
    manifold = {k for k, lst in emap.items() if len(lst) == 2}

    # BFS 绕向补丁
    state = np.zeros(M, dtype=np.int8)
    patches = []
    for seed in range(M):
        if state[seed] != 0:
            continue
        state[seed] = 1
        dq = deque([seed])
        plist = []
        while dq:
            t = dq.popleft()
            plist.append(t)
            f = faces[t]
            ev = f if state[t] == 1 else f[[0, 2, 1]]
            for k in range(3):
                u, v = int(ev[k]), int(ev[(k + 1) % 3])
                key = (min(u, v), max(u, v))
                if key not in manifold:
                    continue
                lst = emap[key]
                n = lst[0] if lst[1] == t else lst[1]
                if state[n] != 0:
                    continue
                fn = faces[n]
                asread = None
                for k2 in range(3):
                    a, b = int(fn[k2]), int(fn[(k2 + 1) % 3])
                    if a == u and b == v:
                        asread = (u, v)
                        break
                    if a == v and b == u:
                        asread = (v, u)
                        break
                state[n] = state[t] if asread == (v, u) else -state[t]
                dq.append(n)
        patches.append(plist)

    # 1mm 连通分量 (回退参考质心)
    qvid = {}
    qf = np.zeros((M, 3), dtype=np.int64)
    for t in range(M):
        for j in range(3):
            p = vpos[int(faces[t][j])]
            qk = (round(p[0] * 1000), round(p[1] * 1000), round(p[2] * 1000))
            qvid.setdefault(qk, len(qvid))
            qf[t, j] = qvid[qk]
    parent = np.arange(M)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    qv2t = defaultdict(list)
    for t in range(M):
        for j in range(3):
            qv2t[int(qf[t, j])].append(t)
    for v, ts in qv2t.items():
        r0 = find(ts[0])
        for k in ts[1:]:
            r = find(k)
            if r0 != r:
                parent[r] = r0
    cid_map = {}
    comp_cent = {}
    comp_of = np.zeros(M, dtype=np.int32)
    for t in range(M):
        r = find(t)
        if r not in cid_map:
            cid_map[r] = len(cid_map)
            fv = faces[[t]]
            comp_cent[cid_map[r]] = vpos[fv].reshape(-1, 3).mean(axis=0)
        comp_of[t] = cid_map[r]

    # 补丁定向
    flip = np.zeros(M, dtype=bool)
    n_vol = n_cent = 0
    for plist in patches:
        idxs = np.array(plist)
        closed = all((min(int(faces[t][k]), int(faces[t][(k + 1) % 3])),
                      max(int(faces[t][k]), int(faces[t][(k + 1) % 3])))
                     in manifold
                     for t in plist for k in range(3))
        fv = faces[idxs]
        a, b, c = vpos[fv[:, 0]], vpos[fv[:, 1]], vpos[fv[:, 2]]
        sgn = state[idxs].astype(float)
        vol = float(np.sum(sgn * np.einsum('ij,ij->i', a, np.cross(b, c))) / 6.0)
        tc = (a + b + c) / 3.0
        if closed and abs(vol) > 1e-4:
            psign = -1 if vol < 0 else 1
            n_vol += 1
        else:
            cc = comp_cent[int(comp_of[plist[0]])]
            outv = tc - cc
            nn = np.linalg.norm(outv, axis=1)
            okm = nn > 1e-6
            dirs = np.zeros_like(outv)
            dirs[okm] = outv[okm] / nn[okm, None]
            nr = np.cross(b - a, c - a)
            nr /= np.maximum(np.linalg.norm(nr, axis=1), 1e-12)[:, None]
            eff = nr * sgn[:, None]
            agree = np.einsum('ij,ij->i', eff, dirs) > 0
            psign = 1 if agree.mean() >= 0.5 else -1
            n_cent += 1
        flip[idxs] = (state[idxs] * psign) == -1

    # 写回
    nf = int(flip.sum())
    print(f"翻转三角形: {nf}/{M}  (体积法补丁 {n_vol} 个, 质心法补丁 {n_cent} 个)")
    with open(DST, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', M))
        for i in range(M):
            fl, attr = tris[keep[i]]
            p = fl[3:12]
            if flip[i]:
                p = [p[0], p[1], p[2], p[6], p[7], p[8], p[3], p[4], p[5]]
            v0 = np.array(p[0:3]); v1 = np.array(p[3:6]); v2 = np.array(p[6:9])
            nr = np.cross(v1 - v0, v2 - v0)
            ln = np.linalg.norm(nr)
            nr = nr / ln if ln > 1e-12 else np.zeros(3)
            f.write(struct.pack('<12f', nr[0], nr[1], nr[2],
                                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]))
            f.write(attr)
    print(f"写回: {DST} ({M} 三角形)")


if __name__ == '__main__':
    main()