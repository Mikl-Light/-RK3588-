#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁力计 2D 椭圆标定分析（2026-10-20）
输入：magcal_collect.py 采的 CSV（tick,ax,ay,az,gx,gy,gz,mx,my,mz,hdg,phase）
输出：
  1) 静止基线噪声
  2) 椭圆拟合结果：硬磁偏移 b(2x2)、软磁矩阵 M(2x2)、轴比、旋转角
  3) 标定前后 磁方位角 vs 陀螺积分角 的线性关系（斜率应≈1）
  4) 每次过零闭环误差
用法：python3 magcal_analyze.py magcal_run1.csv
"""
import sys
import csv
import math
import numpy as np


def load(path):
    rows = []
    with open(path) as fp:
        r = csv.DictReader(fp)
        for d in r:
            rows.append({k: float(v) for k, v in d.items()})
    return rows


def fit_ellipse(x, y):
    """直接最小二乘（齐次 SVD）拟合二次曲线
       A x^2 + B xy + C y^2 + D x + E y + F = 0
       返回 (center(2), S(2x2), r, 椭圆参数)"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    # 数值归一化，避免 x^2 量级差
    xm, ym = x.mean(), y.mean()
    xs, ys = x.std(), y.std()
    xn = (x - xm) / xs
    yn = (y - ym) / ys
    Z = np.column_stack([xn * xn, xn * yn, yn * yn, xn, yn, np.ones_like(xn)])
    # 齐次最小二乘：求最小奇异向量
    _, _, Vt = np.linalg.svd(Z, full_matrices=False)
    A, B, C, D, E, F = Vt[-1]
    S = np.array([[A, B / 2.0], [B / 2.0, C]])
    d = np.array([D, E])
    if abs(np.linalg.det(S)) < 1e-14:
        raise ValueError('拟合退化，数据可能共线')
    # u_n = T (u - m0)，方程 u_n^T S u_n + d^T u_n + F = 0
    # 展开后对绝对坐标 u 的二次型矩阵仍是 S_o = T^T S T，
    # 但圆心是 "相对 m0 的偏移" c_rel，绝对圆心必须加回 m0。
    # 原实现漏加 m0 —— 导致圆心被错报成 (-1.0, -0.9) 而不是真实的 (~15, -3)。
    T = np.diag([1.0 / xs, 1.0 / ys])
    S_o = T.T @ S @ T
    d_o = T.T @ d
    c_rel = -0.5 * np.linalg.solve(S_o, d_o)
    r_o = float(c_rel @ S_o @ c_rel - F)
    c_abs = np.array([xm, ym]) + c_rel
    return c_abs, S_o, r_o


def ellipse_to_matrix(c, S, r):
    """把 u^T S u = r 转成 归一化线性变换 M 使得 ||M(u-c)|| = avg_radius
       返回 (M, 半轴 a,b, 主轴方向 deg)"""
    # S 正定化（S 与 -S 等价，取使特征值为正的符号）
    evals, evecs = np.linalg.eigh(S)
    if np.all(evals < 0):
        evals = -evals
        S = -S
    elif np.any(evals < 0):
        raise ValueError('拟合结果非椭圆: 特征值符号不一致 %s' % evals)
    # u^T S u = r 的半轴 = sqrt(r/eval)
    a = math.sqrt(r / evals[0])
    b = math.sqrt(r / evals[1])
    avg = (a + b) / 2.0
    # M 使 ||M u|| = avg  (u^T S u = r  ->  u^T (S/r) avg^2 u = avg^2)
    # 正确写法：M = avg/sqrt(r) * Λ^(1/2) V^T （不是 V Λ^(1/2) V^T）
    M = np.diag(np.sqrt(evals / r)) @ evecs.T * avg
    # 软磁矩阵物理上必须是"缩放+旋转"，det>0；若拟合给出镜像(det<0)，
    # 翻一个轴消除反射（圆映射到圆，翻轴只让角度反号，不破坏圆度）
    if np.linalg.det(M) < 0:
        M[:, 1] = -M[:, 1]
    ang = math.degrees(math.atan2(evecs[1, 0], evecs[0, 0]))
    return M, a, b, ang, evecs, evals


def unwrap(seq):
    out = [seq[0]]
    for v in seq[1:]:
        d = v - out[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    return out


def main(path):
    rows = load(path)
    print('总帧数 %d' % len(rows))

    base = [r for r in rows if r['phase'] == 0]
    rot = [r for r in rows if r['phase'] == 1]
    post = [r for r in rows if r['phase'] == 2]
    print('阶段: 基线 %d / 旋转 %d / 停止后 %d' % (len(base), len(rot), len(post)))

    # ---------- 1. 静止基线 ----------
    for name, seg in (('开机基线', base), ('停止后', post)):
        if not seg:
            continue
        for ax, lbl in ((('mx', 'my', 'mz'), 'mag'),):
            vals = {k: np.array([r[k] for r in seg]) / 100.0 for k in ax}
            print('%s %s: mx=%.2f±%.2f my=%.2f±%.2f mz=%.2f±%.2f µT' % (
                name, lbl,
                vals['mx'].mean(), vals['mx'].std(),
                vals['my'].mean(), vals['my'].std(),
                vals['mz'].mean(), vals['mz'].std()))

    # ---------- 2. 椭圆拟合 ----------
    mx = np.array([r['mx'] for r in rot]) / 100.0
    my = np.array([r['my'] for r in rot]) / 100.0
    mz = np.array([r['mz'] for r in rot]) / 100.0
    print('\n旋转段: mx=[%.1f,%.1f] my=[%.1f,%.1f] mz=[%.1f,%.1f] µT' % (
        mx.min(), mx.max(), my.min(), my.max(), mz.min(), mz.max()))
    print('mz 变化幅度 %.2f µT（大→车体倾斜或软磁 3D 串扰）' % (mz.max() - mz.min()))

    c, S, r = fit_ellipse(mx, my)
    M, a, b, ang, evecs, evals = ellipse_to_matrix(c, S, r)
    print('\n=== 椭圆拟合 ===')
    print('硬磁圆心 b = (%+.3f, %+.3f) µT   偏移模 %.3f µT' % (c[0], c[1], np.hypot(*c)))
    print('长半轴 a=%.3f  短半轴 b=%.3f  轴比 b/a=%.4f' % (a, b, min(a, b) / max(a, b)))
    print('主轴方向 %.1f°' % ang)
    print('软磁矩阵 M =')
    print('   [%+.5f %+.5f]' % (M[0, 0], M[0, 1]))
    print('   [%+.5f %+.5f]' % (M[1, 0], M[1, 1]))
    print('地磁水平分量估计 avg_radius = %.2f µT' % ((a + b) / 2.0))

    # ---------- 3. 标定前/后 磁方位角 vs 陀螺积分 ----------
    # 陀螺积分（每帧用与上一帧的 dt）
    ticks = np.array([r['tick'] for r in rot], dtype=float) / 1000.0
    gz = np.array([r['gz'] for r in rot]) / 100.0 * math.pi / 180.0   # rad/s
    gyro_yaw = [0.0]
    for i in range(1, len(rot)):
        dt = ticks[i] - ticks[i - 1]
        if dt <= 0 or dt > 0.5:
            dt = 0.035
        gyro_yaw.append(gyro_yaw[-1] + gz[i] * dt)
    gyro_deg = np.degrees(np.array(gyro_yaw))
    gyro_deg -= gyro_deg[0]

    def mag_deg(px, py):
        # 与固件 MagHeadingDeg 同口径: north=my, east=mx
        return np.degrees(np.arctan2(px, py))

    raw = mag_deg(mx, my)
    cal = np.array([M @ (np.array([x, y]) - c) for x, y in zip(mx, my)])
    cald = mag_deg(cal[:, 0], cal[:, 1])

    for lbl, seq in (('标定前', unwrap(list(raw))), ('标定后', unwrap(list(cald)))):
        n = len(seq)
        g = gyro_deg - gyro_deg[0]
        # 线性回归 slope = d(mag)/d(gyro)
        A = np.vstack([g, np.ones_like(g)]).T
        slope, off = np.linalg.lstsq(A, np.array(seq), rcond=None)[0]
        resid = np.array(seq) - (slope * g + off)
        print('\n=== %s 磁方位角 vs 陀螺积分 ===' % lbl)
        print('  斜率 d(mag)/d(gyro) = %+.4f   (目标 1.0000)' % slope)
        print('  残差 RMS = %.2f°   最大 %.2f°   峰峰 %.2f°' % (
            resid.std(), np.abs(resid).max(), resid.max() - resid.min()))
        print('  总转角: 磁 %+.1f°  陀螺 %+.1f°' % (seq[-1] - seq[0], g[-1] - g[0]))

    # ---------- 4. 闭环检查 ----------
    # 标定健康度：水平分量模长应恒定
    r_raw = np.hypot(mx, my)
    r_cal = np.hypot(cal[:, 0], cal[:, 1])
    print('\n=== 水平分量模长 |m_xy|（理想应恒定）===')
    print('  标定前: %.2f ± %.2f µT  [%.2f, %.2f]  波动 %.0f%%' % (
        r_raw.mean(), r_raw.std(), r_raw.min(), r_raw.max(),
        (r_raw.max() - r_raw.min()) / r_raw.mean() * 100))
    print('  标定后: %.2f ± %.2f µT  [%.2f, %.2f]  波动 %.0f%%' % (
        r_cal.mean(), r_cal.std(), r_cal.min(), r_cal.max(),
        (r_cal.max() - r_cal.min()) / r_cal.mean() * 100))
    # 静止基线与停止后基线应当在同一姿态（转整数圈）
    if base and post:
        b0 = np.array([np.mean([r['mx'] for r in base]), np.mean([r['my'] for r in base])]) / 100.0
        b1 = np.array([np.mean([r['mx'] for r in post]), np.mean([r['my'] for r in post])]) / 100.0
        print('\n=== 闭环（转 %d 圈后回到原位）===' % round(abs(gyro_deg[-1]) / 360.0))
        print('  标定前 起点(%.2f,%.2f) 终点(%.2f,%.2f)  差 %.2f µT / %.1f°'
              % (b0[0], b0[1], b1[0], b1[1], np.linalg.norm(b1 - b0),
                 math.degrees(abs(raw[-1] - raw[0]))))
        c0 = M @ (b0 - c)
        c1 = M @ (b1 - c)
        print('  标定后 起点(%.3f,%.3f) 终点(%.3f,%.3f)  差 %.3f (归一化) / %.1f°'
              % (c0[0], c0[1], c1[0], c1[1], np.linalg.norm(c1 - c0),
                 abs(math.degrees(math.atan2(c1[0], c1[1]) - math.atan2(c0[0], c0[1])))))

    # ---------- 5. 导出参数 ----------
    print('\n=== 固件参数（m_cal = M * (m_raw - b)）===')
    print('#define MAG_HARD_X   (%+.4ff)' % c[0])
    print('#define MAG_HARD_Y   (%+.4ff)' % c[1])
    print('#define MAG_SOFT_00   (%+.6ff)' % M[0, 0])
    print('#define MAG_SOFT_01   (%+.6ff)' % M[0, 1])
    print('#define MAG_SOFT_10   (%+.6ff)' % M[1, 0])
    print('#define MAG_SOFT_11   (%+.6ff)' % M[1, 1])


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'magcal_run1.csv')
