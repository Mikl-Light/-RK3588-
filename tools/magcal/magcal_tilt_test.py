#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证假设：磁航向误差的主因是"没有倾角补偿"，而非硬磁/软磁。
做法：用加速度计估计机体系下的地垂方向 g_hat，
      把磁场矢量投影到垂直于 g_hat 的平面内，再算方位角。
      与陀螺积分比对，看残差是否从 ~38° 掉到很小。
"""
import sys
import csv
import math
import numpy as np


def load(path):
    out = []
    with open(path) as fp:
        for d in csv.DictReader(fp):
            out.append({k: float(v) for k, v in d.items()})
    return out


def unwrap(seq):
    o = [seq[0]]
    for v in seq[1:]:
        d = v - o[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        o.append(o[-1] + d)
    return o


def lin_report(name, mag_deg, gyro_deg):
    g = np.array(gyro_deg)
    m = np.array(mag_deg)
    A = np.vstack([g, np.ones_like(g)]).T
    slope, off = np.linalg.lstsq(A, m, rcond=None)[0]
    res = m - (slope * g + off)
    print('  %-22s 斜率 %+.4f  残差RMS %6.2f°  最大 %6.2f°' % (
        name, slope, res.std(), np.abs(res).max()))
    return res.std()


def main(path):
    rows = [r for r in load(path) if r['phase'] == 1]
    ticks = np.array([r['tick'] for r in rows]) / 1000.0
    ax = np.array([r['ax'] for r in rows]) / 1000.0     # g
    ay = np.array([r['ay'] for r in rows]) / 1000.0
    az = np.array([r['az'] for r in rows]) / 1000.0
    gz = np.array([r['gz'] for r in rows]) / 100.0 * math.pi / 180.0
    m = np.column_stack([[r['mx'] for r in rows],
                         [r['my'] for r in rows],
                         [r['mz'] for r in rows]]) / 100.0

    # 陀螺积分
    gyro = [0.0]
    for i in range(1, len(rows)):
        dt = ticks[i] - ticks[i - 1]
        if dt <= 0 or dt > 0.5:
            dt = 0.035
        gyro.append(gyro[-1] + gz[i] * dt)
    gyro_deg = np.degrees(np.array(gyro))
    gyro_deg -= gyro_deg[0]

    print('=== 加速度计读数（判倾斜）===')
    print('  |a|: 均值 %.3f g  范围 [%.3f, %.3f]' % (
        np.linalg.norm(np.column_stack([ax, ay, az]), axis=1).mean(),
        np.linalg.norm(np.column_stack([ax, ay, az]), axis=1).min(),
        np.linalg.norm(np.column_stack([ax, ay, az]), axis=1).max()))
    print('  az: %.3f ± %.3f  [%.3f, %.3f] g  (偏离 1g → 有倾斜)' % (
        az.mean(), az.std(), az.min(), az.max()))
    print('  ax: %.3f ± %.3f   ay: %.3f ± %.3f g' % (
        ax.mean(), ax.std(), ay.mean(), ay.std()))
    tilt = np.degrees(np.arccos(np.clip(az / np.linalg.norm(
        np.column_stack([ax, ay, az]), axis=1), -1, 1)))
    print('  由 az 估计的倾角: %.2f ± %.2f°  [%.2f, %.2f]' % (
        tilt.mean(), tilt.std(), tilt.min(), tilt.max()))

    # 零倾角（固件现状）：north=my east=mx
    h_none = unwrap(list(np.degrees(np.arctan2(m[:, 0], m[:, 1]))))

    # 倾角补偿：把 B 投影到垂直重力平面
    g_hat = np.column_stack([ax, ay, az])
    g_hat /= np.linalg.norm(g_hat, axis=1, keepdims=True)
    bh = m - np.sum(m * g_hat, axis=1, keepdims=True) * g_hat
    # 参考基：取初始帧 bh 方向 e1，e2 = g_hat × e1
    # 若 g_hat 恒定，则 e1/e2 在世界系固定 → 角度即真实转角
    e1 = bh[0] / np.linalg.norm(bh[0])
    e2 = np.cross(g_hat[0], e1)
    e2 /= np.linalg.norm(e2)
    h_tilt = unwrap(list(np.degrees(np.arctan2(bh @ e2, bh @ e1))))

    # 用每帧自己的 g_hat 构造 e2（车体晃动时更准），e1 仍需固定
    e2t = np.cross(g_hat, np.tile(e1, (len(g_hat), 1)))
    e2t /= np.linalg.norm(e2t, axis=1, keepdims=True)
    h_tilt2 = unwrap(list(np.degrees(np.arctan2(
        np.sum(bh * e2t, axis=1), bh @ e1))))

    print('\n=== 磁方位角 vs 陀螺积分（残差越小越好）===')
    s1 = lin_report('零倾角(固件现状)', h_none, gyro_deg)
    s2 = lin_report('倾角补偿(固定g)', h_tilt, gyro_deg)
    s3 = lin_report('倾角补偿(逐帧g)', h_tilt2, gyro_deg)
    print('\n  改善: 零倾角 %.2f° → 逐帧倾角补偿 %.2f°  (降低 %.0f%%)' % (
        s1, s3, (1 - s3 / s1) * 100 if s1 > 0 else 0))

    # 用椭球标定后的值再试一次
    c = np.array([-1.0371, -0.8556])
    M = np.array([[1.013316, 0.002989], [0.002989, 0.987047]])
    m2 = m.copy()
    m2[:, :2] = (m[:, :2] - c) @ M.T
    bh2 = m2 - np.sum(m2 * g_hat, axis=1, keepdims=True) * g_hat
    e1b = bh2[0] / np.linalg.norm(bh2[0])
    e2b = np.cross(g_hat, np.tile(e1b, (len(g_hat), 1)))
    e2b /= np.linalg.norm(e2b, axis=1, keepdims=True)
    h_both = unwrap(list(np.degrees(np.arctan2(
        np.sum(bh2 * e2b, axis=1), bh2 @ e1b))))
    print()
    lin_report('椭球+倾角补偿', h_both, gyro_deg)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'magcal_run1.csv')
