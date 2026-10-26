#!/usr/bin/env python3
"""分析瞬态 CSV：定位 wheel 与 gyro 的偏差发生在哪一段。

对每份 CSV 回答三个问题：
1. wheel 轨迹是否平滑（有没有跳变/抖动）
2. wheel 与 gyro 的累计差在什么时刻开始拉开
3. 偏差是集中在起动/停止的瞬态，还是全程均匀分布
"""
import sys
import numpy as np


def load(p):
    rows = []
    with open(p) as f:
        hdr = f.readline().strip().split(',')
        for line in f:
            v = line.strip().split(',')
            if len(v) != len(hdr):
                continue
            rows.append(dict(zip(hdr, v)))
    return rows


def analyze(p):
    rows = load(p)
    t = np.array([float(r['t_rel']) for r in rows])
    ts = np.array([float(r['t_from_start']) for r in rows])
    te = np.array([float(r['t_from_stop']) for r in rows])
    yw = np.array([float(r['yaw_wheel_acc']) for r in rows])
    gy = np.array([float(r['gyro_acc']) for r in rows])
    gz = np.array([float(r['gyro_z']) for r in rows])
    still = np.array([int(r['still']) for r in rows])

    print('=' * 78)
    print('文件: %s   帧数 %d' % (p, len(rows)))
    print()
    print('【1】整体')
    print('  wheel 净 %+8.2f°   gyro 净 %+8.2f°   差 %+7.2f°'
          % (yw[-1], gy[-1], yw[-1] - gy[-1]))
    print('  wheel 轨迹单调性：相邻帧最大回退 %.3f°'
          % max(0.0, -np.min(np.diff(yw))))
    print('  gyro_z 范围 [%+.3f, %+.3f] rad/s' % (gz.min(), gz.max()))
    print('  静止帧占比 %.1f%%' % (100.0 * still.mean()))

    # 差分
    dt = np.diff(t)
    dt_safe = np.where((dt > 0) & (dt < 0.4), dt, np.nan)
    dw = np.diff(yw)
    dg = np.diff(gy)
    rate_w = dw / dt_safe
    rate_g = dg / dt_safe
    err = dw - dg          # 每帧 wheel 比 gyro 多走的度数
    cum = np.concatenate([[0], np.cumsum(np.nan_to_num(err))])

    print()
    print('【2】偏差累计曲线（起动后，每 0.5s 采样）')
    print('      t(s)    wheel_acc     gyro_acc      累计差      本段转速w/s')
    print('  ' + '-' * 68)
    tgt = -0.5
    for i in range(len(t)):
        if ts[i] < tgt:
            continue
        if ts[i] > 6.2:
            break
        rw = np.degrees(np.nan_to_num(gz[i]))
        print('  %+7.2f  %10.2f  %11.2f  %10.3f  %14.2f'
              % (ts[i], yw[i], gy[i], cum[i], rw))
        tgt += 0.5

    print()
    print('【3】偏差集中在哪里')
    spin = (ts >= 0) & (ts <= 5.0)
    decel = (ts > 5.0) & (te < 0)
    stopped = (te >= 0)
    pre = (ts < 0)
    for name, m in (('静止基线', pre), ('匀速转动', spin),
                    ('减速/停车', decel), ('完全静止', stopped)):
        if m.sum() < 2:
            continue
        idx = np.where(m)[0]
        d = np.nansum([err[max(0, i - 1)] for i in idx]) if len(idx) else 0.0
        print('  %-10s 帧 %4d   wheel−gyro 增量 %+8.3f°' % (name, len(idx), d))

    # 最大单帧偏差
    k = int(np.nanargmax(np.abs(err)))
    print()
    print('  最大单帧偏差 %.3f°  @ t_from_start=%+.2f s（wheel %.2f°/s，gyro %.2f°/s）'
          % (err[k], ts[k + 1], np.nan_to_num(rate_w[k]), np.nan_to_num(rate_g[k])))
    print()


if __name__ == '__main__':
    for p in sys.argv[1:]:
        analyze(p)
