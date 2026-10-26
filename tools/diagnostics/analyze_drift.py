#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_drift.py — 分析 /fusion_diag 采集日志里"静止期间"的 yaw 漂移

用法：python3 analyze_drift.py /tmp/fd2.log
日志由 `ros2 topic echo /fusion_diag --field data` 生成，每帧一行 JSON，帧间以 --- 分隔。
"""
import json
import sys


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def main(path):
    rows = []
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                d = json.loads(line.rstrip('-'))
            except Exception:
                continue
            rows.append(d)

    if not rows:
        print('没有解析到任何数据')
        return

    t0 = rows[0]['t']
    print('样本数: %d   时长: %.1f s' % (len(rows), rows[-1]['t'] - t0))
    print()

    # 只统计静止帧
    still = [r for r in rows if r.get('still')]
    print('静止帧: %d / %d' % (len(still), len(rows)))
    if not still:
        print('警告：没有 still=true 的帧，车可能在动')
        return

    t_ref = still[0]
    yf0 = t_ref['yaw_fused_deg']
    yh0 = t_ref['yaw_wheel_deg']

    print()
    print('时刻(s)  yaw_fused  yaw_wheel  融合-轮式   mag_outlier')
    print('-' * 62)
    last_print = -999.0
    for r in still:
        dt = r['t'] - t_ref['t']
        if dt - last_print < 10.0:
            continue
        last_print = dt
        print('%7.1f  %9.3f  %9.3f  %9.3f   %s'
              % (dt, r['yaw_fused_deg'], r['yaw_wheel_deg'],
                 wrap180(r['yaw_fused_deg'] - r['yaw_wheel_deg']),
                 r.get('mag_outlier')))

    yf_last = still[-1]['yaw_fused_deg']
    yh_last = still[-1]['yaw_wheel_deg']
    dur = still[-1]['t'] - t_ref['t']
    print()
    print('=== 静止 %.1f s 结果 ===' % dur)
    print('yaw_fused 变化量 : %+.3f °' % wrap180(yf_last - yf0))
    print('yaw_wheel 变化量 : %+.3f °' % wrap180(yh_last - yh0))
    print('融合-轮式 终值偏差: %+.3f °' % wrap180(yf_last - yh_last))
    if dur > 1:
        print('等效漂移率      : %+.4f °/min' % (wrap180(yf_last - yf0) / dur * 60.0))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/tmp/fd2.log')
