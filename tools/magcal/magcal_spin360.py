#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环判圈验证：用磁航向(HDG)自己判断"转满 360°"，到点自动停车。
判据完全来自磁力计（不使用陀螺），这是 2026-10-18 约定的判圈标准。
用法：python3 magcal_spin360.py [port] [omega]
"""
import sys
import time
import serial

port  = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS1'
omega = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30

ser = serial.Serial(port, 460800, timeout=0.05, write_timeout=0.5)
ser.reset_input_buffer()
time.sleep(0.3)
ser.write(b'TELEM,33\r\n')
time.sleep(0.3)
ser.reset_input_buffer()


def send(s):
    ser.write((s + '\r\n').encode())


def read_hdg():
    """返回 (tick_s, hdg_deg) 或 None"""
    line = ser.readline()
    if not line.startswith(b'$T,'):
        return None
    try:
        f = line.decode('ascii', 'ignore').strip().split(',')
        return int(f[1]) / 1000.0, int(f[17]) / 10.0
    except (ValueError, IndexError):
        return None


def wrap(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


try:
    # 1) 静止基线
    send('V,0,0,0')
    base = []
    t_end = time.time() + 2.0
    while time.time() < t_end:
        r = read_hdg()
        if r:
            base.append(r[1])
    hdg0 = sum(base) / len(base)
    print('起始磁航向 HDG0 = %.1f°  (基线 %d 帧，峰峰 %.2f°)' % (
        hdg0, len(base), max(base) - min(base)))

    # 2) 开始转，用磁航向累计判满 360°
    send('V,0,0,%.3f' % omega)
    t_start = time.time()
    total = 0.0
    last = hdg0
    hdg_break = hdg0
    last_cmd = time.time()
    frames = 0
    while True:
        if time.time() - last_cmd > 0.1:
            send('V,0,0,%.3f' % omega)
            last_cmd = time.time()
        r = read_hdg()
        if r is None:
            if time.time() - t_start > 120:
                print('超时（120s）未满 360°，累计 %.1f°' % total)
                break
            continue
        _, hdg = r
        total += wrap(hdg - last)
        last = hdg
        frames += 1
        if frames % 60 == 0:
            print('  t=%5.1fs  累计 %+7.1f°  HDG=%7.1f°' % (
                time.time() - t_start, total, hdg))
        if total >= 360.0:
            hdg_break = hdg
            break

    # 3) 停车
    send('V,0,0,0')
    for _ in range(5):
        send('V,0,0,0')
        time.sleep(0.04)
    t_stop = time.time()
    print('判满 360° → 已下停车命令（累计 %+.1f°，用时 %.1fs）' % (total, t_stop - t_start))

    # 4) 停稳后读 3 秒
    time.sleep(0.5)
    post = []
    t_end = time.time() + 3.0
    while time.time() < t_end:
        r = read_hdg()
        if r:
            post.append(r[1])
    hdg1 = sum(post) / len(post)

    print()
    print('=== 结果 ===')
    print('  起始 HDG0 = %.1f°' % hdg0)
    print('  判满时刻 HDG = %.1f°   （应≈HDG0，差 %+.2f° = 判圈准确度）' % (
        hdg_break, wrap(hdg_break - hdg0)))
    print('  停止 HDG1 = %.1f°  (停稳后 %d 帧，峰峰 %.2f°)' % (
        hdg1, len(post), max(post) - min(post)))
    overshoot = wrap(hdg1 - hdg_break)
    print('  停车过冲  = %+.1f°   （判满→停稳期间继续转，属控制延迟非磁力计）' % overshoot)
    print('  实际总转角 = %+.1f°   (目标 360°)' % (360.0 + overshoot))

finally:
    try:
        for _ in range(6):
            send('V,0,0,0')
            time.sleep(0.05)
    except Exception:
        pass
    ser.close()
