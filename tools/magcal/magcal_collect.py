#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁力计标定数据采集（2026-10-20）
原理：车只能绕 z 轴转，所以磁力计在机体系 xy 平面画一个"椭圆环"。
     硬磁干扰 → 圆心偏离原点；软磁干扰 → 圆被拉成椭圆并旋转。
     采集整圈数据后，用椭圆拟合求 (圆心 b, 2x2 矩阵 M)，令
         m_cal = M · (m_raw - b)
     即可把椭圆拉回以原点为圆心的正圆。

流程：静止基线 → 匀速原地转 N 圈 → 停止后基线
用法：python3 magcal_collect.py [port] [omega_rad_s] [seconds] [out.csv]

注意：直接占用串口，必须先停 serial_bridge。
"""
import sys
import time
import csv
import serial

port   = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS1'
omega  = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
dur    = float(sys.argv[3]) if len(sys.argv) > 3 else 63.0
out    = sys.argv[4] if len(sys.argv) > 4 else '/tmp/magcal.csv'

ser = serial.Serial(port, 460800, timeout=0.05, write_timeout=0.5)
ser.reset_input_buffer()
time.sleep(0.3)
# OTA 重启后遥测流默认是关的，必须先开（33ms ≈ 30Hz ASCII $T）
ser.write(b'TELEM,33\r\n')
time.sleep(0.3)
ser.reset_input_buffer()


def send(vx, vy, wz):
    ser.write(('V,%.3f,%.3f,%.3f\r\n' % (vx, vy, wz)).encode())


def parse(line):
    """$T 帧解析。字段: f[1]=tick f[6..8]=accel(mg) f[9..11]=gyro(0.01dps)
       f[12..14]=mag(0.01uT) f[17]=heading(0.1deg)"""
    try:
        f = line.decode('ascii', 'ignore').strip().split(',')
        if len(f) < 18 or f[0] != '$T':
            return None
        return (int(f[1]),
                int(f[6]), int(f[7]), int(f[8]),
                int(f[9]), int(f[10]), int(f[11]),
                int(f[12]), int(f[13]), int(f[14]),
                int(f[17]))
    except (ValueError, IndexError):
        return None


rows = []


def pump(seconds, vx, vy, wz, phase, label):
    """读 seconds 秒，期间每 100ms 重发一次速度命令（防固件看门狗停车）"""
    t_end = time.time() + seconds
    n0 = len(rows)
    last_cmd = 0.0
    while time.time() < t_end:
        now = time.time()
        if now - last_cmd > 0.1:
            send(vx, vy, wz)
            last_cmd = now
        line = ser.readline()
        if not line:
            continue
        p = parse(line)
        if p is not None:
            rows.append(p + (phase,))
    print('  阶段%d %s: %d 帧' % (phase, label, len(rows) - n0))


try:
    print('串口已打开 %s，开始采集（omega=%.3f rad/s, 旋转 %.0fs）' % (port, omega, dur))
    send(0, 0, 0)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print('[0] 静止基线 6s ...')
    pump(6.0, 0, 0, 0, 0, '静止基线')

    print('[1] 原地旋转 %.0fs ...' % dur)
    pump(dur, 0, 0, omega, 1, '旋转')

    print('[2] 停止后基线 6s ...')
    pump(6.0, 0, 0, 0, 2, '停止后')

finally:
    try:
        send(0, 0, 0)
        for _ in range(5):
            send(0, 0, 0)
            time.sleep(0.05)
    except Exception:
        pass
    ser.close()

hdr = ['tick', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 'hdg', 'phase']
with open(out, 'w', newline='') as fp:
    w = csv.writer(fp)
    w.writerow(hdr)
    w.writerows(rows)

print('共 %d 帧 → %s' % (len(rows), out))
