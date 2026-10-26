#!/usr/bin/env python3
"""probe_serial.py — 探测 STM32 是否活着、遥测是否在流

用法（在车上跑）:
  python3 probe_serial.py                 # 只读，被动等 $T
  python3 probe_serial.py --wake          # 先发 TELEM ON 再读
  python3 probe_serial.py --port /dev/ttyS1 --sec 3

判读:
  - 读到 `$T,...` 行        → 固件活着，遥测已开
  - 读到 `OK ...`           → 串口通，命令被接受
  - 读到开机日志(STM32...)  → 固件刚复位过
  - 什么都没有              → STM32 未上电 / HSE-PLL 卡死 / 串口被占
"""
import argparse
import re
import sys
import time

import serial

TELEM_FIELDS = 30          # $T 帧字段数（v3.4.x）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyS1')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--sec', type=float, default=3.0, help='读多久')
    ap.add_argument('--wake', action='store_true', help='先发 TELEM ON')
    a = ap.parse_args()

    try:
        s = serial.Serial(a.port, a.baud, timeout=0.5)
    except Exception as e:
        print('打开串口失败: %s' % e)
        return 2

    s.reset_input_buffer()

    if a.wake:
        # 预热：先排空可能残留的开机日志，避免 "TELEM ON" 与日志粘连被当成非法命令
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b'TELEM ON\r\n')
        time.sleep(0.3)

    t0 = time.time()
    buf = b''
    while time.time() - t0 < a.sec:
        chunk = s.read(4096)
        if chunk:
            buf += chunk
    s.close()

    text = buf.decode('ascii', 'ignore')
    lines = [l.strip() for l in text.splitlines() if l.strip()]

    print('收到 %d 字节, %d 行' % (len(buf), len(lines)))
    if not lines:
        print('结果: 空 —— STM32 未上电 / 卡死 / 串口错')
        return 1

    tlines = [l for l in lines if l.startswith('$T')]
    print('\n--- 前 3 行原始 ---')
    for l in lines[:3]:
        print('  ' + l[:120])

    if tlines:
        last = tlines[-1]
        f = last.split(',')
        print('\n--- $T 帧 ---')
        print('  字段数 %d (期望 %d)' % (len(f) - 1, TELEM_FIELDS))
        print('  帧: ' + last[:140])
        # f[0]='$T'，其后为数据；按 MEMORY 记录的下标取关键量
        try:
            if len(f) > 24:
                print('  gz   f[11] = %s (0.01°/s)' % f[11])
                print('  mag  f[12..14] = %s,%s,%s (x100uT)' % (f[12], f[13], f[14]))
                print('  pos  f[15..16] = %s,%s (mm)' % (f[15], f[16]))
                print('  hdg  f[17] = %s (x10 deg)' % f[17])
                print('  rpm  f[21..24] = %s,%s,%s,%s' % (f[21], f[22], f[23], f[24]))
        except Exception as e:
            print('  解析字段异常: %s' % e)
        print('结果: 固件活着，遥测在流 (%d 帧 / %.1fs)' % (len(tlines), a.sec))
        return 0

    oks = [l for l in lines if l.startswith('OK') or l.startswith('ERR')]
    if oks:
        print('\n--- 应答 ---')
        for l in oks[:5]:
            print('  ' + l[:120])
        print('结果: 串口通，但没收到 $T —— 遥测未开，试 --wake')
        return 0

    boot = [l for l in lines if re.search(r'STM32|Boot|v3\.|init', l, re.I)]
    if boot:
        print('\n--- 疑似开机日志 ---')
        for l in boot[:5]:
            print('  ' + l[:120])
        print('结果: 固件刚复位过，日志在流但遥测未开')
        return 0

    print('结果: 收到数据但无法识别，内容如下')
    for l in lines[:5]:
        print('  ' + l[:120])
    return 0


if __name__ == '__main__':
    sys.exit(main())
