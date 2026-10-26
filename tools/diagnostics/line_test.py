#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线 / 方向测试 —— 直接占用串口发命令，从 $T 帧判定车到底往哪走。

为什么要这个脚本：
  FWD,/BACK, 到底是不是"前进"一直没实车验证过（代码级可疑：前进速度被塞进 vy 横移槽）。
  靠肉眼看轮子会有歧义，这里直接读四轮转速符号 + odom 的 vx/vy，用数据说话。

$T 帧字段（按逗号 split 后的下标）：
  0  = "$T"            1  = tick_ms
  2..5   = enc_delta[0..3]  (FL,FR,RL,RR 原始增量)
  6..8   = accel xyz
  9..11  = gyro  xyz        (11 = gz, 0.01°/s)
  12..14 = mag   xyz
  15,16  = pose.x, pose.y   (mm)
  17     = heading          (0.1°)
  18,19  = pose.vx, pose.vy (mm/s)
  20     = pose.omega       (mrad/s)
  21..24 = wheel_rpm[0..3]  (0.1 RPM)

用法：
  python3 line_test.py --cmd "FWD,0.3"   --dur 3      # 架空验方向
  python3 line_test.py --cmd "V,0.15,0,0" --dur 3     # 落地测直线
  python3 line_test.py --cmd "VERSION"  --dur 0.5     # 只验链路
"""
import argparse
import time
import sys

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial: sudo apt install python3-serial")


def send(ser, cmd):
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


def collect(ser, dur, out):
    """读 dur 秒，把 $T 帧解析进 out。返回收到的帧数。"""
    t_end = time.time() + dur
    buf = b""
    n = 0
    while time.time() < t_end:
        try:
            chunk = ser.read(4096)
        except Exception as e:            # 串口掉线不要整个崩
            print("串口读异常: %s" % e)
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"$T,"):
                if line and not line.startswith(b"$"):
                    print("  [dev] %s" % line.decode("ascii", "replace"))
                continue
            try:
                f = line.split(b",")
                if len(f) < 25:
                    continue
                out.append({
                    "t":  time.time(),
                    "x":  int(f[15]) / 1000.0,
                    "y":  int(f[16]) / 1000.0,
                    "hdg": int(f[17]) / 10.0,
                    "vx": int(f[18]) / 1000.0,
                    "vy": int(f[19]) / 1000.0,
                    "om": int(f[20]) / 1000.0,
                    "rpm": [int(f[i]) / 10.0 for i in (21, 22, 23, 24)],
                })
                n += 1
            except (ValueError, IndexError):
                continue
    return n


def avg(vals):
    return sum(vals) / len(vals) if vals else 0.0


def verdict(rpm, vx, vy, om):
    """按四轮转速符号判定运动类型。"""
    fl, fr, rl, rr = rpm
    s = [1 if v > 1.0 else (-1 if v < -1.0 else 0) for v in (fl, fr, rl, rr)]
    if all(v == 0 for v in s):
        return "静止（四轮都没转）"
    # 前进: 四轮同号  左移: FL/RR 与 FR/RL 反号  旋转: 左侧轮与右侧轮反号
    if s[0] == s[1] == s[2] == s[3]:
        return "前进/后退（四轮同向 -> %s）" % ("前进" if s[0] > 0 else "后退")
    if s[0] == s[3] and s[1] == s[2] and s[0] != s[1]:
        return "横移（FL/RR 一组、FR/RL 另一组）-> %s" % ("左移" if s[0] < 0 else "右移")
    if s[0] == s[2] and s[1] == s[3] and s[0] != s[1]:
        return "原地旋转（左两轮一组、右两轮另一组）-> %s" % ("顺时针" if s[0] > 0 else "逆时针")
    return "混合（转速符号不一致，可能打滑或过渡态）"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--cmd", required=True, help="要发的命令，如 'FWD,0.3'")
    ap.add_argument("--dur", type=float, default=3.0, help="发命令后采样秒数")
    ap.add_argument("--pre", type=float, default=0.8, help="发命令前静止基线秒数")
    ap.add_argument("--post", type=float, default=0.6, help="STOP 后观察秒数")
    ap.add_argument("--no-stop", action="store_true", help="结束后不发 STOP")
    ap.add_argument("--warmup", type=float, default=0.8, help="开串口后先排空启动日志的秒数")
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    ser.reset_input_buffer()

    print("=" * 62)
    print("串口 %s @ %d" % (a.port, a.baud))

    # 预热：刚打开串口时可能正撞上固件启动日志（"[ 1] I2C ..."），
    # 若直接发命令会被拼进那半截未结束的行 -> 固件报 "ERR Unknown"。
    # 先空转一段把残留吐干净，再发一个空行冲掉断行。
    time.sleep(a.warmup)
    ser.reset_input_buffer()
    send(ser, "")
    time.sleep(0.15)
    ser.reset_input_buffer()

    # TELEM 是幂等的，发两次确保第一发若被吞掉也能生效
    send(ser, "TELEM ON")
    time.sleep(0.25)
    send(ser, "TELEM ON")
    send(ser, "VERSION")
    time.sleep(0.4)
    pre_msgs = []
    collect(ser, 0.4, pre_msgs)
    print("预热后收到 %d 帧遥测" % len(pre_msgs))

    # 仍无遥测 -> 再补一发（只看帧数，不重复打印）
    if len(pre_msgs) == 0:
        send(ser, "TELEM ON")
        time.sleep(0.3)
        pre_msgs = []
        collect(ser, 0.5, pre_msgs)
        print("补发 TELEM ON 后收到 %d 帧遥测" % len(pre_msgs))

    # --- 静止基线 ---
    send(ser, "STOP")
    time.sleep(0.2)
    ser.reset_input_buffer()
    base = []
    n = collect(ser, a.pre, base)
    if n == 0:
        print("!! 收不到 $T 帧。检查：TELEM 是否开启 / 串口是否被 serial_bridge 占用")
        ser.close()
        sys.exit(1)
    hdg0 = base[-1]["hdg"]
    x0, y0 = base[-1]["x"], base[-1]["y"]
    print("\n【基线 %.1fs】%d 帧" % (a.pre, n))
    print("  四轮rpm = %s" % "  ".join("%7.1f" % v for v in base[-1]["rpm"]))
    print("  x=%.3f y=%.3f hdg=%.1f  vx=%.3f vy=%.3f om=%.3f"
          % (x0, y0, hdg0, base[-1]["vx"], base[-1]["vy"], base[-1]["om"]))

    # --- 发命令 ---
    print("\n>>> 发送: %s   （采样 %.1fs）" % (a.cmd, a.dur))
    send(ser, a.cmd)
    motion = []
    t_start = time.time()
    n = collect(ser, a.dur, motion)
    if n == 0:
        print("!! 运动段收不到 $T 帧")
        ser.close()
        sys.exit(1)

    # --- 停止 ---
    if not a.no_stop:
        send(ser, "STOP")
    after = []
    collect(ser, a.post, after)

    # --- 结果 ---
    rpms = [[f["rpm"][i] for f in motion] for i in range(4)]
    mean_rpm = [avg(v) for v in rpms]
    vx_m = avg([f["vx"] for f in motion])
    vy_m = avg([f["vy"] for f in motion])
    om_m = avg([f["om"] for f in motion])
    dx = motion[-1]["x"] - x0
    dy = motion[-1]["y"] - y0
    dhdg = motion[-1]["hdg"] - hdg0
    while dhdg > 180:
        dhdg -= 360
    while dhdg < -180:
        dhdg += 360
    dist = (dx * dx + dy * dy) ** 0.5

    print("\n【运动段 %.1fs】%d 帧 (%.1f Hz)" % (a.dur, n, n / max(1e-6, motion[-1]["t"] - t_start)))
    print("  平均四轮rpm  FL=%8.1f  FR=%8.1f  RL=%8.1f  RR=%8.1f"
          % tuple(mean_rpm))
    print("  平均速度      vx=%+.4f m/s   vy=%+.4f m/s   om=%+.4f rad/s" % (vx_m, vy_m, om_m))
    print("  odom 位移     dx=%+.3f m   dy=%+.3f m   |d|=%.3f m" % (dx, dy, dist))
    print("  航向变化      dhdg=%+.2f°" % dhdg)

    print("\n【判定】%s" % verdict(mean_rpm, vx_m, vy_m, om_m))

    # 一致性检查：命令意图 vs 实际 odom 主轴
    c = a.cmd.upper()
    if c.startswith("FWD"):
        primary = "vx"
    elif c.startswith("BACK"):
        primary = "vx"
    elif c.startswith("V,"):
        try:
            parts = [float(x) for x in a.cmd.split(",")[1:4]]
            vals = {"vx": parts[0], "vy": parts[1], "om": parts[2]}
            primary = max(vals, key=lambda k: abs(vals[k]))
        except Exception:
            primary = "?"
    else:
        primary = "?"

    if primary == "vx":
        ratio = abs(vx_m) / max(1e-6, abs(vy_m))
        print("  期望主轴 = vx，实测 |vx|/|vy| = %.2f  ->  %s"
              % (ratio, "符合（确实沿 vx）" if ratio > 3 else "!! 不符合，实际主运动在 vy"))
    elif primary == "vy":
        ratio = abs(vy_m) / max(1e-6, abs(vx_m))
        print("  期望主轴 = vy，实测 |vy|/|vx| = %.2f  ->  %s"
              % (ratio, "符合（确实沿 vy）" if ratio > 3 else "!! 不符合，实际主运动在 vx"))
    elif primary == "om":
        print("  期望主轴 = omega，实测 om=%.4f rad/s" % om_m)
    print("=" * 62)

    ser.close()


if __name__ == "__main__":
    main()
