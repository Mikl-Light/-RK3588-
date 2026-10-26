#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前进时航向假漂移的来源分离 —— 陀螺？磁力计？还是轮速差？

问题背景（2026-10-22）：
  直线前进 3s，odom 的 heading 变了 3.8°，约 1.27°/s。
  但激光点云几何互相关显示车走得很直（dyaw = 0.00°，两次都是）。
  → 3.8° 是假偏航。走 10m 会歪 12.7°，导航直接跑偏。

三个嫌疑人，必须分开，不能猜：
  A. **陀螺 z 轴零偏**在电机运行时变了（振动 / 温升 / 电流干扰 / ZARU 校的是静止态）
  B. **磁力计**在电机电流磁场下被拉偏，互补滤波把错航向吸进 yaw
  C. **四轮真差速**（机械阻力 / 装配），车确实拧了一下，只是点云没分辨出来

采集分三段：
  ① 静置 4s  —— 基线：陀螺零偏、磁力计读数、HDG 是否自己会漂
  ② 前进 3s  —— 运动段：看 gz / mag 是否偏离基线
  ③ 静置 4s  —— 停止后：看是否回落到基线（磁滞、温漂会留下痕迹）

判读规则：
  ∫gz dt ≈ ΔHDG                 → 纯陀螺问题（A）
  ∫gz dt ≈ 0 而 ΔHDG 明显      → 是磁修正拉进去的（B）
  四轮 rpm 相对差 > 1%          → 可能是真转（C）

$T 帧字段（逗号 split 后下标）：
  11 = gz (0.01°/s)      12..14 = mag xyz (0.01µT)
  17 = heading (0.1°)    18,19  = vx,vy (mm/s)   20 = omega (mrad/s)
  21..24 = wheel_rpm (0.1 RPM)

用法：
  python3 ytrack.py                 # 静置 4s + 前进 3s + 静置 4s
  python3 ytrack.py --speed 0.05 --dur 3
  python3 ytrack.py --still-only    # 只测静止段，看零偏本身稳不稳
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


def parse(f):
    """$T 逗号段 → 字典。字段不够或非数字返回 None。"""
    try:
        if len(f) < 25:
            return None
        return {
            "gz": int(f[11]) / 100.0,          # °/s
            "mx": int(f[12]) / 100.0,          # µT
            "my": int(f[13]) / 100.0,
            "mz": int(f[14]) / 100.0,
            "hdg": int(f[17]) / 10.0,          # °
            "vx": int(f[18]) / 1000.0,         # m/s
            "vy": int(f[19]) / 1000.0,
            "om": int(f[20]) / 1000.0,         # rad/s
            "rpm": [int(f[i]) / 10.0 for i in range(21, 25)],
        }
    except (ValueError, IndexError):
        return None


def collect(ser, dur):
    """读 dur 秒，返回 [(t, rec), ...]"""
    rows = []
    buf = b""
    t0 = time.time()
    while time.time() - t0 < dur:
        try:
            chunk = ser.read(4096)
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"$T,"):
                continue
            rec = parse(line.split(b","))
            if rec is not None:
                rows.append((time.time() - t0, rec))
    return rows


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def sd(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5


def wrap180(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def report(name, rows, extra=""):
    if not rows:
        print("  !! %s 没有收到帧" % name)
        return None
    recs = [r for _, r in rows]
    gzs = [r["gz"] for r in recs]
    hdgs = [r["hdg"] for r in recs]

    # 陀螺积分：Δθ = Σ gz·dt（用相邻帧时间差）
    integ = 0.0
    for i in range(1, len(rows)):
        integ += recs[i]["gz"] * (rows[i][0] - rows[i - 1][0])
    d_hdg = wrap180(hdgs[-1] - hdgs[0])

    print("  %s  帧数 %d  时长 %.2fs" % (name, len(recs), rows[-1][0]))
    print("    gz     均值 %+7.3f °/s   标准差 %.3f   范围 %+.2f ~ %+.2f"
          % (mean(gzs), sd(gzs), min(gzs), max(gzs)))
    print("    ∫gz·dt = %+7.3f °        ΔHDG = %+7.3f °     差 %+.3f °"
          % (integ, d_hdg, d_hdg - integ))
    for ax, k in (("x", "mx"), ("y", "my"), ("z", "mz")):
        v = [r[k] for r in recs]
        print("    mag%-2s  均值 %+7.2f µT   标准差 %5.2f" % (ax, mean(v), sd(v)))
    rpm = [[r["rpm"][i] for r in recs] for i in range(4)]
    m4 = [mean(c) for c in rpm]
    spread = (max(m4) - min(m4)) / max(1e-6, abs(mean(m4))) * 100.0
    print("    rpm    FL %6.2f  FR %6.2f  RL %6.2f  RR %6.2f   相对差 %.2f%%"
          % (m4[0], m4[1], m4[2], m4[3], spread))
    print("    vx %+.4f  vy %+.4f  om %+.4f" % (mean([r["vx"] for r in recs]),
                                               mean([r["vy"] for r in recs]),
                                               mean([r["om"] for r in recs])))
    if extra:
        print("    %s" % extra)
    return {"integ": integ, "d_hdg": d_hdg, "gzs": gzs,
            "mag": [(mean([r["mx"] for r in recs]),
                     mean([r["my"] for r in recs]),
                     mean([r["mz"] for r in recs]))],
            "mag_sd": (sd([r["mx"] for r in recs]),
                       sd([r["my"] for r in recs]),
                       sd([r["mz"] for r in recs])),
            "rpm": m4, "spread": spread}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--speed", type=float, default=0.1, help="前进速度 m/s")
    ap.add_argument("--dur", type=float, default=3.0, help="运动段时长")
    ap.add_argument("--pre", type=float, default=4.0, help="运动前静置时长")
    ap.add_argument("--post", type=float, default=4.0, help="运动后静置时长")
    ap.add_argument("--still-only", action="store_true", help="只测静止段")
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    ser.reset_input_buffer()
    time.sleep(0.5)                 # 等开机日志吐完（半截行会和命令粘在一起）
    ser.reset_input_buffer()
    send(ser, "TELEM ON")
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("=" * 66)
    print("① 静置基线 %.1fs —— 陀螺零偏 / 磁力计基线 / HDG 自己会不会漂" % a.pre)
    p1 = report("静置A", collect(ser, a.pre))

    p2 = p3 = None
    if not a.still_only:
        print("\n② 前进 %.2fs  v=%.2f m/s" % (a.dur, a.speed))
        send(ser, "V,%.3f,0,0" % a.speed)
        try:
            p2 = report("运动", collect(ser, a.dur))
        finally:
            send(ser, "STOP")
            send(ser, "STOP")

        print("\n   （停车，等 1.5s 让车身停稳）")
        time.sleep(1.5)
        ser.reset_input_buffer()

        print("\n③ 静置 %.1fs —— 停止后是否回落到基线" % a.post)
        p3 = report("静置B", collect(ser, a.post))
    ser.close()

    if not (p1 and p2):
        return

    print("\n" + "=" * 66)
    print("【判读】")
    d_int = p2["integ"]
    d_hdg = p2["d_hdg"]
    print("  运动段假偏航 ΔHDG     = %+.3f °   (%.3f °/s)" % (d_hdg, d_hdg / a.dur))
    print("  陀螺积分 ∫gz·dt       = %+.3f °   (%.3f °/s)" % (d_int, d_int / a.dur))
    print("  两者之差（磁修正贡献） = %+.3f °" % (d_hdg - d_int))
    print()
    if abs(d_int) > abs(d_hdg) * 0.6 and abs(d_int) > 0.5:
        print("  → 主因是**陀螺**：∫gz 已经能解释大部分 ΔHDG。")
        print("    运动段 gz 均值 %+.3f °/s，静止基线 %+.3f °/s，"
              % (mean(p2["gzs"]), mean(p1["gzs"])))
        print("    说明零偏在电机运行时变了（振动/温升/电流），不是磁力计的事。")
    elif abs(d_hdg - d_int) > abs(d_hdg) * 0.5 and abs(d_hdg) > 0.5:
        print("  → 主因是**磁力计**：陀螺基本没错（∫gz≈%.2f°），是磁修正把航向拉偏。"
              % d_int)
        m1, m2 = p1["mag"][0], p2["mag"][0]
        print("    磁基线 (%.2f, %.2f, %.2f) µT → 运动段 (%.2f, %.2f, %.2f) µT"
              % (m1 + m2))
        print("    变化 (%.2f, %.2f, %.2f) µT" % (m2[0] - m1[0], m2[1] - m1[1],
                                                 m2[2] - m1[2]))
    else:
        print("  → 陀螺和磁力计都不足以解释，怀疑四轮真差速或 EKF 内部。")
        print("    四轮 rpm 相对差 %.2f%%（>1% 就要怀疑机械）" % p2["spread"])
    print()
    print("  磁力计噪声：静置 (%.2f, %.2f, %.2f) µT → 运动 (%.2f, %.2f, %.2f) µT"
          % (p1["mag_sd"] + p2["mag_sd"]))
    if p3:
        print("  停止后：∫gz %+.3f°  ΔHDG %+.3f°  gz均值 %+.3f °/s"
              % (p3["integ"], p3["d_hdg"], mean(p3["gzs"])))
        if abs(mean(p3["gzs"]) - mean(p1["gzs"])) > 0.3:
            print("  → 停止后零偏没回到基线（%.3f vs %.3f °/s），存在温漂或磁滞"
                  % (mean(p3["gzs"]), mean(p1["gzs"])))
    print("=" * 66)


if __name__ == "__main__":
    main()
