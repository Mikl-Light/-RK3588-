#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32F407 Bootloader 上位机烧写脚本 (RK OTA 方案配套)
用法:
    python bl_flash.py COM3 firmware.bin          # 触发 OTA 并烧写
    python bl_flash.py COM3 firmware.bin --no-run # 只烧写不跳转
    python bl_flash.py COM3 --version             # 查询 App 固件版本
    python bl_flash.py COM3 --info                # 查询 bootloader 信息

流程 (与 RK3588 ROS 节点升级流程一致):
    打开串口(460800)
      -> 发 CMD_GET_VERSION (可选, 查询当前版本)
      -> 发 CMD_OTA_START   (App 写升级标记 0x0800FC00 并软复位)
      -> 等 Bootloader 横幅 "BOOTLOADER ... ready"
      -> UPDATE 握手 -> ADDR -> 分块 DATA -> RUN (成功后 Bootloader 清标记跳转)
    若 App 不存在/无应答, 自动回退为直接发 UPDATE (裸升级模式)。
"""
import sys
import time
import argparse

import serial  # pip install pyserial

CHUNK = 64  # 每行 DATA 最多 64 字节 (与 bootloader 端一致)


def readline(ser, timeout=3.0):
    ser.timeout = timeout
    data = ser.read_until(b"\n")
    return data.decode(errors="replace").strip()


def send_line(ser, line):
    ser.write((line + "\r\n").encode())
    ser.flush()


def wait_ok(ser, what):
    resp = readline(ser)
    if not resp.startswith("OK"):
        print(f"[ERR] {what} 失败: {resp!r}")
        sys.exit(1)
    return resp


def query_version(ser):
    """CMD_GET_VERSION: App 回 'VERSION:v2.1 26.08.15'"""
    ser.reset_input_buffer()
    send_line(ser, "CMD_GET_VERSION")
    deadline = time.time() + 2
    while time.time() < deadline:
        line = readline(ser, 0.5)
        if line.startswith("VERSION:"):
            return line
    return None


def enter_update(ser):
    """CMD_OTA_START 触发 App 复位进 Bootloader; 失败则回退直接 UPDATE"""
    ser.reset_input_buffer()
    send_line(ser, "CMD_OTA_START")

    deadline = time.time() + 6
    got_banner = False
    while time.time() < deadline:
        line = readline(ser, 0.5)
        if line.startswith("BOOTLOADER"):
            print(f"已进入 Bootloader: {line}")
            got_banner = True
            break
        if "OK OTA reboot" in line:
            print("App 已写升级标记, 等待复位进 Bootloader ...")
            continue
    if not got_banner:
        print("App 无应答 (未烧 App 或版本过旧), 尝试直接进入升级模式 ...")

    # UPDATE 握手确认 (裸 Bootloader / 保险起见)
    ser.timeout = 0.2
    deadline = time.time() + 5
    while time.time() < deadline:
        ser.reset_input_buffer()
        ser.write(b"UPDATE\r\n")
        ser.flush()
        resp = ser.readline().decode(errors="replace").strip()
        if resp.startswith("OK"):
            return
    print("[ERR] 未能进入升级模式, 请确认波特率为 460800 且固件带 OTA 命令")
    sys.exit(1)


def flash(ser, path, run=True):
    with open(path, "rb") as f:
        fw = f.read()
    print(f"固件大小: {len(fw)} 字节 ({len(fw)/1024:.1f} KB)")
    if len(fw) > 960 * 1024:
        print("[ERR] 固件超过 App 区 960KB 限制")
        sys.exit(1)

    addr = 0x08010000
    send_line(ser, f"ADDR,{addr:08X}")
    wait_ok(ser, "设置地址")

    sent = 0
    t0 = time.time()
    for off in range(0, len(fw), CHUNK):
        block = fw[off:off + CHUNK]
        send_line(ser, "DATA," + block.hex().upper())
        wait_ok(ser, f"写入 @0x{addr + off:08X}")
        sent += len(block)
        pct = sent * 100 // len(fw)
        print(f"\r进度: {pct:3d}%  ({sent}/{len(fw)} 字节)", end="")
    print(f"\n写入完成, 用时 {time.time() - t0:.1f}s")

    if run:
        send_line(ser, "RUN")
        wait_ok(ser, "跳转")
        print("已清除升级标记并跳转到新固件!")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="串口号, 如 COM3")
    ap.add_argument("bin", nargs="?", help="固件 .bin 文件 (链接基址 0x08010000)")
    ap.add_argument("--no-run", action="store_true", help="烧写后不跳转")
    ap.add_argument("--version", action="store_true", help="仅查询 App 固件版本")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 460800, timeout=1)

    if args.version:
        v = query_version(ser)
        print(v if v else "App 未应答 CMD_GET_VERSION")
        ser.close()
        return

    if args.bin is None:
        enter_update(ser)
        send_line(ser, "INFO")
        print(readline(ser))
        ser.close()
        return

    enter_update(ser)
    flash(ser, args.bin, run=not args.no_run)
    ser.close()


if __name__ == "__main__":
    main()
