#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32F407 Bootloader 上位机烧写脚本 (RK3588调试专用，免拔插ST-LINK)
RK设备串口示例：/dev/ttyS4 /dev/ttyUSB0
Windows串口示例：COM3

用法:
    python3 bl_flash.py /dev/ttyS4 firmware.bin          # 烧录完成自动跳转APP
    python3 bl_flash.py /dev/ttyS4 firmware.bin --no-run # 仅烧录不跳转
    python3 bl_flash.py /dev/ttyS4 --version             # 查询当前APP版本
    python3 bl_flash.py /dev/ttyS4 --info                # 查询Bootloader信息
"""
import sys
import time
import argparse
import serial
import serial.serialutil

CHUNK = 64          # 单包DATA最大字节，和bootloader严格对齐
BAUD = 460800
LINE_END = b"\r\n"

def read_line(ser, timeout=3.0):
    """统一行读取：以\r\n分割，返回去除首尾空白字符串"""
    ser.timeout = timeout
    raw = ser.read_until(LINE_END)
    return raw.decode(encoding="ascii", errors="replace").strip()

def send_line(ser, line_str: str):
    """发送带\r\n的命令行，强制刷新缓冲区"""
    data = (line_str + "\r\n").encode("ascii")
    ser.write(data)
    ser.flush()

def wait_ok(ser, desc: str):
    """等待OK应答，非OK直接退出程序"""
    resp = read_line(ser)
    if not resp.startswith("OK"):
        print(f"[错误] {desc} 失败，返回内容：{repr(resp)}")
        sys.exit(1)
    return resp

def query_app_version(ser):
    """向运行中的APP查询版本 CMD_GET_VERSION"""
    ser.reset_input_buffer()
    send_line(ser, "CMD_GET_VERSION")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        line = read_line(ser, timeout=0.5)
        if line.startswith("VERSION:"):
            return line
    return None

def enter_bootloader_mode(ser):
    # 第一步：尝试发送OTA启动指令，让APP写标记+软复位
    ser.reset_input_buffer()
    send_line(ser, "CMD_OTA_START")
    deadline = time.time() + 6.0
    boot_banner_found = False

    print("尝试触发APP OTA复位...等待Bootloader启动横幅")
    while time.time() < deadline:
        line = read_line(ser, timeout=0.5)
        if not line:
            continue
        if line.startswith("BOOTLOADER"):
            print(f"成功进入Bootloader：{line}")
            boot_banner_found = True
            break
        if "OK OTA reboot" in line:
            print("APP已写入升级标记，正在软复位...")
            continue

    if not boot_banner_found:
        print("未收到APP复位应答（无APP/APP版本过低），直接裸连接Bootloader")

    # 第二步：握手 UPDATE，确保和Bootloader建立通信
    ser.timeout = 0.2
    handshake_deadline = time.time() + 5.0
    while time.time() < handshake_deadline:
        ser.reset_input_buffer()
        send_line(ser, "UPDATE")
        resp = read_line(ser)
        if resp.startswith("OK"):
            print("Bootloader握手成功！")
            return
    print("[致命错误] 无法与Bootloader建立通信，请检查：波特率460800、硬件接线、芯片是否上电")
    sys.exit(1)

def flash_firmware(ser, bin_path: str, auto_jump: bool):
    # 读取固件bin
    try:
        with open(bin_path, "rb") as f:
            fw_data = f.read()
    except FileNotFoundError:
        print(f"[错误] 固件文件不存在：{bin_path}")
        sys.exit(1)

    fw_size = len(fw_data)
    print(f"固件总大小：{fw_size} 字节 ({fw_size / 1024:.1f} KB)")
    if fw_size == 0:
        print("[错误] 固件文件为空，禁止烧录")
        sys.exit(1)
    if fw_size > 960 * 1024:
        print("[错误] 固件超出APP分区最大960KB限制")
        sys.exit(1)

    flash_addr = 0x08010000
    # 设置起始地址
    send_line(ser, f"ADDR,{flash_addr:08X}")
    wait_ok(ser, "设置Flash写入起始地址")

    send_total = 0
    start_time = time.time()
    # 分块下发DATA
    for offset in range(0, fw_size, CHUNK):
        block = fw_data[offset : offset + CHUNK]
        hex_str = block.hex().upper()
        send_line(ser, f"DATA,{hex_str}")
        current_write_addr = flash_addr + offset
        wait_ok(ser, f"写入 0x{current_write_addr:08X}")
        send_total += len(block)
        progress = send_total * 100 // fw_size
        print(f"\r烧录进度：{progress:3d}%  ({send_total}/{fw_size})", end="")

    cost_sec = time.time() - start_time
    print(f"\n固件全部写入完成，耗时 {cost_sec:.1f} 秒")

    if auto_jump:
        send_line(ser, "RUN")
        wait_ok(ser, "发送跳转APP指令")
        print("Bootloader已清除升级标记，自动跳转新APP！")
    else:
        print("已完成烧录，不执行跳转，停留在Bootloader升级模式")

def main():
    parser = argparse.ArgumentParser(description="STM32 Bootloader UART OTA工具(RK调试版)")
    parser.add_argument("port", help="串口设备，RK示例：/dev/ttyS4；Windows示例：COM3")
    parser.add_argument("bin_file", nargs="?", help="待烧录app.bin文件路径")
    parser.add_argument("--no-run", action="store_true", help="烧写完不自动跳转APP")
    parser.add_argument("--version", action="store_true", help="仅查询当前运行APP版本")
    args = parser.parse_args()

    ser = None
    try:
        ser = serial.Serial(args.port, BAUD, timeout=1)
        print(f"串口打开成功：{args.port} @ {BAUD}")

        if args.version:
            ver = query_app_version(ser)
            print(ver if ver else "APP未响应版本查询指令，可能当前运行Bootloader或无APP")
            return

        if args.bin_file is None:
            enter_bootloader_mode(ser)
            send_line(ser, "INFO")
            info = read_line(ser)
            print("Bootloader信息：", info)
            return

        # 完整OTA烧录流程
        enter_bootloader_mode(ser)
        flash_firmware(ser, args.bin_file, auto_jump=not args.no_run)

    except serial.serialutil.SerialException as e:
        print(f"[串口异常] 打开/读写失败：{str(e)}")
        print("排查：RK串口权限 chmod 666 /dev/ttyS4；确认设备已上电、接线正常")
        sys.exit(1)
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("串口已正常关闭")

if __name__ == "__main__":
    main()