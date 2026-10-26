#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ota.py — STM32F407 Bootloader UART 烧写工具 (Windows / RK3588 通用)

配合 "V1 - 副本" 的 Bootloader V1.2.0+ 使用:
  协议: CMD_OTA_START(触发App复位) -> BOOTLOADER横幅 -> UPDATE握手
        -> ADDR,08010000 -> DATA,<hex> xN -> RUN(校验并跳转)

用法 (Windows):
    python ota.py --list                          # 列出可用串口
    python ota.py COM3 app.bin                    # 烧录并自动跳转 (推荐)
    python ota.py COM3 app.bin --no-run           # 只烧不跳
    python ota.py COM3 app.bin --erase            # 先整片擦除 App 区再烧
    python ota.py COM3 --info                     # 查询 Bootloader 信息
    python ota.py COM3 --version                  # 查询运行中 App 版本

用法 (RK3588 / Linux):
    python3 ota.py /dev/ttyS1 app.bin

注意:
    - COM10 及以上 pyserial 需要 \\\\.\\ 前缀, 脚本自动处理
    - 若板子无 App / App 无 OTA 支持, 需按住 PA15 按键上电进入 Bootloader
"""

import sys
import time
import argparse
import serial

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

CHUNK = 64          # 单包 DATA 最大字节, 与 bootloader 严格对齐
LINE_END = b"\r\n"
BAUD_DEFAULT = 460800
APP_ADDR = 0x08010000
APP_MAX = 960 * 1024


# ================================ 串口工具 ================================

def fix_windows_port(port: str) -> str:
    """Windows COM10+ 需要 \\\\.\\ 前缀, 自动补全"""
    if sys.platform.startswith("win") and port.upper().startswith("COM"):
        try:
            n = int(port[3:])
            if n >= 10 and not port.startswith("\\\\.\\"):
                return "\\\\.\\" + port
        except ValueError:
            pass
    return port


def list_serial_ports():
    if list_ports is None:
        print("未安装 serial.tools.list_ports, 请手动指定串口 (如 COM3)")
        return
    ports = list_ports.comports()
    if not ports:
        print("未检测到任何串口!")
        return
    print("可用串口:")
    for p in ports:
        desc = p.description if p.description else ""
        hwid = p.hwid if p.hwid else ""
        print(f"  {p.device:12s} {desc}  {hwid}")


def read_line(ser, timeout=3.0):
    """读取一行 (\r\n 结尾), 返回去除首尾空白字符串; 超时返回 ''"""
    ser.timeout = timeout
    raw = ser.read_until(LINE_END)
    return raw.decode(encoding="ascii", errors="replace").strip()


def send_line(ser, line: str):
    ser.write((line + "\r\n").encode("ascii"))
    ser.flush()


def wait_ok(ser, desc: str, timeout=15.0):
    """等待OK应答 (默认 15s: 首次DATA含64KB扇区擦除, 需预留时间)"""
    resp = read_line(ser, timeout)
    if not resp.startswith("OK"):
        print(f"[错误] {desc} 失败, 返回: {repr(resp)}")
        sys.exit(1)
    return resp


# ================================ 阶段流程 ================================

def query_app_version(ser):
    """向运行中的 App 查询版本 CMD_GET_VERSION"""
    ser.reset_input_buffer()
    send_line(ser, "CMD_GET_VERSION")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        line = read_line(ser, timeout=0.5)
        if line.startswith("VERSION:"):
            return line
    return None


def enter_bootloader(ser):
    """触发 App OTA 复位 -> 等待 BOOTLOADER 横幅 -> UPDATE 握手"""
    ser.reset_input_buffer()
    send_line(ser, "CMD_OTA_START")
    deadline = time.time() + 6.0
    boot_found = False

    print("尝试触发 App OTA 复位... 等待 Bootloader 横幅")
    while time.time() < deadline:
        line = read_line(ser, timeout=0.5)
        if not line:
            continue
        if line.startswith("BOOTLOADER"):
            print(f"已进入 Bootloader: {line}")
            boot_found = True
            break
        if "OK OTA reboot" in line:
            print("App 已写升级标记, 正在软复位...")

    if not boot_found:
        print("未收到 App 复位应答 (无App/版本过旧), 直接裸连 Bootloader")
        print("若仍无法握手: 按住 PA15 按键上电强制进入升级模式")

    # UPDATE 握手
    ser.timeout = 0.2
    deadline = time.time() + 5.0
    while time.time() < deadline:
        ser.reset_input_buffer()
        send_line(ser, "UPDATE")
        resp = read_line(ser, timeout=0.5)
        if resp.startswith("OK"):
            print("Bootloader 握手成功!")
            return
    print("[致命错误] 无法与 Bootloader 建立通信: 检查波特率/接线/是否在升级模式")
    sys.exit(1)


def flash_firmware(ser, bin_path: str, auto_jump: bool, do_erase: bool, chunk: int = CHUNK):
    try:
        with open(bin_path, "rb") as f:
            fw = f.read()
    except FileNotFoundError:
        print(f"[错误] 固件文件不存在: {bin_path}")
        sys.exit(1)

    size = len(fw)
    print(f"固件大小: {size} 字节 ({size / 1024:.1f} KB)")
    if size == 0:
        print("[错误] 固件为空")
        sys.exit(1)
    if size > APP_MAX:
        print(f"[错误] 超出 App 分区最大 {APP_MAX // 1024}KB")
        sys.exit(1)

    # 可选: 整片擦除
    if do_erase:
        print("擦除 App 区 (扇区 4~11)...")
        send_line(ser, "ERASE")
        wait_ok(ser, "整片擦除")

    # 设置起始地址
    send_line(ser, f"ADDR,{APP_ADDR:08X}")
    wait_ok(ser, "设置写入地址")

    # 分块写入
    t0 = time.time()
    sent = 0
    last_report = 0
    for offset in range(0, size, chunk):
        block = fw[offset:offset + chunk]
        send_line(ser, "DATA," + block.hex().upper())
        wait_ok(ser, f"写入 0x{APP_ADDR + offset:08X}")
        sent += len(block)

        pct = sent * 100 // size
        if pct >= last_report + 10 or pct == 100:
            last_report = pct
            print(f"  进度 {pct:3d}%  ({sent}/{size})")

    cost = time.time() - t0
    rate = size / cost / 1024 if cost > 0 else 0
    print(f"写入完成: {size} 字节, 耗时 {cost:.1f}s, 速率 {rate:.1f} KB/s")

    if auto_jump:
        send_line(ser, "RUN")
        wait_ok(ser, "跳转 App")
        print("Bootloader 已清标记, 自动跳转新 App!")
    else:
        print("烧录完成 (--no-run), 停留在升级模式")


# ================================ 主流程 ================================

def main():
    parser = argparse.ArgumentParser(
        description="STM32F407 UART OTA 烧写工具 (Windows/RK3588)")
    parser.add_argument("port", nargs="?", help="串口: COM3 (Windows) 或 /dev/ttyS1 (Linux)")
    parser.add_argument("bin_file", nargs="?", help="待烧录 app.bin")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT, help=f"波特率 (默认 {BAUD_DEFAULT})")
    parser.add_argument("--no-run", action="store_true", help="烧录后不跳转")
    parser.add_argument("--erase", action="store_true", help="烧录前整片擦除 App 区")
    parser.add_argument("--info", action="store_true", help="查询 Bootloader 信息后退出")
    parser.add_argument("--version", action="store_true", help="查询运行中 App 版本后退出")
    parser.add_argument("--list", action="store_true", help="列出可用串口后退出")
    parser.add_argument("--erase-only", action="store_true", help="只擦除 App 区并退出 (诊断 Flash 擦除用)")
    parser.add_argument("--diag", action="store_true", help="DATA 写入诊断: 短行(4B) + 长行(64B) 各发一条看响应")
    parser.add_argument("--chunk", type=int, default=CHUNK, choices=[16, 32, 64],
                        help=f"单包字节数 (默认 {CHUNK}; 旧bootloader无ORE修复时用 16/32 应急)")
    args = parser.parse_args()

    if args.list or (args.port is None and args.bin_file is None):
        list_serial_ports()
        return

    port = fix_windows_port(args.port)
    ser = None
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        print(f"串口打开成功: {args.port} @ {args.baud}")

        if args.version:
            ver = query_app_version(ser)
            print(ver if ver else "App 未响应版本查询 (可能处于 Bootloader 或无 App)")
            return

        enter_bootloader(ser)

        if args.info:
            send_line(ser, "INFO")
            print("Bootloader:", read_line(ser, timeout=2.0))
            return

        if args.erase_only:
            print("擦除 App 区 (扇区 4~11)...")
            send_line(ser, "ERASE")
            wait_ok(ser, "整片擦除")
            print("OK: App 区已擦除")
            return

        if args.diag:
            print("--- DATA 写入诊断 ---")
            # 短行: 4 字节
            send_line(ser, "ADDR,08010000")
            wait_ok(ser, "ADDR(短行)")
            send_line(ser, "DATA,00000000")
            r = read_line(ser, timeout=5.0)
            print(f"短 DATA (4B):  响应={repr(r)}")
            # 长行: 64 字节 = 128 hex 字符
            send_line(ser, "ADDR,08010000")
            wait_ok(ser, "ADDR(长行)")
            send_line(ser, "DATA," + "AA" * 64)
            r = read_line(ser, timeout=5.0)
            print(f"长 DATA (64B): 响应={repr(r)}")
            print("--- 诊断结束: 短行无响应=写入路径问题; 长行无响应=长行接收问题 ---")
            return

        if args.bin_file is None:
            print("[错误] 未指定固件文件")
            return

        flash_firmware(ser, args.bin_file, auto_jump=not args.no_run,
                       do_erase=args.erase, chunk=args.chunk)

    except serial.SerialException as e:
        print(f"[串口异常] {e}")
        print("排查: 端口是否正确 / 设备已上电 / 驱动已装 / 权限 (Linux: chmod 666 /dev/ttyS1)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n用户中断, 退出")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    main()
