#!/usr/bin/env python3
"""
JDY-16 BLE 桥接命令行工具
========================
通过 BLE 连接 JDY-16 透传模块，执行 MCU 自定义指令。

协议: PC 发送 !<cmd> → JDY-16 透传 → MCU (mid_ble.c) → 执行并回复

依赖: pip install bleak
运行: python jdy16_ble.py

MCU 指令集:
  !SPD <dur> <int> [save] - 定时采样电机转速 (m/s)
       dur=持续时长(ms)  int=采样间隔(ms)  save=1/s/save 保存CSV
       例: !SPD 5000 100     每100ms读一次，持续5秒，实时输出
       例: !SPD 3000 50 save 每50ms读一次，结束后保存到 log/*.csv
  !START [speed]          - 直行启动 (speed: 0.05-0.80 m/s, 默认 0.10)
  !STOP                   - 停止

内置命令:
  .scan            - 重新扫描 BLE 设备
  .info            - 显示当前连接信息
  .disconnect      - 断开连接
  .reconnect       - 重新连接
  .help            - 显示帮助
  .quit            - 退出程序
"""

import asyncio
import os
import sys
import platform
from datetime import datetime
from typing import Optional

try:
    from bleak import BleakScanner, BleakClient
    from bleak.exc import BleakError
except ImportError:
    print("错误: 未安装 bleak 库，请运行: pip install bleak")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════════════════
# 常量
# ══════════════════════════════════════════════════════════════════════════════

JDY16_CHAR_FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "log")

# ══════════════════════════════════════════════════════════════════════════════
# 全局状态
# ══════════════════════════════════════════════════════════════════════════════

g_client: Optional[BleakClient] = None
g_device_name: str = ""
g_device_addr: str = ""
g_running: bool = True
g_disconnect_intentional: bool = False
g_notify_queue: asyncio.Queue = asyncio.Queue()

# SPD 录制
g_spd_recording: bool = False
g_spd_samples: list = []
g_spd_interval_ms: int = 0

# 行缓冲 (处理 BLE 通知分包)
g_line_buf: str = ""


# ══════════════════════════════════════════════════════════════════════════════
# 回调
# ══════════════════════════════════════════════════════════════════════════════

def is_wsl() -> bool:
    try:
        with open("/proc/version", "r") as f:
            return "microsoft" in f.read().lower()
    except (FileNotFoundError, PermissionError):
        return False


def on_notify(sender_handle, data: bytearray):
    try:
        text = data.decode("utf-8", errors="replace")
        g_notify_queue.put_nowait(text)
    except Exception:
        g_notify_queue.put_nowait(repr(data))


def on_disconnect(client):
    global g_running
    if not g_disconnect_intentional:
        print("\n[!] 蓝牙连接已断开")
        g_running = False


# ══════════════════════════════════════════════════════════════════════════════
# SPD CSV 保存
# ══════════════════════════════════════════════════════════════════════════════

def save_spd_csv():
    global g_spd_recording, g_spd_samples, g_spd_interval_ms

    if not g_spd_samples:
        print("\n[!] 没有编码器数据可保存")
        g_spd_recording = False
        return

    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"spd_{timestamp}.csv"
    filepath = os.path.join(LOG_DIR, filename)

    try:
        with open(filepath, "w", encoding="utf-8", newline="") as f:
            f.write("time_ms,speed_a_mps,speed_b_mps\n")
            for i, (a, b) in enumerate(g_spd_samples):
                time_ms = i * g_spd_interval_ms
                f.write(f"{time_ms},{a:.6f},{b:.6f}\n")
        print(f"\n[√] 编码器数据已保存: {filepath}")
        print(f"    采样点数: {len(g_spd_samples)}, 间隔: {g_spd_interval_ms}ms")
    except OSError as e:
        print(f"\n[X] 保存 CSV 失败: {e}")

    g_spd_recording = False
    g_spd_samples = []
    g_spd_interval_ms = 0


def start_spd_recording(cmd: str):
    global g_spd_recording, g_spd_samples, g_spd_interval_ms

    upper = cmd.upper().replace("!SPD", "").strip()
    parts = upper.split()

    if len(parts) >= 3:
        if parts[2] in ("1", "SAVE", "S"):
            g_spd_recording = True
            g_spd_samples = []
            try:
                g_spd_interval_ms = int(parts[1])
            except ValueError:
                g_spd_interval_ms = 100
            return True
    return False


# ══════════════════════════════════════════════════════════════════════════════
# 设备扫描
# ══════════════════════════════════════════════════════════════════════════════

async def scan_devices(timeout: float = 8.0) -> list:
    print(f"正在扫描 BLE 设备 ({timeout}秒)...")
    print("提示: 请确保 JDY-16 模块已上电且未与其它设备连接\n")

    devices = []
    jdy_devices = []

    def detection_callback(device, advertisement_data):
        if device.address not in [d.address for d in devices]:
            devices.append(device)
            name = device.name or "(未知)"
            rssi = advertisement_data.rssi
            name_lower = name.lower()
            addr_lower = device.address.lower()
            if any(kw in name_lower or kw in addr_lower
                   for kw in ("jdy", "mlt", "bt", "ble")):
                jdy_devices.append((device, name, rssi))
                print(f"  [*] 发现疑似 JDY 设备: {name:<20} RSSI: {rssi:>4} dBm  MAC: {device.address}")

    try:
        scanner = BleakScanner(detection_callback)
        await scanner.start()
        await asyncio.sleep(timeout)
        await scanner.stop()
    except BleakError as e:
        reason = str(e).lower()
        if "powered" in reason or "turn on" in reason:
            print("\n[X] 蓝牙未开启，请在 Windows 设置中打开蓝牙后重试。")
        elif "not available" in reason or "not supported" in reason:
            print("\n[X] 此设备没有蓝牙适配器，或蓝牙驱动未正确安装。")
        else:
            print(f"\n[X] 蓝牙错误: {e}")
        return []

    if jdy_devices:
        print(f"\n[√] 扫描完成，发现 {len(jdy_devices)} 个 JDY 兼容设备:")
        return jdy_devices

    device_list = [(d, d.name or "(未知)", getattr(d, 'rssi', 0) or 0) for d in devices]
    print(f"\n扫描完成，共发现 {len(device_list)} 个 BLE 设备:")
    for i, (d, name, rssi) in enumerate(device_list):
        print(f"  [{i}] {name:<25} MAC: {d.address}")
    return device_list


async def select_device(devices: list):
    if not devices:
        print("\n未发现任何 BLE 设备！")
        print("请检查:")
        print("  1. JDY-16 模块是否已上电")
        print("  2. 模块是否处于广播状态 (LED 快闪)")
        print("  3. 蓝牙适配器是否正常工作")
        return None, None

    if len(devices) == 1:
        d, name, rssi = devices[0]
        print(f"\n自动选择唯一设备: {name} [{d.address}]")
        return d.address, name

    print(f"\n发现 {len(devices)} 个可选设备:")
    for i, (d, name, rssi) in enumerate(devices):
        print(f"  [{i}] {name:<25} RSSI: {rssi:>4} dBm  MAC: {d.address}")

    loop = asyncio.get_event_loop()
    while True:
        try:
            choice = (await loop.run_in_executor(
                None, lambda: input(f"\n请选择设备 [0-{len(devices) - 1}] (q 退出): ").strip()
            ))
            if choice.lower() == "q":
                return None, None
            idx = int(choice)
            if 0 <= idx < len(devices):
                d, name, rssi = devices[idx]
                return d.address, name
            print(f"输入无效，请输入 0-{len(devices) - 1}")
        except ValueError:
            print(f"输入无效，请输入数字或 'q'")
        except (EOFError, KeyboardInterrupt):
            return None, None


# ══════════════════════════════════════════════════════════════════════════════
# 连接管理
# ══════════════════════════════════════════════════════════════════════════════

async def connect_device(address: str) -> Optional[BleakClient]:
    global g_client, g_disconnect_intentional

    print(f"\n正在连接 {address} ...")
    client = BleakClient(address, disconnected_callback=on_disconnect)

    try:
        await client.connect(timeout=15.0)
        if not client.is_connected:
            print("[X] 连接失败")
            return None

        print(f"[√] 连接成功! (MCU 桥接模式)")

        target_char = JDY16_CHAR_FFE1
        found = False
        for service in client.services:
            for char in service.characteristics:
                if char.uuid.lower().endswith("ffe1"):
                    target_char = char.uuid
                    found = True
                    break
            if found:
                break

        if found:
            print(f"[√] 通信特征值: {target_char}")
        else:
            print("[!] 未找到 FFE1 特征值，尝试第一个可写可通知的特征值")
            for service in client.services:
                for char in service.characteristics:
                    if "write" in char.properties and "notify" in char.properties:
                        target_char = char.uuid
                        found = True
                        break
                if found:
                    break

        if not found:
            print("[X] 无法找到可用的通信特征值")
            await client.disconnect()
            return None

        try:
            await client.start_notify(target_char, on_notify)
            print(f"[√] 已启用通知监听")
        except Exception as e:
            print(f"[!] 启用通知失败: {e}")

        g_client = client
        g_disconnect_intentional = False
        return client

    except asyncio.TimeoutError:
        print("[X] 连接超时")
    except Exception as e:
        print(f"[X] 连接失败: {e}")

    g_client = None
    return None


async def disconnect_device():
    global g_client, g_disconnect_intentional
    g_disconnect_intentional = True
    if g_client and g_client.is_connected:
        try:
            await g_client.disconnect()
        except Exception:
            pass
        g_client = None
        print("[√] 已断开连接")


# ══════════════════════════════════════════════════════════════════════════════
# 命令发送
# ══════════════════════════════════════════════════════════════════════════════

async def send_command(cmd: str) -> None:
    if g_client is None or not g_client.is_connected:
        print("[!] 未连接到设备")
        return

    if cmd.upper().startswith("!SPD"):
        start_spd_recording(cmd)

    data = (cmd + "\r\n").encode("utf-8")
    try:
        await g_client.write_gatt_char(JDY16_CHAR_FFE1, data, response=False)
    except Exception as e:
        print(f"[X] 发送失败: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# 通知处理
# ══════════════════════════════════════════════════════════════════════════════

async def notification_printer():
    global g_line_buf, g_spd_recording, g_spd_samples

    while True:
        data = await g_notify_queue.get()
        g_line_buf += data

        while "\n" in g_line_buf:
            line, g_line_buf = g_line_buf.split("\n", 1)
            line = line.strip("\r").strip()
            if not line:
                continue

            sys.stdout.write(f"\r← {line}\n> ")
            sys.stdout.flush()

            if g_spd_recording and line.startswith("SPD:") and line != "SPD_DONE":
                content = line[4:]
                parts = content.split(",")
                if len(parts) == 2:
                    try:
                        a = float(parts[0].strip())
                        b = float(parts[1].strip())
                        g_spd_samples.append((a, b))
                    except ValueError:
                        pass

            if g_spd_recording and line == "SPD_DONE":
                save_spd_csv()


# ══════════════════════════════════════════════════════════════════════════════
# 帮助与信息
# ══════════════════════════════════════════════════════════════════════════════

def print_help():
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║              JDY-16 BLE 桥接命令行帮助 (MCU Relay)                    ║
╠══════════════════════════════════════════════════════════════════════╣
║  内置命令:                                                           ║
║    .scan        重新扫描 BLE 设备                                    ║
║    .info        显示当前连接信息                                      ║
║    .disconnect  断开连接                                             ║
║    .reconnect   重新连接                                             ║
║    .help        显示此帮助                                           ║
║    .quit        退出程序                                             ║
╠══════════════════════════════════════════════════════════════════════╣
║  MCU 指令 (以 ! 开头):                                               ║
║    !SPD <dur> <int> [save]  定时采样电机转速 (m/s)                     ║
║         dur = 持续时长 (ms)   例: 5000 = 5秒                          ║
║         int = 采样间隔 (ms)   例: 100  = 每100ms读一次                 ║
║         save = 可选参数，加 "save"/"s"/"1" 启用CSV保存                ║
║         例: !SPD 5000 100      实时输出，不保存文件                    ║
║         例: !SPD 3000 50 save  采集后保存到 log/spd_*.csv             ║
║    !START [speed]          直行启动 (speed: 0.05-0.80, 默认0.10)       ║
║    !STOP                   停止                                       ║
╚══════════════════════════════════════════════════════════════════════╝
""")


def print_info():
    status = "已连接" if (g_client and g_client.is_connected) else "未连接"
    rec = "录制中" if g_spd_recording else "空闲"
    print(f"""
╔══════════════════════════════════════════════╗
║  连接信息 (MCU 桥接模式)                       ║
╠══════════════════════════════════════════════╣
║  设备名称:  {g_device_name:<32} ║
║  MAC 地址:  {g_device_addr:<32} ║
║  连接状态:  {status:<32} ║
║  SPD 录制:  {rec:<32} ║
║  CSV 目录:  {LOG_DIR:<32} ║
╚══════════════════════════════════════════════╝
""")


# ══════════════════════════════════════════════════════════════════════════════
# 交互主循环
# ══════════════════════════════════════════════════════════════════════════════

def _is_connected() -> bool:
    return g_client is not None and g_client.is_connected


def _print_banner():
    print("\n" + "=" * 60)
    print("JDY-16 BLE 桥接模式 (MCU Relay)")
    print("输入 ! 前缀的 MCU 指令，输入 .help 查看帮助，输入 .quit 退出")
    print("=" * 60 + "\n")


async def _dot_scan():
    global g_device_name, g_device_addr
    print("正在扫描设备...")
    devices = await scan_devices()
    addr, name = await select_device(devices)
    if addr:
        await disconnect_device()
        g_device_name = name
        g_device_addr = addr
        await connect_device(addr)
        _print_banner()
    else:
        print("已取消扫描")


async def _dot_reconnect():
    global g_device_addr
    if g_device_addr:
        await disconnect_device()
        await connect_device(g_device_addr)
        _print_banner()
    else:
        print("[!] 没有可用的设备地址，请先使用 .scan")


# ── Dot 命令分发 ──

async def _handle_dot_command(full_cmd: str):
    parts = full_cmd.split(maxsplit=1)
    dot_cmd = parts[0].lower()
    arg = parts[1] if len(parts) > 1 else ""

    if dot_cmd in (".quit", ".exit"):
        global g_running
        print("正在退出...")
        g_running = False
        return

    if dot_cmd == ".help":
        print_help()
        return

    if dot_cmd == ".info":
        print_info()
        return

    if dot_cmd == ".scan":
        await _dot_scan()
        return

    if dot_cmd == ".disconnect":
        await disconnect_device()
        return

    if dot_cmd == ".reconnect":
        await _dot_reconnect()
        return

    print(f"未知命令: {dot_cmd}，输入 .help 查看帮助")


# ── 文本命令处理 ──

async def _handle_text_command(cmd: str):
    if not _is_connected():
        print("[!] 未连接，请先使用 .scan 连接设备")
        return

    if cmd.startswith("!"):
        print(f"发送: {cmd}")
        await send_command(cmd)
    else:
        print(f"[!] 桥接模式仅支持 ! 前缀的 MCU 指令，输入 .help 查看帮助")


# ── 主循环 ──

async def interactive_loop():
    global g_running

    printer_task = asyncio.create_task(notification_printer())
    _print_banner()

    loop = asyncio.get_event_loop()

    while g_running:
        try:
            if not _is_connected():
                print("[!] 设备已断开，输入 .scan 重新扫描或 .quit 退出")

            cmd = await loop.run_in_executor(
                None, lambda: input("> ").strip()
            )

            if not cmd:
                continue

            if cmd.startswith("."):
                await _handle_dot_command(cmd)
            else:
                await _handle_text_command(cmd)

        except (EOFError, KeyboardInterrupt):
            print("\n正在退出...")
            g_running = False
            break
        except Exception as e:
            print(f"[X] 错误: {e}")

    printer_task.cancel()
    try:
        await printer_task
    except asyncio.CancelledError:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════════════

async def main():
    global g_device_name, g_device_addr

    print("=" * 60)
    print("  JDY-16 BLE Bridge Tool v2.0")
    print("  平台:", platform.system(), platform.release())
    print("=" * 60)

    if is_wsl():
        print("\n[!] 检测到 WSL 环境，BLE 蓝牙功能可能不可用。")
        print("    建议在 Windows 原生 Python 环境中运行本脚本。")
        resp = input("\n是否继续尝试? [y/N]: ").strip().lower()
        if resp != "y":
            return

    devices = await scan_devices()
    addr, name = await select_device(devices)

    if addr is None:
        print("未选择设备，退出。")
        return

    g_device_name = name
    g_device_addr = addr

    client = await connect_device(addr)
    if client is None or not client.is_connected:
        print("无法连接到设备，退出。")
        return

    try:
        await interactive_loop()
    finally:
        await disconnect_device()
        print("程序已退出。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n用户中断，退出。")
    except BleakError as e:
        reason = str(e).lower()
        if "powered" in reason or "turn on" in reason:
            print("\n[X] 蓝牙未开启，请在 Windows 设置中打开蓝牙后重试。")
        elif "not available" in reason or "not supported" in reason:
            print("\n[X] 此设备没有蓝牙适配器，或蓝牙驱动未正确安装。")
        else:
            print(f"\n程序错误: {e}")
    except Exception as e:
        print(f"\n程序异常: {e}")
