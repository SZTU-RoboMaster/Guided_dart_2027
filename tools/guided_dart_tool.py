"""@file guided_dart_tool.py
@brief Guided Dart 固件镜像生成、校验及 LoRa 模块维护工具。
@details 将独立 Boot/App BIN 组合成首次 SWD 烧录镜像，并为 App-SWD
         镜像附上末页元数据；普通 LoRa OTA 仍只传独立 App BIN。
@author Guided Dart 项目维护者
@date 2026-10-07
@version 1.0
@note 合并 ELF 仅封装整片装载内容，不包含 Boot/App 源码调试符号。
"""

from __future__ import annotations

import argparse
import sys
import struct
import subprocess
import time
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lora_profiles import (DEFAULT_AIR_RATE, DEFAULT_CHANNEL, DEFAULT_CONFIG_BAUD,
                           DEFAULT_DATA_BAUD, Ewm220Config, ewm220_read_all,
                           ewm220_write_all)

BOOT_SIZE = 0x4000
APP_ADDRESS = 0x4000
META_ADDRESS = 0x1F800
FLASH_SIZE = 0x20000
APP_FLASH_ADDRESS = 0x08004000
META_FLASH_ADDRESS = 0x0801F800
META_MAGIC = 0x31505041
META_COMMIT = 0xA55AA55A


def validate_app(app: bytes) -> None:
    """检查独立 App BIN 的长度、栈顶和复位向量是否匹配分区。"""
    if not app or len(app) > META_ADDRESS - APP_ADDRESS:
        raise ValueError("App BIN 超出 110KB 分区")
    if len(app) < 8:
        raise ValueError("App BIN 缺少向量表")
    stack, reset = struct.unpack_from("<II", app, 0)
    if not 0x20000000 <= stack <= 0x20007FE0 or stack % 8:
        raise ValueError("App 初始栈地址错误")
    if not APP_FLASH_ADDRESS <= (reset & ~1) < META_FLASH_ADDRESS:
        raise ValueError("App 复位入口不在 App 分区")


def make_image(boot_path: Path, app_path: Path, output: Path, version: int,
               full_elf: Path | None = None, objcopy: Path | None = None) -> None:
    """生成固定 128 KB 整片镜像，并在末页写入 App 的版本及 CRC32。"""
    boot = boot_path.read_bytes()
    app = app_path.read_bytes()
    if len(boot) > BOOT_SIZE:
        raise ValueError("Bootloader 超过 16KB")
    validate_app(app)

    image = bytearray(b"\xFF" * FLASH_SIZE)
    image[:len(boot)] = boot
    image[APP_ADDRESS:APP_ADDRESS + len(app)] = app
    crc = zlib.crc32(app) & 0xFFFFFFFF
    metadata = struct.pack("<6I", META_MAGIC, len(app), crc, version,
                           META_COMMIT, 0)
    image[META_ADDRESS:META_ADDRESS + len(metadata)] = metadata
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(image)
    print(f"生成 {output}，App 大小={len(app)}，CRC32=0x{crc:08X}")
    if full_elf is not None and objcopy is not None:
        make_full_elf(output, full_elf, objcopy)


def verify_image(path: Path) -> None:
    """复核整片镜像元数据及 App CRC32，避免烧录不一致内容。"""
    image = path.read_bytes()
    if len(image) != FLASH_SIZE:
        raise ValueError(f"整片镜像长度错误：{len(image)}")
    magic, size, expected_crc, version, commit, _ = struct.unpack_from(
        "<6I", image, META_ADDRESS)
    app = image[APP_ADDRESS:APP_ADDRESS + size]
    validate_app(app)
    actual_crc = zlib.crc32(app) & 0xFFFFFFFF
    if magic != META_MAGIC or commit != META_COMMIT:
        raise ValueError("元数据 magic/commit 错误")
    if actual_crc != expected_crc:
        raise ValueError(f"App CRC 错误：0x{actual_crc:08X} != 0x{expected_crc:08X}")
    print(f"镜像有效：版本={version}，App 大小={size}，CRC32=0x{actual_crc:08X}")


def make_app_swd(image_path: Path, output: Path) -> None:
    """从已校验的整片镜像提取 App 和末页元数据，供 SWD 单独烧录。"""
    verify_image(image_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(image_path.read_bytes()[APP_ADDRESS:])


def verify_app_swd(path: Path) -> None:
    """确认 App-SWD 镜像包含与 App 匹配的末页元数据。"""
    image = path.read_bytes()
    if len(image) != FLASH_SIZE - APP_ADDRESS:
        raise ValueError("App-SWD 镜像长度错误")
    verify_image_bytes = b"\xFF" * BOOT_SIZE + image
    magic, size, expected_crc, _, commit, _ = struct.unpack_from(
        "<6I", verify_image_bytes, META_ADDRESS)
    app = verify_image_bytes[APP_ADDRESS:APP_ADDRESS + size]
    validate_app(app)
    if magic != META_MAGIC or commit != META_COMMIT or zlib.crc32(app) & 0xFFFFFFFF != expected_crc:
        raise ValueError("App-SWD 元数据或 CRC 错误")


def make_full_elf(image_path: Path, output: Path, objcopy: Path) -> None:
    """把整片 BIN 包装成带 0x08000000 装载地址的 ELF。"""
    if not image_path.is_file():
        raise ValueError(f"整片 BIN 不存在：{image_path}")
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        str(objcopy), "-I", "binary", "-O", "elf32-littlearm", "-B", "arm",
        "--rename-section=.data=.flash,alloc,load,readonly,data,contents",
        "--change-addresses=0x08000000", str(image_path), str(output)
    ], check=True)
    print(f"生成 {output}，大小={output.stat().st_size} 字节")


def _open_serial(port_name: str, baud: int):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请安装 groundstation/requirements.txt") from exc

    return serial.Serial(port_name, baud, timeout=1.0)


def configure_ewm32(port_name: str, air_rate: int, local_baud: int, address: int, channel: int) -> None:
    if not 0 <= address <= 65535:
        raise ValueError("EWM32 地址必须为 0~65535")
    if not 0 <= channel <= 69:
        raise ValueError("EWM32 信道必须为 0~69")
    commands = [
        "AT+UART=7,0",
        f"AT+ADDR={address}",
        f"AT+CHANNEL={channel}",
        f"AT+RATE={air_rate}",
        "AT+TRANS=0",
        "AT+FEC=1",
        "AT+POWER=0",
    ]
    with _open_serial(port_name, local_baud) as port:
        for command in commands:
            port.write((command + "\r\n").encode("ascii"))
            time.sleep(0.15)
            response = port.read_all().decode("ascii", errors="replace").strip()
            print(f"{command} -> {response or '(无响应)'}")
    print(f"配置完成：当前配置串口 {local_baud}，退出配置模式并重新上电后，数据 UART 使用 {DEFAULT_DATA_BAUD} 8N1。")


def configure_ewm220(port_name: str, air_rate: int, uart_baud: int, channel: int) -> None:
    config = Ewm220Config(air_rate=air_rate, uart_baud=uart_baud, channel=channel)
    with _open_serial(port_name, DEFAULT_CONFIG_BAUD) as port:
        port.reset_input_buffer()
        port.write(ewm220_write_all(config))
        port.flush()
        time.sleep(0.25)
        response = port.read(32)
    print(f"EWM220 写入：{response.hex(' ') or '(无响应)'}")
    if response and response[:3] != b"\xC1\x00\x09":
        raise RuntimeError("EWM220 未返回 C1 参数确认帧")
    print(f"配置完成：信道={channel}，空口={air_rate}，模块数据 UART={uart_baud}；恢复 M0=M1=0 后重新上电")


def probe_lora(port_name: str, local_baud: int, profile: str) -> None:
    """读取 EWM32M 的型号和保存参数；模块必须处于 M0=M1=1。"""
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请安装 groundstation/requirements.txt") from exc

    baud = DEFAULT_CONFIG_BAUD if profile == "ewm220" else local_baud
    with _open_serial(port_name, baud) as port:
        commands = (("parameters", ewm220_read_all()),) if profile == "ewm220" else (
            ("model", b"\xC3\xC3\xC3"), ("parameters", b"\xC1\xC1\xC1"))
        for name, command in commands:
            port.reset_input_buffer()
            port.write(command)
            port.flush()
            time.sleep(0.25)
            response = port.read(64)
            print(f"{name}: {response.hex(' ') or '(无响应；请确认 M0=M1=1、波特率和接线)'}")
            if profile == "ewm220" and response:
                config = Ewm220Config.from_response(response)
                print(f"EWM220: UART={config.uart_baud}，空口编号={config.air_rate}，信道={config.channel}，透明={config.transparent}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Guided Dart 工具链")
    sub = parser.add_subparsers(dest="command", required=True)

    image = sub.add_parser("make-image", help="生成首次 SWD 整片镜像")
    image.add_argument("--boot", type=Path, required=True)
    image.add_argument("--app", type=Path, required=True)
    image.add_argument("--output", type=Path, required=True)
    image.add_argument("--version", type=int, default=1)
    image.add_argument("--full-elf", type=Path)
    image.add_argument("--objcopy", type=Path)

    verify = sub.add_parser("verify-image", help="校验整片镜像")
    verify.add_argument("image", type=Path)

    app_swd = sub.add_parser("make-app-swd", help="提取带元数据的 App-SWD 镜像")
    app_swd.add_argument("image", type=Path)
    app_swd.add_argument("output", type=Path)

    verify_app = sub.add_parser("verify-app-swd", help="校验 App-SWD 镜像")
    verify_app.add_argument("image", type=Path)

    full_elf = sub.add_parser("make-full-elf", help="生成带 Flash 地址的整片 ELF")
    full_elf.add_argument("image", type=Path)
    full_elf.add_argument("output", type=Path)
    full_elf.add_argument("--objcopy", type=Path, required=True)

    lora = sub.add_parser("configure-lora", help="配置电脑端 LoRa 模块")
    lora.add_argument("port")
    lora.add_argument("--profile", choices=("ewm32", "ewm220"), default="ewm32",
                      help="模块协议；电脑端 EWM32M 用 ewm32，主控端 EWM220M 用 ewm220")
    lora.add_argument("--air-rate", type=int, default=DEFAULT_AIR_RATE, choices=tuple(range(8)),
                      help="空口速率编号；默认 5（19.2 kbps），必须与主控端一致")
    lora.add_argument("--local-baud", type=int, default=DEFAULT_CONFIG_BAUD,
                      choices=(1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200),
                      help="配置模式 UART 波特率；按手册默认 9600")
    lora.add_argument("--uart-baud", type=int, default=DEFAULT_DATA_BAUD,
                      choices=tuple((1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)),
                      help="EWM220 退出配置模式后的数据 UART 波特率")
    lora.add_argument("--channel", type=int, default=DEFAULT_CHANNEL,
                      help="信道；EWM220 为 0~83，EWM32-433 为 0~31；23 对应 433 MHz")
    lora.add_argument("--address", type=int, default=0,
                      help="EWM32 通信地址，默认 0")
    probe = sub.add_parser("probe-lora", help="读取 EWM32M 型号和保存参数")
    probe.add_argument("port")
    probe.add_argument("--profile", choices=("ewm32", "ewm220"), default="ewm32")
    probe.add_argument("--local-baud", type=int, default=DEFAULT_CONFIG_BAUD,
                       choices=(1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200),
                       help="配置模式 UART 波特率，手册默认 9600")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.command == "make-image":
        make_image(args.boot, args.app, args.output, args.version, args.full_elf, args.objcopy)
    elif args.command == "verify-image":
        verify_image(args.image)
    elif args.command == "make-app-swd":
        make_app_swd(args.image, args.output)
    elif args.command == "verify-app-swd":
        verify_app_swd(args.image)
    elif args.command == "make-full-elf":
        make_full_elf(args.image, args.output, args.objcopy)
    elif args.command == "configure-lora":
        if args.profile == "ewm220":
            configure_ewm220(args.port, args.air_rate, args.uart_baud, args.channel)
        else:
            configure_ewm32(args.port, args.air_rate, args.local_baud, args.address, args.channel)
    else:
        probe_lora(args.port, args.local_baud, args.profile)


if __name__ == "__main__":
    main()
