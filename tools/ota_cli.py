"""通过 LoRa 串口执行一次 App OTA，并校验目标板版本与 CRC。"""

from __future__ import annotations

import argparse
import struct
import time
import zlib
from pathlib import Path

import serial
import sys
sys.path.insert(0, str(Path(__file__).parents[1] / "groundstation"))
from protocol import (FrameParser, MSG_DEVICE_INFO, MSG_UPDATE_BEGIN, MSG_UPDATE_DATA,
                      MSG_UPDATE_END, UPDATE_CHUNK_SIZE, build_device_info,
                      build_update_begin, build_update_data, build_update_end, image_crc32,
                      parse_ack, validate_app_image)


def wait_ack(port: serial.Serial, parser: FrameParser, message_type: int, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in parser.feed(port.read(256)):
            ack = parse_ack(frame)
            if ack and ack[0] == message_type:
                return ack
        time.sleep(0.01)
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("image", type=Path)
    args = parser.parse_args()
    image = args.image.read_bytes()
    validate_app_image(image)
    crc = image_crc32(image)
    version = crc
    frame_parser = FrameParser()
    with serial.Serial(args.port, 115200, timeout=0.1) as port:
        port.reset_input_buffer()
        begin = build_update_begin(len(image), crc, version)
        ack = None
        for attempt in range(12):
            port.write(begin)
            ack = wait_ack(port, frame_parser, MSG_UPDATE_BEGIN, 1.0)
            if ack and ack[1] == 0:
                break
            print(f"等待 Bootloader（{attempt + 1}/12）")
            time.sleep(0.25)
        if not ack or ack[1] != 0:
            raise RuntimeError(f"BEGIN 失败：{ack}")
        index = ack[2]
        total = (len(image) + UPDATE_CHUNK_SIZE - 1) // UPDATE_CHUNK_SIZE
        while index < total:
            target = min(total, index + 4)
            for chunk in range(index, target):
                data = image[chunk * UPDATE_CHUNK_SIZE:(chunk + 1) * UPDATE_CHUNK_SIZE]
                port.write(build_update_data(chunk, data))
                time.sleep(0.03)
            ack = wait_ack(port, frame_parser, MSG_UPDATE_DATA, 5.0)
            if not ack or ack[1] not in (0, 2) or ack[2] <= index:
                raise RuntimeError(f"数据块 {index} 确认失败：{ack}")
            index = ack[2]
            print(f"数据 {index}/{total}")
        port.write(build_update_end())
        ack = wait_ack(port, frame_parser, MSG_UPDATE_END, 30.0)
        if not ack or ack[1] != 0:
            raise RuntimeError(f"END 失败：{ack}")
        for _ in range(12):
            time.sleep(0.4)
            port.write(build_device_info())
            deadline = time.monotonic() + 1.3
            while time.monotonic() < deadline:
                for frame in frame_parser.feed(port.read(256)):
                    if frame.message_type == MSG_DEVICE_INFO and len(frame.payload) >= 15:
                        actual_version = int.from_bytes(frame.payload[7:11], "little")
                        actual_crc = int.from_bytes(frame.payload[11:15], "little")
                        if actual_version == version and actual_crc == crc:
                            print(f"OTA 成功：版本={version} CRC=0x{crc:08X}")
                            return
                time.sleep(0.01)
    raise RuntimeError("镜像已提交，但未收到匹配版本/CRC 的 App 确认")


if __name__ == "__main__":
    main()
