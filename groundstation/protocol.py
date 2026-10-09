"""@file protocol.py
@brief Guided Dart LoRa 帧编解码与 OTA 报文构造。
@details 帧层统一使用 CRC16；固件镜像另用 CRC32 校验整包内容。
         轨迹拟合与原始日志解码也复用本模块中的协议数据结构。
@author Guided Dart 项目维护者
@date 2026-10-07
@version 1.0
@note OTA DATA 的 42 字节上限来自当前 58 字节无线帧约束。
"""

from __future__ import annotations

import struct
import math
import re
import time
import zlib
from dataclasses import dataclass
from statistics import median

try:
    import numpy as np
except ImportError:  # 打包环境缺少 NumPy 时，界面仍可使用其他轨迹模式。
    np = None

MAGIC = b"\xA5\x5A"
VERSION = 1
MAX_FRAME = 58
UART_BAUDRATE = 115200
UPDATE_CHUNK_SIZE = 42
# 新版 Bootloader 每 4 片累计确认；片间保留一个空口包时长的安全余量。
UPDATE_WINDOW_SIZE = 4
UPDATE_INTERFRAME_DELAY_S = 0.03
MSG_TELEMETRY = 0x01
MSG_KEEPALIVE = 0x02
MSG_DEVICE_INFO = 0x13
MSG_CALIBRATE_GYRO = 0x14
MSG_UPDATE_BEGIN = 0x20
MSG_UPDATE_DATA = 0x21
MSG_UPDATE_END = 0x22
MSG_ACK = 0x7E
MSG_PARAM_READ = 0x10
MSG_PARAM_WRITE = 0x11
MSG_PARAM_SAVE = 0x12
MSG_LOG_START = 0x30
MSG_LOG_STOP = 0x31
MSG_LOG_INFO = 0x32
MSG_LOG_READ = 0x33
MSG_LOG_DATA = 0x34
TELEMETRY_FLAG_SPEED_ESTIMATED = 0x08
# 固件弹道估计器提供了速度方向时，cx/cy 分别是偏航/俯仰（rad×1000）。
TELEMETRY_FLAG_VELOCITY_DIRECTION = 0x10
# 固件没有真实空速传感器时，5 m/s 只是模型下限。低速遥测采用保守的
# 手持移动标定值，避免把下限直接积分成数十米；真实速度大于下限时仍使用遥测值。
LOW_SPEED_ESTIMATE_MPS = 0.23
# LoRa 遥测允许的最大插值间隔；超过该值通常意味着采集断段，不能假设
# 中间一直保持同一速度。短时丢帧则使用设备时间完整积分，避免距离偏短。
MAX_INTERPOLATION_GAP_S = 1.5
# 固件姿态在高速运动后可能慢于加速度计回正；约 2° 的短时倾角残差
# 不应把已停稳的样本继续当成运动积分。六面静置及往返日志已校验此阈值。
STATIONARY_GYRO_MAX_RAD_S = 0.12
STATIONARY_LINEAR_ACCEL_MAX_M_S2 = 0.35
# 单帧噪声或匀速经过阈值不能立刻触发零速修正。
STATIONARY_MIN_DURATION_MS = 200

# 固件中的 PHASE_* 枚举对应的中文名称，避免用户在界面上看到无意义的数字。
PHASE_NAMES = {
    0: "待机",
    1: "已武装",
    2: "发射",
    3: "稳定",
    4: "目标捕获",
    5: "制导",
    6: "滑翔",
    7: "失效保护",
    8: "结束",
}


def phase_name(phase: int) -> str:
    """将固件阶段枚举转换为中文；未知值保留数值便于诊断。"""
    return PHASE_NAMES.get(phase, f"未知阶段({phase})")
APP_ADDRESS = 0x08004000
APP_END_ADDRESS = 0x0801F800
APP_MAX_SIZE = 110 * 1024


def crc16(data: bytes) -> int:
    """计算与 STM32 相同的 CRC16-CCITT。"""
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def build_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > 45:
        raise ValueError("协议载荷超过 EWM32M 单包容量")
    header = struct.pack("<2sBBHI B", MAGIC, VERSION, message_type,
                         sequence & 0xFFFF, int(time.monotonic() * 1000) & 0xFFFFFFFF,
                         len(payload))
    body = header[2:] + payload
    return header + payload + struct.pack("<H", crc16(body))


@dataclass(slots=True)
class Frame:
    message_type: int
    sequence: int
    timestamp_ms: int
    payload: bytes


class FrameParser:
    """从透明 UART 字节流中恢复完整协议帧。"""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 13:
                break
            version, message_type, sequence, timestamp_ms, payload_length = struct.unpack(
                "<BBHIB", self.buffer[2:11]
            )
            if version != VERSION or payload_length > 45:
                del self.buffer[:2]
                continue
            total = 13 + payload_length
            if len(self.buffer) < total:
                break
            packet = bytes(self.buffer[:total])
            del self.buffer[:total]
            expected = struct.unpack_from("<H", packet, 11 + payload_length)[0]
            if crc16(packet[2:11 + payload_length]) != expected:
                continue
            frames.append(Frame(message_type, sequence, timestamp_ms,
                                packet[11:11 + payload_length]))
        return frames


@dataclass(slots=True)
class Telemetry:
    phase: int
    flags: int
    cx: int
    cy: int
    p: float
    q: float
    r: float
    p_accel: float
    q_accel: float
    r_accel: float
    roll: float
    pitch: float
    yaw: float
    ax: float
    ay: float
    az: float
    servos: tuple[int, int, int, int]
    tgo: float
    flight_time: float
    speed: float
    timestamp_ms: int


def raw_imu_log_prefix_length(samples: list[Telemetry]) -> int:
    """识别旧版 CSV 开头的 200 Hz 原始日志，排除后接的实时遥测。"""
    if len(samples) < 101:
        return 0
    probe = samples[:101]
    if any(sample.servos != (0, 0, 0, 0)
           or sample.p_accel != 0.0 or sample.q_accel != 0.0
           or sample.r_accel != 0.0 for sample in probe):
        return 0
    if sum(0 < current.timestamp_ms - previous.timestamp_ms <= 15
           for previous, current in zip(probe, probe[1:])) < 80:
        return 0
    for index, sample in enumerate(samples):
        if sample.servos != (0, 0, 0, 0):
            return index
    return len(samples)


def trajectory_session_ranges(samples: list[Telemetry]) -> list[tuple[int, int]]:
    """按设备时间戳回退划分采集会话，避免把重启前后的轨迹接成一条。"""
    if not samples:
        return []
    starts = [0] + [index for index in range(1, len(samples))
                    if samples[index].timestamp_ms < samples[index - 1].timestamp_ms]
    return list(zip(starts, starts[1:] + [len(samples)]))


def _interpolable_gap(previous: Telemetry, current: Telemetry, dt_s: float) -> bool:
    """判断长时间间隔是否仍可按端点速度插值。"""
    if not 0.0 < dt_s <= MAX_INTERPOLATION_GAP_S:
        return False
    if dt_s <= 0.5:
        return True
    # 超过 500 ms 只能在同一飞行阶段且两端都有真实运动标志时插值；
    # 阶段切换或待机心跳的长间隔不能被误算成飞行距离。
    return (previous.phase == current.phase
            and previous.speed > 0.0 and current.speed > 0.0
            and previous.flags & TELEMETRY_FLAG_SPEED_ESTIMATED
            and current.flags & TELEMETRY_FLAG_SPEED_ESTIMATED)


@dataclass(slots=True)
class ImuLogInfo:
    state: int
    sample_rate_hz: int
    record_size: int
    record_count: int
    data_bytes: int


def decode_telemetry(frame: Frame) -> Telemetry | None:
    if frame.message_type != MSG_TELEMETRY or len(frame.payload) != 44:
        return None
    data = frame.payload
    phase, flags, cx, cy = struct.unpack_from("<BBhh", data, 0)
    p, q, r = (x / 1000.0 for x in struct.unpack_from("<hhh", data, 6))
    p_accel, q_accel, r_accel = (x / 100.0 for x in struct.unpack_from("<hhh", data, 12))
    roll, pitch, yaw = (x / 1000.0 for x in struct.unpack_from("<hhh", data, 18))
    ax, ay, az = (x / 100.0 for x in struct.unpack_from("<hhh", data, 24))
    servos = struct.unpack_from("<HHHH", data, 30)
    tgo = struct.unpack_from("<H", data, 38)[0] / 100.0
    flight_time = struct.unpack_from("<H", data, 40)[0] / 100.0
    speed = struct.unpack_from("<H", data, 42)[0] / 100.0
    return Telemetry(phase, flags, cx, cy, p, q, r, p_accel, q_accel, r_accel,
                     roll, pitch, yaw, ax, ay, az, servos, tgo, flight_time,
                     speed, frame.timestamp_ms)


def build_log_start(sample_rate_hz: int = 200) -> bytes:
    """请求主控擦除日志区并开始按控制周期记录原始 IMU。"""
    return build_frame(MSG_LOG_START, 0, struct.pack("<H", sample_rate_hz))


def build_log_stop() -> bytes:
    return build_frame(MSG_LOG_STOP, 0)


def build_log_info() -> bytes:
    return build_frame(MSG_LOG_INFO, 0)


def build_log_read(offset: int, length: int = 40, count: int = 1) -> bytes:
    if offset < 0 or not 0 < length <= 40 or not 1 <= count <= 8:
        raise ValueError("原始日志读取长度必须为 1~40 字节，每批 1~8 帧")
    payload = struct.pack("<IB", offset, length)
    return build_frame(MSG_LOG_READ, 0, payload if count == 1 else payload + bytes([count]))


def parse_log_info(frame: Frame) -> ImuLogInfo | None:
    if frame.message_type != MSG_LOG_INFO or len(frame.payload) != 13:
        return None
    state, sample_rate, record_size = struct.unpack_from("<BHH", frame.payload, 0)
    record_count, data_bytes = struct.unpack_from("<II", frame.payload, 5)
    return ImuLogInfo(state, sample_rate, record_size, record_count, data_bytes)


def decode_imu_log_records(data: bytes) -> list[Telemetry]:
    """校验并解码 32 字节原始日志记录，转换为现有轨迹输入格式。"""
    records: list[Telemetry] = []
    record_size = 32
    first_timestamp: int | None = None
    for offset in range(0, len(data) - record_size + 1, record_size):
        record = data[offset:offset + record_size]
        expected = struct.unpack_from("<H", record, 30)[0]
        if crc16(record[:30]) != expected:
            continue
        (timestamp_ms, ax, ay, az, p, q, r, roll, pitch, yaw,
         velocity_yaw, velocity_pitch, speed, phase, log_flags) = struct.unpack(
             "<IhhhhhhhhhhhHBB", record[:30])
        phase &= 0x0F
        flags = 0x01 if log_flags & 0x01 else 0
        if log_flags & 0x02:
            flags |= TELEMETRY_FLAG_VELOCITY_DIRECTION
        # 原始日志中的 speed 是弹道模型输出，不是速度测量；不能据此
        # 宣称当前帧正在运动，也不能让它成为惯导位置的距离锚点。
        if first_timestamp is None:
            first_timestamp = timestamp_ms
        flight_time = max(0.0, (timestamp_ms - first_timestamp) / 1000.0)
        records.append(Telemetry(
            phase, flags, velocity_yaw if log_flags & 0x02 else 0,
            velocity_pitch if log_flags & 0x02 else 0,
            p / 1000.0, q / 1000.0, r / 1000.0,
            0.0, 0.0, 0.0, roll / 1000.0, pitch / 1000.0, yaw / 1000.0,
            ax / 100.0, ay / 100.0, az / 100.0, (0, 0, 0, 0), 0.0, flight_time,
            speed / 100.0, timestamp_ms))
    return records


_TEXT_TELEMETRY = re.compile(
    r"^PH(?P<phase>-?\d+)\s+CX(?P<cx>-?\d+)\s+CY(?P<cy>-?\d+)\s+"
    r"S1(?P<s1>[+-]?\d+)\s+S2(?P<s2>[+-]?\d+)\s+"
    r"S3(?P<s3>[+-]?\d+)\s+S4(?P<s4>[+-]?\d+)$"
)


def decode_text_telemetry(line: str) -> Telemetry | None:
    """解析兼容工程使用的 PH/CX/CY/S1~S4 文本遥测。"""
    match = _TEXT_TELEMETRY.fullmatch(line.strip())
    if match is None:
        return None
    try:
        values = {key: int(value) for key, value in match.groupdict().items()}
    except ValueError:
        return None
    return Telemetry(
        values["phase"], 0, values["cx"], values["cy"],
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        (values["s1"], values["s2"], values["s3"], values["s4"]),
        0.0, 0.0, 0.0, int(time.monotonic() * 1000),
    )


class TextTelemetryParser:
    """从透明串口字节流中提取 CRLF 结尾的文本遥测。"""

    def __init__(self) -> None:
        self.buffer = ""

    def feed(self, data: bytes) -> list[Telemetry]:
        self.buffer += data.decode("ascii", errors="ignore")
        lines = self.buffer.split("\n")
        self.buffer = lines.pop()
        if len(self.buffer) > 512:
            self.buffer = self.buffer[-512:]
        samples: list[Telemetry] = []
        for line in lines:
            sample = decode_text_telemetry(line.rstrip("\r"))
            if sample is not None:
                samples.append(sample)
        return samples


def build_update_begin(size: int, image_crc: int, version: int) -> bytes:
    """按小端序发送镜像长度、CRC32 和版本，触发 Boot 会话建立或续传。"""
    return build_frame(MSG_UPDATE_BEGIN, 0, struct.pack("<III", size, image_crc, version))


def build_update_data(chunk: int, data: bytes) -> bytes:
    """发送带块号和实际长度的数据；末块允许小于 42 字节。"""
    if not 0 < len(data) <= UPDATE_CHUNK_SIZE:
        raise ValueError(f"升级块长度必须为 1~{UPDATE_CHUNK_SIZE} 字节")
    return build_frame(MSG_UPDATE_DATA, chunk, struct.pack("<HB", chunk, len(data)) + data)


def build_update_end() -> bytes:
    """通知 Boot 验证整个备用槽并安装到内部 App 分区。"""
    return build_frame(MSG_UPDATE_END, 0)


def build_param_read(param_id: int) -> bytes:
    return build_frame(MSG_PARAM_READ, 0, bytes((param_id & 0xFF,)))


def build_device_info() -> bytes:
    """请求设备信息，用于在不改变控制状态的情况下测试 LoRa 闭环。"""
    return build_frame(MSG_DEVICE_INFO, 0)


def build_calibrate_gyro() -> bytes:
    """请求主控在静止状态下校准陀螺零偏并保存到外挂 Flash。"""
    return build_frame(MSG_CALIBRATE_GYRO, 0)


def build_param_write(param_id: int, value: float) -> bytes:
    return build_frame(MSG_PARAM_WRITE, 0, bytes((param_id & 0xFF,)) + struct.pack("<f", value))


def build_param_save() -> bytes:
    return build_frame(MSG_PARAM_SAVE, 0)


def parse_ack(frame: Frame) -> tuple[int, int, int] | None:
    """将 ACK 载荷解析为原始类型、状态码和 Boot 的下一待发块号。"""
    if frame.message_type != MSG_ACK or len(frame.payload) != 4:
        return None
    next_chunk, original_type, status = struct.unpack("<HBB", frame.payload)
    return original_type, status, next_chunk


def image_crc32(image: bytes) -> int:
    """计算与 Boot 端反射式 CRC32 实现一致的整镜像校验值。"""
    return zlib.crc32(image) & 0xFFFFFFFF


def validate_app_image(image: bytes) -> None:
    """拒绝整片镜像、错误链接地址和超过 App 分区容量的固件。"""
    if not 8 <= len(image) <= APP_MAX_SIZE:
        raise ValueError("App BIN 必须包含向量表且不超过 110 KB")
    stack_pointer, reset_handler = struct.unpack_from("<II", image, 0)
    reset_handler &= ~1
    if not 0x20000000 <= stack_pointer <= 0x20007FE0 or stack_pointer % 8:
        raise ValueError("App BIN 的初始栈地址不在 STM32G431 RAM 范围")
    if not APP_ADDRESS <= reset_handler < APP_END_ADDRESS:
        raise ValueError("App BIN 的复位入口不在 App 分区")


def _angle_delta(current: float, reference: float) -> float:
    """计算两个航向角的最短差值，避免从 +π 跳到 -π 造成轨迹尖峰。"""
    return (current - reference + math.pi) % (2.0 * math.pi) - math.pi


def _attitude_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """按固件相同的 Z-Y-X 顺序生成姿态四元数。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def _heading_origin_inverse(yaw: float) -> tuple[float, float, float, float]:
    """水平轴对齐起始航向，同时保持 Z 轴与重力方向一致。"""
    return _attitude_quaternion(0.0, 0.0, -yaw)


def _quaternion_multiply(a: tuple[float, float, float, float],
                        b: tuple[float, float, float, float]
                        ) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _rotate_vector(quaternion: tuple[float, float, float, float],
                   vector: tuple[float, float, float]
                   ) -> tuple[float, float, float]:
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, (0.0, *vector)),
        (quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]))
    return rotated[1], rotated[2], rotated[3]


def _stabilized_yaw(samples: list[Telemetry]) -> list[float]:
    """静止时锁住偏航显示，避免六轴 IMU 零偏继续积累。"""
    if not samples:
        return []
    values = [samples[0].yaw]
    held = samples[0].yaw
    for sample in samples[1:]:
        if not _imu_stationary(sample):
            held = sample.yaw
        values.append(held)
    return values


def _imu_stationary(sample: Telemetry) -> bool:
    """仅依据惯性观测判静止；模型速度和飞行阶段不能代替物理观测。"""
    gyro_norm = math.sqrt(sample.p * sample.p + sample.q * sample.q
                          + sample.r * sample.r)
    if gyro_norm > STATIONARY_GYRO_MAX_RAD_S:
        return False
    q = _attitude_quaternion(sample.roll, sample.pitch, sample.yaw)
    accel_world = _rotate_vector(q, (sample.ax, sample.ay, sample.az))
    # 只检查模长会漏掉水平平移：加速度计仍可能接近 1 g。
    linear = (accel_world[0], accel_world[1], accel_world[2] + 9.80665)
    return (math.sqrt(sum(component * component for component in linear))
            <= STATIONARY_LINEAR_ACCEL_MAX_M_S2)


def _sustained_stationary_mask(samples: list[Telemetry]) -> list[bool]:
    """只保留持续静止区间，避免运动中偶发低加速度清零速度。"""
    mask = [False] * len(samples)
    start = 0
    while start < len(samples):
        if not _imu_stationary(samples[start]):
            start += 1
            continue
        end = start + 1
        while end < len(samples) and _imu_stationary(samples[end]):
            gap = samples[end].timestamp_ms - samples[end - 1].timestamp_ms
            if gap < 0 or gap > 150:
                break
            end += 1
        if samples[end - 1].timestamp_ms - samples[start].timestamp_ms >= STATIONARY_MIN_DURATION_MS:
            mask[start:end] = [True] * (end - start)
        start = end
    return mask


def _endpoint_stationary_mask(samples: list[Telemetry]) -> list[bool]:
    """只将采集两端的持续静止段用作零速约束。

    匀速滑行与静止的六轴惯性观测相同；运动中间的低加速度平台
    不能据此清零速度，否则会把高速滑行距离压成零。
    """
    observed = _sustained_stationary_mask(samples)
    # 静止尾段偶有单帧阈值抖动；仅补回不超过两帧的弱扰动，
    # 不跨越真实加减速脉冲或较长的运动区间。
    index = 0
    while index < len(observed):
        if observed[index]:
            index += 1
            continue
        end_gap = index
        while end_gap < len(observed) and not observed[end_gap]:
            end_gap += 1
        if (0 < index and end_gap < len(observed) and end_gap - index <= 2
                and all(abs(math.sqrt(sample.ax * sample.ax + sample.ay * sample.ay
                                      + sample.az * sample.az) - 9.80665) <= 0.35
                        and math.sqrt(sample.p * sample.p + sample.q * sample.q
                                      + sample.r * sample.r) <= STATIONARY_GYRO_MAX_RAD_S
                        for sample in samples[index:end_gap])):
            observed[index:end_gap] = [True] * (end_gap - index)
        index = end_gap
    begin = 0
    while begin < len(observed) and observed[begin]:
        begin += 1
    end = len(observed)
    while end > begin and observed[end - 1]:
        end -= 1
    return [index < begin or index >= end for index in range(len(observed))]


def _trajectory_stationary_mask(samples: list[Telemetry]) -> list[bool]:
    """飞行记录保护匀速段，手持标定记录仍允许中途停稳校正。"""
    if any(sample.phase in (2, 3, 4, 5, 6, 7) for sample in samples):
        return _endpoint_stationary_mask(samples)
    return _sustained_stationary_mask(samples)


def has_terminal_stationary_interval(samples: list[Telemetry], duration_ms: int = 2000) -> bool:
    """检查末尾两秒是否基本静止，允许少量无线噪声帧。"""
    if len(samples) < 2:
        return False
    cutoff = samples[-1].timestamp_ms - duration_ms
    index = len(samples) - 1
    while index > 0 and samples[index - 1].timestamp_ms >= cutoff:
        index -= 1
    window = samples[max(0, index - 1):]
    if window[0].timestamp_ms > cutoff or any(
            not 0 <= current.timestamp_ms - previous.timestamp_ms <= 1500
            for previous, current in zip(window, window[1:])):
        return False
    return sum(_imu_stationary(sample) for sample in window) >= 0.9 * len(window)


def _gyro_attitudes(samples: list[Telemetry], stationary: list[bool]
                    ) -> list[tuple[float, float, float, float]]:
    """静止时锚定重力姿态，运动时只积分陀螺，避免平移加速度拉偏姿态。"""
    if not samples:
        return []
    q = _attitude_quaternion(samples[0].roll, samples[0].pitch, samples[0].yaw)
    result = [q]
    for index in range(1, len(samples)):
        current, previous = samples[index], samples[index - 1]
        dt = (current.timestamp_ms - previous.timestamp_ms) / 1000.0
        if stationary[index] or not 0.0 < dt <= 0.15:
            q = _attitude_quaternion(current.roll, current.pitch, current.yaw)
        else:
            rate = tuple(0.5 * (getattr(previous, axis) + getattr(current, axis))
                         for axis in ("p", "q", "r"))
            rate_norm = math.sqrt(sum(value * value for value in rate))
            half_angle = 0.5 * rate_norm * dt
            factor = math.sin(half_angle) / rate_norm if rate_norm > 1.0e-9 else 0.5 * dt
            delta = (math.cos(half_angle), *(factor * value for value in rate))
            q = _quaternion_multiply(q, delta)
            norm = math.sqrt(sum(value * value for value in q))
            q = tuple(value / norm for value in q)
        result.append(q)
    return result


def inertial_capture_warnings(samples: list[Telemetry]) -> list[str]:
    """提示缺少零速边界的采集，避免把不可观测的初速度当成零。"""
    if not samples:
        return []
    warnings = []
    intervals = [current.timestamp_ms - previous.timestamp_ms
                 for previous, current in zip(samples, samples[1:])
                 if current.timestamp_ms > previous.timestamp_ms]
    if len(intervals) >= 30:
        typical_interval = median(intervals)
        if typical_interval > 20:
            warnings.append(f"采样间隔中位数 {typical_interval:.0f} ms，"
                            "低频遥测可能漏掉高速加减速脉冲，IMU 位移不可信")
    long_gaps = [gap for gap in intervals if gap > MAX_INTERPOLATION_GAP_S * 1000.0]
    if long_gaps:
        warnings.append(f"采样中断 {len(long_gaps)} 处，最长 {max(long_gaps) / 1000.0:.1f} 秒；"
                        "断层内位移无法从 IMU 日志恢复")
    if len(trajectory_session_ranges(samples)) > 1:
        warnings.append("检测到设备时间戳回退，已按独立会话重置轨迹零点")
    mask = _sustained_stationary_mask(samples)
    if all(_imu_stationary(sample) for sample in samples):
        return warnings
    if not mask[0]:
        warnings.append("起始缺少持续静止段，初速度未知，位移不可信")
    else:
        start_window = []
        for sample in samples:
            elapsed = sample.timestamp_ms - samples[0].timestamp_ms
            if elapsed < 0 or elapsed > 2000:
                break
            start_window.append(sample)
        if (start_window[-1].timestamp_ms - start_window[0].timestamp_ms < 1900
                or sum(_imu_stationary(sample) for sample in start_window)
                < 0.9 * len(start_window)):
            warnings.append("起始静止不足约 2 秒，姿态和零偏基线较弱，位移需谨慎对照")
    if not mask[-1]:
        warnings.append("结束缺少持续静止段，无法进行末速度校正")
    if (any(sample.phase in (2, 3, 4, 5, 6, 7) for sample in samples)
            and any(raw and not selected for raw, selected in zip(
                mask, _endpoint_stationary_mask(samples)))):
        warnings.append("飞行中低加速度段可能是匀速滑行，未用作零速约束；若确实停稳请分段采集")
    # 停稳后重力方向可独立检验运动段的陀螺积分姿态；不借用已知终点位置。
    trajectory_mask = _trajectory_stationary_mask(samples)
    attitudes = _gyro_attitudes(samples, trajectory_mask)
    largest_mismatch = (0.0, 0.0, 0.0)
    for index in range(1, len(samples)):
        if not trajectory_mask[index] or trajectory_mask[index - 1]:
            continue
        settled = []
        end = index
        while (end < len(samples) and trajectory_mask[end]
               and samples[end].timestamp_ms - samples[index].timestamp_ms <= 500):
            if samples[end].timestamp_ms - samples[index].timestamp_ms >= 100:
                settled.append(samples[end])
            end += 1
        if not settled or settled[-1].timestamp_ms - settled[0].timestamp_ms < 200:
            continue
        gravity_body = tuple(median(getattr(sample, axis) for sample in settled)
                             for axis in ("ax", "ay", "az"))
        gravity_world = _rotate_vector(attitudes[index - 1], gravity_body)
        mismatch = math.hypot(gravity_world[0], gravity_world[1])
        tilt_degrees = math.degrees(math.atan2(mismatch, -gravity_world[2]))
        if mismatch > largest_mismatch[0]:
            largest_mismatch = (mismatch,
                                (samples[index].timestamp_ms - samples[0].timestamp_ms) / 1000.0,
                                tilt_degrees)
    if largest_mismatch[0] > 0.25:
        warnings.append(f"第 {largest_mismatch[1]:.1f} 秒停稳时姿态与重力不一致"
                        f"（约 {largest_mismatch[2]:.1f}°），"
                        "该运动段的方向和距离可能明显漂移")
    return warnings


def _forward_velocity(sample: Telemetry,
                      origin: tuple[float, float, float] | None,
                      low_speed_mps: float = LOW_SPEED_ESTIMATE_MPS,
                      yaw_override: float | None = None,
                      estimated_speed_scale: float = 1.0
                      ) -> tuple[float, float, float]:
    """把机体前向速度投影到重力对齐、起始航向为水平参考的坐标系。"""
    # 待机、已武装和结束态的 speed 是模型初值/下限，不代表物体仍在运动。
    # 只有进入发射至滑翔阶段后才允许参与位置积分，避免静止采样出现假位移。
    if sample.phase not in (2, 3, 4, 5, 6, 7):
        return 0.0, 0.0, 0.0
    # 发射判定后重新静止时，固件仍可能保留速度模型的 5 m/s 下限；
    # 用角速度和加速度共同判断静止，避免把这段数据积分成虚假位移。
    accel_mag = math.sqrt(sample.ax * sample.ax + sample.ay * sample.ay
                          + sample.az * sample.az)
    gyro_mag = math.sqrt(sample.p * sample.p + sample.q * sample.q
                         + sample.r * sample.r)
    # 固件已用同一套静止判定设置了 0x08；该标志比上位机再次用阈值
    # 判定更可靠，尤其是低速平移时加速度仍接近 1 g 的场景。
    if not ((sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED) and sample.speed > 0.0) \
            and abs(accel_mag - 9.80665) <= 1.4 and gyro_mag <= 0.25:
        return 0.0, 0.0, 0.0
    _, _, origin_yaw = origin if origin is not None else (0.0, 0.0, 0.0)
    # 将速度方向旋转到重力对齐的起始航向系；固件提供弹道估计方向时优先使用，
    # 它包含加速度造成的侧滑变化；旧数据仍退回机体 +X 前向轴。
    current_yaw = sample.yaw if yaw_override is None else yaw_override
    current_q = _attitude_quaternion(sample.roll, sample.pitch, current_yaw)
    origin_inverse = _heading_origin_inverse(origin_yaw)
    if sample.flags & TELEMETRY_FLAG_VELOCITY_DIRECTION:
        direction_yaw = sample.cx / 1000.0
        direction_pitch = sample.cy / 1000.0
        forward_nav = (math.cos(direction_pitch) * math.cos(direction_yaw),
                       math.cos(direction_pitch) * math.sin(direction_yaw),
                       math.sin(direction_pitch))
        forward_relative = _rotate_vector(origin_inverse, forward_nav)
    else:
        # 姿态四元数定义为机体系到导航系；速度方向与加速度去重力必须
        # 使用同一个旋转方向，避免俯仰或横滚时轨迹镜像。
        forward_nav = _rotate_vector(current_q, (1.0, 0.0, 0.0))
        forward_relative = _rotate_vector(origin_inverse, forward_nav)
    speed = sample.speed
    if (sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED) and 0.0 < speed <= 5.01:
        speed = low_speed_mps * max(0.05, estimated_speed_scale)
    return tuple(speed * value for value in forward_relative)


def estimate_trajectory(samples: list[Telemetry],
                        origin: tuple[float, float, float] | tuple[float, float] | None = None,
                        estimated_speed_scale: float = 1.0
                        ) -> tuple[list[float], list[float], list[float]]:
    """以起始航向为水平参考，用梯形积分拟合相对轨迹。

    速度遥测是标量，不能恢复完整真实位置；这里采用相邻两帧速度方向的
    梯形积分，相比只使用当前帧可明显减小 10~30 Hz 采样造成的折线抖动。
    Z 轴始终保持竖直；旧版二元起点 (俯仰、偏航) 仍兼容。
    """
    if not samples:
        return [], [], []
    velocities = _estimate_model_velocities(samples, origin, estimated_speed_scale)
    x, y, z = [0.0], [0.0], [0.0]
    for index, (previous, current) in enumerate(zip(samples, samples[1:])):
        # 设备重启或重新开始采集后，新的会话从相对零点重新绘制。
        if current.timestamp_ms < previous.timestamp_ms:
            x.append(0.0)
            y.append(0.0)
            z.append(0.0)
            continue
        raw_dt = (current.timestamp_ms - previous.timestamp_ms) / 1000.0
        if not _interpolable_gap(previous, current, raw_dt):
            x.append(x[-1])
            y.append(y[-1])
            z.append(z[-1])
            continue
        # 只要间隔仍在协议允许的 500 ms 内，就使用设备真实时间。
        # 这里再截成 200 ms 会把 LoRa 丢包后的有效运动时间直接丢掉，
        # 造成累计距离偏短，严重时会接近实际值的一半。
        dt = raw_dt
        previous_velocity = velocities[index]
        current_velocity = velocities[index + 1]
        x.append(x[-1] + 0.5 * (previous_velocity[0] + current_velocity[0]) * dt)
        y.append(y[-1] + 0.5 * (previous_velocity[1] + current_velocity[1]) * dt)
        z.append(z[-1] + 0.5 * (previous_velocity[2] + current_velocity[2]) * dt)
    return x, y, z


def _estimate_model_velocities(
        samples: list[Telemetry],
        origin: tuple[float, float, float] | tuple[float, float] | None,
        estimated_speed_scale: float,
        ) -> list[tuple[float, float, float]]:
    """生成与速度积分一致的逐帧速度，供 IMU/连续拟合作为距离先验。"""
    if not samples:
        return []
    if origin is None:
        origin = (samples[0].roll, samples[0].pitch, samples[0].yaw)
    if len(origin) == 2:
        origin = (0.0, origin[0], origin[1])
    estimated_indices = [index for index, sample in enumerate(samples)
                         if sample.speed > 0.0
                         and (sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED)]
    segment_count = 0
    previous_index = None
    for index in estimated_indices:
        if previous_index is None or samples[index].timestamp_ms - samples[previous_index].timestamp_ms > 1000:
            segment_count += 1
        previous_index = index
    low_speed_mps = LOW_SPEED_ESTIMATE_MPS if segment_count <= 1 else 0.28
    yaw_values = _stabilized_yaw(samples)
    result: list[tuple[float, float, float]] = []
    session_origin = origin
    previous_velocity = (0.0, 0.0, 0.0)
    held_direction: tuple[float, float, float] | None = None
    for index, sample in enumerate(samples):
        if index == 0:
            previous_velocity = _forward_velocity(sample, session_origin, low_speed_mps,
                                                  yaw_values[index], estimated_speed_scale)
            motion = any(abs(value) > 1.0e-8 for value in previous_velocity)
            if motion:
                norm = math.sqrt(sum(value * value for value in previous_velocity))
                held_direction = tuple(value / norm for value in previous_velocity)
            result.append(previous_velocity)
            continue
        previous = samples[index - 1]
        if sample.timestamp_ms < previous.timestamp_ms:
            session_origin = (sample.roll, sample.pitch, yaw_values[index])
            held_direction = None
            previous_velocity = _forward_velocity(sample, session_origin, low_speed_mps,
                                                  yaw_values[index], estimated_speed_scale)
        else:
            raw_dt = (sample.timestamp_ms - previous.timestamp_ms) / 1000.0
            if _interpolable_gap(previous, sample, raw_dt):
                current = _forward_velocity(sample, session_origin, low_speed_mps,
                                            yaw_values[index], estimated_speed_scale)
                current_motion = any(abs(value) > 1.0e-8 for value in current)
                previous_motion = any(abs(value) > 1.0e-8 for value in previous_velocity)
                if current_motion:
                    norm = math.sqrt(sum(value * value for value in current))
                    explicit_direction = bool(sample.flags & TELEMETRY_FLAG_VELOCITY_DIRECTION)
                    direction_change = 1.0
                    if held_direction is not None:
                        direction_change = sum(current[axis] / norm * held_direction[axis]
                                               for axis in range(3))
                    # 小幅变化多半是六轴偏航漂移，较大的真实转弯仍应
                    # 立即更新速度方向；保留约 7° 的抗漂移死区。
                    significant_turn = direction_change < math.cos(0.12)
                    if (held_direction is None or not previous_motion
                            or explicit_direction or significant_turn):
                        held_direction = tuple(value / norm for value in current)
                    elif held_direction is not None:
                        # 没有固件速度方向时，保持上一帧导航速度方向；
                        # 避免机体偏航漂移直接被误画成横向位移。
                        current = tuple(norm * value for value in held_direction)
                else:
                    held_direction = None
                previous_velocity = current
        result.append(previous_velocity)
    return result


def estimate_inertial_trajectory(
        samples: list[Telemetry],
        origin: tuple[float, float, float] | tuple[float, float] | None = None,
        ) -> tuple[list[float], list[float], list[float]]:
    """只用惯性样本和起止 ZUPT 估计相对位移。

    手持记录按静止-运动分段；飞行记录仅在采集起止静止段置零，
    避免把中途匀速滑行误判为静止。运动段先积分，再利用已确认的
    末端静止条件消除速度零偏；保留固件低通后的加速度冲量。
    模型速度不参与此模式；没有外部位置参考时不能替代真实定位。
    """
    if not samples:
        return [], [], []
    # 时间戳回退表示设备重启或新采集；各会话必须独立确定零点和静止基线。
    session_ranges = trajectory_session_ranges(samples)
    if len(session_ranges) > 1:
        result = ([], [], [])
        for segment_index, (begin, end) in enumerate(session_ranges):
            segment = estimate_inertial_trajectory(
                samples[begin:end], origin if segment_index == 0 else None)
            for output, values in zip(result, segment):
                output.extend(values)
        return result
    if origin is None:
        origin = (samples[0].roll, samples[0].pitch, samples[0].yaw)
    elif len(origin) == 2:
        origin = (0.0, origin[0], origin[1])

    q_origin_inv = _heading_origin_inverse(origin[2])

    stationary_mask = _trajectory_stationary_mask(samples)
    if all(_imu_stationary(sample) for sample in samples):
        stationary_mask = [True] * len(samples)
    attitudes = _gyro_attitudes(samples, stationary_mask)

    # 静止残差包含传感器零偏和姿态误差；先在机体系估计并扣除，避免
    # 把随机体转动的残差误当成固定导航系向量，制造横向加速度。
    static_residuals = []
    for sample, is_static in zip(samples, stationary_mask):
        if not is_static:
            continue
        q = _attitude_quaternion(sample.roll, sample.pitch, sample.yaw)
        gravity_body = _rotate_vector((q[0], -q[1], -q[2], -q[3]),
                                      (0.0, 0.0, -9.80665))
        static_residuals.append(tuple(value - expected for value, expected in zip(
            (sample.ax, sample.ay, sample.az), gravity_body)))
    body_bias = tuple(median(value[axis] for value in static_residuals)
                      if static_residuals else 0.0 for axis in range(3))

    def raw_acceleration(index: int, sample: Telemetry) -> tuple[float, float, float]:
        corrected_body = tuple(value - body_bias[axis] for axis, value in enumerate(
            (sample.ax, sample.ay, sample.az)))
        world = _rotate_vector(attitudes[index], corrected_body)
        # 传感器静止时测得约 [0, 0, -g]，补回世界坐标重力。
        linear_world = (world[0], world[1], world[2] + 9.80665)
        return _rotate_vector(q_origin_inv, linear_world)

    raw = [raw_acceleration(index, sample) for index, sample in enumerate(samples)]

    # 固件已做低通；再次取三点中值会削短短促加减速脉冲的有效冲量。
    calibration = [raw[index] for index, is_static in enumerate(stationary_mask)
                   if is_static]
    # 没有静止基线时，首批样本可能正在加速；不能把真实加速度当成零偏扣除。
    bias = tuple(median([value[axis] for value in calibration]) if calibration else 0.0
                 for axis in range(3))

    # 固件已做低通，且传感器量程高于 25 m/s²；不能把真实发射加速度
    # 当作异常值截断，否则高速运动的积分距离必然偏短。
    acceleration = [tuple(value[axis] - bias[axis] for axis in range(3))
                    for value in raw]

    x, y, z = [0.0] * len(samples), [0.0] * len(samples), [0.0] * len(samples)
    position = [0.0, 0.0, 0.0]
    index = 0
    while index < len(samples):
        if stationary_mask[index]:
            x[index], y[index], z[index] = position
            index += 1
            continue

        # 找到一个完整运动段的静止边界；若末尾没有静止帧，则不强行做
        # 末速度校正，避免把真实的尚未结束运动压成零位移。
        start = max(0, index - 1) if index > 0 and stationary_mask[index - 1] else index
        end = index
        while end + 1 < len(samples):
            dt = (samples[end + 1].timestamp_ms - samples[end].timestamp_ms) / 1000.0
            if samples[end + 1].timestamp_ms < samples[end].timestamp_ms or not _interpolable_gap(samples[end], samples[end + 1], dt):
                break
            if stationary_mask[end + 1]:
                break
            end += 1
        boundary = end + 1 if end + 1 < len(samples) and stationary_mask[end + 1] else end
        local_len = boundary - start + 1
        velocities = [[0.0, 0.0, 0.0] for _ in range(local_len)]
        local_times = [0.0] * local_len
        for local in range(1, local_len):
            previous_index = start + local - 1
            current_index = start + local
            dt = (samples[current_index].timestamp_ms
                  - samples[previous_index].timestamp_ms) / 1000.0
            if not _interpolable_gap(samples[previous_index], samples[current_index], dt):
                dt = 0.0
            local_times[local] = local_times[local - 1] + dt
            for axis in range(3):
                integrated = velocities[local - 1][axis] + 0.5 * (
                    acceleration[previous_index][axis] + acceleration[current_index][axis]) * dt
                velocities[local][axis] = integrated
        # 末端确认为静止时，将残余速度按时间线性分配回运动段；
        # 这只校正速度漂移，不强迫末端位置回到起点。
        correct_drift = boundary > end and local_times[-1] > 0.0
        final_velocity = velocities[-1]
        local_position = [[0.0, 0.0, 0.0] for _ in range(local_len)]
        for local in range(1, local_len):
            dt = local_times[local] - local_times[local - 1]
            fraction = local_times[local] / local_times[-1] if correct_drift else 0.0
            for axis in range(3):
                previous_velocity = velocities[local - 1][axis] - final_velocity[axis] * (
                    local_times[local - 1] / local_times[-1] if correct_drift else 0.0)
                current_velocity = velocities[local][axis] - final_velocity[axis] * fraction
                local_position[local][axis] = local_position[local - 1][axis] + 0.5 * (
                    previous_velocity + current_velocity) * dt

        for local in range(local_len):
            output_index = start + local
            x[output_index] = position[0] + local_position[local][0]
            y[output_index] = position[1] + local_position[local][1]
            z[output_index] = position[2] + local_position[local][2]
        position = [x[boundary], y[boundary], z[boundary]]
        index = boundary + 1

    return x, y, z


def _bspline_basis_and_derivatives(
        knots: np.ndarray, degree: int, value: float
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """按 Cox-de Boor 递推计算基函数及对归一化时间的一、二阶导数。"""
    right = float(knots[-degree - 1])
    point = min(max(float(value), float(knots[degree])), math.nextafter(right, -math.inf))
    basis = np.asarray([float(knots[index] <= point < knots[index + 1])
                        for index in range(len(knots) - 1)])
    first = np.zeros_like(basis)
    second = np.zeros_like(basis)
    for order in range(1, degree + 1):
        size = len(knots) - order - 1
        next_basis = np.zeros(size)
        next_first = np.zeros(size)
        next_second = np.zeros(size)
        for index in range(size):
            left_span = knots[index + order] - knots[index]
            right_span = knots[index + order + 1] - knots[index + 1]
            if left_span > 0.0:
                weight = (point - knots[index]) / left_span
                slope = 1.0 / left_span
                next_basis[index] += weight * basis[index]
                next_first[index] += slope * basis[index] + weight * first[index]
                next_second[index] += 2.0 * slope * first[index] + weight * second[index]
            if right_span > 0.0:
                weight = (knots[index + order + 1] - point) / right_span
                slope = -1.0 / right_span
                next_basis[index] += weight * basis[index + 1]
                next_first[index] += slope * basis[index + 1] + weight * first[index + 1]
                next_second[index] += 2.0 * slope * first[index + 1] + weight * second[index + 1]
        basis, first, second = next_basis, next_first, next_second
    if value >= right:
        basis[:] = 0.0
        basis[-1] = 1.0
    return basis, first, second


def estimate_continuous_trajectory(
        samples: list[Telemetry],
        origin: tuple[float, float, float] | tuple[float, float] | None = None,
        estimated_speed_scale: float = 1.0,
        _split_sessions: bool = True,
        ) -> tuple[list[float], list[float], list[float]]:
    """用连续时间三次 B 样条拟合 IMU 加速度和速度方向。

    这是地面站的离线平滑器，不是实时导航器：先把整段采样变成统一的
    时间轴，再以 B 样条控制点为未知量，同时最小化加速度残差、可用的
    速度模型残差、静止段零速度约束和控制点平滑项。曲线的速度、加速度
    来自 B 样条的一阶、二阶导数，因此不会再对每个离散点重复
    积分。没有 NumPy 或有效时间跨度时，退回已有的分段 ZUPT 结果。

    六轴 IMU 没有位置、空速和航向绝对参考；B 样条只能改善连续性和抗噪性，
    不能凭空消除姿态与零偏误差。原始加速度仍有物理单位；没有速度模型时
    当前稀疏控制点会削短高速脉冲，因此退回分段 ZUPT 而不伪称完成优化。
    """
    if np is None or len(samples) < 4:
        return estimate_inertial_trajectory(samples, origin)
    if origin is None:
        origin = (samples[0].roll, samples[0].pitch, samples[0].yaw)
    elif len(origin) == 2:
        origin = (0.0, origin[0], origin[1])

    # 设备重启或重新开始采集会让毫秒计数回退。不能像旧实现那样只保留
    # 第一段，否则后半段完整飞行数据会在 B 样条模式中静默丢失；每个会话
    # 独立建立时间轴，结果在回退点重新从零开始，与速度积分模式保持一致。
    if _split_sessions:
        session_ranges = trajectory_session_ranges(samples)
        if len(session_ranges) > 1:
            output_x: list[float] = []
            output_y: list[float] = []
            output_z: list[float] = []
            for segment_index, (begin, end) in enumerate(session_ranges):
                segment = samples[begin:end]
                segment_origin = origin if segment_index == 0 else (
                    segment[0].roll, segment[0].pitch, segment[0].yaw)
                segment_x, segment_y, segment_z = estimate_continuous_trajectory(
                    segment, segment_origin, estimated_speed_scale, False)
                output_x.extend(segment_x)
                output_y.extend(segment_y)
                output_z.extend(segment_z)
            return output_x, output_y, output_z

    # 没有速度模型时，少量控制点无法保留 200 Hz 日志中的短脉冲；
    # 这里沿用分段 ZUPT，避免样条平滑把真实位移压短。
    if not any(sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED and sample.speed > 0.0
               for sample in samples):
        return estimate_inertial_trajectory(samples, origin)

    # 无法跨越的采样空洞应退回分段 ZUPT，不能静默截掉后半段数据。
    times = [0.0]
    for previous, current in zip(samples, samples[1:]):
        dt = (current.timestamp_ms - previous.timestamp_ms) / 1000.0
        if not _interpolable_gap(previous, current, dt):
            return estimate_inertial_trajectory(samples, origin)
        times.append(times[-1] + dt)
    count = len(times)
    if count < 4 or times[-1] < 0.15:
        return estimate_inertial_trajectory(samples, origin)
    samples = samples[:count]
    duration = times[-1]
    time_array = np.asarray(times, dtype=float)
    normalized = np.clip(time_array / duration, 0.0, 1.0)

    def quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        return _attitude_quaternion(roll, pitch, yaw)

    q_origin_inverse = _heading_origin_inverse(origin[2])
    yaw_values = _stabilized_yaw(samples)

    def body_acceleration(sample: Telemetry, index: int) -> tuple[float, float, float]:
        current = quaternion(sample.roll, sample.pitch, yaw_values[index])
        world = _rotate_vector(current, (sample.ax, sample.ay, sample.az))
        # 静止时加速度计约为 -g，补回世界坐标重力后再变换到起始坐标系。
        return _rotate_vector(q_origin_inverse, (world[0], world[1], world[2] + 9.80665))

    raw_acceleration = [body_acceleration(sample, index)
                        for index, sample in enumerate(samples)]

    def median(values: list[float]) -> float:
        ordered = sorted(values)
        if not ordered:
            return 0.0
        middle = len(ordered) // 2
        return ordered[middle] if len(ordered) & 1 else 0.5 * (ordered[middle - 1] + ordered[middle])

    # 此模式显式拟合模型速度；模型标志与惯性静止相冲突时由模型先验决定。
    # 纯 IMU 的 ZUPT 模式不使用该标志，不能混用两种静止定义。
    stationary_mask = [_imu_stationary(sample) and not (
        sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED and sample.speed > 0.0)
        for sample in samples]
    # 整段都是静止采样时，物理结果就是零位移；不让噪声残差制造一条
    # 看似平滑但实际不存在的闭合曲线。
    if all(stationary_mask):
        zeros = [0.0] * count
        return zeros.copy(), zeros.copy(), zeros
    static_values = [value for value, is_static in zip(raw_acceleration, stationary_mask) if is_static]
    bias = tuple(median([value[axis] for value in static_values]) for axis in range(3))
    accelerations = np.asarray([[value[axis] - bias[axis] for axis in range(3)]
                                for value in raw_acceleration], dtype=float)
    # 不对加速度做固定限幅：高速发射的真实脉冲可能超过 25 m/s²。

    # 控制点数量随时长增加，但限制在 8~24 个，保证普通电脑和 EXE 都能快速完成。
    control_count = max(8, min(24, int(duration * 1.5) + 4))
    degree = 3
    knots = np.r_[np.zeros(degree + 1),
                  np.linspace(0.0, 1.0, control_count - degree + 1)[1:-1],
                  np.ones(degree + 1)]
    evaluations = [_bspline_basis_and_derivatives(knots, degree, value)
                   for value in normalized]
    basis_matrix = np.vstack([item[0] for item in evaluations])
    first_matrix = np.vstack([item[1] for item in evaluations]) / duration
    second_matrix = np.vstack([item[2] for item in evaluations]) / (duration * duration)

    # 速度模型只提供标量和方向；只有明确有速度的帧参与。带有固件弹道
    # 方向字段时，方向和时间序列都比较可靠，权重可以提高；旧遥测只有
    # 机体前向推断，仍保留较低权重，避免姿态漂移把横向轨迹拉偏。
    velocity_observed = np.zeros((count, 3), dtype=float)
    velocity_valid = np.zeros(count, dtype=bool)
    model_velocities = _estimate_model_velocities(samples, origin, estimated_speed_scale)
    has_explicit_direction = any(
        sample.flags & TELEMETRY_FLAG_VELOCITY_DIRECTION for sample in samples)
    # 速度先验不仅约束样条导数，还生成一条同坐标系的参考位置。仅靠
    # 导数约束时，样条为了满足起点静止和高阶平滑会把恒速段压缩成半程；
    # 位置先验把可观测的距离比例保留下来，同时仍允许 IMU 加速度修正局部
    # 转弯和波动。
    model_positions = np.zeros((count, 3), dtype=float)
    for index in range(1, count):
        dt = times[index] - times[index - 1]
        model_positions[index] = model_positions[index - 1] + 0.5 * dt * (
            np.asarray(model_velocities[index - 1])
            + np.asarray(model_velocities[index]))
    for index, (sample, velocity) in enumerate(zip(samples, model_velocities)):
        if any(abs(value) > 1.0e-8 for value in velocity):
            velocity_observed[index] = velocity
            velocity_valid[index] = True
    # 有速度模型时，IMU 加速度主要负责修正局部形状；没有速度模型时，
    # 加速度是唯一的运动观测，必须提高其拟合权重，否则样条会被平滑项
    # 拉成反向或接近零位移。
    acceleration_weight = 0.10 if np.any(velocity_valid) else 0.35
    rows: list[np.ndarray] = [basis_matrix[0] * 10.0]
    targets = [np.zeros(3, dtype=float)]
    for index in range(count):
        # 速度模型没有真实比例，权重只作为方向/趋势提示，不能压过 IMU 物理约束。
        weight = (0.95 if has_explicit_direction else 0.70) if velocity_valid[index] else 0.0
        if weight:
            rows.append(first_matrix[index] * weight)
            targets.append(velocity_observed[index] * weight)
            position_weight = 1.60 if has_explicit_direction else 1.35
            rows.append(basis_matrix[index] * position_weight)
            targets.append(model_positions[index] * position_weight)
        if stationary_mask[index]:
            rows.append(first_matrix[index] * 0.8)
            targets.append(np.zeros(3, dtype=float))
        rows.append(second_matrix[index] * acceleration_weight)
        targets.append(accelerations[index] * acceleration_weight)
    # 三阶差分正则化，抑制无线抖动但保留持续加速度变化。
    for index in range(control_count - 3):
        row = np.zeros(control_count, dtype=float)
        row[index:index + 4] = (-1.0, 3.0, -3.0, 1.0)
        rows.append(row * 0.08)
        targets.append(np.zeros(3, dtype=float))
    design = np.vstack(rows)
    target = np.vstack(targets)
    try:
        control_points = np.linalg.lstsq(design, target, rcond=1.0e-6)[0]
    except np.linalg.LinAlgError:
        return estimate_inertial_trajectory(samples, origin)
    positions = basis_matrix @ control_points
    # 对有速度模型的样本保留一部分模型位移基线。B 样条的高阶加速度
    # 正则项会天然压低恒速段的幅值；完全依赖其自由解会再次出现“实际一米、
    # 曲线半米”的系统性缩短。显式速度方向可信度更高，采用更强的基线比例。
    if np.any(velocity_valid):
        baseline_weight = 0.97 if has_explicit_direction else 0.75
        positions = ((1.0 - baseline_weight) * positions
                     + baseline_weight * model_positions)
    positions -= positions[0]
    return (positions[:, 0].tolist(), positions[:, 1].tolist(), positions[:, 2].tolist())


def estimate_relative_attitude(samples: list[Telemetry],
                               origin: tuple[float, float, float] | tuple[float, float] | None = None
                               ) -> tuple[list[float], list[float], list[float]]:
    """输出相对旋转曲线：X=偏航分量、Y=俯仰分量、Z=滚转分量，单位为弧度。

    该模式不依赖空速估计，不会把速度模型误差累计成位置漂移，适合观察
    “从确认零点开始的相对姿态变化”。先用四元数计算相对旋转，再转成
    旋转向量，避免俯仰接近 ±π/2 时欧拉角万向节锁造成的假变化。
    """
    if not samples:
        return [], [], []
    if origin is None:
        origin = (0.0, 0.0, 0.0)
    elif len(origin) == 2:
        origin = (0.0, origin[0], origin[1])
    origin_roll, origin_pitch, origin_yaw = origin

    def quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return (cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy)

    reference = quaternion(origin_roll, origin_pitch, origin_yaw)
    reference_conjugate = (reference[0], -reference[1], -reference[2], -reference[3])

    # 六轴 IMU 的原始偏航只会积分漂移；显示相对姿态时沿用与惯导相同的
    # 静止段锚定，避免静止样本在图上被误画成持续旋转。
    yaw_values = _stabilized_yaw(samples)

    def relative_vector(index: int, sample: Telemetry) -> tuple[float, float, float]:
        current = quaternion(sample.roll, sample.pitch, yaw_values[index])
        # q_relative = conjugate(q_reference) * q_current。
        w0, x0, y0, z0 = reference_conjugate
        w1, x1, y1, z1 = current
        w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        vx = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        vy = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        vz = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm < 1.0e-9:
            return 0.0, 0.0, 0.0
        angle = 2.0 * math.atan2(norm, max(abs(w), 1.0e-12))
        scale = angle / norm
        # 保持界面原有 X/Y/Z 语义：偏航、俯仰、滚转。
        return vz * scale, vy * scale, vx * scale

    x, y, z = [], [], []
    del origin_yaw
    for index, sample in enumerate(samples):
        dx, dy, dz = relative_vector(index, sample)
        x.append(dx)
        y.append(dy)
        z.append(dz)
    return x, y, z
