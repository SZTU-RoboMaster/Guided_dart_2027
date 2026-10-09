"""验证上位机协议与单包长度约束。"""

from pathlib import Path
from dataclasses import replace
import csv
import math
import struct
import sys
import tempfile
import threading
import time
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).parents[1] / "groundstation"))

from protocol import (  # noqa: E402
    Frame,
    FrameParser,
    MAX_FRAME,
    MSG_UPDATE_DATA,
    Telemetry,
    build_frame,
    build_calibrate_gyro,
    build_update_data,
    crc16,
    estimate_trajectory,
    estimate_inertial_trajectory,
    estimate_continuous_trajectory,
    _bspline_basis_and_derivatives,
    np as protocol_numpy,
    estimate_relative_attitude,
    _attitude_quaternion,
    _rotate_vector,
    _sustained_stationary_mask,
    raw_imu_log_prefix_length,
    trajectory_session_ranges,
    inertial_capture_warnings,
    has_terminal_stationary_interval,
    build_log_read,
    build_log_start,
    build_log_stop,
    decode_imu_log_records,
    parse_log_info,
    ImuLogInfo,
    MSG_LOG_INFO,
    MSG_LOG_READ,
    MSG_LOG_DATA,
    MSG_LOG_STOP,
    TELEMETRY_FLAG_SPEED_ESTIMATED,
    TELEMETRY_FLAG_VELOCITY_DIRECTION,
    phase_name,
    parse_ack,
    validate_app_image,
)


def test_capture_buttons() -> None:
    """采集按钮必须走原始日志流程，不能把一帧预览当成拟合结果。"""
    from main import GroundStation

    station = GroundStation.__new__(GroundStation)
    sample = Telemetry(0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, -9.80665,
                       (1500,) * 4, 0.0, 0.0, 0.0, 100)
    steady = [replace(sample, timestamp_ms=index * 100) for index in range(22)]
    assert has_terminal_stationary_interval(steady)
    steady[10] = replace(steady[10], p=0.2)
    assert has_terminal_stationary_interval(steady)
    for index in range(12, 18):
        steady[index] = replace(steady[index], p=0.2)
    assert not has_terminal_stationary_interval(steady)
    slow_preview = [replace(sample, timestamp_ms=index * 1000) for index in range(4)]
    assert has_terminal_stationary_interval(slow_preview)
    assert not has_terminal_stationary_interval(slow_preview[:2])
    station.serial_port = object()
    station.last_telemetry_time = time.monotonic()
    station.telemetry = [sample]
    station.trajectory_samples = []
    station.trajectory_origin = None
    station.trajectory_capturing = False
    station.capture_state = "idle"
    station.finish_requested = False
    station.log_lock = threading.Lock()
    station.update_lock = threading.Lock()
    station.status_var = Mock()
    station.capture_var = Mock()
    station.capture_progress_var = Mock()
    station.capture_percent_var = Mock()
    station.trajectory_mode_var = Mock()
    station.plot_canvas = Mock()
    station.update_idletasks = Mock()
    station.after = lambda _delay, callback, *args: callback(*args)
    station._refresh_trajectory = Mock(side_effect=lambda: assert_fitting_progress(station))
    station._show_telemetry = Mock()
    with patch("main.messagebox.askyesno", return_value=True), patch("main.threading.Thread") as worker:
        station._start_trajectory_capture()
        assert station.capture_state == "starting"
        assert not station.trajectory_capturing
        worker.return_value.start.assert_called_once()
        station._capture_started(200)
        assert station.trajectory_capturing
        station._accept_telemetry(sample)
        assert len(station.trajectory_samples) == 2
        with patch("main.messagebox.askyesno", return_value=False):
            station._finish_trajectory_capture()
        assert station.capture_state == "recording"
        assert worker.return_value.start.call_count == 1
        station.trajectory_samples = slow_preview
        station.last_telemetry_time = time.monotonic() - 1.2
        with patch("main.messagebox.askyesno") as warning:
            station._finish_trajectory_capture()
        warning.assert_not_called()
        assert station.capture_state == "downloading"
        assert not station.trajectory_capturing
        assert "已拟合" not in str(station.status_var.set.call_args)
        assert worker.return_value.start.call_count == 2
        station._finish_trajectory_capture()
        assert worker.return_value.start.call_count == 2
        station._accept_downloaded_log([sample] * 4000)
        assert len(station.trajectory_samples) == 4000
        assert station.capture_state == "idle"
        station.capture_progress_var.set.assert_called_with(100)
        station.plot_canvas.draw.assert_called_once()
        station._start_trajectory_capture()
        station._finish_trajectory_capture()
        assert station.capture_state == "starting" and station.finish_requested
        assert worker.return_value.start.call_count == 3
        station._capture_started(200)
        assert station.capture_state == "downloading"
        assert worker.return_value.start.call_count == 4


def assert_fitting_progress(station) -> None:
    """绘图执行期间不能提前宣称拟合已完成。"""
    if station.capture_state == "fitting":
        station.capture_progress_var.set.assert_called_with(99)


def test_burst_download() -> None:
    """批量下载必须覆盖整段日志，包括末尾不足 40 字节的记录。"""
    from main import GroundStation

    records = []
    for index in range(11):
        body = struct.pack("<IhhhhhhhhhhhHBB", index * 5, 0, 0, -981,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0)
        records.append(body + struct.pack("<H", crc16(body)))
    raw = b"".join(records)
    replies = []
    requests = []

    def write(packet):
        request = FrameParser().feed(packet)[0]
        if request.message_type != MSG_LOG_READ:
            return
        requests.append(request.payload)
        offset, length = struct.unpack_from("<IB", request.payload)
        count = request.payload[5] if len(request.payload) == 6 else 1
        for index in range(count):
            block_offset = offset + index * length
            replies.append(Frame(MSG_LOG_DATA, 0, 0,
                                 struct.pack("<IB", block_offset, length)
                                 + raw[block_offset:block_offset + length]))

    station = GroundStation.__new__(GroundStation)
    station.serial_port = Mock()
    station.serial_port.write.side_effect = write
    station.log_lock = threading.Lock()
    station._wait_ack = Mock(return_value=(MSG_LOG_STOP, 0, 0))
    station._wait_log_info = Mock(return_value=ImuLogInfo(2, 200, 32, 11, len(raw)))
    station._wait_frame = Mock(side_effect=lambda *_args, **_kwargs: replies.pop(0))
    station.after = lambda _delay, callback, *args: callback(*args)
    station._accept_downloaded_log = Mock()
    station._capture_download_failed = Mock()
    station.capture_var = Mock()
    station.capture_progress_var = Mock()
    station.capture_percent_var = Mock()
    station._raw_log_stop_worker()
    station._capture_download_failed.assert_not_called()
    assert len(station._accept_downloaded_log.call_args.args[0]) == 11
    assert len(requests) == 2 and requests[0][5] == 8 and len(requests[1]) == 5


def test_log_finalizing_info() -> None:
    """停止记录后应等待异步提交完成，不能把提交中误判为下载失败。"""
    from main import GroundStation

    station = GroundStation.__new__(GroundStation)
    station.serial_port = Mock()
    station._wait_frame = Mock(side_effect=[
        Frame(MSG_LOG_INFO, 0, 0, struct.pack("<BHHII", state, 200, 32, 8, 256))
        for state in (4, 2)
    ])
    assert station._wait_log_info(expected_state=2).state == 2
    station.serial_port.write.assert_called_once()


def test_log_csv_roundtrip() -> None:
    """旧日志导入和重新导出都只保留原始采样，并清除假速度标志。"""
    from main import GroundStation

    base = Telemetry(3, TELEMETRY_FLAG_SPEED_ESTIMATED, 0, 0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, -9.80665,
                     (0, 0, 0, 0), 0.0, 0.0, 5.0, 0)
    mixed = [replace(base, timestamp_ms=index * 5) for index in range(120)]
    mixed.append(replace(base, timestamp_ms=800, servos=(1500,) * 4))
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "记录.csv"
        with path.open("w", newline="", encoding="utf-8-sig") as output:
            writer = csv.writer(output)
            fields = tuple(Telemetry.__dataclass_fields__)
            writer.writerow(fields)
            writer.writerows(tuple(getattr(sample, key) for key in fields)
                             for sample in mixed)
        station = GroundStation.__new__(GroundStation)
        station.trajectory_mode_var = Mock()
        station._refresh_trajectory = Mock()
        station.status_var = Mock()
        with patch("main.filedialog.askopenfilename", return_value=str(path)):
            station._import_csv()
        assert len(station.telemetry) == 120
        assert not any(sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED
                       for sample in station.telemetry)
        station.trajectory_mode_var.set.assert_called_with("IMU积分（ZUPT）")
        station.telemetry.append(mixed[-1])
        with patch("main.filedialog.asksaveasfilename", return_value=str(path)):
            station._export_csv()
        with path.open("r", newline="", encoding="utf-8-sig") as source:
            assert len(list(csv.DictReader(source))) == 120


def main() -> None:
    payload = bytes(range(45))
    frame = build_frame(MSG_UPDATE_DATA, 7, payload)
    assert len(frame) == MAX_FRAME
    parser = FrameParser()
    assert parser.feed(frame[:5]) == []
    decoded = parser.feed(frame[5:])
    assert len(decoded) == 1
    assert decoded[0].payload == payload

    corrupted = bytearray(frame)
    corrupted[-1] ^= 0x01
    assert parser.feed(corrupted) == []

    update = build_update_data(3, b"abc")
    assert len(update) <= MAX_FRAME
    assert len(build_update_data(4, b"x" * 42)) == MAX_FRAME
    assert phase_name(0) == "待机"
    assert phase_name(8) == "结束"
    assert crc16(b"123456789") == 0x29B1
    info = parse_log_info(Frame(MSG_LOG_INFO, 0, 0,
                                bytes([2, 200, 0, 32, 0])
                                + struct.pack("<II", 3, 96)))
    assert info == ImuLogInfo(2, 200, 32, 3, 96)
    assert len(build_log_read(40, 40)) <= MAX_FRAME
    burst = FrameParser().feed(build_log_read(0, 40, 8))[0]
    assert burst.payload == struct.pack("<IBB", 0, 40, 8)
    assert len(build_log_start(200)) <= MAX_FRAME
    assert len(build_log_stop()) <= MAX_FRAME
    log_record = struct.pack(
        "<IhhhhhhhhhhhHBB", 1234, 100, -200, -981, 10, 20, 30,
        40, 50, 60, 70, 80, 500, 3, 3)
    log_record += struct.pack("<H", crc16(log_record))
    decoded_log = decode_imu_log_records(log_record)
    assert len(decoded_log) == 1 and decoded_log[0].timestamp_ms == 1234
    assert decoded_log[0].flags & TELEMETRY_FLAG_VELOCITY_DIRECTION
    assert not decoded_log[0].flags & TELEMETRY_FLAG_SPEED_ESTIMATED
    logged = [replace(decoded_log[0], timestamp_ms=index * 5)
              for index in range(120)]
    logged.append(replace(logged[-1], timestamp_ms=700, servos=(1500,) * 4))
    assert raw_imu_log_prefix_length(logged) == 120
    assert raw_imu_log_prefix_length(logged[120:]) == 0
    generation = 0x155
    tagged_record = struct.pack(
        "<IhhhhhhhhhhhHBB", 1239, 100, -200, -981, 10, 20, 30,
        40, 50, 60, 70, 80, 500, 8 | ((generation & 15) << 4),
        3 | ((generation >> 4) << 3))
    tagged_record += struct.pack("<H", crc16(tagged_record))
    assert decode_imu_log_records(tagged_record)[0].phase == 8
    assert parse_ack(decoded[0]) is None
    assert FrameParser().feed(build_calibrate_gyro())[0].message_type == 0x14

    valid_image = struct.pack("<II", 0x20007FE0, 0x08004009) + b"\x00" * 32
    validate_app_image(valid_image)
    try:
        validate_app_image(struct.pack("<II", 0x20001000, 0x08000009))
        raise AssertionError("错误链接地址未被拒绝")
    except ValueError:
        pass

    def sample(flight_time: float, pitch: float, yaw: float,
               phase: int = 2) -> Telemetry:
        return Telemetry(phase, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, pitch, yaw, 0.0, 0.0, 0.0, (1500,) * 4,
                         0.0, flight_time, 10.0, int(flight_time * 1000))

    x, y, z = estimate_trajectory([sample(0.0, 0.0, 0.0),
                                   sample(0.1, math.pi / 2.0, 0.0)])
    assert abs(x[-1] - 0.5) < 1.0e-6 and abs(y[-1]) < 1.0e-6
    # 四元数按机体系到导航系变换；正俯仰把前向轴转向负 Z。
    assert abs(z[-1] + 0.5) < 1.0e-6
    # LoRa 丢帧后 264 ms 的有效时间不能被旧的 200 ms 上限截断，
    # 10 m/s 速度应完整积出 2.64 m。
    gap_start = sample(0.0, 0.0, 0.0)
    gap_end = sample(0.264, 0.0, 0.0)
    gap_start.flags = gap_end.flags = TELEMETRY_FLAG_SPEED_ESTIMATED
    gap_start.timestamp_ms, gap_end.timestamp_ms = 0, 264
    gx, gy, gz = estimate_trajectory([gap_start, gap_end])
    assert abs(gx[-1] - 2.64) < 1.0e-6
    gap_end.timestamp_ms = 800
    gap_end.flight_time = 0.8
    gx, gy, gz = estimate_trajectory([gap_start, gap_end])
    assert abs(gx[-1] - 8.0) < 1.0e-6
    # 固件提供速度方向时，地面站应优先使用 cx/cy 中的方向。
    directed = [Telemetry(3, TELEMETRY_FLAG_SPEED_ESTIMATED | TELEMETRY_FLAG_VELOCITY_DIRECTION,
                          0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, -9.81, (1500,) * 4,
                          0.0, 0.0, 5.0, 0),
                Telemetry(3, TELEMETRY_FLAG_SPEED_ESTIMATED | TELEMETRY_FLAG_VELOCITY_DIRECTION,
                          1571, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, -9.81, (1500,) * 4,
                          0.0, 0.0, 5.0, 100)]
    dx, dy, dz = estimate_trajectory(directed)
    assert abs(dx[-1]) < 0.02 and dy[-1] > 0.01 and abs(dz[-1]) < 1.0e-5
    # 起始坐标系非零偏航时，导航系速度应通过起始姿态共轭变换回去；
    # 该断言用于防止 X/Y 轴再次发生镜像或串扰。
    def directed_sample(timestamp: int) -> Telemetry:
        return Telemetry(3, TELEMETRY_FLAG_SPEED_ESTIMATED, 0, 0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, -9.81,
                         (1500,) * 4, 0.0, 0.0, 10.0, timestamp)
    rotated_x, rotated_y, rotated_z = estimate_trajectory(
        [directed_sample(0), directed_sample(100)], origin=(0.0, 0.0, math.pi / 2.0))
    assert abs(rotated_x[-1]) < 1.0e-6 and rotated_y[-1] < -0.9
    assert abs(rotated_z[-1]) < 1.0e-6
    # 两段同向运动之间的停顿不提供反向证据，不能凭停顿翻转位移。
    same_direction = [directed_sample(t) for t in (0, 100, 200, 1400, 1500)]
    same_direction[2].flags = same_direction[3].flags = 0
    same_direction[2].speed = same_direction[3].speed = 0.0
    same_direction[0].flags = same_direction[1].flags = TELEMETRY_FLAG_SPEED_ESTIMATED
    same_direction[4].flags = TELEMETRY_FLAG_SPEED_ESTIMATED
    forward_x, _, _ = estimate_trajectory(same_direction)
    assert forward_x[-1] > forward_x[1]
    # 固件提供速度方向时，连续转向应同时产生 X/Y 轨迹，而不是全部挤在
    # 机体前向轴上。
    turning = []
    for index in range(21):
        direction_yaw = math.pi * index / 40.0
        turning.append(Telemetry(
            3, TELEMETRY_FLAG_SPEED_ESTIMATED | TELEMETRY_FLAG_VELOCITY_DIRECTION,
            int(direction_yaw * 1000.0), 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -9.81, (1500,) * 4,
            0.0, index * 0.1, 10.0, index * 100))
    tx, ty, tz = estimate_trajectory(turning)
    assert tx[-1] > 1.0 and ty[-1] > 1.0 and max(abs(value) for value in tz) < 1.0e-6
    ax, ay, az = estimate_relative_attitude([sample(0.0, 0.0, 0.0),
                                             sample(0.0, math.pi / 2.0, 0.0)],
                                            origin=(0.0, 0.0, 0.0))
    assert abs(ax[-1]) < 1.0e-6 and abs(ay[-1] - math.pi / 2.0) < 1.0e-6
    # 结束阶段 flight_time 停止，设备时间戳虽继续前进，也不能把速度下限当作位移。
    ended = sample(0.0, 0.0, 0.0, phase=8)
    later = sample(0.0, 0.0, 0.0, phase=8)
    later.timestamp_ms = ended.timestamp_ms + 100
    ex, _, _ = estimate_trajectory([ended, later])
    assert abs(ex[-1]) < 1.0e-6
    # 结束态的速度下限不是实际运动速度，静止数据不能产生假位移。
    static = [sample(0.0, 0.0, 0.0, phase=8),
              sample(0.0, 0.2, 0.1, phase=8)]
    sx, sy, sz = estimate_trajectory(static, origin=(0.0, 0.0, 0.0))
    assert sx == [0.0, 0.0] and sy == [0.0, 0.0] and sz == [0.0, 0.0]
    # 发射判定后重新静止时，速度模型下限同样不能产生假位移。
    stationary_flight = [Telemetry(3, 0, 0, 0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, -9.81, (1500,) * 4,
                                   0.0, 10.0, 5.0, 0),
                         Telemetry(3, 0, 0, 0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, -9.81, (1500,) * 4,
                                   0.0, 10.0, 5.0, 100)]
    fx, fy, fz = estimate_trajectory(stationary_flight)
    assert fx == [0.0, 0.0] and fy == [0.0, 0.0] and fz == [0.0, 0.0]
    ix, iy, iz = estimate_inertial_trajectory(stationary_flight)
    assert ix == [0.0, 0.0] and iy == [0.0, 0.0] and iz == [0.0, 0.0]
    # 连续时间优化至少应对一段有效采样返回同长度、有限值的曲线，
    # 避免 B 样条端点或退化时间轴把界面变成 NaN。
    smooth_samples = [Telemetry(3, 0, 0, 0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                1.0, 0.0, -9.80665, (1500,) * 4,
                                0.0, index * 0.1, 0.0, index * 100)
                      for index in range(8)]
    if protocol_numpy is not None:
        # 三次 B 样条基函数必须分割单位，导数与数值差分一致。
        knots = protocol_numpy.r_[protocol_numpy.zeros(4),
                                  protocol_numpy.linspace(0.0, 1.0, 6)[1:-1],
                                  protocol_numpy.ones(4)]
        for point in (0.0, 0.13, 0.37, 0.63, 0.87, 1.0):
            basis, first, second = _bspline_basis_and_derivatives(knots, 3, point)
            assert abs(sum(basis) - 1.0) < 1.0e-12
            assert abs(sum(first)) < 1.0e-11
            assert abs(sum(second)) < 1.0e-10
            if 0.0 < point < 1.0:
                step = 1.0e-5
                next_basis, next_first, _ = _bspline_basis_and_derivatives(
                    knots, 3, point + step)
                prev_basis, prev_first, _ = _bspline_basis_and_derivatives(
                    knots, 3, point - step)
                assert max(abs((next_basis - prev_basis) / (2.0 * step) - first)) < 1.0e-4
                assert max(abs((next_first - prev_first) / (2.0 * step) - second)) < 1.0e-3
        assert sum(_bspline_basis_and_derivatives(knots, 3, 0.37)[0] > 0.0) == 4
        for knot in (0.2, 0.4, 0.6, 0.8):
            before = _bspline_basis_and_derivatives(knots, 3, knot - 1.0e-7)
            after = _bspline_basis_and_derivatives(knots, 3, knot + 1.0e-7)
            assert max(abs(after[0] - before[0])) < 1.0e-5
            assert max(abs(after[1] - before[1])) < 1.0e-4
            assert max(abs(after[2] - before[2])) < 1.0e-2
    sx, sy, sz = estimate_continuous_trajectory(smooth_samples)
    assert len(sx) == len(smooth_samples)
    assert all(math.isfinite(value) for value in sx + sy + sz)
    # 没有速度先验时，连续模式应退回 ZUPT；对已知的加速-减速轨迹，
    # 末端位移应接近解析值 1.25 m，而不是被样条正则项拟合成反向曲线。
    pure_imu = []
    for index in range(201):
        acceleration = 5.0 if index < 100 else (-5.0 if index < 200 else 0.0)
        pure_imu.append(Telemetry(
            3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, acceleration, 0.0, -9.80665,
            (1500,) * 4, 0.0, index * 0.005, 0.0, index * 5))
    px, py, pz = estimate_continuous_trajectory(pure_imu)
    assert abs(px[-1] - 1.25) < 0.08
    assert max(abs(value) for value in py + pz) < 1.0e-6
    # 量程内的 40 m/s² 高速脉冲不能再被旧版 25 m/s² 限幅削短。
    pulse = []
    for index in range(140):
        acceleration = 40.0 if 50 <= index < 70 else (-40.0 if 70 <= index < 90 else 0.0)
        pulse.append(Telemetry(
            3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, acceleration, 0.0, -9.80665,
            (0, 0, 0, 0), 0.0, index * 0.005, 0.0, index * 5))
    pulse_x, pulse_y, pulse_z = estimate_inertial_trajectory(pulse)
    assert abs(pulse_x[-1] - 0.4) < 0.03
    assert max(abs(value) for value in pulse_y + pulse_z) < 1.0e-6
    # 设备重启后时间戳回退，新会话必须从相对零点重新积分。
    joined_pulses = pulse + [replace(sample) for sample in pulse]
    joined_x, joined_y, joined_z = estimate_inertial_trajectory(joined_pulses)
    assert trajectory_session_ranges(joined_pulses) == [
        (0, len(pulse)), (len(pulse), len(joined_pulses))]
    assert joined_x[len(pulse)] == 0.0
    assert abs(joined_x[-1] - 0.4) < 0.03
    assert max(abs(value) for value in joined_y + joined_z) < 1.0e-6
    assert any("时间戳回退" in warning for warning in inertial_capture_warnings(joined_pulses))
    sparse = [replace(sample, timestamp_ms=index * 66)
              for index, sample in enumerate(pulse)]
    assert any("采样间隔中位数" in warning for warning in inertial_capture_warnings(sparse))
    # 固件低通后的单采样脉冲仍保留正确冲量；离线处理中值滤波不能再次削短位移。
    filtered_pulse = []
    filtered_acceleration = 0.0
    for index in range(600):
        input_acceleration = (40.0 if index == 100 else
                              -40.0 if index == 500 else 0.0)
        filtered_acceleration += 0.28 * (input_acceleration - filtered_acceleration)
        filtered_pulse.append(Telemetry(
            3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, filtered_acceleration, 0.0, -9.80665,
            (0, 0, 0, 0), 0.0, index * 0.005, 0.0, index * 5))
    filtered_x, filtered_y, filtered_z = estimate_inertial_trajectory(filtered_pulse)
    assert abs(filtered_x[-1] - 0.4) < 0.015
    assert max(abs(value) for value in filtered_y + filtered_z) < 1.0e-6
    # 飞行中的匀速段与静止有相同惯性观测；不能在中途清零速度。
    coast = []
    for index in range(600):
        acceleration = (40.0 if 100 <= index < 120 else
                        -40.0 if 520 <= index < 540 else 0.0)
        coast.append(Telemetry(
            3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, acceleration, 0.0, -9.80665,
            (0, 0, 0, 0), 0.0, index * 0.005, 0.0, index * 5))
    coast_x, coast_y, coast_z = estimate_inertial_trajectory(coast)
    assert abs(coast_x[-1] - 8.4) < 0.05
    assert max(abs(value) for value in coast_y + coast_z) < 1.0e-6
    assert any("匀速滑行" in warning for warning in inertial_capture_warnings(coast))
    assert not any("姿态与重力不一致" in warning for warning in inertial_capture_warnings(coast))
    wrong_gyro = [replace(sample, ax=0.0 if index == 570 else sample.ax,
                          p=0.15 if 100 <= index < 540 else 0.0)
                  for index, sample in enumerate(coast)]
    wrong_gyro.extend(replace(wrong_gyro[-1], timestamp_ms=index * 5)
                      for index in range(600, 700))
    assert any("姿态与重力不一致" in warning
               for warning in inertial_capture_warnings(wrong_gyro))
    # 末端确已停稳时，单帧轻微超阈值不应让末速度校正失效。
    coast[570].ax = 0.25
    noisy_x, _, _ = estimate_inertial_trajectory(coast)
    assert abs(noisy_x[-1] - 8.4) < 0.05
    # 加速度会在运动段拉偏固件重力姿态；陀螺为零时不能跟随这段假俯仰。
    # 起止静止段同时提供机体系加速度残差，不靠已知位移调整倍率。
    tilted_motion = []
    for index in range(320):
        ax = 5.0 if 60 <= index < 160 else (-5.0 if 160 <= index < 260 else 0.0)
        false_pitch = 0.12 if 60 <= index < 160 else 0.0
        tilted_motion.append(Telemetry(
            3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, false_pitch, 0.0, ax, 0.0, -9.80665,
            (0, 0, 0, 0), 0.0, index * 0.005, 0.0, index * 5))
    tilt_x, tilt_y, tilt_z = estimate_inertial_trajectory(tilted_motion)
    assert abs(tilt_x[-1] - 1.25) < 0.02
    assert abs(tilt_y[-1]) < 1.0e-6 and abs(tilt_z[-1]) < 0.02
    # 导航系 Z 向下：竖直抬高产生负 Z 位移，界面绘图再翻转为正高度。
    lifted = [replace(sample, ax=0.0, az=-9.80665 - sample.ax,
                      pitch=0.0) for sample in tilted_motion]
    lift_x, lift_y, lift_z = estimate_inertial_trajectory(lifted)
    assert abs(lift_z[-1] + 1.25) < 0.02
    assert max(abs(value) for value in lift_x + lift_y) < 0.02
    # 起始机体有倾角时，纯竖直抬高不能被投影成横向位移。
    roll, pitch, yaw = -0.12, 0.03, 0.4
    orientation = _attitude_quaternion(roll, pitch, yaw)
    inverse = (orientation[0], -orientation[1], -orientation[2], -orientation[3])
    tilted_lift = []
    for index in range(320):
        vertical = -5.0 if 60 <= index < 160 else (5.0 if 160 <= index < 260 else 0.0)
        ax, ay, az = _rotate_vector(inverse, (0.0, 0.0, -9.80665 + vertical))
        tilted_lift.append(Telemetry(
            1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            roll, pitch, yaw, ax, ay, az, (0, 0, 0, 0),
            0.0, index * 0.005, 0.0, index * 5))
    level_x, level_y, level_z = estimate_inertial_trajectory(tilted_lift)
    assert abs(level_z[-1] + 1.25) < 0.02
    assert math.hypot(level_x[-1], level_y[-1]) < 0.02
    assert any("起始静止不足" in warning for warning in inertial_capture_warnings(tilted_lift))
    model_start = replace(stationary_flight[0], flags=TELEMETRY_FLAG_SPEED_ESTIMATED,
                          pitch=0.2, speed=5.0)
    model_end = replace(model_start, timestamp_ms=100)
    _, _, model_z = estimate_trajectory([model_start, model_end])
    assert model_z[-1] < -0.001
    # 只有速度模型声称在运动、IMU 始终测不到起步加速度时，纯惯导
    # 没有速度初值证据，不能凭模型下限虚构位移。
    constant_speed = []
    for index in range(21):
        moving = index >= 2
        constant_speed.append(Telemetry(
            3 if moving else 0, TELEMETRY_FLAG_SPEED_ESTIMATED if moving else 0,
            0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -9.80665,
            (1500,) * 4, 0.0, index * 0.1, 5.0 if moving else 0.0, index * 100))
    cx, cy, cz = estimate_inertial_trajectory(constant_speed)
    assert cx[-1] == 0.0 and cy[-1] == 0.0 and cz[-1] == 0.0
    # 原始日志即使携带旧版模型速度标志，静止判断仍只看 IMU；
    # 运动中短暂低于阈值的单帧不能把已经积累的速度清零。
    stable = [Telemetry(3, TELEMETRY_FLAG_SPEED_ESTIMATED, 0, 0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, -9.80665,
                        (0, 0, 0, 0), 0.0, index * 0.005, 5.0, index * 5)
              for index in range(120)]
    stable[60].ax = 0.5
    stable_mask = _sustained_stationary_mask(stable)
    assert all(stable_mask[:60]) and all(stable_mask[61:])
    assert not stable_mask[60]
    # 固件俯仰回正稍慢时，低角速度且实际已停稳的样本不能继续积分成假位移。
    lagged_attitude = [replace(sample, flags=0, ax=0.0, pitch=0.03)
                       for sample in stable]
    assert all(_sustained_stationary_mask(lagged_attitude))
    lagged_x, lagged_y, lagged_z = estimate_inertial_trajectory(lagged_attitude)
    assert max(abs(value) for value in lagged_x + lagged_y + lagged_z) < 1.0e-6
    # 已停稳后固件俯仰缓慢回正；若误把这段当作运动，重力泄漏会让位移增大。
    settling = []
    for index in range(800):
        acceleration = (4.0 if 100 <= index < 150 else
                        -4.0 if 150 <= index < 200 else 0.0)
        settling.append(Telemetry(
            1, 0, 0, 0, 0.0, 0.08 if 100 <= index < 200 else 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.03 if 200 <= index < 650 else 0.0, 0.0,
            acceleration, 0.0, -9.80665, (0, 0, 0, 0),
            0.0, index * 0.005, 0.0, index * 5))
    settling_x, settling_y, settling_z = estimate_inertial_trajectory(settling)
    assert abs(settling_x[-1] - 0.25) < 0.02
    assert max(abs(value) for value in settling_y + settling_z) < 0.02
    moving = [replace(sample, ax=0.5) for sample in stable]
    moving[60].ax = 0.0
    assert not any(_sustained_stationary_mask(moving))
    assert inertial_capture_warnings(moving)
    assert not inertial_capture_warnings(stable[:60])
    # 连续时间模式也必须保留速度模型给出的距离比例；否则高阶平滑项会
    # 把恒速段压成半程，正是现场“实际一米、曲线半米”的回归场景。
    bx, by, bz = estimate_continuous_trajectory(constant_speed)
    assert bx[-1] > 0.35 and abs(by[-1]) < 1.0e-6 and abs(bz[-1]) < 1.0e-6
    # 带固件速度方向时，连续优化应同时产生两个水平分量，不能把转弯
    # 折叠成单一机体前向轴。
    explicit_turn = []
    for index in range(21):
        direction_yaw = math.pi * index / 40.0
        explicit_turn.append(Telemetry(
            3, TELEMETRY_FLAG_SPEED_ESTIMATED | TELEMETRY_FLAG_VELOCITY_DIRECTION,
            int(direction_yaw * 1000.0), 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -9.81, (1500,) * 4,
            0.0, index * 0.1, 10.0, index * 100))
    bx, by, bz = estimate_continuous_trajectory(explicit_turn)
    assert bx[-1] > 1.0 and by[-1] > 1.0 and max(abs(value) for value in bz) < 1.0e-5
    # 高速场景回归：18 m/s、30°水平转弯两秒的路径应保持约 36 m，且
    # 横向分量不能被压成零；这是实际高速目标比手持低速数据更接近的工况。
    high_speed = []
    for index in range(401):
        flight_time = index * 0.005
        direction_yaw = math.radians(30.0) * min(1.0, flight_time / 1.2)
        high_speed.append(Telemetry(
            3, TELEMETRY_FLAG_SPEED_ESTIMATED | TELEMETRY_FLAG_VELOCITY_DIRECTION,
            int(direction_yaw * 1000.0), 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -9.80665, (1500,) * 4,
            0.0, flight_time, 18.0, index * 5))
    hx, hy, hz = estimate_continuous_trajectory(high_speed)
    assert math.hypot(hx[-1], hy[-1]) > 32.0 and hy[-1] > 8.0
    # 长时间采样空洞不能导致 B 样条静默丢弃后半段样本。
    interrupted = [replace(sample, timestamp_ms=sample.timestamp_ms + 3000)
                   if index >= 200 else sample
                   for index, sample in enumerate(high_speed)]
    interrupted_x, interrupted_y, interrupted_z = estimate_continuous_trajectory(interrupted)
    assert len(interrupted_x) == len(interrupted_y) == len(interrupted_z) == len(interrupted)
    assert any("采样中断" in warning for warning in inertial_capture_warnings(interrupted))
    # 时间戳回退时，B 样条必须拟合回退后的会话，而不是只返回第一段。
    restarted = []
    for session in range(2):
        for index in range(4):
            restarted.append(Telemetry(
                3, TELEMETRY_FLAG_SPEED_ESTIMATED | TELEMETRY_FLAG_VELOCITY_DIRECTION,
                0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, -9.80665, (1500,) * 4,
                0.0, index * 0.1, 10.0, index * 100))
    rx, ry, rz = estimate_continuous_trajectory(restarted)
    assert len(rx) == len(restarted) and rx[-1] > 2.0
    # 末帧静止不代表回到了起点，不能把真实位移强制校正为零。
    displaced = [stationary_flight[0],
                 Telemetry(3, 0, 0, 0, 0.3, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           3.0, 0.0, -9.81, (1500,) * 4,
                           0.0, 10.0, 5.0, 100),
                 Telemetry(3, 0, 0, 0, 0.3, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           3.0, 0.0, -9.81, (1500,) * 4,
                           0.0, 10.0, 5.0, 200),
                 stationary_flight[1]]
    displaced[-1].timestamp_ms = 300
    dx, _, _ = estimate_inertial_trajectory(displaced)
    assert dx[-1] > 0.0
    # 阶段为“结束”时仍可能被搬动，不能仅凭阶段把加速度全部封零。
    displaced[1].phase = displaced[2].phase = 8
    moved_after_end, _, _ = estimate_inertial_trajectory(displaced)
    assert moved_after_end[-1] > 0.0
    # 角度跨越 ±π 时不能产生约 2π 的伪跳变。
    wrapped = [sample(0.0, 0.0, math.pi - 0.01),
               sample(0.1, 0.0, -math.pi + 0.01)]
    wx, _, _ = estimate_relative_attitude(wrapped, origin=(0.0, 0.0, wrapped[0].yaw))
    assert abs(wx[-1] + 0.02) < 1.0e-6
    test_capture_buttons()
    test_burst_download()
    test_log_finalizing_info()
    test_log_csv_roundtrip()
    print("protocol tests passed")


if __name__ == "__main__":
    main()
