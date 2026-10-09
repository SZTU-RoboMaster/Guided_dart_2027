"""离线检查遥测 CSV 的时间轴、阶段和轨迹估计结果。"""

from __future__ import annotations

import ast
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "groundstation"))
from protocol import (Telemetry, TELEMETRY_FLAG_SPEED_ESTIMATED,
                      estimate_inertial_trajectory, estimate_trajectory,
                      raw_imu_log_prefix_length)  # noqa: E402


def load(path: Path) -> list[Telemetry]:
    result: list[Telemetry] = []
    with path.open("r", newline="", encoding="utf-8-sig") as source:
        for row in csv.DictReader(source):
            result.append(Telemetry(
                int(row["phase"]), int(row["flags"]), int(row["cx"]), int(row["cy"]),
                float(row["p"]), float(row["q"]), float(row["r"]),
                float(row["p_accel"]), float(row["q_accel"]), float(row["r_accel"]),
                float(row["roll"]), float(row["pitch"]), float(row["yaw"]),
                float(row["ax"]), float(row["ay"]), float(row["az"]),
                tuple(ast.literal_eval(row["servos"])), float(row["tgo"]),
                float(row["flight_time"]), float(row["speed"]), int(row["timestamp_ms"])))
    return result


def check(path: Path) -> None:
    samples = load(path)
    assert samples, f"{path} 没有遥测帧"
    log_length = raw_imu_log_prefix_length(samples)
    if log_length:
        omitted = len(samples) - log_length
        samples = samples[:log_length]
        for sample in samples:
            sample.flags &= ~TELEMETRY_FLAG_SPEED_ESTIMATED
        if omitted:
            print(f"{path.name}: 已排除日志后的 {omitted} 帧实时预览")
    inertial = estimate_inertial_trajectory(samples)
    model = None if log_length else estimate_trajectory(samples)
    values = (*inertial[0], *inertial[1], *inertial[2])
    assert all(math.isfinite(value) for value in values)
    duration = sum(min(0.5, max(0.0, (current.timestamp_ms - previous.timestamp_ms) / 1000.0))
                   for previous, current in zip(samples, samples[1:])
                   if current.timestamp_ms >= previous.timestamp_ms)
    print(f"{path.name}: {len(samples)}帧，阶段={sorted({s.phase for s in samples})}，时长={duration:.3f}s")
    print(f"  IMU终点=({inertial[0][-1]:+.3f}, {inertial[1][-1]:+.3f}, {inertial[2][-1]:+.3f})m")
    if model is not None:
        print(f"  模型终点=({model[0][-1]:+.3f}, {model[1][-1]:+.3f}, {model[2][-1]:+.3f})m")
        model_path = sum(math.sqrt((model[0][i] - model[0][i - 1]) ** 2
                                   + (model[1][i] - model[1][i - 1]) ** 2
                                   + (model[2][i] - model[2][i - 1]) ** 2)
                         for i in range(1, len(model[0])))
        print(f"  模型累计路径={model_path:.3f}m")
    if log_length:
        print("  注意：非零速度全部为模型估计/下限，数据没有可用于判断前进或返回的速度符号。")


if __name__ == "__main__":
    paths = [Path(arg) for arg in sys.argv[1:]]
    if not paths:
        raise SystemExit("用法：python tools/check_telemetry_csv.py 数据.csv [...]")
    for csv_path in paths:
        check(csv_path)
