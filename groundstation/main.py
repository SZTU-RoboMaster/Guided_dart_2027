"""@file main.py
@brief Guided Dart 上位机界面及 LoRa OTA 调度。
@details 同一串口承载遥测、原始 IMU 日志和固件升级；升级在后台线程中
         按 Bootloader 的累计确认进度发送，界面线程只更新状态。
@author Guided Dart 项目维护者
@date 2026-10-07
@version 1.0
@note OTA 成功以新 App 回传的版本和 CRC32 为准；发送结束不等于运行成功。
"""

from __future__ import annotations

import csv
import ast
import math
import sys
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 运行时给出清晰提示，便于打包后的诊断
    serial = None
    list_ports = None

from protocol import (Frame, FrameParser, MSG_ACK, MSG_CALIBRATE_GYRO, MSG_DEVICE_INFO,
                      MSG_KEEPALIVE, MSG_PARAM_READ, MSG_PARAM_WRITE,
                      MSG_LOG_START, MSG_LOG_STOP, MSG_LOG_INFO, MSG_LOG_READ, MSG_LOG_DATA,
                      Telemetry, TextTelemetryParser, build_param_read, build_param_save, build_param_write,
                      build_update_begin, build_update_data, build_update_end,
                      build_calibrate_gyro, build_device_info, decode_telemetry,
                      build_log_start, build_log_stop, build_log_info, build_log_read,
                      decode_imu_log_records, parse_log_info,
                      TELEMETRY_FLAG_SPEED_ESTIMATED,
                      TELEMETRY_FLAG_VELOCITY_DIRECTION,
                      estimate_trajectory, estimate_inertial_trajectory,
                      estimate_continuous_trajectory,
                      image_crc32, parse_ack, phase_name, raw_imu_log_prefix_length,
                      trajectory_session_ranges,
                      inertial_capture_warnings, has_terminal_stationary_interval,
                      UART_BAUDRATE, UPDATE_CHUNK_SIZE, UPDATE_INTERFRAME_DELAY_S,
                      UPDATE_WINDOW_SIZE, estimate_relative_attitude, validate_app_image)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lora_profiles import (DEFAULT_AIR_RATE, DEFAULT_CHANNEL, DEFAULT_CONFIG_BAUD,
                           DEFAULT_DATA_BAUD, Ewm220Config, ewm220_read_all,
                           ewm220_write_all)


class GroundStation(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Guided Dart LoRa 地面站")
        self.geometry("1100x720")
        self.serial_port = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.frame_parser = FrameParser()
        self.text_parser = TextTelemetryParser()
        self.frame_queue: queue.Queue[Frame] = queue.Queue()
        self.text_queue: queue.Queue[Telemetry] = queue.Queue()
        # 遥测和控制应答分开排队，避免主界面轮询把 ACK 取走后丢弃。
        self.control_queue: queue.Queue[Frame] = queue.Queue()
        self.update_lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.telemetry: list[Telemetry] = []
        self.trajectory_samples: list[Telemetry] = []
        self.trajectory_origin: tuple[float, float, float] | None = None
        self.trajectory_capturing = False
        self.capture_state = "idle"
        self.finish_requested = False
        self.last_log_status_query = 0.0
        self.last_log_status_time = 0.0
        self.board_log_count = 0
        self.last_telemetry_time = 0.0
        self.capture_var = tk.StringVar(value="未采集；Flash 保存 200 Hz IMU（已换算和滤波，并非芯片原码）")
        self.capture_progress_var = tk.DoubleVar(value=0.0)
        self.capture_percent_var = tk.StringVar(value="0%")
        self.trajectory_scale_var = tk.DoubleVar(value=1.0)
        self.trajectory_distance_scale_var = tk.StringVar(value="1.00")
        self.trajectory_mode_var = tk.StringVar(value="相对姿态")
        self.last_plot_time = 0.0
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="未连接")
        self.update_var = tk.StringVar(value="未开始升级")
        self.update_progress_var = tk.DoubleVar(value=0.0)
        self.param_id_var = tk.StringVar(value="3")
        self.param_value_var = tk.StringVar(value="2.8")
        self.param_status_var = tk.StringVar(value="参数当前仅在本次上电有效")
        self.lora_profile_var = tk.StringVar(value="EWM32M（电脑端）")
        self.lora_baud_var = tk.StringVar(value=str(DEFAULT_CONFIG_BAUD))
        self.lora_uart_var = tk.StringVar(value=str(DEFAULT_DATA_BAUD))
        self.lora_air_rate_var = tk.StringVar(value=str(DEFAULT_AIR_RATE))
        self.lora_channel_var = tk.StringVar(value=str(DEFAULT_CHANNEL))
        self.lora_status_var = tk.StringVar(value="配置模式要求 M0=M1=1，UART 9600 8N1")
        self.value_vars = {name: tk.StringVar(value="-") for name in (
            "phase", "time", "speed", "pqr", "apqr", "attitude", "accel",
            "vision", "servos", "tgo")}
        self._build_ui()
        self._refresh_ports()
        # 观测界面 40 Hz 轮询；无线预览为 10 Hz，原始 IMU 在主控记录 200 Hz。
        self.after(25, self._poll_frames)

    def _build_ui(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=26, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Button(top, text="连接", command=self._connect).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="断开", command=self._disconnect).pack(side=tk.LEFT)
        ttk.Button(top, text="链路测试", command=self._link_test).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="清空数据", command=self._clear_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="导出 CSV", command=self._export_csv).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="导入 CSV", command=self._import_csv).pack(side=tk.LEFT, padx=5)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT)

        self.tabs = ttk.Notebook(self)
        self.tabs.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.monitor_tab = ttk.Frame(self.tabs, padding=12)
        self.trajectory_tab = ttk.Frame(self.tabs, padding=12)
        self.update_tab = ttk.Frame(self.tabs, padding=12)
        self.param_tab = ttk.Frame(self.tabs, padding=12)
        self.lora_tab = ttk.Frame(self.tabs, padding=12)
        self.tabs.add(self.monitor_tab, text="实时观测")
        self.tabs.add(self.trajectory_tab, text="轨迹拟合")
        self.tabs.add(self.update_tab, text="无线升级")
        self.tabs.add(self.param_tab, text="参数管理")
        self.tabs.add(self.lora_tab, text="LoRa 配置")
        self._build_monitor()
        self._build_trajectory()
        self._build_update()
        self._build_params()
        self._build_lora()

    def _build_monitor(self) -> None:
        fields = [
            ("phase", "飞行阶段"), ("time", "飞行时间"), ("speed", "估计速度"),
            ("pqr", "角速度 p/q/r"), ("apqr", "角加速度"),
            ("attitude", "姿态 roll/pitch/yaw"), ("accel", "加速度"),
            ("vision", "视觉坐标"), ("servos", "舵机 PWM"), ("tgo", "剩余时间"),
        ]
        for row, (key, label) in enumerate(fields):
            ttk.Label(self.monitor_tab, text=label, width=20).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Label(self.monitor_tab, textvariable=self.value_vars[key], width=90).grid(
                row=row, column=1, sticky="w", pady=5)

    def _build_trajectory(self) -> None:
        self.trajectory_text = tk.Text(self.trajectory_tab, height=6, state="disabled")
        self.trajectory_text.pack(fill=tk.X)
        self.trajectory_text.configure(state="normal")
        self.trajectory_text.insert(tk.END, "等待确认零点：先接收姿态遥测，再点击“以当前位姿为起点”。")
        self.trajectory_text.configure(state="disabled")
        controls = ttk.Frame(self.trajectory_tab)
        controls.pack(anchor="w", pady=6)
        ttk.Button(controls, text="以当前位姿为起点", command=self._mark_trajectory_origin).pack(side=tk.LEFT)
        ttk.Button(controls, text="开始采集", command=self._start_trajectory_capture).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="结束采集并拟合", command=self._finish_trajectory_capture).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="恢复并下载已有日志", command=self._download_existing_log).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="放大", command=lambda: self._zoom_trajectory(1.25)).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="缩小", command=lambda: self._zoom_trajectory(0.8)).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="复位缩放", command=lambda: self._set_trajectory_scale(1.0)).pack(side=tk.LEFT)
        ttk.Label(controls, text="曲线类型").pack(side=tk.LEFT, padx=(12, 4))
        mode_combo = ttk.Combobox(controls, textvariable=self.trajectory_mode_var,
                                  values=("相对姿态", "速度积分（模型估计）", "IMU积分（ZUPT）",
                                          "连续时间优化（B样条）"), width=22, state="readonly")
        mode_combo.pack(side=tk.LEFT)
        mode_combo.bind("<<ComboboxSelected>>", lambda _event: self._refresh_trajectory())
        ttk.Label(controls, text="视图缩放").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Scale(controls, from_=0.25, to=8.0, variable=self.trajectory_scale_var,
                  command=lambda _value: self._refresh_trajectory()).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        ttk.Label(controls, text="鼠标滚轮也可缩放").pack(side=tk.LEFT, padx=8)
        ttk.Label(controls, text="模型距离倍率").pack(side=tk.LEFT, padx=(10, 4))
        ttk.Entry(controls, textvariable=self.trajectory_distance_scale_var,
                  width=6).pack(side=tk.LEFT)
        ttk.Button(controls, text="应用", command=self._refresh_trajectory).pack(side=tk.LEFT, padx=4)
        ttk.Label(self.trajectory_tab, textvariable=self.capture_var).pack(anchor="w")
        progress_row = ttk.Frame(self.trajectory_tab)
        progress_row.pack(fill=tk.X, pady=(2, 6))
        ttk.Progressbar(progress_row, maximum=100.0, variable=self.capture_progress_var).pack(
            side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(progress_row, textvariable=self.capture_percent_var, width=6).pack(side=tk.LEFT, padx=(8, 0))
        self.plot_canvas = None
        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
            from matplotlib import rcParams
            rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans SC", "DejaVu Sans"]
            rcParams["axes.unicode_minus"] = False
            self.figure = Figure(figsize=(8, 4), dpi=100)
            self.axis = self.figure.add_subplot(111, projection="3d")
            self.axis.set_xlabel("前向 X (m)")
            self.axis.set_ylabel("右向 Y (m)")
            self.axis.set_zlabel("估计 Z (m)")
            self.axis.grid(True)
            self.plot_canvas = FigureCanvasTkAgg(self.figure, master=self.trajectory_tab)
            self.plot_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            canvas_widget = self.plot_canvas.get_tk_widget()
            canvas_widget.bind("<MouseWheel>", self._on_trajectory_wheel)
            canvas_widget.bind("<Button-4>", lambda _event: self._zoom_trajectory(1.25))
            canvas_widget.bind("<Button-5>", lambda _event: self._zoom_trajectory(0.8))
        except Exception:
            ttk.Label(self.trajectory_tab, text="未加载绘图库，仍可导出遥测 CSV。").pack()

    def _build_update(self) -> None:
        ttk.Label(self.update_tab, text="点击开始后，程序会自动请求目标板进入 Bootloader 并显示当前阶段。\n"
                                      "升级失败时 Bootloader 会保留并等待重新传输。\n"
                                      f"本地 UART：{DEFAULT_DATA_BAUD} 8N1；两端 LoRa 空口速率必须一致。遥测预览 10 Hz，原始 IMU 本地记录 200 Hz，当前默认空口 19.2 kbps。",
                  justify=tk.LEFT).pack(anchor="w")
        row = ttk.Frame(self.update_tab)
        row.pack(fill=tk.X, pady=12)
        self.firmware_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.firmware_var, width=80).pack(side=tk.LEFT)
        ttk.Button(row, text="选择 BIN", command=self._choose_firmware).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.update_tab, text="开始无线升级", command=self._start_update).pack(anchor="w")
        ttk.Label(self.update_tab, textvariable=self.update_var).pack(anchor="w", pady=10)
        self.update_progress = ttk.Progressbar(self.update_tab, orient="horizontal",
                                               mode="determinate", maximum=100.0,
                                               variable=self.update_progress_var, length=760)
        self.update_progress.pack(anchor="w", fill=tk.X, pady=(0, 10))

    def _build_params(self) -> None:
        ttk.Label(self.param_tab, text="运行期参数（当前仍为临时值；外挂 Flash 参数保存待 LoRa 闭环后接入）",
                  justify=tk.LEFT).pack(anchor="w")
        row = ttk.Frame(self.param_tab)
        row.pack(anchor="w", pady=12)
        ttk.Label(row, text="参数 ID").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.param_id_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(row, text="数值").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.param_value_var, width=16).pack(side=tk.LEFT, padx=5)
        ttk.Button(row, text="读取", command=self._read_param).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="写入", command=self._write_param).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="校准陀螺并保存", command=self._calibrate_gyro).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="保存参数（当前不支持）", command=self._save_param).pack(side=tk.LEFT, padx=4)
        ttk.Label(self.param_tab, text="1 相机偏航偏置(rad)  2 相机俯仰偏置(rad)  3 导航比  4 俯仰配平(rad/s)  5 角速度限幅(rad/s)").pack(anchor="w")
        ttk.Label(self.param_tab, textvariable=self.param_status_var).pack(anchor="w", pady=8)

    def _build_lora(self) -> None:
        ttk.Label(self.lora_tab, text="先将待配置模块 M0、M1 同时拉高。EWM32M 使用 AT 配置；EWM220M 使用 C0/C1 寄存器帧。完成后恢复 M0=M1=0。",
                  justify=tk.LEFT).pack(anchor="w")
        row = ttk.Frame(self.lora_tab)
        row.pack(anchor="w", pady=12)
        ttk.Label(row, text="模块型号").pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.lora_profile_var,
                     values=("EWM32M（电脑端）", "EWM220M-400T22S（主控端）"),
                     width=23, state="readonly").pack(side=tk.LEFT, padx=5)
        ttk.Label(row, text="配置 UART").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Combobox(row, textvariable=self.lora_baud_var, values=("9600", "115200"),
                     width=10, state="readonly").pack(side=tk.LEFT, padx=5)
        ttk.Label(row, text="数据 UART").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Combobox(row, textvariable=self.lora_uart_var,
                     values=("9600", "115200"), width=9, state="readonly").pack(side=tk.LEFT, padx=5)
        ttk.Label(row, text="空口编号").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Combobox(row, textvariable=self.lora_air_rate_var, values=tuple(str(i) for i in range(8)),
                     width=6, state="readonly").pack(side=tk.LEFT, padx=5)
        ttk.Label(row, text="信道").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(row, textvariable=self.lora_channel_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(row, text="读取型号/参数", command=self._probe_lora).pack(side=tk.LEFT, padx=8)
        ttk.Button(row, text="写入配置", command=self._configure_lora).pack(side=tk.LEFT)
        ttk.Label(self.lora_tab, textvariable=self.lora_status_var, justify=tk.LEFT).pack(anchor="w", pady=8)

    def _open_lora_config_port(self):
        if serial is None:
            raise RuntimeError("缺少 pyserial")
        if self.serial_port is not None:
            raise RuntimeError("请先断开实时观测串口")
        baud = DEFAULT_CONFIG_BAUD if self.lora_profile_var.get().startswith("EWM220") else int(self.lora_baud_var.get())
        return serial.Serial(self.port_var.get(), baud, timeout=1.0)

    def _probe_lora(self) -> None:
        try:
            with self._open_lora_config_port() as port:
                results = []
                ewm220 = self.lora_profile_var.get().startswith("EWM220")
                commands = (("参数", ewm220_read_all()),) if ewm220 else (("型号", b"\xC3\xC3\xC3"), ("参数", b"\xC1\xC1\xC1"))
                for name, command in commands:
                    port.reset_input_buffer()
                    port.write(command)
                    port.flush()
                    time.sleep(0.25)
                    response = port.read(64)
                    text = f"{name}: {response.hex(' ') or '无响应，请确认 M0=M1=1'}"
                    if ewm220 and response:
                        config = Ewm220Config.from_response(response)
                        text += f"\nEWM220：UART={config.uart_baud}，空口={config.air_rate}，信道={config.channel}"
                    results.append(text)
            self.lora_status_var.set("\n".join(results))
        except Exception as exc:
            self.lora_status_var.set(f"读取失败：{exc}")

    def _configure_lora(self) -> None:
        try:
            air_rate = int(self.lora_air_rate_var.get())
            with self._open_lora_config_port() as port:
                ewm220 = self.lora_profile_var.get().startswith("EWM220")
                if ewm220:
                    config = Ewm220Config(air_rate=air_rate, uart_baud=int(self.lora_uart_var.get()), channel=int(self.lora_channel_var.get()))
                    port.write(ewm220_write_all(config))
                    port.flush()
                    time.sleep(0.25)
                    response = port.read(32)
                    if response and response[:3] != b"\xC1\x00\x09":
                        raise RuntimeError(f"EWM220 应答异常：{response.hex(' ')}")
                    result = f"EWM220 配置完成：UART={config.uart_baud}，空口={air_rate}，信道={config.channel}"
                else:
                    channel = int(self.lora_channel_var.get())
                    if not 0 <= channel <= 69:
                        raise ValueError("EWM32 信道必须为 0~69")
                    # EWM32 与 EWM220 的地址、信道和空速必须保持一致。
                    commands = ("AT+UART=7,0", "AT+ADDR=0", f"AT+CHANNEL={channel}",
                                f"AT+RATE={air_rate}", "AT+TRANS=0", "AT+FEC=1", "AT+POWER=0")
                    replies = []
                    for command in commands:
                        port.write((command + "\r\n").encode("ascii"))
                        port.flush()
                        time.sleep(0.15)
                        replies.append(f"{command} -> {port.read_all().decode('ascii', errors='replace').strip() or '无响应'}")
                    result = f"EWM32 配置完成：恢复 M0=M1=0 并重新上电后使用 {DEFAULT_DATA_BAUD}。\n" + "\n".join(replies)
            self.lora_status_var.set(result)
        except Exception as exc:
            self.lora_status_var.set(f"配置失败：{exc}")

    def _param_id(self) -> int:
        value = int(self.param_id_var.get())
        if value not in range(1, 6):
            raise ValueError("参数 ID 必须为 1~5")
        return value

    def _read_param(self) -> None:
        try:
            if self.serial_port is None:
                raise RuntimeError("请先连接串口")
            param_id = self._param_id()
            self.serial_port.write(build_param_read(param_id))
            threading.Thread(target=self._param_read_worker, args=(param_id,), daemon=True).start()
        except Exception as exc:
            self.param_status_var.set(f"读取失败：{exc}")

    def _param_read_worker(self, param_id: int) -> None:
        try:
            frame = self._wait_frame(MSG_PARAM_READ, 3.0)
            if len(frame.payload) != 5 or frame.payload[0] != param_id:
                raise RuntimeError("参数应答内容异常")
            import struct
            value = f"{struct.unpack('<f', frame.payload[1:])[0]:.6g}"
            self.after(0, self.param_value_var.set, value)
            self.after(0, self.param_status_var.set, "读取成功")
        except Exception as exc:
            self.after(0, self.param_status_var.set, f"读取失败：{exc}")

    def _write_param(self) -> None:
        try:
            if self.serial_port is None:
                raise RuntimeError("请先连接串口")
            param_id = self._param_id()
            value = float(self.param_value_var.get())
            self.serial_port.write(build_param_write(param_id, value))
            threading.Thread(target=self._param_write_worker, daemon=True).start()
        except Exception as exc:
            self.param_status_var.set(f"写入失败：{exc}")

    def _param_write_worker(self) -> None:
        try:
            ack = self._wait_ack(MSG_PARAM_WRITE, 3.0)
            message = "写入成功" if ack[1] == 0 else f"写入被拒绝，状态码 {ack[1]}"
            self.after(0, self.param_status_var.set, message)
        except Exception as exc:
            self.after(0, self.param_status_var.set, f"写入失败：{exc}")

    def _save_param(self) -> None:
        if self.serial_port is None:
            self.param_status_var.set("请先连接串口")
            return
        self.serial_port.write(build_param_save())
        self.param_status_var.set("当前参数仅运行期生效；外挂 Flash 持久化待 LoRa 闭环后接入")

    def _calibrate_gyro(self) -> None:
        if self.serial_port is None:
            self.param_status_var.set("请先连接串口")
            return
        self.serial_port.write(build_calibrate_gyro())
        self.param_status_var.set("正在校准，请保持主控静止约 2 秒…")
        threading.Thread(target=self._calibrate_gyro_worker, daemon=True).start()

    def _calibrate_gyro_worker(self) -> None:
        try:
            ack = self._wait_ack(MSG_CALIBRATE_GYRO, 8.0)
            message = "陀螺校准完成，已保存到外挂 Flash" if ack[1] == 0 else f"校准失败，状态码 {ack[1]}"
            self.after(0, self.param_status_var.set, message)
        except Exception as exc:
            self.after(0, self.param_status_var.set, f"校准失败：{exc}")

    def _refresh_ports(self) -> None:
        if list_ports is None:
            self.port_combo["values"] = []
            self.status_var.set("缺少 pyserial")
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _connect(self) -> None:
        if serial is None:
            messagebox.showerror("缺少依赖", "请安装 pyserial 后再运行上位机。")
            return
        if self.serial_port is not None:
            return
        try:
            self.serial_port = serial.Serial(self.port_var.get(), UART_BAUDRATE, timeout=0.1)
            self.reader_stop.clear()
            self.reader_thread = threading.Thread(target=self._reader, daemon=True)
            self.reader_thread.start()
            self.status_var.set(f"已连接 {self.port_var.get()}")
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def _disconnect(self) -> None:
        self.reader_stop.set()
        if self.serial_port is not None:
            self.serial_port.close()
        self.serial_port = None
        self.status_var.set("未连接")

    def _link_test(self) -> None:
        """请求设备信息；不发送控制量，不改变飞行参数。"""
        if self.serial_port is None:
            self.status_var.set("链路测试失败：请先连接 COM 口")
            return
        self.serial_port.write(build_device_info())
        self.status_var.set("正在等待主控设备应答…")
        threading.Thread(target=self._link_test_worker, daemon=True).start()

    def _link_test_worker(self) -> None:
        try:
            frame = self._wait_frame(MSG_DEVICE_INFO, 3.0)
            self.after(0, self.status_var.set, f"LoRa 链路正常：设备信息 {frame.payload.hex(' ')}")
        except Exception as exc:
            self.after(0, self.status_var.set, f"LoRa 链路无应答：{exc}")

    def _reader(self) -> None:
        while not self.reader_stop.is_set() and self.serial_port is not None:
            try:
                data = self.serial_port.read(256)
                if data:
                    for frame in self.frame_parser.feed(data):
                        self.frame_queue.put(frame)
                    for sample in self.text_parser.feed(data):
                        self.text_queue.put(sample)
            except Exception:
                break

    def _poll_frames(self) -> None:
        received_telemetry = False
        while True:
            try:
                sample = self.text_queue.get_nowait()
            except queue.Empty:
                break
            self._accept_telemetry(sample)
            received_telemetry = True
        while True:
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break
            if frame.message_type == 0x01:
                sample = decode_telemetry(frame)
                if sample is not None:
                    self._accept_telemetry(sample)
                    received_telemetry = True
            else:
                if frame.message_type == MSG_LOG_INFO and self.capture_state == "recording":
                    info = parse_log_info(frame)
                    if info is not None:
                        self._accept_recording_info(info)
                    continue
                if frame.message_type != MSG_KEEPALIVE:
                    self.control_queue.put(frame)
                if (frame.message_type == MSG_KEEPALIVE and len(frame.payload) >= 2
                        and self.capture_state == "idle"):
                    calibrated = "已校准" if frame.payload[0] else "待校准"
                    self.status_var.set(f"LoRa 保活正常（{calibrated}，{phase_name(frame.payload[1])}）")
        if received_telemetry and time.monotonic() - self.last_plot_time >= 0.2:
            self._refresh_trajectory()
            self.last_plot_time = time.monotonic()
        if (self.capture_state == "recording" and self.serial_port is not None
                and not self.log_lock.locked() and time.monotonic() - self.last_log_status_query >= 5.0):
            self.last_log_status_query = time.monotonic()
            try:
                self.serial_port.write(build_log_info())
            except Exception as exc:
                self.capture_var.set(f"日志状态查询暂时失败：{exc}；主控记录状态尚未确认")
        if self.capture_state == "recording" and time.monotonic() - self.last_telemetry_time > 3.0:
            confirmed = time.monotonic() - self.last_log_status_time < 10.0
            self.capture_var.set("实时遥测暂时中断；板上原始 IMU 仍在记录"
                                 if confirmed else "实时遥测暂时中断；板上记录状态待确认，可稍后结束下载")
        self.after(25, self._poll_frames)

    def _accept_recording_info(self, info) -> None:
        if info.state == 1:
            self.board_log_count = info.record_count
            self.last_log_status_time = time.monotonic()
        else:
            self.trajectory_capturing = False
            self.capture_state = "retry" if info.state == 2 else "uncertain"
            self.capture_var.set("板上记录已结束；点击结束采集并拟合下载"
                                 if info.state == 2 else "板上日志状态异常；请检查供电与 Flash")

    def _accept_telemetry(self, sample: Telemetry) -> None:
        self.telemetry.append(sample)
        self.last_telemetry_time = time.monotonic()
        if self.trajectory_origin is not None and self.trajectory_capturing:
            self.trajectory_samples.append(sample)
            if self.capture_state == "recording":
                self.capture_var.set(f"实时遥测预览 {len(self.trajectory_samples)} 帧；"
                                     f"板上原始 IMU 已确认 {self.board_log_count} 帧")
        self._show_telemetry(sample)

    def _mark_trajectory_origin(self) -> None:
        if self.capture_state != "idle":
            self.status_var.set("采集或下载期间不能重置轨迹零点")
            return
        if not self.telemetry:
            messagebox.showinfo("暂无位姿", "先连接并等待一帧姿态遥测。")
            return
        sample = self.telemetry[-1]
        self.trajectory_origin = (sample.roll, sample.pitch, sample.yaw)
        self.trajectory_samples = [sample]
        self.status_var.set("已确认零点；点击开始采集后主控记录原始 IMU")
        self._refresh_trajectory()

    def _start_trajectory_capture(self) -> None:
        """启动主控原始 IMU 记录，同时使用实时遥测预览。"""
        if self.serial_port is None:
            messagebox.showwarning("链路未就绪", "请先连接 LoRa 串口。")
            return
        if self.capture_state != "idle" or self.log_lock.locked() or self.update_lock.locked():
            messagebox.showwarning("设备忙", "已有采集、下载或 OTA 任务正在运行。")
            return
        if not messagebox.askyesno("覆盖旧日志", "开始采集会覆盖上一段原始 IMU 日志，旧数据将无法在上位机恢复，是否继续？"):
            return
        sample = self.telemetry[-1] if self.telemetry else None
        self.trajectory_origin = ((sample.roll, sample.pitch, sample.yaw)
                                  if sample is not None else (0.0, 0.0, 0.0))
        self.trajectory_samples = [sample] if sample is not None else []
        self.trajectory_mode_var.set("相对姿态")
        self.trajectory_capturing = False
        self.capture_state = "starting"
        self.finish_requested = False
        self.last_log_status_query = time.monotonic()
        self.last_log_status_time = 0.0
        self.board_log_count = 0
        self._set_capture_progress(0)
        self.capture_var.set("正在启动 200 Hz 原始 IMU 记录；实时遥测仅用于预览")
        self._refresh_trajectory()
        threading.Thread(target=self._raw_log_start_worker, daemon=True).start()

    def _finish_trajectory_capture(self) -> None:
        """停止原始记录，完整下载并拟合，不使用遥测预览帧充数。"""
        if self.capture_state == "starting":
            self.finish_requested = True
            self.capture_var.set("正在等待主控确认开始记录；确认后自动结束并拟合")
            return
        if self.capture_state in ("downloading", "fitting"):
            self.status_var.set("日志正在下载或拟合；再次点击不会暂停任务")
            return
        if self.capture_state not in ("recording", "retry", "uncertain"):
            self.status_var.set("没有待结束的采集；请先确认开始成功，或选择恢复并下载已有日志")
            return
        if self.update_lock.locked():
            self.status_var.set("OTA 正在运行，暂不能下载日志")
            return
        if self.log_lock.locked():
            self.finish_requested = True
            self.after(100, self._finish_trajectory_capture)
            return
        if self.capture_state == "recording":
            fresh = time.monotonic() - self.last_telemetry_time <= 2.0
            still = has_terminal_stationary_interval(self.trajectory_samples) if fresh else False
            if not still and not messagebox.askyesno(
                    "终点尚未确认静止",
                    "最近 2 秒未检测到连续静止。现在结束会缺少末速度校正，"
                    "位移可能严重漂移。请将主控固定在终点后继续等待；"
                    "仍要强制结束并下载吗？"):
                self.capture_var.set("主控仍在记录；请固定在终点并持续静止 2 秒后再次点击结束")
                return
        self.finish_requested = False
        self.capture_state = "downloading"
        self.trajectory_capturing = False
        self._set_capture_progress(1)
        self.capture_var.set("正在停止记录并下载完整原始日志；请保持主控和 LoRa 供电")
        threading.Thread(target=self._raw_log_stop_worker, daemon=True).start()

    def _set_capture_progress(self, value: float) -> None:
        progress = max(0.0, min(100.0, value))
        self.capture_progress_var.set(progress)
        self.capture_percent_var.set("100%" if progress == 100.0 else f"{progress:.1f}%")

    def _download_existing_log(self) -> None:
        """重新连接后读取外挂 Flash 中尚未下载完成的日志。"""
        if self.serial_port is None or self.capture_state != "idle":
            self.status_var.set("请先连接串口，并等待当前采集或下载结束")
            return
        self.capture_state = "retry"
        self._set_capture_progress(0)
        self._finish_trajectory_capture()

    def _raw_log_start_worker(self) -> None:
        if not self.log_lock.acquire(blocking=False):
            self.after(0, self._capture_start_failed, "设备忙", False)
            return
        try:
            ack = None
            for attempt in range(12):
                self.serial_port.write(build_log_start(200))
                try:
                    ack = self._wait_ack(MSG_LOG_START, timeout=1.2)
                except TimeoutError:
                    time.sleep(0.07 * (attempt + 1))
                    continue
                break
            if ack is None:
                raise TimeoutError("开始记录命令未获确认")
            original_type, status, _ = ack
            if original_type != MSG_LOG_START or status != 0:
                reason = {2: "主控仍运行限制飞行阶段的旧固件，请更新 App",
                          3: "外挂 Flash 擦写失败", 4: "采样率不支持"}.get(status, f"状态={status}")
                self.after(0, self._capture_start_failed,
                           f"主控拒绝原始日志启动：{reason}", False)
                return
            info = self._wait_log_info(expected_state=1)
            if info.state != 1:
                raise RuntimeError(f"主控日志状态异常：{info.state}")
            self.after(0, self._capture_started, info.sample_rate_hz)
        except Exception as exc:
            self.after(0, self._capture_start_failed, str(exc), True)
        finally:
            self.log_lock.release()

    def _capture_started(self, sample_rate_hz: int) -> None:
        self.capture_state = "recording"
        self.trajectory_capturing = True
        self.last_log_status_time = time.monotonic()
        self.status_var.set(f"原始 IMU 日志正在主控记录：{sample_rate_hz} Hz")
        if self.finish_requested:
            self.after(100, self._finish_trajectory_capture)

    def _capture_start_failed(self, error: str, uncertain: bool) -> None:
        self.capture_state = "uncertain" if uncertain else "idle"
        self.trajectory_capturing = False
        self.capture_var.set("启动结果未确认；可结束并尝试恢复日志" if uncertain
                             else f"主控未开始记录：{error}")
        self.status_var.set(f"原始日志启动失败：{error}")
        if uncertain and self.finish_requested:
            self.after(100, self._finish_trajectory_capture)
        else:
            self.finish_requested = False

    def _raw_log_stop_worker(self) -> None:
        if not self.log_lock.acquire(blocking=False):
            self.after(0, self._capture_download_failed, "设备忙，请重试")
            return
        try:
            ack = None
            for attempt in range(12):
                self.serial_port.write(build_log_stop())
                try:
                    ack = self._wait_ack(MSG_LOG_STOP, timeout=1.2)
                except TimeoutError:
                    time.sleep(0.07 * (attempt + 1))
                    continue
                break
            if ack is None:
                raise TimeoutError("停止记录命令未获确认")
            original_type, status, _ = ack
            if original_type != MSG_LOG_STOP or status != 0:
                raise RuntimeError(f"主控停止日志失败，状态={status}")
            self.after(0, self._set_capture_progress, 3)
            info = self._wait_log_info(expected_state=2)
            if info.state != 2 or info.record_size != 32 or info.data_bytes != info.record_count * 32:
                raise RuntimeError("主控没有返回可读取的日志")
            self.after(0, self._set_capture_progress, 5)
            raw = bytearray(info.data_bytes)
            offset = 0
            while offset < info.data_bytes:
                remaining = info.data_bytes - offset
                count = min(8, remaining // 40) if remaining >= 40 else 1
                length = 40 if remaining >= 40 else remaining
                pending = {offset + i * length for i in range(count)}
                for burst_try in range(2):
                    self.serial_port.write(build_log_read(offset, length, count))
                    deadline = time.monotonic() + 2.0
                    while pending and time.monotonic() < deadline:
                        try:
                            frame = self._wait_frame(MSG_LOG_DATA, timeout=deadline - time.monotonic())
                        except TimeoutError:
                            break
                        reply_offset = int.from_bytes(frame.payload[:4], "little")
                        if reply_offset in pending and len(frame.payload) == length + 5 \
                                and frame.payload[4] == length:
                            raw[reply_offset:reply_offset + length] = frame.payload[5:]
                            pending.remove(reply_offset)
                    if not pending:
                        break
                for missing in sorted(pending):
                    for _retry in range(40):
                        self.serial_port.write(build_log_read(missing, length))
                        try:
                            frame = self._wait_frame(MSG_LOG_DATA, timeout=1.8)
                        except TimeoutError:
                            continue
                        if len(frame.payload) == length + 5 and frame.payload[4] == length \
                                and int.from_bytes(frame.payload[:4], "little") == missing:
                            raw[missing:missing + length] = frame.payload[5:]
                            break
                    else:
                        raise TimeoutError(f"日志块 {missing} 下载失败")
                completed = offset + count * length
                if offset % 3200 == 0 or completed == info.data_bytes:
                    percent = 5.0 + 90.0 * completed / max(1, info.data_bytes)
                    self.after(0, self._set_capture_progress, percent)
                    self.after(0, self.capture_var.set,
                               f"正在下载原始日志：{completed}/{info.data_bytes} 字节")
                offset = completed
            samples = decode_imu_log_records(bytes(raw))
            if len(samples) != info.record_count or len(samples) < 2:
                raise RuntimeError(f"日志记录不完整：校验通过 {len(samples)}/{info.record_count} 帧")
            self.after(0, self._set_capture_progress, 98)
            self.after(0, self._accept_downloaded_log, samples)
        except Exception as exc:
            self.after(0, self._capture_download_failed, str(exc))
        finally:
            self.log_lock.release()

    def _capture_download_failed(self, error: str) -> None:
        self.capture_state = "retry"
        self.capture_var.set("下载未完成；可再次点击结束采集并拟合重试，旧日志未清除")
        self.status_var.set(f"原始日志下载失败：{error}")

    def _wait_log_info(self, expected_state: int | None = None):
        for attempt in range(6):
            if attempt:
                self.serial_port.write(build_log_info())
            try:
                info = parse_log_info(self._wait_frame(MSG_LOG_INFO, timeout=2.0))
            except TimeoutError:
                info = None
            if info is not None and info.state == 3:
                raise RuntimeError("主控日志 Flash 处理失败")
            if info is not None and (expected_state is None or info.state == expected_state):
                return info
            time.sleep(0.07 * (attempt + 1))
        raise TimeoutError("日志信息未获确认")

    def _accept_downloaded_log(self, samples: list[Telemetry]) -> None:
        self.capture_state = "fitting"
        self._set_capture_progress(99)
        self.capture_var.set("日志已完整校验，正在拟合并绘制曲线")
        self.update_idletasks()
        self.telemetry = samples.copy()
        self.trajectory_samples = samples.copy()
        self.trajectory_origin = (samples[0].roll, samples[0].pitch, samples[0].yaw)
        self.trajectory_capturing = False
        self.trajectory_mode_var.set("IMU积分（ZUPT）")
        try:
            if self.plot_canvas is None:
                raise RuntimeError("缺少绘图库，无法生成拟合曲线")
            self._refresh_trajectory()
            self.plot_canvas.draw()
        except Exception as exc:
            self.capture_state = "idle"
            self.capture_var.set(f"日志已校验，但拟合失败：{exc}")
            self.status_var.set(f"拟合失败：{exc}")
            return
        self.capture_state = "idle"
        self._set_capture_progress(100)
        self.capture_var.set(f"原始 IMU 日志 {len(samples)} 帧已完整校验并拟合；实时预览帧未用于最终拟合")
        self.status_var.set(f"原始 IMU 日志下载及 ZUPT 拟合完成：{len(samples)} 帧")

    def _set_trajectory_scale(self, scale: float) -> None:
        self.trajectory_scale_var.set(max(0.25, min(8.0, scale)))
        self._refresh_trajectory()

    def _zoom_trajectory(self, factor: float) -> None:
        self._set_trajectory_scale(self.trajectory_scale_var.get() * factor)

    def _on_trajectory_wheel(self, event) -> None:
        self._zoom_trajectory(1.25 if event.delta > 0 else 0.8)

    def _trajectory_distance_scale(self) -> float:
        """读取模型距离校准倍率；该倍率只作用于模型下限速度。"""
        try:
            value = float(self.trajectory_distance_scale_var.get())
        except (TypeError, ValueError):
            value = 1.0
        value = max(0.05, min(10.0, value))
        self.trajectory_distance_scale_var.set(f"{value:.2f}")
        return value

    def _show_telemetry(self, sample: Telemetry) -> None:
        self.value_vars["phase"].set(phase_name(sample.phase))
        self.value_vars["time"].set(f"{sample.flight_time:.2f} s")
        speed_label = "估计" if sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED else "未测量"
        self.value_vars["speed"].set(f"{sample.speed:.2f} m/s（{speed_label}）")
        self.value_vars["pqr"].set(", ".join(f"{math.degrees(v):+.2f}" for v in (sample.p, sample.q, sample.r)) + " °/s")
        self.value_vars["apqr"].set(", ".join(f"{math.degrees(v):+.2f}" for v in (sample.p_accel, sample.q_accel, sample.r_accel)) + " °/s²")
        self.value_vars["attitude"].set(", ".join(f"{math.degrees(v):+.2f}" for v in (sample.roll, sample.pitch, sample.yaw)) + " °")
        self.value_vars["accel"].set(f"{sample.ax:+.2f}, {sample.ay:+.2f}, {sample.az:+.2f} m/s²")
        self.value_vars["vision"].set(f"CX={sample.cx}, CY={sample.cy}, flags=0x{sample.flags:02X}")
        self.value_vars["servos"].set(" / ".join(str(v) for v in sample.servos) + " us")
        self.value_vars["tgo"].set(f"{sample.tgo:.2f} s")

    def _clear_data(self) -> None:
        """清空已采集的数据和零点，下一次确认零点后开始新曲线。"""
        if self.capture_state != "idle":
            self.status_var.set("采集或下载期间不能清空数据")
            return
        self.telemetry.clear()
        self.trajectory_samples.clear()
        self.trajectory_origin = None
        self.trajectory_capturing = False
        for pending in (self.text_queue, self.frame_queue):
            while True:
                try:
                    frame = pending.get_nowait()
                except queue.Empty:
                    break
                if isinstance(frame, Frame) and frame.message_type not in (0x01, MSG_KEEPALIVE):
                    self.control_queue.put(frame)
        if self.plot_canvas is not None:
            self.axis.clear()
            self.plot_canvas.draw_idle()
        self.trajectory_text.configure(state="normal")
        self.trajectory_text.delete("1.0", tk.END)
        self.trajectory_text.insert(tk.END, "数据已清空；收到新遥测后请重新确认零点。")
        self.trajectory_text.configure(state="disabled")
        self.status_var.set("数据已清空")

    def _export_csv(self) -> None:
        """导出当前数据；原始日志后续的实时预览帧不混入日志 CSV。"""
        if not self.telemetry:
            messagebox.showinfo("暂无数据", "当前没有可导出的遥测，请先采集数据。")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        log_length = raw_imu_log_prefix_length(self.telemetry)
        samples = self.telemetry[:log_length] if log_length else self.telemetry
        with open(path, "w", newline="", encoding="utf-8-sig") as output:
            writer = csv.writer(output)
            fields = tuple(Telemetry.__dataclass_fields__)
            writer.writerow(fields)
            writer.writerows(tuple(getattr(sample, key) for key in fields) for sample in samples)
        self.status_var.set(f"已导出 {len(samples)} 帧{'原始 IMU 日志' if log_length else '遥测'}")

    def _import_csv(self) -> None:
        """导入已有遥测，直接复用实时轨迹和状态显示，便于离线复盘。"""
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv")])
        if not path:
            return
        loaded: list[Telemetry] = []
        try:
            with open(path, "r", newline="", encoding="utf-8-sig") as source:
                for row in csv.DictReader(source):
                    loaded.append(Telemetry(
                        int(row["phase"]), int(row["flags"]), int(row["cx"]), int(row["cy"]),
                        float(row["p"]), float(row["q"]), float(row["r"]),
                        float(row["p_accel"]), float(row["q_accel"]), float(row["r_accel"]),
                        float(row["roll"]), float(row["pitch"]), float(row["yaw"]),
                        float(row["ax"]), float(row["ay"]), float(row["az"]),
                        tuple(ast.literal_eval(row["servos"])), float(row["tgo"]),
                        float(row["flight_time"]), float(row["speed"]), int(row["timestamp_ms"])))
        except (KeyError, TypeError, ValueError, SyntaxError) as exc:
            messagebox.showerror("CSV 格式错误", f"无法读取遥测字段：{exc}")
            return
        if not loaded:
            messagebox.showinfo("没有数据", "CSV 中没有可用遥测帧。")
            return
        log_length = raw_imu_log_prefix_length(loaded)
        if log_length:
            loaded = loaded[:log_length]
            # 旧版导出的原始日志曾误带模型速度标志；原始记录没有速度测量。
            for sample in loaded:
                sample.flags &= ~TELEMETRY_FLAG_SPEED_ESTIMATED
        self.telemetry = loaded
        self.trajectory_samples = loaded.copy()
        self.trajectory_origin = (loaded[0].roll, loaded[0].pitch, loaded[0].yaw)
        self.trajectory_capturing = False
        if log_length:
            self.trajectory_mode_var.set("IMU积分（ZUPT）")
        self._refresh_trajectory()
        self.status_var.set(f"已导入 {len(loaded)} 帧{'原始 IMU 日志' if log_length else '遥测'}：{Path(path).name}")

    def _refresh_trajectory(self) -> None:
        if self.plot_canvas is None or not self.trajectory_samples:
            return
        sessions = trajectory_session_ranges(self.trajectory_samples)
        mode = "相对姿态" if self.trajectory_capturing else self.trajectory_mode_var.get()
        if mode == "相对姿态":
            x, y, z = estimate_relative_attitude(self.trajectory_samples, self.trajectory_origin)
            x, y, z = [list(map(math.degrees, axis)) for axis in (x, y, z)]
            times = [((sample.timestamp_ms - self.trajectory_samples[0].timestamp_ms) & 0xFFFFFFFF) / 1000.0
                     for sample in self.trajectory_samples]
            self.figure.clear()
            self.axis = self.figure.add_subplot(111)
            self.axis.plot(times, x, label="偏航", color="tab:blue")
            self.axis.plot(times, y, label="俯仰", color="tab:orange")
            self.axis.plot(times, z, label="滚转", color="tab:green")
            self.axis.set_xlabel("确认零点后的时间 (s)")
            self.axis.set_ylabel("相对角度 (°)")
            self.axis.legend(loc="upper left")
            self.axis.grid(True)
            window = max(1.0, times[-1] * 1.05) / self.trajectory_scale_var.get()
            left = max(0.0, times[-1] - window)
            self.axis.set_xlim(left, left + window)
            visible = [value for t, values in zip(times, zip(x, y, z))
                       if t >= left for value in values]
            center = 0.5 * (min(visible) + max(visible))
            half = max(0.5, 0.6 * (max(visible) - min(visible))) / self.trajectory_scale_var.get()
            self.axis.set_ylim(center - half, center + half)
            self.plot_canvas.draw_idle()
            summary = (f"相对姿态：{len(times)} 帧，持续 {times[-1]:.1f} s；"
                       f"偏航 {x[-1]:+.2f}°、俯仰 {y[-1]:+.2f}°、滚转 {z[-1]:+.2f}°。")
            if len(times) == 1:
                summary += " 已确认零点，等待后续遥测。"
            elif max(visible) - min(visible) < 0.1:
                summary += " 当前姿态读数尚无可见变化。"
        elif mode in ("速度积分（模型估计）", "速度积分（估计位移）"):
            distance_scale = self._trajectory_distance_scale()
            x, y, z = estimate_trajectory(
                self.trajectory_samples, self.trajectory_origin,
                estimated_speed_scale=distance_scale)
            # 固件导航系 Z 轴向下，界面改用用户直观的“高度向上为正”。
            z = [-value for value in z]
            self.figure.clear()
            self.axis = self.figure.add_subplot(111, projection="3d")
            for begin, end in sessions:
                self.axis.plot3D(x[begin:end], y[begin:end], z[begin:end], color="tab:blue")
            self.axis.scatter([x[0]], [y[0]], [z[0]], color="tab:green", label="起点")
            self.axis.scatter([x[-1]], [y[-1]], [z[-1]], color="tab:red", label="当前")
            self.axis.set_xlabel("估计 X (m)")
            self.axis.set_ylabel("估计 Y (m)")
            self.axis.set_zlabel("高度 Z（向上为正，m）")
            scale = self.trajectory_scale_var.get()
            points = (x, y, z)
            ranges = [max(values) - min(values) for values in points]
            centers = [0.5 * (min(values) + max(values)) for values in points]
            # 每个方向独立留白，再按真实范围设置盒体比例；旧版用最大轴
            # 统一范围，导致厘米级的侧向/竖向变化在图上完全看不见。
            limits = []
            for center, extent in zip(centers, ranges):
                margin = max(0.02, extent * 0.12) / scale
                limits.append((center - 0.5 * extent - margin,
                               center + 0.5 * extent + margin))
            self.axis.set_xlim(*limits[0])
            self.axis.set_ylim(*limits[1])
            self.axis.set_zlim(*limits[2])
            self.axis.set_box_aspect(tuple(max(extent, 0.05) for extent in ranges))
            self.axis.legend(loc="upper left")
            self.axis.grid(True)
            self.plot_canvas.draw_idle()
            path_length = sum(((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2
                               + (z[i] - z[i - 1]) ** 2) ** 0.5
                              for begin, end in sessions for i in range(begin + 1, end))
            endpoint = (x[-1] ** 2 + y[-1] ** 2 + z[-1] ** 2) ** 0.5
            summary = (f"速度积分仅为模型估计，非真实定位；{len(x)} 帧，"
                       f"终点距离起点 {endpoint:.2f} m，累计路径 {path_length:.2f} m；"
                       f"模型距离倍率 {distance_scale:.2f}。")
            if self.trajectory_samples and all(
                    (sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED) != 0
                    for sample in self.trajectory_samples if sample.speed > 0.0):
                summary += " 当前速度为模型估计，距离比例和前进/返回符号没有传感器依据。"
            if not any(sample.flags & TELEMETRY_FLAG_VELOCITY_DIRECTION
                       for sample in self.trajectory_samples):
                summary += " 未收到固件速度方向字段，三轴方向仍由姿态推断。"
        else:
            continuous = mode == "连续时间优化（B样条）"
            has_model_prior = any(
                sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED and sample.speed > 0.0
                for sample in self.trajectory_samples)
            spline_active = continuous and has_model_prior
            if continuous:
                x, y, z = estimate_continuous_trajectory(
                    self.trajectory_samples, self.trajectory_origin,
                    estimated_speed_scale=self._trajectory_distance_scale())
            else:
                x, y, z = estimate_inertial_trajectory(
                    self.trajectory_samples, self.trajectory_origin)
            z = [-value for value in z]
            self.figure.clear()
            self.axis = self.figure.add_subplot(111, projection="3d")
            for begin, end in sessions:
                self.axis.plot3D(x[begin:end], y[begin:end], z[begin:end], color="tab:purple")
            self.axis.scatter([x[0]], [y[0]], [z[0]], color="tab:green", label="起点")
            self.axis.scatter([x[-1]], [y[-1]], [z[-1]], color="tab:red", label="当前")
            label = "连续时间 B 样条" if spline_active else "IMU 积分"
            self.axis.set_xlabel(f"{label} 前向 X (m)")
            self.axis.set_ylabel(f"{label} 右向 Y (m)")
            self.axis.set_zlabel(f"{label} 高度 Z（向上为正，m）")
            scale = self.trajectory_scale_var.get()
            points = (x, y, z)
            ranges = [max(values) - min(values) for values in points]
            centers = [0.5 * (min(values) + max(values)) for values in points]
            limits = []
            for center, extent in zip(centers, ranges):
                margin = max(0.02, extent * 0.12) / scale
                limits.append((center - 0.5 * extent - margin,
                               center + 0.5 * extent + margin))
            self.axis.set_xlim(*limits[0])
            self.axis.set_ylim(*limits[1])
            self.axis.set_zlim(*limits[2])
            self.axis.set_box_aspect(tuple(max(extent, 0.05) for extent in ranges))
            self.axis.legend(loc="upper left")
            self.axis.grid(True)
            self.plot_canvas.draw_idle()
            path_length = sum(((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2
                               + (z[i] - z[i - 1]) ** 2) ** 0.5
                              for begin, end in sessions for i in range(begin + 1, end))
            endpoint = (x[-1] ** 2 + y[-1] ** 2 + z[-1] ** 2) ** 0.5
            if spline_active:
                summary = (f"连续时间 B 样条离线优化（非定位结果）；{len(x)} 帧，"
                           f"终点 X={x[-1]:+.2f}、Y={y[-1]:+.2f}、高度={z[-1]:+.2f} m，"
                           f"三维距离 {endpoint:.2f} m，累计路径 {path_length:.2f} m；"
                           "已联合拟合加速度、速度模型和静止零速约束。")
            else:
                summary = (f"IMU 积分与静止零速修正（非定位结果）；{len(x)} 帧，"
                           f"终点 X={x[-1]:+.2f}、Y={y[-1]:+.2f}、高度={z[-1]:+.2f} m，"
                           f"三维距离 {endpoint:.2f} m，累计路径 {path_length:.2f} m。")
                summary += " 此模式不使用模型速度作距离先验；没有外部速度观测，结果仍会漂移。"
                if continuous and not spline_active:
                    summary += " 本次数据未使用 B 样条，已退回 IMU 零速修正。"
                warnings = inertial_capture_warnings(self.trajectory_samples)
                if warnings:
                    summary += " 注意：" + "；".join(warnings) + "。"
            # 三轴分离的可信度取决于固件是否持续提供速度方向，而不是曲线
            # 是否经过了平滑拟合；在所有轨迹模式中统一提示覆盖率，避免把
            # 只有机体前向推断的历史数据误认为完整三维速度。
            motion_count = sum(1 for sample in self.trajectory_samples
                               if sample.speed > 0.0
                               and sample.flags & TELEMETRY_FLAG_SPEED_ESTIMATED)
            direction_count = sum(1 for sample in self.trajectory_samples
                                  if sample.speed > 0.0
                                  and sample.flags & TELEMETRY_FLAG_VELOCITY_DIRECTION)
            if not spline_active:
                summary += " 三轴来自姿态旋转后的加速度；缺少绝对航向参考，方向仍可能漂移。"
            elif direction_count == 0:
                summary += " 未收到速度方向字段，模型速度只能按机体前向推断。"
            elif direction_count < max(3, motion_count // 5):
                summary += (f" 速度方向字段覆盖率仅 {direction_count / motion_count:.0%}，"
                            "三轴曲线可能存在方向跳变。")
        self.trajectory_text.configure(state="normal")
        self.trajectory_text.delete("1.0", tk.END)
        self.trajectory_text.insert(tk.END, summary)
        self.trajectory_text.configure(state="disabled")

    def _choose_firmware(self) -> None:
        """选择链接到 App 分区的独立 BIN，不接受整片 Boot+App 镜像。"""
        path = filedialog.askopenfilename(filetypes=[("Firmware BIN", "*.bin")])
        if path:
            self.firmware_var.set(path)

    def _wait_ack(self, expected_type: int, timeout: float = 10.0) -> tuple[int, int, int]:
        """只消费指定原始消息类型的 ACK，返回状态和下一待发块号。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                frame = self.control_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            ack = parse_ack(frame)
            if ack is not None and ack[0] == expected_type:
                return ack
        raise TimeoutError(f"等待消息 0x{expected_type:02X} 应答超时")

    def _wait_frame(self, message_type: int, timeout: float = 10.0) -> Frame:
        """等待控制队列中的指定类型帧；遥测帧由串口接收流程另行处理。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                frame = self.control_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if frame.message_type == message_type:
                return frame
        raise TimeoutError("等待设备应答超时")

    def _start_update(self) -> None:
        """检查串口及文件的基本条件，再启动不阻塞界面的升级线程。"""
        if self.serial_port is None:
            messagebox.showwarning("未连接", "请先连接 PC 端 LoRa 模块。")
            return
        path = Path(self.firmware_var.get())
        if not path.is_file():
            messagebox.showwarning("文件无效", "请选择有效的 App BIN 文件。")
            return
        if path.stat().st_size == 0 or path.stat().st_size > 110 * 1024:
            messagebox.showwarning("文件无效", "App BIN 必须大于 0 且不超过 110 KB。")
            return
        if self.update_lock.locked():
            messagebox.showwarning("升级进行中", "当前已有升级任务，请等待结束。")
            return
        if self.capture_state != "idle" or self.log_lock.locked():
            messagebox.showwarning("采集进行中", "请先结束原始 IMU 记录和下载，再进行 OTA。")
            return
        threading.Thread(target=self._update_worker, args=(path,), daemon=True).start()

    def _update_worker(self, path: Path) -> None:
        """执行 App 唤醒、Boot 续传、安装及新 App 身份确认全过程。"""
        if not self.update_lock.acquire(blocking=False):
            return
        try:
            image = path.read_bytes()
            validate_app_image(image)
            image_crc = image_crc32(image)
            # 同一 BIN 的版本字段固定，断线后再次点击升级才能命中 Flash 检查点。
            version = image_crc
            begin = build_update_begin(len(image), image_crc, version)
            self._set_update_status("正在请求目标板进入 Bootloader", 0.0)
            # App 的过渡 ACK 只表示“即将复位”，不是可写入确认。复位期间
            # 每 800 ms 重发 BEGIN，Bootloader 在 3 秒窗口内收到任意一次即可应答。
            ack = None
            for attempt in range(12):
                self.serial_port.write(begin)
                try:
                    candidate = self._wait_ack(0x20, 0.8)
                except TimeoutError:
                    self._set_update_status(f"等待 Bootloader 响应（{attempt + 1}/12）", 0.0)
                    continue
                if candidate[1] == 0xFE:
                    self._set_update_status("目标板正在重启到 Bootloader", 0.0)
                    time.sleep(0.25)
                    continue
                ack = candidate
                break
            if ack is None:
                raise TimeoutError("Bootloader 未在重试窗口内应答 BEGIN")
            if ack[1] != 0:
                raise RuntimeError(f"Bootloader 拒绝升级，状态码 {ack[1]}")
            # BEGIN ACK 的 value 是 Bootloader 最近一次持久化的 next_chunk。
            # 从该块继续发送，前一个检查点之后的少量块会被安全重传。
            start_chunk = ack[2]
            total = (len(image) + UPDATE_CHUNK_SIZE - 1) // UPDATE_CHUNK_SIZE
            self._set_update_status(
                f"已进入 Bootloader，{('从第 ' + str(start_chunk) + ' 块续传') if start_chunk else '从第 0 块开始'}",
                start_chunk * 100.0 / total)
            index = start_chunk
            retries = 0
            ack_mode = None  # 兼容旧 Bootloader 的逐块 ACK 和新版累计 ACK。
            while index < total:
                target = min(total, ((index // UPDATE_WINDOW_SIZE) + 1) * UPDATE_WINDOW_SIZE)
                if ack_mode == "legacy":
                    target = index + 1
                # 首块先单独探测：旧 Bootloader 会立即应答，新 Bootloader
                # 会在第 4 块累计确认，因而无需有线升级也能兼容现有设备。
                first_end = index + 1 if ack_mode is None else target
                for chunk in range(index, first_end):
                    self.serial_port.write(build_update_data(
                        chunk, image[chunk * UPDATE_CHUNK_SIZE:(chunk + 1) * UPDATE_CHUNK_SIZE]))
                    time.sleep(UPDATE_INTERFRAME_DELAY_S)
                if ack_mode is None:
                    try:
                        candidate = self._wait_ack(0x21, 0.8)
                    except TimeoutError:
                        candidate = None
                    if candidate is not None and candidate[1] == 0 and candidate[2] == index + 1:
                        ack_mode = "legacy"
                        index += 1
                        retries = 0
                        self._set_update_status(f"正在发送固件数据 {index}/{total}", index * 100.0 / total)
                        continue
                    ack_mode = "cumulative"
                    for chunk in range(first_end, target):
                        self.serial_port.write(build_update_data(
                            chunk, image[chunk * UPDATE_CHUNK_SIZE:(chunk + 1) * UPDATE_CHUNK_SIZE]))
                        time.sleep(UPDATE_INTERFRAME_DELAY_S)
                deadline = time.monotonic() + 5.0
                progress = None
                while time.monotonic() < deadline:
                    try:
                        candidate = self._wait_ack(0x21, deadline - time.monotonic())
                    except TimeoutError:
                        break
                    if candidate[1] == 0 and candidate[2] == target:
                        progress = target
                        break
                    if candidate[1] == 2 and index <= candidate[2] <= target:
                        progress = candidate[2]
                        break
                    if candidate[1] not in (0, 2):
                        raise RuntimeError(f"数据块写入失败，状态码 {candidate[1]}")
                if progress is None or progress == index:
                    retries += 1
                    if retries >= 3:
                        raise TimeoutError(f"块 {index}～{target - 1} 未获确认")
                    continue
                index = progress
                retries = 0
                self._set_update_status(f"正在发送固件数据 {index}/{total}",
                                        index * 100.0 / total)
            self._set_update_status("数据已发送，正在校验和写入 App", 100.0)
            self.serial_port.write(build_update_end())
            ack = self._wait_ack(0x22, 30.0)
            if ack[1] != 0:
                raise RuntimeError(f"整包校验失败，状态码 {ack[1]}")
            # END ACK 表示 Boot 已完成镜像提交，但复位和新 App 启动可能失败；
            # 因此继续主动查询设备信息，只有版本和 CRC32 都匹配才显示成功。
            self._set_update_status("镜像已提交，等待新 App 重启确认", 100.0)
            confirmed = False
            for attempt in range(12):
                time.sleep(0.4)
                self.serial_port.write(build_device_info())
                try:
                    frame = self._wait_frame(MSG_DEVICE_INFO, 1.3)
                except TimeoutError:
                    continue
                if len(frame.payload) >= 15:
                    actual_version = int.from_bytes(frame.payload[7:11], "little")
                    actual_crc = int.from_bytes(frame.payload[11:15], "little")
                    if actual_version == version and actual_crc == image_crc:
                        confirmed = True
                        break
            if confirmed:
                self._set_update_status("升级成功：新 App 已重启并确认版本/CRC", 100.0)
            else:
                raise TimeoutError("镜像已提交，但未收到匹配版本/CRC 的新 App 确认")
        except Exception as exc:
            self._set_update_status(f"升级失败：{exc}", self.update_progress_var.get())
        finally:
            self.update_lock.release()

    def _set_update_status(self, message: str, progress: float | None = None) -> None:
        """跨线程提交界面文字与百分比，防止工作线程直接操作 Tk 控件。"""
        self.after(0, self.update_var.set, message)
        if progress is not None:
            self.after(0, self.update_progress_var.set, max(0.0, min(100.0, progress)))


if __name__ == "__main__":
    GroundStation().mainloop()
