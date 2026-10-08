"""
RoboMaster Dart OpenMV vision front end, protocol V5 (2027 project).

Target platform: OpenMV Cam H7 Plus / color camera (RGB565, e.g. OV5640).

Design goals:
  - Stable green LED blob detection for guided dart experiments.
  - QVGA RGB565, fixed ISP settings, target FPS >= 50 Hz.
  - ROI tracking with periodic full-frame recovery.
  - Multi-candidate scoring: color blob quality + roundness/ratio + history.
  - Raw frame detection, jump rejection, and ROI history for reacquisition.
  - 24-byte big-endian UART frame at 921600 bps, CRC16-CCITT.

UART frame, 24 bytes:
  0..1  SOF 0xAA, 0x55
  2     protocol version 0x05
  3     flags: bit7=fresh valid, bit6=reserved (always zero)
  4     frame id
  5     confidence, 0..100
  6..7  current-frame target cx int16
  8..9  current-frame target cy int16
  10..11 target vx in px/s int16
  12..13 target vy in px/s int16
  14..15 radius px uint16
  16..17 area px uint16
  18..19 calibrated distance dm uint16; zero when uncalibrated
  20..21 capture timestamp low uint16 ms
  22..23 CRC16-CCITT over bytes 0..21
"""

import sensor
import image
import time
import math
import gc

try:
    import uos
except ImportError:
    uos = None

try:
    import mjpeg
    HAS_MJPEG = True
except ImportError:
    HAS_MJPEG = False

from pyb import UART, LED

try:
    from pyb import SD
    HAS_SD_CLASS = True
except ImportError:
    HAS_SD_CLASS = False


# ============================================================
# Configuration
# ============================================================

SENSOR_WIDTH = 320
SENSOR_HEIGHT = 240
CENTER_X = SENSOR_WIDTH // 2
CENTER_Y = SENSOR_HEIGHT // 2
FPS_TARGET = 60

UART_PORT = 3
UART_BAUD = 921600

EXPOSURE_US = 2500
GAIN_DB = 10

# LAB thresholds for green LED in RGB565 mode.
# Tune these in OpenMV IDE Threshold Editor under the real field lighting.
GREEN_THRESHOLD_STRICT = (27, 100, -84, 8, -51, 68)    #(28, 100, -89, 7, -19, 96)
GREEN_THRESHOLD_WIDE = (25, 100, -85, -15, -20, 90)
USE_WIDE_GREEN_FALLBACK = False
GREEN_THRESHOLDS = [GREEN_THRESHOLD_STRICT]
if USE_WIDE_GREEN_FALLBACK:
    GREEN_THRESHOLDS.append(GREEN_THRESHOLD_WIDE)

BLOB_PIXELS_MIN = 8
BLOB_AREA_MIN = 12
BLOB_AREA_MAX = 50000
BLOB_MERGE = True
BLOB_MARGIN = 2

# CONF_ACCEPT 是唯一的发送门, 必须 <= STM32 的 DART_VISION_CONF_HOLD,
# 否则 STM32 侧迟滞的低门限永远收不到对应的帧, 形同虚设。
# 语义分工: 本门限负责"是否继续跟踪", STM32 的 CONF_ACQUIRE 负责"是否进入制导"。
CONF_ACCEPT = 52

# CONF_STRONG / CONF_REACQUIRE 也尚未按 12.5 m 实测标定。
#
# 失效模式是**静默的**: 若目标在 12.5 m 处的置信度**始终到不了 68/72**,
# 那么依赖它们的逻辑永不触发, 而且不会报错:
#   CONF_STRONG -> dynamic_gate() 不放宽 (门控偏紧)
#               -> RECORD_MODE="auto" 永不启动 (现已改 "always", 不受影响)
#   CONF_REACQUIRE -> 跳变门控与重捕获判定始终走严格分支
#
# 这三个门限必须满足 CONF_ACCEPT <= CONF_REACQUIRE < CONF_STRONG。
# 定值方法: 跑一次台架剖面, 读遥测 CF 字段 (本次飞行观测到的置信度峰值),
# 再取 CONF_STRONG ≈ 0.85 × CF、CONF_REACQUIRE ≈ 0.75 × CF。
# 若 CF 本身低于 CONF_ACCEPT + 8, 说明 12.5 m 处目标根本进不了稳定跟踪,
# 那是采集参数 (GREEN_THRESHOLD_*/EXPOSURE_US) 的问题, 不要靠降门限掩盖。
CONF_STRONG = 72
CONF_REACQUIRE = 68
LOCK_CONFIRM_FRAMES = 2

MIN_BLOB_DENSITY = 0.24
MIN_ROUNDNESS_ACCEPT = 0.42
MIN_RATIO_ACCEPT = 0.58
MAX_RATIO_ACCEPT = 1.72
MIN_CENTER_L = 32
MAX_CENTER_A = -18
MIN_CENTER_B = -25
MAX_CENTER_B = 95

ROI_HALF_TRACK = 60
ROI_FULL_SCAN_EVERY = 40

JUMP_BASE_PX = 30.0
JUMP_VEL_SCALE = 1.4
JUMP_CONF_RELAX = 18.0

LOST_TIMEOUT_MS = 250
# STM32 负责唯一的视线滤波和延迟补偿，视觉端不做位置超前预测。

# Distance model. Replace with a six-point least-squares calibration:
#   Z_mm = DIST_A / radius_px + DIST_B
DISTANCE_CALIBRATED = True
DIST_A = 26000.0
DIST_B = 0.0
DIST_MIN_DM = 2
DIST_MAX_DM = 250

# 距离不可用时的输出值。协议 (vision_protocol.h) 已约定 0 = "未标定或不可用",
# STM32 的 estimate_tgo() 会据此走它自己的 fallback / 光学分支。
#
# 这里**不能**填一个"看起来合理"的默认距离 (旧值 90 dm = 9 m)。填了真实数值,
# STM32 的第一分支 (distance_dm > 0) 就会把占位当测量采信。12.5 m 处目标半径
# 在 1/2 px 之间抖动, r=1 的帧会发出 9 m, 使 t_go 在 0.54/0.78 s 之间以帧率
# 翻转 —— 恰好发生在捕获瞬间, 直接污染 NEAR_TGO_S/FAR_TGO_S 的标定。
DIST_UNAVAILABLE_DM = 0

# 距离一阶低通时间常数 (s)。滤波作用在 radius_px 上, 不是 dist_mm —— 见
# estimate_distance_dm() 的说明。12.5 m 处 r 只有 2 px, 半径的整数量化和
# 阈值抖动经 Z=A/r 会被放大成数米的距离阶跃。
#
# 用时间常数而不是固定每帧系数: 系数按 alpha = 1 - exp(-dt/tau) 逐帧换算,
# 帧率变化时滤波带宽不变。本值即 30 fps 下的等效 tau。
#
# 代价: 这个 tau 就是滞后本身 —— 使 t_go 系统性偏高同样时长, 与制导窗口
# 0.75 s 同量级。标定 DART_GUIDANCE_NEAR_TGO_S / FAR_TGO_S 时要把这段
# 滞后算进去 (它是常数偏置, 会被回填一并吸收)。
DIST_LPF_TAU_S = 0.077

# 默认开启画面叠加，便于在 OpenMV IDE 中观察目标框、中心偏差和锁定状态。
OSD_ENABLED = True
DEBUG_PRINT = False
DEBUG_EVERY_N = 30

# Recording. Options: "off", "auto", "always".
#
# 设为 "always": 捕获时刻的诊断需要**捕获之前**的画面 —— 目标在
# 12.5 m 处才第一次出现, 而 "auto" 要等 conf >= CONF_STRONG 才开始录,
# 那正好错过最该看的那几帧。台架剖面测试期间保持 "always"。
#
# ⚠ 代价是帧率。MJPEG 编码是 CPU 负载, 而制导窗口只有 0.75 s,
#   帧率从 30 掉到 20 就少了三分之一的制导帧。每次改完这个开关,
#   都必须确认实际帧率 —— 遥测已不含帧率字段 (F 已移除), 只能看这两处:
#     - OSD 左上角的 "FPS:xx"
#     - IDE 串口 print 的 "fps:xx.x"
#   低于 25 fps 就把 RECORD_FPS 降到 10 再试, 仍然低就说明本平台带不动,
#   只能关录像。
RECORD_MODE = "off"
RECORD_FPS = 15
RECORD_QUALITY = 50
RECORD_SEGMENT_SECONDS = 60
RECORD_SYNC_EVERY = 90

FPS_EMA_ALPHA = 0.15


# ============================================================
# Small utilities
# ============================================================

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def lerp(a, b, t):
    if t <= 0.0:
        return a
    if t >= 1.0:
        return b
    return a + (b - a) * t


# def crc8_calc(data):
#     crc = 0x00
#     for byte in data:
#         crc ^= byte
#         for _ in range(8):
#             if crc & 0x80:
#                 crc = ((crc << 1) ^ 0x07) & 0xFF
#             else:
#                 crc = (crc << 1) & 0xFF
#     return crc


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def put_u16_be(buf, idx, value):
    value = int(clamp(value, 0, 65535))
    buf[idx] = (value >> 8) & 0xFF
    buf[idx + 1] = value & 0xFF


def put_i16_be(buf, idx, value):
    value = int(clamp(value, -32768, 32767))
    if value < 0:
        value += 65536
    buf[idx] = (value >> 8) & 0xFF
    buf[idx + 1] = value & 0xFF


def safe_roundness(blob):
    try:
        return blob.roundness()
    except Exception:
        ratio = blob.w() / float(blob.h()) if blob.h() > 0 else 99.0
        if ratio < 1.0:
            ratio = 1.0 / max(ratio, 0.001)
        return clamp(1.0 / ratio, 0.0, 1.0)


def center_color_ok(img, blob):
    # Verify the blob center is still bright green, not just a loose merged ROI.
    cx = blob.cx()
    cy = blob.cy()
    side = int(clamp(min(blob.w(), blob.h()) // 2, 3, 12))
    x = int(clamp(cx - side // 2, 0, SENSOR_WIDTH - 1))
    y = int(clamp(cy - side // 2, 0, SENSOR_HEIGHT - 1))
    w = int(clamp(side, 1, SENSOR_WIDTH - x))
    h = int(clamp(side, 1, SENSOR_HEIGHT - y))
    try:
        st = img.get_statistics(roi=(x, y, w, h))
        l_mean = st.l_mean()
        a_mean = st.a_mean()
        b_mean = st.b_mean()
        return (l_mean >= MIN_CENTER_L and
                a_mean <= MAX_CENTER_A and
                b_mean >= MIN_CENTER_B and
                b_mean <= MAX_CENTER_B)
    except Exception:
        return True


def blob_radius(blob):
    box_r = 0.25 * (blob.w() + blob.h())
    area_r = math.sqrt(max(1.0, float(blob.area())) / 3.14159)
    return int(clamp(0.55 * box_r + 0.45 * area_r, 1.0, 65535.0))


# 距离低通的滤波状态。目标丢失时必须复位, 否则旧半径会拖进下一次锁定。
dist_r_filt = 0.0
dist_r_filt_ready = False


def reset_distance_filter():
    global dist_r_filt, dist_r_filt_ready
    dist_r_filt = 0.0
    dist_r_filt_ready = False


def estimate_distance_dm(radius_px, dt_s):
    """带一阶低通的距离反演。

    低通作用在半径域而非距离域, 有两个原因:
      1. 半径是整数。在 12.5 m 处 r = 2, 1 px 的量化跳变对应
         dZ/dr = -A/r^2 ≈ -6.5 m/px。先在半径域平均掉量化阶跃, 再过 1/r,
         避免把阶跃放大成数米的距离跳变。
      2. 帧间平均整数半径可以恢复亚像素半径, 而 1/r 是凸函数 ——
         对距离取均值会系统性高估 (Jensen), 对半径取均值不会。

    radius_px < 2 的帧返回 0 且**不更新滤波器**: r=1 按模型对应 26 m, 说明
    blob 被阈值截断了, 不是真的在 26 m 处。把这种截断值喂进滤波器会把半径
    拉低、距离拉高。代价是 12.5 m 处只有一半的帧参与平均, 有效时间常数翻倍。
    """
    global dist_r_filt, dist_r_filt_ready
    if not DISTANCE_CALIBRATED:
        return DIST_UNAVAILABLE_DM
    if radius_px < 2:
        return DIST_UNAVAILABLE_DM
    if not dist_r_filt_ready:
        # 首帧直接用原始半径, 避免从 0 缓慢爬升
        dist_r_filt = float(radius_px)
        dist_r_filt_ready = True
    else:
        alpha = 1.0 - math.exp(-clamp(dt_s, 0.005, 0.200) / DIST_LPF_TAU_S)
        dist_r_filt += alpha * (float(radius_px) - dist_r_filt)
    if dist_r_filt < 2.0:
        return DIST_UNAVAILABLE_DM
    dist_mm = DIST_A / dist_r_filt + DIST_B
    dist_dm = int(dist_mm / 100.0 + 0.5)
    return int(clamp(dist_dm, DIST_MIN_DM, DIST_MAX_DM))


def roi_around(cx, cy, half):
    x = int(cx - half)
    y = int(cy - half)
    x = int(clamp(x, 0, SENSOR_WIDTH - 1))
    y = int(clamp(y, 0, SENSOR_HEIGHT - 1))
    w = int(clamp(half * 2, 1, SENSOR_WIDTH - x))
    h = int(clamp(half * 2, 1, SENSOR_HEIGHT - y))
    return (x, y, w, h)


# ============================================================
# Camera and UART
# ============================================================

def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    try:
        sensor.set_framerate(FPS_TARGET)
    except Exception:
        pass
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.set_auto_whitebal(False)
    sensor.skip_frames(30)


def init_uart():
    return UART(UART_PORT, UART_BAUD, timeout_char=10)


# ============================================================
# Candidate scoring
# ============================================================

def base_confidence(img, blob):
    area = blob.area()
    w = blob.w()
    h = blob.h()
    rect_area = w * h
    if rect_area <= 0 or area < BLOB_AREA_MIN or area > BLOB_AREA_MAX:
        return 0.0

    density = area / float(rect_area)
    ratio = w / float(h) if h > 0 else 99.0
    if density < MIN_BLOB_DENSITY:
        return 0.0
    if ratio < MIN_RATIO_ACCEPT or ratio > MAX_RATIO_ACCEPT:
        return 0.0
    if not center_color_ok(img, blob):
        return 0.0

    if ratio < 1.0:
        inv_ratio = 1.0 / max(ratio, 0.001)
    else:
        inv_ratio = ratio

    small = area < 35
    density_floor = 0.42 if small else 0.62
    round_floor = 0.48 if small else 0.62

    score_density = 30.0 * clamp(density / density_floor, 0.0, 1.0)

    if 20 <= area <= 5000:
        score_area = 25.0
    elif area < 20:
        score_area = 25.0 * clamp((area - BLOB_AREA_MIN) / 15.0, 0.0, 1.0)
    else:
        score_area = 25.0 * clamp(1.0 - (area - 5000) / 3000.0, 0.0, 1.0)

    if 0.70 <= ratio <= 1.40:
        score_ratio = 20.0
    elif 0.50 <= ratio <= 2.00:
        score_ratio = 10.0
    else:
        score_ratio = 0.0

    roundness = safe_roundness(blob)
    if roundness < MIN_ROUNDNESS_ACCEPT:
        return 0.0
    score_round = 25.0 * clamp(roundness / round_floor, 0.0, 1.0)

    if inv_ratio > 2.3 or density < 0.12:
        return 0.0

    return clamp(score_density + score_area + score_ratio + score_round, 0.0, 100.0)


def choose_best_blob(img, blobs, tracker):
    best_blob = None
    best_conf = 0.0
    best_score = -999.0

    for blob in blobs:
        conf = base_confidence(img, blob)
        if conf <= 0.0:
            continue

        score = conf
        if tracker.has_history():
            dx = blob.cx() - tracker.x
            dy = blob.cy() - tracker.y
            d = math.sqrt(dx * dx + dy * dy)
            gate = tracker.dynamic_gate(conf)
            if d <= gate:
                score += 22.0 * (1.0 - d / max(gate, 1.0))
            elif conf < CONF_REACQUIRE:
                score -= 35.0
            else:
                score -= 12.0
        else:
            dx = blob.cx() - CENTER_X
            dy = blob.cy() - CENTER_Y
            center_dist = math.sqrt(dx * dx + dy * dy)
            score += 8.0 * (1.0 - clamp(center_dist / 200.0, 0.0, 1.0))

        if score > best_score:
            best_score = score
            best_blob = blob
            best_conf = conf

    return best_blob, int(clamp(best_conf + 0.5, 0, 100))


# ============================================================
# 原始测量跟踪器
# ============================================================

class TargetTracker:
    def __init__(self):
        self.x = float(CENTER_X)
        self.y = float(CENTER_Y)
        self.r = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.valid = False
        self.lost_ms = LOST_TIMEOUT_MS
        self.last_fresh_ms = 0
        self.last_area = 0
        self.last_conf = 0
        self.pending_x = 0.0
        self.pending_y = 0.0
        self.pending_count = 0

    def has_history(self):
        return self.valid or self.lost_ms < LOST_TIMEOUT_MS

    def speed(self):
        return math.sqrt(self.vx * self.vx + self.vy * self.vy)

    def dynamic_gate(self, conf):
        gate = JUMP_BASE_PX + self.speed() * JUMP_VEL_SCALE
        if conf >= CONF_STRONG:
            gate += JUMP_CONF_RELAX
        return clamp(gate, 24.0, 60.0)

    def update(self, blob, conf, dt_ms, now_ms):
        if blob is None or conf < CONF_ACCEPT:
            return self.mark_missed(dt_ms)

        mx = float(blob.cx())
        my = float(blob.cy())
        mr = float(blob_radius(blob))

        if not self.has_history():
            if self.pending_count > 0:
                pdx = mx - self.pending_x
                pdy = my - self.pending_y
                if math.sqrt(pdx * pdx + pdy * pdy) <= 18.0:
                    self.pending_count += 1
                else:
                    self.pending_count = 1
            else:
                self.pending_count = 1
            self.pending_x = mx
            self.pending_y = my

            if self.pending_count < LOCK_CONFIRM_FRAMES:
                self.x = mx
                self.y = my
                self.r = mr
                self.last_area = blob.area()
                self.last_conf = conf
                return False

            self.x = mx
            self.y = my
            self.r = mr
            self.vx = 0.0
            self.vy = 0.0
            self.valid = True
            self.lost_ms = 0
            self.last_fresh_ms = now_ms
            self.last_area = blob.area()
            self.last_conf = conf
            self.pending_count = 0
            return True

        residual_x = mx - self.x
        residual_y = my - self.y
        residual = math.sqrt(residual_x * residual_x + residual_y * residual_y)
        gate = self.dynamic_gate(conf)

        if residual > gate and conf < CONF_REACQUIRE:
            return self.mark_missed(dt_ms)

        # 只输出当前帧的原始测量；像素速度是相邻有效帧的差分观测量。
        sample_dt_ms = max(dt_ms, self.lost_ms + dt_ms)
        dt_s = clamp(sample_dt_ms / 1000.0, 0.001, 0.250)
        self.vx = clamp(residual_x / dt_s, -32768.0, 32767.0)
        self.vy = clamp(residual_y / dt_s, -32768.0, 32767.0)
        self.x = mx
        self.y = my
        self.r = mr

        self.x = clamp(self.x, 0.0, SENSOR_WIDTH - 1.0)
        self.y = clamp(self.y, 0.0, SENSOR_HEIGHT - 1.0)
        self.valid = True
        self.lost_ms = 0
        self.last_fresh_ms = now_ms
        self.last_area = blob.area()
        self.last_conf = conf
        return True

    def mark_missed(self, dt_ms):
        self.lost_ms += dt_ms
        self.valid = False
        if self.lost_ms > LOST_TIMEOUT_MS:
            self.vx = 0.0
            self.vy = 0.0
            self.pending_count = 0
        return False

    def output_xy(self):
        return int(clamp(self.x + 0.5, 0, SENSOR_WIDTH - 1)), int(clamp(self.y + 0.5, 0, SENSOR_HEIGHT - 1))


# ============================================================
# Optional MJPEG recorder
# ============================================================

class VideoRecorder:
    def __init__(self, mode):
        self.mode = mode
        self.enabled = mode != "off" and HAS_MJPEG and uos is not None
        self.root = None
        self.dir = None
        self.writer = None
        self.recording = False
        self.frame_index = 0
        self.file_index = 0
        self.segment_start = 0
        self.faulted = False

        if not self.enabled:
            return

        self.root = self.find_sd_root()
        if self.root is None:
            self.enabled = False
            return

        self.dir = self.join(self.root, "REC") if self.root != "/" else "/REC"
        try:
            uos.mkdir(self.dir)
        except OSError:
            pass

    def find_sd_root(self):
        for path in ("/sdcard", "/sd"):
            try:
                if (uos.stat(path)[0] & 0x4000) != 0:
                    return path
            except OSError:
                pass
        try:
            if HAS_SD_CLASS:
                sd = SD()
                sd.init()
                if (uos.stat("/sd")[0] & 0x4000) != 0:
                    return "/sd"
        except Exception:
            pass
        return None

    def join(self, a, b):
        if a.endswith("/"):
            a = a[:-1]
        return a + "/" + b

    def next_path(self):
        if self.file_index == 0:
            try:
                for fn in uos.listdir(self.dir):
                    if fn.startswith("DART_") and fn.endswith(".mjpeg"):
                        try:
                            n = int(fn[5:9])
                            if n > self.file_index:
                                self.file_index = n
                        except Exception:
                            pass
            except Exception:
                pass
        self.file_index += 1
        return self.join(self.dir, "DART_%04d.mjpeg" % self.file_index)

    def update(self, fresh_valid, conf):
        if not self.enabled or self.faulted:
            return
        if self.mode == "always":
            if not self.recording:
                self.start()
        elif self.mode == "auto":
            if fresh_valid and conf >= CONF_STRONG:
                if not self.recording:
                    self.start()
            elif self.recording and conf == 0:
                # Keep the clip open; rotate/stop only on segment end.
                pass
        self.rotate_if_needed()

    def start(self):
        if not self.enabled or self.recording:
            return
        try:
            self.writer = mjpeg.Mjpeg(self.next_path())
            self.recording = True
            self.frame_index = 0
            self.segment_start = time.ticks_ms()
        except Exception:
            self.stop()
            self.faulted = True

    def add_frame(self, img):
        if not self.recording or self.writer is None:
            return
        interval = max(1, int(FPS_TARGET / max(1, RECORD_FPS)))
        self.frame_index += 1
        if self.frame_index % interval != 0:
            return
        try:
            self.writer.add_frame(img, quality=RECORD_QUALITY)
            if RECORD_SYNC_EVERY > 0 and self.frame_index % RECORD_SYNC_EVERY == 0:
                try:
                    self.writer.sync()
                except Exception:
                    pass
        except Exception:
            self.stop()
            self.faulted = True

    def rotate_if_needed(self):
        if not self.recording or RECORD_SEGMENT_SECONDS <= 0:
            return
        if time.ticks_diff(time.ticks_ms(), self.segment_start) >= RECORD_SEGMENT_SECONDS * 1000:
            self.stop()
            self.start()

    def stop(self):
        try:
            if self.writer is not None:
                self.writer.close()
        except Exception:
            pass
        self.writer = None
        self.recording = False
        gc.collect()


# ============================================================
# OSD and protocol
# ============================================================

class OSD:
    def __init__(self):
        self.enabled = OSD_ENABLED
        self.white = (255, 255, 255)
        self.cyan = (0, 255, 255)
        self.green = (0, 255, 0)
        self.yellow = (255, 255, 0)
        self.red = (255, 0, 0)
        self.magenta = (255, 0, 255)

    def draw(self, img, fps, tracker, fresh_valid, conf, radius, area, dist_dm, recording):
        if not self.enabled:
            return

        cx, cy = tracker.output_xy()
        img.draw_line(CENTER_X - 10, CENTER_Y, CENTER_X + 10, CENTER_Y, color=self.cyan)
        img.draw_line(CENTER_X, CENTER_Y - 10, CENTER_X, CENTER_Y + 10, color=self.cyan)

        if fresh_valid:
            color = self.green
            r = max(4, int(radius))
            img.draw_rectangle(cx - r, cy - r, r * 2, r * 2, color=color)
            img.draw_cross(cx, cy, color=color, size=5)
            img.draw_line(CENTER_X, CENTER_Y, cx, cy, color=self.yellow)
            state = "TRACK"
        else:
            state = "LOST"

        dx = cx - CENTER_X
        dy = cy - CENTER_Y
        rec = "REC" if recording else "   "
        img.draw_string(2, 2, "FPS:%3.0f %s %s C:%3d" % (fps, rec, state, conf), color=self.white)
        img.draw_string(2, 14, "dX:%+4d dY:%+4d R:%3d A:%5d" % (dx, dy, radius, area), color=self.white)
        dist_text = ("%4.1fm" % (dist_dm / 10.0)) if dist_dm > 0 else "  --  "
        img.draw_string(2, 26, "D:%s V:%4.1f Exp:%dus" % (dist_text, tracker.speed(), EXPOSURE_US), color=self.white)
        if recording:
            img.draw_circle(SENSOR_WIDTH - 10, 8, 3, color=self.red, fill=True)


def pack_frame(frame_id, cx, cy, vx_px_s, vy_px_s, radius, area,
               confidence, dist_dm, fresh_valid, timestamp_ms):
    payload = bytearray(24)
    payload[0] = 0xAA
    payload[1] = 0x55
    payload[2] = 0x05
    flags = 0x80 if fresh_valid else 0x00
    payload[3] = flags
    payload[4] = frame_id & 0xFF
    payload[5] = int(clamp(confidence, 0, 100)) & 0xFF
    put_i16_be(payload, 6, cx)
    put_i16_be(payload, 8, cy)
    put_i16_be(payload, 10, int(tracker_velocity_limit(vx_px_s)))
    put_i16_be(payload, 12, int(tracker_velocity_limit(vy_px_s)))
    put_u16_be(payload, 14, radius)
    put_u16_be(payload, 16, area)
    put_u16_be(payload, 18, dist_dm)
    put_u16_be(payload, 20, timestamp_ms & 0xFFFF)
    crc = crc16_ccitt(payload[:22])
    put_u16_be(payload, 22, crc)
    return payload


def tracker_velocity_limit(value):
    return clamp(value, -32768, 32767)


# ============================================================
# Main loop
# ============================================================

def main():
    print("Dart Vision Guidance 2027 / protocol V5")
    print("UART %d bps, QVGA RGB565, CRC16 protocol" % UART_BAUD)

    init_sensor()
    uart = init_uart()
    tracker = TargetTracker()
    osd = OSD()
    recorder = VideoRecorder(RECORD_MODE)

    led_power = LED(1)
    led_lock = LED(2)
    led_lost = LED(3)
    led_power.on()
    led_lock.off()
    led_lost.off()

    frame_id = 0
    fps_ema = float(FPS_TARGET)
    last_tick = time.ticks_ms()

    try:
        while True:
            now = time.ticks_ms()
            dt_ms = time.ticks_diff(now, last_tick)
            last_tick = now
            if dt_ms <= 0:
                dt_ms = int(1000 / FPS_TARGET)
            instant_fps = 1000.0 / float(dt_ms)
            fps_ema = (1.0 - FPS_EMA_ALPHA) * fps_ema + FPS_EMA_ALPHA * instant_fps

            img = sensor.snapshot()
            capture_ms = time.ticks_ms()
            use_full = (frame_id % ROI_FULL_SCAN_EVERY == 0) or (not tracker.has_history())
            roi = None
            if not use_full:
                half = ROI_HALF_TRACK
                cx_roi, cy_roi = tracker.output_xy()
                roi = roi_around(cx_roi, cy_roi, half)

            if roi is None:
                blobs = img.find_blobs(GREEN_THRESHOLDS,
                                       pixels_threshold=BLOB_PIXELS_MIN,
                                       area_threshold=BLOB_AREA_MIN,
                                       merge=BLOB_MERGE,
                                       margin=BLOB_MARGIN)
            else:
                blobs = img.find_blobs(GREEN_THRESHOLDS,
                                       roi=roi,
                                       pixels_threshold=BLOB_PIXELS_MIN,
                                       area_threshold=BLOB_AREA_MIN,
                                       merge=BLOB_MERGE,
                                       margin=BLOB_MARGIN)

            best_blob, conf = choose_best_blob(img, blobs, tracker) if blobs else (None, 0)
            fresh_valid = tracker.update(best_blob, conf, dt_ms, now)

            cx_out, cy_out = tracker.output_xy()
            radius_out = int(clamp(tracker.r + 0.5, 0, 65535)) if fresh_valid else 0
            area_out = tracker.last_area if fresh_valid else 0
            conf_out = tracker.last_conf if fresh_valid else 0
            # 只在真丢目标时复位距离低通。r < 2 不算丢失 —— 那只是目标太小、
            # 量不出尺寸; 在这里复位会让滤波器在整个 12.5 m 捕获段反复清零,
            # 恰好在该生效的距离永远攒不起平均。
            if not fresh_valid:
                reset_distance_filter()
            dist_out = (estimate_distance_dm(radius_out, dt_ms * 0.001)
                        if fresh_valid else DIST_UNAVAILABLE_DM)

            osd.draw(img, fps_ema, tracker, fresh_valid, conf_out,
                     radius_out, area_out, dist_out, recorder.recording)

            packet = pack_frame(frame_id, cx_out, cy_out,
                                tracker.vx, tracker.vy,
                                radius_out, area_out, conf_out, dist_out,
                                fresh_valid, capture_ms)
            uart.write(packet)

            recorder.update(fresh_valid, conf_out)
            recorder.add_frame(img)

            if fresh_valid:
                led_lock.on()
                led_lost.off()
            else:
                led_lock.off()
                led_lost.on()

            if DEBUG_PRINT and frame_id % DEBUG_EVERY_N == 0:
                print("fps:%4.1f valid:%d cx:%3d cy:%3d r:%3d conf:%3d dist:%3d blobs:%d" %
                      (fps_ema, 1 if fresh_valid else 0,
                       cx_out, cy_out, radius_out, conf_out, dist_out,
                       len(blobs) if blobs else 0))

            frame_id = (frame_id + 1) & 0xFF
    finally:
        recorder.stop()
        led_lock.off()
        led_lost.off()


if __name__ == "__main__":
    main()
