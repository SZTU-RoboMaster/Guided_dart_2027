/**
 * @file    lora_telemetry.c
 * @brief   LoRa 串口遥测实现 — USART1 DMA 发送，约 20Hz
 */

#include "lora_telemetry.h"
#include "dart_data.h"       /* dart_data_get() → phase, impact_detected */
#include "vision_link.h"     /* vision_link_get_latest() → 视觉 cx/cy */
#include "servo.h"           /* servo_get_pulse_us() → 四路舵机脉宽 */
#include "usart.h"           /* huart1 */
#include "main.h"            /* HAL_GetTick */
#include <stdio.h>           /* snprintf */

/* ── 发送间隔 ── */
#define TX_INTERVAL_MS   50   /* 20Hz */

/* ── DMA 发送缓冲 (静态, 确保 DMA 异步读取期间有效) ── */
static char  g_tx_buf[384];
static uint32_t g_last_tx_tick = 0;

/* ═══════════════════════════════════════════════════════════════
 *  lora_telemetry_send — 每 50ms 发送一帧
 *
 *  数据格式 (一次 snprintf 组帧, 以 \r\n 结尾):
 *    PH<n> CX<x> CY<y> S1+<us> S2+<us> S3+<us> S4+<us>
 *
 *  例:
 *    PH5 CX156 CY112 S1+1500 S2+1500 S3+1543 S4+1462
 *
 *  字段说明:
 *    PH      — 飞行阶段索引 0~8, 与 flight_phase_name() 的数组下标一致:
 *                0 IDLE    1 ARMED    2 LAUNCH    3 STABILIZE  4 ACQUIRE
 *                5 GUIDE   6 COAST    7 FAILSAFE  8 END
 *              取自 dart_data_get() 快照, 与控制环同拍。
 *              ⚠ 只有 PH>=3 才发得出来 —— 见文末说明。
 *    CX/CY   — 视觉最新帧的目标图像坐标 (px), 取自 vision_link_get_latest()。
 *              尚未收到任何有效帧时为 0。
 *    S1..S4  — 四个舵机当前实际输出的脉宽 (μs), 由 servo_get_pulse_us()
 *              回读 TIM3/TIM4 的 CCR 得到。中位 1500, 正常行程 ±500
 *              (即 1000~2000), 硬限幅 SERVO_PULSE_MIN/MAX = 500/2500。
 *
 *              ⚠ 这是**指令**, 不是舵面实际位置。舵机无位置反馈, 卡死、
 *                堵转、掉电在遥测上完全看不出来 —— 只能说明"驱动到线上
 *                的波形是什么"。
 *              ⚠ 报文 20 Hz 而控制环 200 Hz, 每 10 拍只采到 1 拍。判断
 *                "动没动 / 是否回中 / 往哪边饱和" 够用; 想还原内环振荡
 *                波形会混叠, 那得提高发送率或改成机载全速记录。
 *
 *  ⚠ 撞击标志 IM、制导量 (P/Q/R、LY/LP、LYR/LPR、TG、TGO/TA/DF/CF)、视觉
 *    帧率 FPS 和原始 IMU 量都已不在报文里。报文现在只剩飞行阶段、视觉目标
 *    位置 (cx/cy) 和舵机输出。相关变量计算 (如 update_vision_fps() 的帧率
 *    估计、T_go 估计) 均保留, 只是不再发送 —— 需要复核时改回完整格式即可
 *    (旧格式见本文件 send 函数内注释块)。
 *
 *  ⚠ 本函数只在 PHASE_STABILIZE 及之后的阶段被调用。dart_control_process()
 *    在 IDLE/ARMED/LAUNCH 三处提前 return, 那条路径走不到发送语句,
 *    所以地面收不到 PH0/1/2 —— PH2 (LAUNCH) 本身也只有 80ms。
 * ═══════════════════════════════════════════════════════════════ */

void lora_telemetry_send(void)
{
  uint32_t now = HAL_GetTick();

  /* ── 节流: 距上次发送不足 50ms 则跳过 ── */
  if (now - g_last_tx_tick < TX_INTERVAL_MS) {
    return;
  }
  g_last_tx_tick = now;

  /* ── 上次 DMA 未完成则跳过本帧 (丢帧, 不阻塞) ── */
  if (huart1.gState != HAL_UART_STATE_READY) {
    return;
  }

  /* ── 飞行状态快照 (与控制环同拍) ── */
  const dart_data_t *dd = dart_data_get();

  /* ── 视觉最新帧 cx/cy (尚未收到任何有效帧时为 0) ── */
  vision_frame_t vision;
  const bool have_vision = vision_link_get_latest(&vision);
  const int16_t cx = have_vision ? vision.cx : 0;
  const int16_t cy = have_vision ? vision.cy : 0;

  /* ── 四个舵机实际输出脉宽 (μs), 回读 TIM3/TIM4 的 CCR ── */
  uint16_t pulse_us[4];
  servo_get_pulse_us(pulse_us);

  /* ── 格式化: 只发飞行阶段 + 视觉 cx/cy + 四路舵机脉宽 ── */
  int len = snprintf(g_tx_buf, sizeof(g_tx_buf),
                     "PH%d CX%d CY%d S1+%u S2+%u S3+%u S4+%u\r\n",
                     (int)dd->phase,
                     (int)cx, (int)cy,
                     (unsigned)pulse_us[0], (unsigned)pulse_us[1],
                     (unsigned)pulse_us[2], (unsigned)pulse_us[3]);

  /* 旧报文格式保留备查 (相关变量计算过程均保留, 仅发送内容裁剪):
   *   FPS%.2f —— 视觉帧率 (Hz)。vision_fps 字段仍在 dart_control.c 的
   *              update_vision_fps() 中持续计算, 只是不再进遥测报文。
   *   PH/IM/P/Q/R/LY/LP/LYR/LPR/TG/S1~S4 —— 完整飞行遥测, 见下注释块。 */
  /*
  int len = snprintf(g_tx_buf, sizeof(g_tx_buf),
                     "PH%d IM%d P%+.3f Q%+.3f R%+.3f "
                     "LY%+.4f LP%+.4f LYR%+.4f LPR%+.4f TG%.3f "
                     "S1+%u S2+%u S3+%u S4+%u\r\n",
                     (int)dd->phase,
                     dd->impact_detected ? 1 : 0,
                     (double)dd->p, (double)dd->q, (double)dd->r,
                     (double)dd->los_yaw, (double)dd->los_pitch,
                     (double)dd->los_yaw_rate, (double)dd->los_pitch_rate,
                     (double)dd->tgo_s,
                     (unsigned)pulse_us[0], (unsigned)pulse_us[1],
                     (unsigned)pulse_us[2], (unsigned)pulse_us[3]);
  */

  /* ── DMA 发送 (非阻塞) ── */
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)g_tx_buf, len);
}
