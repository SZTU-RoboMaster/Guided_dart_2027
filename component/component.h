/**
 * @file    component.h
 * @brief   制导飞镖飞控组件 — 顶层头文件
 *
 *          当前模式: IMU 姿态自稳 (纯惯性, 无视觉依赖)
 *
 *          模块清单:
 *            ahrs              — Mahony AHRS 姿态估计 (互补滤波)
 *            imu               — IMU SPI 驱动 (ICM-20600/42688)
 *            servo             — 舵机 PWM 驱动 (TIM3_CH1/2 + TIM4_CH1/2)
 *            rate_control      — 角速率内环 PD 控制器
 *            control_allocation— 加权伪逆控制分配 (X翼面)
 *            flight_state      — 飞行状态机 (发射检测 → 自稳 → 结束)
 *            dart_control      — 主控制循环 (200Hz, 全部集成)
 *
 *          可选扩展 (未在当前控制链路中使用):
 *            ab_filter         — STM32 侧视觉 α-β 滤波器
 *            guidance          — 视线角制导律 (视觉依赖)
 *
 *          IOC 引脚映射:
 *            舵机 1：TIM3_CH1 → PB4 (AF2)
 *            舵机 2：TIM3_CH2 → PB5 (AF2)
 *            舵机 3：TIM4_CH1 → PB6 (AF2)
 *            舵机 4：TIM4_CH2 → PB7 (AF2)
 *            IMU SPI: PA5(SCK), PA6(MISO), PA7(MOSI), PA4(CS)
 *            LED/状态：PA1（GPIO 输出）
 *
 *          PWM 参数 (来自 IOC):
 *            频率：200Hz（预分频器=169，周期=4999，时钟=170MHz）
 *            中位: 1500μs, 范围: 500~2500μs
 *
 *          使用方法:
 *
 *            // main.c
 *            #include "component.h"
 *
 *            int main(void) {
 *                HAL_Init();
 *                SystemClock_Config();
 *                MX_GPIO_Init();
 *                MX_DMA_Init();
 *                MX_SPI1_Init();
 *                MX_TIM2_Init();
 *                MX_TIM3_Init();
 *                MX_TIM4_Init();
 *                // ... 其余 CubeMX 初始化 ...
 *
 *                dart_control_init();  // IMU校准 + 启动200Hz控制
 *
 *                while (1) {
 *                    // 主循环空闲 — 所有控制在 TIM2 中断中执行
 *                }
 *            }
 *
 *            // main.c — TIM2 周期回调 (200Hz)
 *            void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *                if (htim->Instance == TIM2) {
 *                    dart_control_tick();
 *                }
 *            }
 *
 *          控制链路:
 *            IMU(200Hz) → AHRS（滚转/俯仰+p/q/r）
 *              → 姿态外环(角度→角速率指令)
 *              → 角速率内环PD(误差→力矩)
 *              → 控制分配(力矩→舵面δ[4])
 *              → 舵机PWM(δ→500-2500μs)
 */

#ifndef COMPONENT_H__
#define COMPONENT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ─── 基础模块 ─── */
#include "ahrs.h"
#include "ab_filter.h"       /* STM32 侧视觉 α-β 滤波 */

/* ─── 硬件驱动 ─── */
#include "imu.h"
#include "servo.h"

/* ─── 控制算法 ─── */
#include "guidance.h"        /* 可选: 制导律, 后续视觉方案用 */
#include "ballistic_estimator.h"
#include "rate_control.h"
#include "control_allocation.h"

/* ─── 系统集成 ─── */
#include "dart_data.h"
#include "vision_link.h"
#include "flight_state.h"
#include "dart_control.h"
#include "lora_telemetry.h"

/* ─── 版本 ─── */
#define COMPONENT_VERSION_MAJOR  2
#define COMPONENT_VERSION_MINOR  0
#define COMPONENT_VERSION_PATCH  0
#define COMPONENT_VERSION_STRING "2.0.0-imu-only"

#ifdef __cplusplus
}
#endif

#endif /* COMPONENT_H__ */
