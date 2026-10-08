/**
 * @file    servo.c
 * @brief   舵机 PWM 驱动实现
 *
 *          TIM3 (CH1, CH2): PB4, PB5
 *          TIM4 (CH1, CH2): PB6, PB7
 *
 *          脉宽转换: pulse_us = center[index] + direction[index] * delta * span
 *          TIM 比较值: ccr = pulse_us (因为 Prescaler=169 后 1 tick = 1μs)
 *
 *          ⚠️ 依赖 CubeMX 生成的外部变量 htim3、htim4
 */

#include "servo.h"
#include "dart_config.h"
#include "tim.h"       /* 提供 htim3, htim4 (由 CubeMX 生成) */
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════
 *  内部状态
 * ═══════════════════════════════════════════════════════════════ */

static bool     servo_initialized = false;
static float    last_delta[SERVO_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static const uint16_t center_us[SERVO_COUNT] = {
    DART_SERVO_1_CENTER_US, DART_SERVO_2_CENTER_US,
    DART_SERVO_3_CENTER_US, DART_SERVO_4_CENTER_US
};
static const float direction[SERVO_COUNT] = {
    DART_SERVO_1_DIRECTION, DART_SERVO_2_DIRECTION,
    DART_SERVO_3_DIRECTION, DART_SERVO_4_DIRECTION
};

/* ═══════════════════════════════════════════════════════════════
 *  脉宽 ↔ 归一化值 转换
 * ═══════════════════════════════════════════════════════════════ */

static inline uint16_t delta_to_pulse(uint8_t index, float delta)
{
    /* delta ∈ [-1, +1]
     * -1.0 → 1500 - 500 = 1000μs (保守, 不直接到 500)
     *  0.0 → 1500
     * +1.0 → 1500 + 500 = 2000μs (保守)
     *
     * 实际限制: SERVO_PULSE_MIN=500, SERVO_PULSE_MAX=2500
     * 舵机行程:  500μs (对应 ±500 归一化范围)
     * 安全余量:  500-1000 和 2000-2500 保留给饱和情况
     */
    float pulse_f = (float)center_us[index]
                  + direction[index] * delta * DART_SERVO_SPAN_US;

    if (pulse_f < (float)SERVO_PULSE_MIN) pulse_f = (float)SERVO_PULSE_MIN;
    if (pulse_f > (float)SERVO_PULSE_MAX) pulse_f = (float)SERVO_PULSE_MAX;

    return (uint16_t)pulse_f;
}

static inline float pulse_to_delta(uint8_t index, uint16_t pulse)
{
    float d = ((float)(int32_t)pulse - (float)center_us[index])
            / (direction[index] * DART_SERVO_SPAN_US);
    if (d < -1.0f) d = -1.0f;
    if (d >  1.0f) d =  1.0f;
    return d;
}

/* ═══════════════════════════════════════════════════════════════
 *  PWM 输出 — 直接写 CCR 寄存器
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief 写 TIM3 CH1 CCR（PB4 → 舵机 1）
 */
static inline void tim3_ch1_set(uint16_t ccr)
{
    TIM3->CCR1 = (uint32_t)ccr;
}

/**
 * @brief 写 TIM3 CH2 CCR（PB5 → 舵机 2）
 */
static inline void tim3_ch2_set(uint16_t ccr)
{
    TIM3->CCR2 = (uint32_t)ccr;
}

/**
 * @brief 写 TIM4 CH1 CCR（PB6 → 舵机 3）
 */
static inline void tim4_ch1_set(uint16_t ccr)
{
    TIM4->CCR1 = (uint32_t)ccr;
}

/**
 * @brief 写 TIM4 CH2 CCR（PB7 → 舵机 4）
 */
static inline void tim4_ch2_set(uint16_t ccr)
{
    TIM4->CCR2 = (uint32_t)ccr;
}

/* ═══════════════════════════════════════════════════════════════
 *  公共接口
 * ═══════════════════════════════════════════════════════════════ */

void servo_init(void)
{
    /* ── 启动 TIM3 PWM 通道 ── */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* PB4 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* PB5 */

    /* ── 启动 TIM4 PWM 通道 ── */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   /* PB6 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   /* PB7 */

    /* ── 所有舵机回中 ── */
    servo_all_center();

    servo_initialized = true;
}


void servo_set_pulse(uint8_t index, uint16_t pulse)
{
    if (!servo_initialized || index >= SERVO_COUNT) return;

    /* 硬件限幅 */
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;

    switch (index) {
    case 0: tim3_ch1_set(pulse);break; /* PB4 */
    case 1: tim3_ch2_set(pulse);break; /* PB5 */
    case 2: tim4_ch1_set(pulse);break; /* PB6 */
    case 3: tim4_ch2_set(pulse);break; /* PB7 */
    default: break;
    }

    last_delta[index] = pulse_to_delta(index, pulse);
}

void servo_set_delta(uint8_t index, float delta)
{
    if (!servo_initialized || index >= SERVO_COUNT) return;

    uint16_t pulse = delta_to_pulse(index, delta);
    servo_set_pulse(index, pulse);
}

void servo_set_all_delta(const float delta[4])
{
    if (!servo_initialized || delta == NULL) return;

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        uint16_t pulse = delta_to_pulse(i, delta[i]);
        servo_set_pulse(i, pulse);
    }
}

void servo_all_center(void)
{
    /* 写四个通道的 CCR */
    tim3_ch1_set(center_us[0]);
    tim3_ch2_set(center_us[1]);
    tim4_ch1_set(center_us[2]);
    tim4_ch2_set(center_us[3]);

    for (int i = 0; i < SERVO_COUNT; i++) {
        last_delta[i] = 0.0f;
    }
}

void servo_stop_all(void)
{
    if (!servo_initialized) return;

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);

    servo_initialized = false;
}

void servo_get_last_delta(float delta_out[4])
{
    if (delta_out != NULL) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            delta_out[i] = last_delta[i];
        }
    }
}

/* 回读 CCR 而不是复用 last_delta: last_delta 只在 servo_set_pulse() 里更新,
 * 而 servo_all_center() 是直接写 CCR 的, 走那条路径时 last_delta 和实际
 * 输出会分叉。CCR 是唯一"所写即所读"的真值来源。
 * Prescaler=169 → 1 tick = 1μs, 所以 CCR 数值本身就是脉宽。 */
void servo_get_pulse_us(uint16_t pulse_out[4])
{
    if (pulse_out == NULL) return;

    pulse_out[0] = (uint16_t)TIM3->CCR1;
    pulse_out[1] = (uint16_t)TIM3->CCR2;
    pulse_out[2] = (uint16_t)TIM4->CCR1;
    pulse_out[3] = (uint16_t)TIM4->CCR2;
}
