/**
 * @file    servo.h
 * @brief   舵机 PWM 驱动
 *
 *          硬件映射 (来自 Guided_Dart.ioc):
 *            舵机 1 → TIM3_CH1 → PB4（AF2）
 *            舵机 2 → TIM3_CH2 → PB5（AF2）
 *            舵机 3 → TIM4_CH1 → PB6（AF2）
 *            舵机 4 → TIM4_CH2 → PB7（AF2）
 *
 *          PWM 参数:
 *            时钟：    170 MHz（APB1 定时器时钟）
 *            预分频器：169 → 170MHz/(169+1) = 1MHz（每计数周期 1μs）
 *            周期：    4999 → 5000 个计数周期 = 5ms = 200Hz
 *
 *          标准数字舵机:
 *            - 中位:  1500 μs
 *            - 范围:  500 ~ 2500 μs
 *            - 频率：200 Hz（仅适用于明确支持高刷新率的数字舵机）
 *
 *          ⚠️ 舵机无位置反馈 — 输出的是指令, 不是实际位置
 */

#ifndef SERVO_H__
#define SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ─── 舵机数量 ─── */
#define SERVO_COUNT     4

/* ─── PWM 脉宽极限 (μs) ─── */
#define SERVO_PULSE_MIN     500     /* 最小安全脉宽 */
#define SERVO_PULSE_MAX     2500    /* 最大安全脉宽 */
#define SERVO_PULSE_MID     1500    /* 中位 */

/* ─── 归一化范围 ─── */
/* 归一化值 [-1, +1] 映射到 [SERVO_PULSE_MIN, SERVO_PULSE_MAX] */
/* 中位 0 → 1500μs */

/* ─── 舵机索引 ─── */
#define SERVO_1     0
#define SERVO_2     1
#define SERVO_3     2
#define SERVO_4     3

/* ─── 接口 ─── */

/**
 * @brief 初始化舵机 PWM
 *
 *        启动 TIM3_CH1, TIM3_CH2, TIM4_CH1, TIM4_CH2 的 PWM 输出。
 *        初始化后所有舵机输出中位 (1500μs)。
 *
 * @note  必须在 MX_TIM3_Init() 和 MX_TIM4_Init() 之后调用。
 *        此函数只启动 PWM 通道, TIM 基础初始化由 CubeMX 生成代码完成。
 */
void servo_init(void);

/**
 * @brief 设置单个舵机脉宽
 * @param index  舵机编号 (0~3)
 * @param pulse  脉宽 (μs), 自动限幅到 [500, 2500]
 */
void servo_set_pulse(uint8_t index, uint16_t pulse);

/**
 * @brief 设置单个舵机归一化偏角
 * @param index  舵机编号 (0~3)
 * @param delta  归一化偏角；实际中位、行程和方向取自 dart_config.h
 */
void servo_set_delta(uint8_t index, float delta);

/**
 * @brief 同时设置所有四个舵机归一化偏角
 * @param delta  数组 [4], 每个元素 ∈ [-1.0, +1.0]
 */
void servo_set_all_delta(const float delta[4]);

/**
 * @brief 所有舵机回中
 */
void servo_all_center(void);

/**
 * @brief 停止所有舵机 PWM (紧急情况)
 */
void servo_stop_all(void);

/**
 * @brief 获取最后的归一化输出值
 */
void servo_get_last_delta(float delta_out[4]);

/**
 * @brief 回读四个舵机当前实际输出的脉宽 (μs)
 *
 *        直接读 TIM3/TIM4 的 CCR 寄存器, 不是软件缓存 —— 所写即所读,
 *        所以 servo_all_center() 那条绕过 delta 转换、直接写 CCR 的路径
 *        也能被如实反映出来。
 *
 * @param pulse_out  数组 [4], 按舵机索引 0~3 依次输出脉宽 (μs)
 */
void servo_get_pulse_us(uint16_t pulse_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H__ */
