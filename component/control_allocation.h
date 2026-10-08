/**
 * @file    control_allocation.h
 * @brief   舵面控制分配器 — 解耦制导需求与舵面执行
 *
 *          问题: X 翼面构型, 4 个舵面 → 3 轴力矩
 *               M = B × δ    (3 = 3×4 × 4)
 *
 *          4 个舵面产生 3 轴力矩 — 冗余 → 需要在约束下求最优解
 *
 *          求解 (加权伪逆):
 *            min  || Wm · (B_q δ − M_cmd) ||² + || Wu · δ ||² + λ ||δ||²
 *            δ
 *            s.t.  δ_min ≤ δ ≤ δ_max
 *                  |δ − δ_last| ≤ rate_limit × dt
 *
 *          B 矩阵 (X 翼面构型 — 初始经验值, 需转台标定):
 *
 *                 滚转    俯仰    偏航
 *            舵面1: -0.5   +0.6   +0.6    (左前翼)
 *            舵面2: -0.5   -0.6   +0.6    (右前翼)
 *            舵面3: -0.5   +0.6   -0.6    (左后翼)
 *            舵面4: -0.5   -0.6   -0.6    (右后翼)
 *
 *          关键特性:
 *            - 动压缩放: B_q = B × (½ρV² / q_nominal)
 *            - 死区逆补偿: 在小指令上叠加偏移量, 确保舵机实际动作
 *            - 控制轴优先级：偏航/俯仰 > 滚转（末端阶段）
 *            - 舵面饱和重分配：优先削减滚转控制量
 */

#ifndef CONTROL_ALLOCATION_H__
#define CONTROL_ALLOCATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ─── 舵机数量 ─── */
#define CA_NUM_SERVOS   4
#define CA_NUM_AXES     3    /* 滚转、俯仰、偏航 */

/* ─── 控制分配器 ─── */
typedef struct {
    /* B 矩阵 (3×4)：[滚转、俯仰、偏航] × [舵机 1 至 4] */
    float B[CA_NUM_AXES][CA_NUM_SERVOS];

    /* 标称动压: q_nominal = ½ρV² @ V=15m/s ≈ 137.8 Pa */
    float q_nominal;

    /* 舵面使用代价 Wu (越大越不愿用) */
    float Wu[CA_NUM_SERVOS];

    /* 正则化系数 λ */
    float lambda_reg;

    /* 约束 */
    float delta_min;        /* 归一化下限 (推荐 -1.0) */
    float delta_max;        /* 归一化上限 (推荐 +1.0) */
    float rate_limit;       /* 最大变化率 (/s) */

    /* 死区补偿 */
    float deadband;         /* 死区宽度 (归一化), 推荐 0.015 */
    float deadband_boost;   /* 逆补偿附加量, 推荐 0.01 */

    /* 上一步舵面位置 */
    float delta_last[CA_NUM_SERVOS];

    /* 采样周期 */
    float dt;

    bool  initialized;
} control_allocator_t;

/* ─── T_go → 控制轴权重映射 ─── */
typedef struct {
    float w_roll;
    float w_pitch;
    float w_yaw;
} ca_weights_t;

/* ─── 接口 ─── */

/**
 * @brief 初始化控制分配器
 */
void ca_init(control_allocator_t *ca, float dt);

/**
 * @brief 设置 B 矩阵 (用于后期转台标定后更新)
 */
void ca_set_B_matrix(control_allocator_t *ca,
                     const float B[CA_NUM_AXES][CA_NUM_SERVOS]);

/**
 * @brief 设置舵机死区参数
 */
void ca_set_deadband(control_allocator_t *ca,
                     float deadband, float deadband_boost);

/**
 * @brief 核心分配函数
 *
 * @param ca          分配器实例
 * @param Mx_cmd, My_cmd, Mz_cmd  期望力矩 (归一化, ±1.0)
 * @param V_est       估计速度 (m/s), 用于动压缩放
 * @param T_go        剩余飞行时间 (s), 用于控制轴权重调度
 * @param delta_out   舵面归一化偏角输出 [-1, +1] × 4
 * @param saturated   输出是否经过饱和限制
 *
 * @note  delta_out 已包含死区逆补偿和速率限制
 */
void ca_allocate(control_allocator_t *ca,
                 float Mx_cmd, float My_cmd, float Mz_cmd,
                 float V_est, float T_go,
                 float delta_out[CA_NUM_SERVOS],
                 bool *saturated);

/**
 * @brief 根据 T_go 计算控制轴权重
 */
void ca_get_T_go_weights(float T_go, ca_weights_t *w);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ALLOCATION_H__ */
