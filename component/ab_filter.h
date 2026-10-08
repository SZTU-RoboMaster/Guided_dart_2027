/**
 * @file    ab_filter.h
 * @brief   α-β 滤波器
 *
 *          用于从含噪测量中提取平滑值和变化率。
 *          在制导飞镖中用于:
 *            - 平滑视线角 (λ_yaw, λ_pitch)
 *            - 提取视线角速率 (λ̇_yaw, λ̇_pitch)
 *            - 平滑像素尺寸 → 估计 T_go
 *
 *          Benedict-Bordner 最优关系: β = α² / (2 - α)
 *          推荐起步值: α=0.20, β=0.02
 *
 *          参考: "雷达数据处理及应用" 何友, 第2章
 */

#ifndef AB_FILTER_H__
#define AB_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ─── α-β 滤波器结构体 ─── */
typedef struct {
    float est;        /* 滤波后的估计值 */
    float rate;       /* 估计的变化率 (一阶导数) */
    float acc;        /* 估计的加速度 (二阶导数, α-β-γ 滤波器) */
    float alpha;      /* 位置平滑系数 (0~1) */
    float beta;       /* 速度提取系数 (0~1) */
    float gamma;      /* 加速度提取系数 (0~1, 0 = 退化为 α-β) */
    float dt;         /* 采样周期 (秒) */

    /* 限幅 */
    float est_limit;  /* 估计值上限 (0 = 不限幅) */
    float rate_limit; /* 变化率上限 (0 = 不限幅) */
    float acc_limit;  /* 加速度上限 (0 = 不限幅) */

    bool  initialized;
} ab_filter_t;

/* ─── 接口 ─── */

/**
 * @brief 初始化 α-β 滤波器
 * @param f       滤波器实例
 * @param alpha   位置平滑系数 (推荐 0.15~0.30)
 * @param beta    速度提取系数 (推荐 0.01~0.05)
 * @param dt      采样周期 (秒)
 */
void ab_filter_init(ab_filter_t *f, float alpha, float beta, float dt);

/**
 * @brief 设置限幅
 */
void ab_filter_set_limits(ab_filter_t *f, float est_limit, float rate_limit);

/**
 * @brief 更新滤波器
 * @param f      滤波器实例
 * @param meas   新测量值
 * @return       滤波后的估计值 (也存储在 f->est 中)
 */
float ab_filter_update(ab_filter_t *f, float meas);

/**
 * @brief 仅设置初始状态 (不发散更新)
 */
void ab_filter_set_state(ab_filter_t *f, float est, float rate);

/**
 * @brief 重置滤波器
 */
void ab_filter_reset(ab_filter_t *f);

/**
 * @brief 动态更新采样周期 (制导按帧率调用时使用)
 */
void ab_filter_set_dt(ab_filter_t *f, float dt);

/**
 * @brief 动态更新增益
 */
void ab_filter_set_gains(ab_filter_t *f, float alpha, float beta);

/**
 * @brief 设置加速度提取系数 gamma (0 = 退化为 α-β, 不估计加速度)
 */
void ab_filter_set_gamma(ab_filter_t *f, float gamma);

/**
 * @brief 设置加速度限幅 (0 = 不限幅)
 */
void ab_filter_set_acc_limit(ab_filter_t *f, float acc_limit);

#ifdef __cplusplus
}
#endif

#endif /* AB_FILTER_H__ */
