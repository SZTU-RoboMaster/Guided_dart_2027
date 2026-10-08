/**
 * @file    ab_filter.c
 * @brief   α-β-γ 滤波器实现
 *
 *          恒加速度模型:
 *          预测步: est  += rate * dt + 0.5 * acc * dt^2
 *                  rate += acc * dt
 *          更新步: residual = meas - est
 *                  est  += α * residual
 *                  rate += (β / dt) * residual
 *                  acc  += (γ / dt^2) * residual   (gamma > 0 时启用)
 *
 *          gamma = 0 时退化为经典 α-β 滤波器 (不估计加速度)。
 */

#include "ab_filter.h"
#include <math.h>
#include <string.h>

void ab_filter_init(ab_filter_t *f, float alpha, float beta, float dt)
{
    memset(f, 0, sizeof(*f));

    f->alpha = alpha;
    f->beta  = beta;
    f->dt    = dt;

    /* 初值: 无有效数据 */
    f->est  = 0.0f;
    f->rate = 0.0f;
    f->acc  = 0.0f;

    /* gamma 默认 0: 保持 α-β 行为不变, 由调用方按需开启加速度估计 */
    f->gamma = 0.0f;

    f->est_limit  = 0.0f;
    f->rate_limit = 0.0f;
    f->acc_limit  = 0.0f;

    f->initialized = true;
}

void ab_filter_set_limits(ab_filter_t *f, float est_limit, float rate_limit)
{
    f->est_limit  = est_limit;
    f->rate_limit = rate_limit;
}

void ab_filter_set_state(ab_filter_t *f, float est, float rate)
{
    f->est  = est;
    f->rate = rate;
    f->acc  = 0.0f;
}

void ab_filter_reset(ab_filter_t *f)
{
    f->est  = 0.0f;
    f->rate = 0.0f;
    f->acc  = 0.0f;
}

float ab_filter_update(ab_filter_t *f, float meas)
{
    if (!f->initialized) return 0.0f;

    /* ── 预测步 (恒加速度模型) ── */
    f->est  += f->rate * f->dt + 0.5f * f->acc * f->dt * f->dt;
    f->rate += f->acc * f->dt;

    /* ── 更新步 ── */
    float residual = meas - f->est;

    f->est  += f->alpha * residual;
    f->rate += (f->beta / f->dt) * residual;
    if (f->gamma > 0.0f) {
        f->acc += (f->gamma / (f->dt * f->dt)) * residual;
    }

    /* ── 限幅 ── */
    if (f->est_limit > 0.0f) {
        if (f->est >  f->est_limit) f->est =  f->est_limit;
        if (f->est < -f->est_limit) f->est = -f->est_limit;
    }

    if (f->rate_limit > 0.0f) {
        if (f->rate >  f->rate_limit) f->rate =  f->rate_limit;
        if (f->rate < -f->rate_limit) f->rate = -f->rate_limit;
    }

    if (f->acc_limit > 0.0f) {
        if (f->acc >  f->acc_limit) f->acc =  f->acc_limit;
        if (f->acc < -f->acc_limit) f->acc = -f->acc_limit;
    }

    return f->est;
}

void ab_filter_set_dt(ab_filter_t *f, float dt)
{
    f->dt = dt;
}

void ab_filter_set_gains(ab_filter_t *f, float alpha, float beta)
{
    f->alpha = alpha;
    f->beta  = beta;
}

void ab_filter_set_gamma(ab_filter_t *f, float gamma)
{
    f->gamma = gamma;
}

void ab_filter_set_acc_limit(ab_filter_t *f, float acc_limit)
{
    f->acc_limit = acc_limit;
}
