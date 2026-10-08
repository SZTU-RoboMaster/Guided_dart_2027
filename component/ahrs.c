/**
 * @file    ahrs.c
 * @brief   Mahony AHRS 姿态估计实现
 *
 *          算法流程:
 *            1. 归一化加速度测量值
 *            2. 从当前四元数估计重力方向
 *            3. 叉积 = 加速度测量 ⊗ 重力估计 → 姿态误差
 *            4. PI 修正陀螺角速度
 *            5. 四元数积分更新
 *            6. 归一化四元数
 */

#include "ahrs.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════
 *  初始化
 * ═══════════════════════════════════════════════════════════════ */

void ahrs_init(ahrs_t *ahrs, float Kp, float Ki, float dt)
{
    memset(ahrs, 0, sizeof(*ahrs));

    /* 初始姿态: 水平 (q = [1, 0, 0, 0]) */
    ahrs->q.q0 = 1.0f;
    ahrs->q.q1 = 0.0f;
    ahrs->q.q2 = 0.0f;
    ahrs->q.q3 = 0.0f;

    ahrs->Kp = Kp;
    ahrs->Ki = Ki;
    ahrs->dt = dt;

    ahrs->integral_fbx = 0.0f;
    ahrs->integral_fby = 0.0f;
    ahrs->integral_fbz = 0.0f;

    ahrs->initialized = true;
}

void ahrs_reset(ahrs_t *ahrs)
{
    ahrs_init(ahrs, ahrs->Kp, ahrs->Ki, ahrs->dt);
}

/* ═══════════════════════════════════════════════════════════════
 *  快速倒数平方根近似（《雷神之锤 III》经典算法）
 *  对于 Cortex-M4F 有 FPU, 用 1.0f/sqrtf() 也可, 此为实现参考
 * ═══════════════════════════════════════════════════════════════ */

static inline float inv_sqrt(float x)
{
    /* 在 STM32G4 (Cortex-M4F) 上直接用硬件 sqrt + 除法更快 */
    return 1.0f / sqrtf(x);
}

/* ═══════════════════════════════════════════════════════════════
 *  主更新函数
 * ═══════════════════════════════════════════════════════════════ */

void ahrs_update(ahrs_t *ahrs,
                 float gx, float gy, float gz,
                 float ax, float ay, float az)
{
    ahrs_update_weighted(ahrs, gx, gy, gz, ax, ay, az, 1.0f);
}

void ahrs_update_weighted(ahrs_t *ahrs,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float accel_weight)
{
    if (!ahrs->initialized) return;

    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;

    if (accel_weight < 0.0f) accel_weight = 0.0f;
    if (accel_weight > 1.0f) accel_weight = 1.0f;

    /* ── 1. 归一化加速度测量值 ── */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        /* 加速度为零 → 跳过修正, 仅陀螺积分 */
        ax = 0.0f; ay = 0.0f; az = -1.0f;
        norm = 1.0f;
    }
    float inv_norm = 1.0f / norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    /* ── 2. 从四元数估计重力方向 (机体坐标系) ── */
    /*     重力在惯性系为 [0, 0, -1], 旋转到机体系 */
    float vx = -2.0f * (q1 * q3 - q0 * q2);
    float vy = -2.0f * (q0 * q1 + q2 * q3);
    float vz = -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

    /* ── 3. 叉积: 测量值 × 估计值 → 姿态误差 ── */
    float ex = (ay * vz - az * vy) * accel_weight;
    float ey = (az * vx - ax * vz) * accel_weight;
    float ez = (ax * vy - ay * vx) * accel_weight;

    /* ── 4. PI 修正陀螺 ── */
    /* 积分项 — 补偿陀螺常值漂移 */
    if (accel_weight > 0.0f) {
        ahrs->integral_fbx += ahrs->Ki * ex * ahrs->dt;
        ahrs->integral_fby += ahrs->Ki * ey * ahrs->dt;
        ahrs->integral_fbz += ahrs->Ki * ez * ahrs->dt;
    }

    /* 积分限幅 — 防止积分饱和 */
    const float integral_limit = 0.3f;
    if (ahrs->integral_fbx >  integral_limit) ahrs->integral_fbx =  integral_limit;
    if (ahrs->integral_fbx < -integral_limit) ahrs->integral_fbx = -integral_limit;
    if (ahrs->integral_fby >  integral_limit) ahrs->integral_fby =  integral_limit;
    if (ahrs->integral_fby < -integral_limit) ahrs->integral_fby = -integral_limit;
    if (ahrs->integral_fbz >  integral_limit) ahrs->integral_fbz =  integral_limit;
    if (ahrs->integral_fbz < -integral_limit) ahrs->integral_fbz = -integral_limit;

    /* 修正后的角速度 */
    gx += ahrs->Kp * ex + ahrs->integral_fbx;
    gy += ahrs->Kp * ey + ahrs->integral_fby;
    gz += ahrs->Kp * ez + ahrs->integral_fbz;

    /* ── 5. 四元数积分 (一阶) ── */
    float half_dt = 0.5f * ahrs->dt;
    const float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    const float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    const float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    const float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * half_dt;
    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    /* ── 6. 归一化 ── */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-6f) {
        inv_norm = 1.0f / norm;
        ahrs->q.q0 = q0 * inv_norm;
        ahrs->q.q1 = q1 * inv_norm;
        ahrs->q.q2 = q2 * inv_norm;
        ahrs->q.q3 = q3 * inv_norm;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  姿态提取
 * ═══════════════════════════════════════════════════════════════ */

void ahrs_get_quaternion(const ahrs_t *ahrs, quaternion_t *q_out)
{
    *q_out = ahrs->q;
}

void ahrs_get_euler(const ahrs_t *ahrs, euler_t *euler_out)
{
    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;

    /* 滚转：绕 X 轴 */
    euler_out->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                              1.0f - 2.0f * (q1 * q1 + q2 * q2));

    /* 俯仰：绕 Y 轴 */
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);

    /* 保护 asinf 参数范围 */
    if (sin_pitch >  1.0f) sin_pitch =  1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;

    euler_out->pitch = asinf(sin_pitch);

    /* 偏航：绕 Z 轴 */
    euler_out->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                             1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

float ahrs_get_roll(const ahrs_t *ahrs)
{
    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;

    return atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

float ahrs_get_pitch(const ahrs_t *ahrs)
{
    float q0 = ahrs->q.q0;
    float q2 = ahrs->q.q2;
    float q1 = ahrs->q.q1;
    float q3 = ahrs->q.q3;

    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_pitch >  1.0f) sin_pitch =  1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    return asinf(sin_pitch);
}

float ahrs_get_yaw(const ahrs_t *ahrs)
{
    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;

    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                   1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
