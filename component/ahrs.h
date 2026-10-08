/**
 * @file    ahrs.h
 * @brief   Mahony AHRS — 姿态航向参考系统
 *
 *          使用陀螺仪积分 + 加速度计修正的互补滤波器
 *          输出四元数姿态和欧拉角（滚转/俯仰/偏航）
 *
 *          飞镖自稳核心：精确的姿态估计是滚转坐标补偿、
 *          角速率阻尼和制导指令正确执行的前提。
 *
 *          参考：Mahony 等，《特殊正交群上的非线性互补滤波器》，
 *                IEEE TAC 2008
 */

#ifndef AHRS_H__
#define AHRS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ─── 四元数结构体 ─── */
typedef struct {
    float q0;    /* w — 实部 */
    float q1;    /* x */
    float q2;    /* y */
    float q3;    /* z */
} quaternion_t;

/* ─── 欧拉角 (弧度) ─── */
typedef struct {
    float roll;   /* 绕 X 轴, [-π, π] */
    float pitch;  /* 绕 Y 轴, [-π/2, π/2] */
    float yaw;    /* 绕 Z 轴, [-π, π] */
} euler_t;

/* ─── 角速度 (rad/s) ─── */
typedef struct {
    float p;      /* 滚转角速度 — 绕机体 X 轴 */
    float q;      /* 俯仰角速度 — 绕机体 Y 轴 */
    float r;      /* 偏航角速度 — 绕机体 Z 轴 */
} body_rate_t;

/* ─── AHRS 主结构体 ─── */
typedef struct {
    /* 姿态四元数 */
    quaternion_t q;

    /* 滤波器增益 */
    float Kp;           /* 比例增益 (推荐 1.0~3.0) */
    float Ki;           /* 积分增益 (推荐 0.01~0.05) */

    /* 积分项 — 补偿陀螺漂移 */
    float integral_fbx;
    float integral_fby;
    float integral_fbz;

    /* 采样周期 (秒) */
    float dt;

    /* 初始化标志 */
    bool initialized;
} ahrs_t;

/* ─── 接口 ─── */

/**
 * @brief 初始化 AHRS
 * @param ahrs   AHRS 实例指针
 * @param Kp     比例增益 (典型值: 1.0)
 * @param Ki     积分增益 (典型值: 0.01)
 * @param dt     采样周期 (秒), 例如 0.005 对应 200Hz
 */
void ahrs_init(ahrs_t *ahrs, float Kp, float Ki, float dt);

/**
 * @brief AHRS 更新 — 每帧调用一次
 * @param ahrs   AHRS 实例指针
 * @param gx,gy,gz  陀螺仪测量值 (rad/s)
 * @param ax,ay,az  加速度计测量值 (m/s², 任意单位也可)
 * @note  加速度计用于修正滚转/俯仰漂移，偏航无修正参考
 *         有磁力计时可扩展偏航修正
 */
void ahrs_update(ahrs_t *ahrs,
                 float gx, float gy, float gz,
                 float ax, float ay, float az);

/* accel_weight=0：仅进行陀螺仪积分；为 1：进行常规 Mahony 修正。 */
void ahrs_update_weighted(ahrs_t *ahrs,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float accel_weight);

/**
 * @brief 获取当前四元数
 */
void ahrs_get_quaternion(const ahrs_t *ahrs, quaternion_t *q_out);

/**
 * @brief 获取欧拉角 (rad)
 */
void ahrs_get_euler(const ahrs_t *ahrs, euler_t *euler_out);

/**
 * @brief 获取滚转角 (rad) — 飞镖自稳常用
 */
float ahrs_get_roll(const ahrs_t *ahrs);

/**
 * @brief 获取俯仰角 (rad)
 */
float ahrs_get_pitch(const ahrs_t *ahrs);

/**
 * @brief 获取偏航角 (rad) — 无磁力计时仅陀螺积分，会漂移
 */
float ahrs_get_yaw(const ahrs_t *ahrs);

/**
 * @brief 重置 AHRS 状态
 */
void ahrs_reset(ahrs_t *ahrs);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H__ */
