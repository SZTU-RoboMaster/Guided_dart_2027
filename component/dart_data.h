/**
 * @file    dart_data.h
 * @brief   飞行数据结构 — 全机统一的姿态/状态信息
 *
 *          通过 dart_data_t 提供:
 *            - 原始 AHRS 姿态（滚转/俯仰/偏航、机体角速度）
 *            - 发射时刻姿态基准 (roll_ref/pitch_ref/yaw_ref)
 *            - 相对姿态 (flight_roll/flight_pitch/flight_yaw)
 *            - 飞行时间、阶段、速度估计等运行状态
 *
 *          写入: dart_control.c 在 200Hz 控制循环中更新
 *          读取: 任何模块通过 dart_data_get() 获取只读指针
 *
 *          ╔══════════════════════════════════════════════════════╗
 *          ║  IMU 轴交换说明                                     ║
 *          ║  ICM-45686 芯片转了 90° 安装:                       ║
 *          ║    芯片滚转轴 → 机体俯仰轴                           ║
 *          ║    芯片俯仰轴 → 机体滚转轴                           ║
 *          ║  本结构体中所有滚转/俯仰量均为交换后的机体坐标系       ║
 *          ║  外部消费者无需关心安装方向                          ║
 *          ╚══════════════════════════════════════════════════════╝
 */

#ifndef DART_DATA_H__
#define DART_DATA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "flight_state.h"

/* ─── 统一飞行数据结构 ─── */
typedef struct {
    /* ── 原始 AHRS 姿态 (机体坐标系, rad, 已做轴交换) ── */
    float roll;                  /* 机体滚转角 */
    float pitch;                 /* 机体俯仰角 */
    float yaw;                   /* 机体偏航角（无磁力计，仅为积分值） */

    /* ── 发射时刻姿态基准 (rad, 进入 STABILIZE 瞬间捕获) ── */
    float roll_ref;              /* 发射基准滚转角 */
    float pitch_ref;             /* 发射基准俯仰角 */
    float yaw_ref;               /* 发射基准偏航角 */

    /* ── 相对姿态 = 当前 − 基准（rad，偏航角已进行角度包裹）── */
    float flight_roll;           /* 滚转角与发射基准滚转角之差 */
    float flight_pitch;          /* 俯仰角与发射基准俯仰角之差 */
    float flight_yaw;            /* 偏航角与发射基准偏航角之差（已包裹） */

    /* ── 机体角速度（rad/s，陀螺仪直接输出）── */
    float p;                     /* 滚转角速度 */
    float q;                     /* 俯仰角速度 */
    float r;                     /* 偏航角速度 */

    /* 视觉制导量（LOS：视线），角度单位 rad，角速度单位 rad/s */
    float los_yaw;               /* 补偿后的 LOS 偏航角 */
    float los_pitch;             /* 补偿后的 LOS 俯仰角 */
    float los_yaw_rate;          /* 补偿后的 LOS 偏航角速度 */
    float los_pitch_rate;        /* 补偿后的 LOS 俯仰角速度 */
    float vision_fps;            /* 视觉帧率（Hz） */

    /* ── 加速度 (m/s²) ── */
    float ax;                    /* 机体 X 轴加速度 */
    float ay;                    /* 机体 Y 轴加速度 */
    float az;                    /* 机体 Z 轴加速度 */
    float accel_mag;             /* 加速度模长, 用于发射/撞击检测 */

    /* ── 基准有效标志 ── */
    bool  ref_valid;             /* true：发射基准已捕获，flight_* 有效 */

    /* ── 飞行状态快照 ── */
    flight_phase_t phase;        /* 当前飞行阶段 */
    flight_phase_t prev_phase;   /* 上一阶段 (用于检测阶段边沿) */
    bool  impact_detected;       /* true = 因撞靶进入 END; false = 超时或尚未结束 */
    float flight_time_s;         /* 从发射起的累计时间 (s) */

    /* ── 估计量 ── */
    float V_est;                 /* 估计空速 (m/s) */

    /* ── 制导时间量 (仅遥测, 不参与控制) ──
     * TA 与 DF 分开锁存, 因为二者通常不是同一帧: 捕获首帧目标往往只有
     * 1 px 半径, 视觉端量不出距离 (返回 0), 此时 TA 来自 fallback_tgo()
     * 而非视觉。DF 记的是第一个真正量到距离的帧。 */
    float tgo_s;                 /* 当前 t_go 估计 (s) */
    float tgo_acquire_s;         /* 首次跑出制导指令瞬间的 t_go (s); <0 = 尚未捕获 */
    uint16_t dist_first_dm;      /* 首个有效视觉距离 (dm); 0 = 尚未量到 */
    uint8_t conf_max;            /* 本次飞行观测到的视觉置信度峰值 (0~100)。
                                  * 用来定 CONF_STRONG / CONF_REACQUIRE ——
                                  * 若峰值低于它们, 依赖它们的逻辑会静默失效 */

    /* 基于 IMU 估计的速度方向，以及速度方向相对机头的偏角。 */
    float velocity_yaw_rad;
    float velocity_pitch_rad;
    float track_yaw_rad;
    float track_pitch_rad;
    float track_yaw_rate_rad_s;
    float track_pitch_rate_rad_s;
    bool ballistic_valid;

} dart_data_t;

/**
 * @brief 获取全局飞行数据只读指针
 * @return 指向 dart_data_t 的常量指针
 */
const dart_data_t *dart_data_get(void);

#ifdef __cplusplus
}
#endif

#endif /* DART_DATA_H__ */
