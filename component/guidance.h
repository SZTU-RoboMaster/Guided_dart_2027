#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <stdbool.h>
#include <stdint.h>
#include "ab_filter.h"
#include "vision_protocol.h"

typedef struct {
    float p_cmd;
    float q_cmd;
    float r_cmd;
    float los_yaw;
    float los_pitch;
    float los_yaw_rate;
    float los_pitch_rate;
    float t_go_s;
    float total_delay_s;
    float authority;
} guidance_output_t;

typedef struct {
    ab_filter_t yaw_filter;
    ab_filter_t pitch_filter;
    float radius_est;
    float radius_rate;
    float last_radius;
    float held_q_cmd;
    float held_r_cmd;
    float last_tgo_s;
    uint8_t last_confidence;
    bool initialized;
    bool target_initialized;

    /* APN λ̈ 估计: 与 λ̇ 同源, 由融合后 λ̇ 帧间差分再低通得到 */
    float yaw_lam_d_prev;   /* 上一帧 λ̇, 用于差分 */
    float pitch_lam_d_prev;
    float yaw_lam_dd;       /* λ̈ 低通值 (rad/s²) */
    float pitch_lam_dd;

    /* ISMCG 滑模状态 (仅 DART_GUIDANCE_MODE_ISMCG = 1 时使用) */
    float yaw_los_integral;    /* ∫λ̇ dt, 滑模面积分项 */
    float pitch_los_integral;
    float yaw_rho;             /* 自适应机动上界估计 (rad/s) */
    float pitch_rho;
} guidance_t;

void guidance_init(guidance_t *g);
void guidance_reset_target(guidance_t *g);

bool guidance_update(guidance_t *g,
                     const vision_frame_t *frame,
                     float roll_rad,
                     float roll_rate_rad_s,
                     float p_rad_s,
                     float q_rad_s,
                     float r_rad_s,
                     float speed_m_s,
                     float flight_time_s,
                     float frame_dt_s,
                     uint32_t frame_age_ms,
                     guidance_output_t *out);

void guidance_hold(const guidance_t *g,
                   float roll_rad,
                   uint32_t frame_age_ms,
                   guidance_output_t *out);

void guidance_failsafe(float roll_rad, guidance_output_t *out);

#endif
