#include "rate_control.h"
#include "dart_config.h"
#include <string.h>

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void rate_control_init(rate_controller_t *rc, float dt,
                       float kp_p, float kp_q, float kp_r,
                       float kd_p, float kd_q, float kd_r)
{
    memset(rc, 0, sizeof(*rc));
    rc->dt = dt;
    rc->kp_p = kp_p; rc->kp_q = kp_q; rc->kp_r = kp_r;
    rc->kd_p = kd_p; rc->kd_q = kd_q; rc->kd_r = kd_r;
    rc->accel_lpf_alpha = DART_RATE_ACCEL_LPF_ALPHA;
    rc->output_limit = 1.0f;
    rc->initialized = true;
}

void rate_control_set_limit(rate_controller_t *rc, float limit)
{
    rc->output_limit = limit;
}

void rate_control_update(rate_controller_t *rc,
                         float p_cmd, float q_cmd, float r_cmd,
                         float p, float q, float r,
                         float *mx, float *my, float *mz)
{
    if (!rc->initialized) { *mx = *my = *mz = 0.0f; return; }
    if (!rc->have_previous) {
        rc->previous_p = p; rc->previous_q = q; rc->previous_r = r;
        rc->have_previous = true;
    }
    const float inv_dt = 1.0f / rc->dt;
    const float raw_dp = (p - rc->previous_p) * inv_dt;
    const float raw_dq = (q - rc->previous_q) * inv_dt;
    const float raw_dr = (r - rc->previous_r) * inv_dt;
    const float a = rc->accel_lpf_alpha;
    rc->p_accel_lpf += a * (raw_dp - rc->p_accel_lpf);
    rc->q_accel_lpf += a * (raw_dq - rc->q_accel_lpf);
    rc->r_accel_lpf += a * (raw_dr - rc->r_accel_lpf);
    rc->previous_p = p; rc->previous_q = q; rc->previous_r = r;

    *mx = rc->kp_p * (p_cmd - p) - rc->kd_p * rc->p_accel_lpf;
    *my = rc->kp_q * (q_cmd - q) - rc->kd_q * rc->q_accel_lpf;
    *mz = rc->kp_r * (r_cmd - r) - rc->kd_r * rc->r_accel_lpf;
    *mx = clampf(*mx, rc->output_limit);
    *my = clampf(*my, rc->output_limit);
    *mz = clampf(*mz, rc->output_limit);
}

void rate_control_reset(rate_controller_t *rc)
{
    rc->p_accel_lpf = rc->q_accel_lpf = rc->r_accel_lpf = 0.0f;
    rc->have_previous = false;
}
