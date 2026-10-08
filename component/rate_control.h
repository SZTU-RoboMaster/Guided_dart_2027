#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include <stdbool.h>

typedef struct {
    float kp_p, kp_q, kp_r;
    float kd_p, kd_q, kd_r;
    float dt;
    float previous_p, previous_q, previous_r;
    float p_accel_lpf, q_accel_lpf, r_accel_lpf;
    float accel_lpf_alpha;
    float output_limit;
    bool initialized;
    bool have_previous;
} rate_controller_t;

void rate_control_init(rate_controller_t *rc, float dt,
                       float kp_p, float kp_q, float kp_r,
                       float kd_p, float kd_q, float kd_r);
void rate_control_set_limit(rate_controller_t *rc, float limit);
void rate_control_update(rate_controller_t *rc,
                         float p_cmd, float q_cmd, float r_cmd,
                         float p_meas, float q_meas, float r_meas,
                         float *mx, float *my, float *mz);
void rate_control_reset(rate_controller_t *rc);

#endif
