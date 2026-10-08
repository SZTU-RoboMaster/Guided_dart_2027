#ifndef BALLISTIC_ESTIMATOR_H
#define BALLISTIC_ESTIMATOR_H

#include <stdbool.h>

#include "ahrs.h"

typedef struct {
    float face_dir_nav[3];
    float velocity_dir_nav[3];
    float accel_nav[3];
    float linear_accel_nav[3];
    float face_yaw_rad;
    float face_pitch_rad;
    float velocity_yaw_rad;
    float velocity_pitch_rad;
    /* 速度方向相对于机体 +X 轴（机头方向）的偏角。 */
    float track_yaw_rad;
    float track_pitch_rad;
    float track_yaw_rate_rad_s;
    float track_pitch_rate_rad_s;
    float filtered_linear_accel_nav[3];
    float previous_track_yaw_rad;
    float previous_track_pitch_rad;
    bool accel_filter_ready;
    bool initialized;
} ballistic_estimator_t;

void ballistic_estimator_init(ballistic_estimator_t *estimator);
void ballistic_estimator_reset(ballistic_estimator_t *estimator,
                               const quaternion_t *attitude);
void ballistic_estimator_invalidate(ballistic_estimator_t *estimator);

bool ballistic_estimator_update(ballistic_estimator_t *estimator,
                                const quaternion_t *attitude,
                                float ax_body,
                                float ay_body,
                                float az_body,
                                float speed_m_s,
                                float dt_s);

#endif
