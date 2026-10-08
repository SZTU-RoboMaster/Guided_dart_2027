#include "ballistic_estimator.h"

#include "dart_config.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float wrap_pi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static bool normalize3(float vector[3])
{
    const float norm_sq = vector[0] * vector[0]
                        + vector[1] * vector[1]
                        + vector[2] * vector[2];
    if (!isfinite(norm_sq) || norm_sq < 1.0e-10f) return false;
    const float inv_norm = 1.0f / sqrtf(norm_sq);
    vector[0] *= inv_norm;
    vector[1] *= inv_norm;
    vector[2] *= inv_norm;
    return true;
}

/* 四元数约定与 component/ahrs.c 保持一致。 */
static void quaternion_to_body_to_nav(const quaternion_t *q, float rotation[3][3])
{
    const float q0 = q->q0;
    const float q1 = q->q1;
    const float q2 = q->q2;
    const float q3 = q->q3;

    rotation[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    rotation[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    rotation[0][2] = 2.0f * (q1 * q3 + q0 * q2);
    rotation[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    rotation[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    rotation[1][2] = 2.0f * (q2 * q3 - q0 * q1);
    rotation[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    rotation[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    rotation[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
}

static void mat3_mul_vec3(const float matrix[3][3],
                          const float vector[3],
                          float output[3])
{
    for (int row = 0; row < 3; ++row) {
        output[row] = matrix[row][0] * vector[0]
                    + matrix[row][1] * vector[1]
                    + matrix[row][2] * vector[2];
    }
}

static void mat3_transpose_mul_vec3(const float matrix[3][3],
                                    const float vector[3],
                                    float output[3])
{
    for (int column = 0; column < 3; ++column) {
        output[column] = matrix[0][column] * vector[0]
                       + matrix[1][column] * vector[1]
                       + matrix[2][column] * vector[2];
    }
}

static void vector_to_yaw_pitch(const float vector[3],
                                float *yaw_rad,
                                float *pitch_rad)
{
    const float horizontal = sqrtf(vector[0] * vector[0]
                                  + vector[1] * vector[1]);
    *yaw_rad = atan2f(vector[1], vector[0]);
    *pitch_rad = atan2f(vector[2], horizontal);
}

static void update_face_direction(ballistic_estimator_t *estimator,
                                  const float rotation[3][3])
{
    /* Guided_dart_2027 将机体 +X 轴定义为机头前向轴。 */
    estimator->face_dir_nav[0] = rotation[0][0];
    estimator->face_dir_nav[1] = rotation[1][0];
    estimator->face_dir_nav[2] = rotation[2][0];
    (void)normalize3(estimator->face_dir_nav);
    vector_to_yaw_pitch(estimator->face_dir_nav,
                        &estimator->face_yaw_rad,
                        &estimator->face_pitch_rad);
}

static void update_track_angles(ballistic_estimator_t *estimator,
                                const float rotation[3][3],
                                float dt_s)
{
    float velocity_body[3];
    mat3_transpose_mul_vec3(rotation, estimator->velocity_dir_nav,
                            velocity_body);
    const float horizontal = sqrtf(velocity_body[0] * velocity_body[0]
                                  + velocity_body[1] * velocity_body[1]);
    const float track_yaw = atan2f(velocity_body[1], velocity_body[0]);
    const float track_pitch = atan2f(velocity_body[2], horizontal);

    if (dt_s > 1.0e-5f) {
        const float raw_yaw_rate = wrap_pi(track_yaw
                                         - estimator->previous_track_yaw_rad) / dt_s;
        const float raw_pitch_rate = wrap_pi(track_pitch
                                           - estimator->previous_track_pitch_rad) / dt_s;
        const float alpha = clampf(DART_BALLISTIC_TRACK_RATE_LPF_ALPHA,
                                   0.0f, 1.0f);
        estimator->track_yaw_rate_rad_s += alpha
            * (raw_yaw_rate - estimator->track_yaw_rate_rad_s);
        estimator->track_pitch_rate_rad_s += alpha
            * (raw_pitch_rate - estimator->track_pitch_rate_rad_s);
    }
    estimator->track_yaw_rad = track_yaw;
    estimator->track_pitch_rad = track_pitch;
    estimator->previous_track_yaw_rad = track_yaw;
    estimator->previous_track_pitch_rad = track_pitch;
}

void ballistic_estimator_init(ballistic_estimator_t *estimator)
{
    if (estimator != NULL) memset(estimator, 0, sizeof(*estimator));
}

void ballistic_estimator_reset(ballistic_estimator_t *estimator,
                               const quaternion_t *attitude)
{
    if (estimator == NULL || attitude == NULL) return;
    memset(estimator, 0, sizeof(*estimator));
    float rotation[3][3];
    quaternion_to_body_to_nav(attitude, rotation);
    update_face_direction(estimator, rotation);
    memcpy(estimator->velocity_dir_nav, estimator->face_dir_nav,
           sizeof(estimator->velocity_dir_nav));
    vector_to_yaw_pitch(estimator->velocity_dir_nav,
                        &estimator->velocity_yaw_rad,
                        &estimator->velocity_pitch_rad);
    estimator->initialized = true;
}

void ballistic_estimator_invalidate(ballistic_estimator_t *estimator)
{
    if (estimator == NULL) return;
    estimator->initialized = false;
    estimator->accel_filter_ready = false;
}

bool ballistic_estimator_update(ballistic_estimator_t *estimator,
                                const quaternion_t *attitude,
                                float ax_body,
                                float ay_body,
                                float az_body,
                                float speed_m_s,
                                float dt_s)
{
    if (estimator == NULL || attitude == NULL || !estimator->initialized) return false;
    if (!(dt_s > 0.0f) || dt_s > 0.05f || !isfinite(speed_m_s)) return false;

    float rotation[3][3];
    quaternion_to_body_to_nav(attitude, rotation);
    update_face_direction(estimator, rotation);

    const float accel_body[3] = {ax_body, ay_body, az_body};
    mat3_mul_vec3(rotation, accel_body, estimator->accel_nav);
    estimator->linear_accel_nav[0] = estimator->accel_nav[0];
    estimator->linear_accel_nav[1] = estimator->accel_nav[1];
    estimator->linear_accel_nav[2] = estimator->accel_nav[2]
                                   - DART_BALLISTIC_GRAVITY_MPS2;

    const float accel_alpha = clampf(DART_BALLISTIC_ACCEL_LPF_ALPHA, 0.0f, 1.0f);
    if (!estimator->accel_filter_ready) {
        memcpy(estimator->filtered_linear_accel_nav,
               estimator->linear_accel_nav,
               sizeof(estimator->filtered_linear_accel_nav));
        estimator->accel_filter_ready = true;
    } else {
        for (int i = 0; i < 3; ++i) {
            estimator->filtered_linear_accel_nav[i] += accel_alpha
                * (estimator->linear_accel_nav[i]
                   - estimator->filtered_linear_accel_nav[i]);
        }
    }

    const float speed = fmaxf(speed_m_s, DART_BALLISTIC_MIN_SPEED_MPS);
    const float *accel = estimator->filtered_linear_accel_nav;
    const float parallel = accel[0] * estimator->velocity_dir_nav[0]
                         + accel[1] * estimator->velocity_dir_nav[1]
                         + accel[2] * estimator->velocity_dir_nav[2];
    float perpendicular[3] = {
        accel[0] - parallel * estimator->velocity_dir_nav[0],
        accel[1] - parallel * estimator->velocity_dir_nav[1],
        accel[2] - parallel * estimator->velocity_dir_nav[2]
    };
    const float perpendicular_norm = sqrtf(perpendicular[0] * perpendicular[0]
                                          + perpendicular[1] * perpendicular[1]
                                          + perpendicular[2] * perpendicular[2]);
    if (perpendicular_norm > DART_BALLISTIC_MAX_LATERAL_ACCEL_MPS2) {
        const float scale = DART_BALLISTIC_MAX_LATERAL_ACCEL_MPS2 / perpendicular_norm;
        perpendicular[0] *= scale;
        perpendicular[1] *= scale;
        perpendicular[2] *= scale;
    }

    for (int i = 0; i < 3; ++i) {
        estimator->velocity_dir_nav[i] += perpendicular[i] / speed * dt_s;
    }
    if (!normalize3(estimator->velocity_dir_nav)) {
        ballistic_estimator_reset(estimator, attitude);
        return false;
    }
    vector_to_yaw_pitch(estimator->velocity_dir_nav,
                        &estimator->velocity_yaw_rad,
                        &estimator->velocity_pitch_rad);
    update_track_angles(estimator, rotation, dt_s);
    return true;
}
