/**
 * @file    control_allocation.c
 * @brief   舵面控制分配器实现
 *
 *          求解: min || Wm^{½} (B_q δ − M_cmd) ||² + λ ||δ||²
 *          方法: 降维伪逆
 *            δ = B_q^T (B_q B_q^T + λI)^{-1} M_cmd
 *
 *          完整步骤:
 *            1. 动压缩放 B → B_q
 *            2. T_go → Wm (控制轴权重)
 *            3. 构造 B̃ = Wm^{½} · B_q
 *            4. M = B̃ B̃^T + λI
 *            5. z = M^{-1} · (Wm^{½} · M_cmd)
 *            6. δ = B̃^T · z
 *            7. 死区逆补偿
 *            8. 位置限幅 + 速率限幅
 * 
 *            滚转补偿
 */

#include "control_allocation.h"
#include "dart_config.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════
 *  3×3 矩阵求逆（高斯-约旦消元法）
 * ═══════════════════════════════════════════════════════════════ */

static bool mat3_inv(const float A[3][3], float A_inv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < 1e-9f) return false;

    float inv_det = 1.0f / det;

    A_inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    A_inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    A_inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    A_inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    A_inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    A_inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
    A_inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    A_inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    A_inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

    return true;
}

/* ═══════════════════════════════════════════════════════════════
 *  死区逆补偿
 * ═══════════════════════════════════════════════════════════════ */

static float deadband_inverse(float delta, float db, float boost)
{
    if (delta > 0.0f) {
        float ratio = fminf(delta / db, 1.0f);
        return delta + boost * ratio;
    } else if (delta < 0.0f) {
        float ratio = fminf(-delta / db, 1.0f);
        return delta - boost * ratio;
    }
    return 0.0f;
}

/* ═══════════════════════════════════════════════════════════════
 *  T_go → 控制轴权重
 * ═══════════════════════════════════════════════════════════════ */

void ca_get_T_go_weights(float T_go, ca_weights_t *w)
{
    if (T_go > 3.0f) {
        /* 远距：保持滚转，温和修正偏航/俯仰 */
        w->w_roll  = 0.8f;
        w->w_pitch = 1.0f;
        w->w_yaw   = 1.0f;
    } else if (T_go > 1.5f) {
        /* 中距: 命中轴权重上升 */
        w->w_roll  = 0.5f;
        w->w_pitch = 1.6f;
        w->w_yaw   = 1.6f;
    } else if (T_go > 0.5f) {
        /* 近距：进一步降低滚转权重 */
        w->w_roll  = 0.3f;
        w->w_pitch = 2.2f;
        w->w_yaw   = 2.2f;
    } else {
        /* 末端：基本不再控制滚转，全力修正偏航/俯仰 */
        w->w_roll  = 0.1f;
        w->w_pitch = 3.0f;
        w->w_yaw   = 3.0f;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  分配器初始化
 * ═══════════════════════════════════════════════════════════════ */

void ca_init(control_allocator_t *ca, float dt)
{
    memset(ca, 0, sizeof(*ca));

    /* ── B 矩阵: X 翼面构型初始值 ── */
    /*
     * 舵面编号约定:
     *   1: 左前翼 (左前象限)
     *   2: 右前翼 (右前象限)
     *   3: 左后翼 (左后象限)
     *   4: 右后翼 (右后象限)
     *
     * 滚转符号：左翼下偏 → 正滚转（右手法则）
     * 俯仰符号：前翼下偏 → 正俯仰
     * 偏航符号：左翼下偏 → 正偏航
     */

    /* 滚转：对角线舵面对 */
    ca->B[0][0] = DART_B_ROLL_1;
    ca->B[0][1] = DART_B_ROLL_2;
    ca->B[0][2] = DART_B_ROLL_3;
    ca->B[0][3] = DART_B_ROLL_4;

    /* 俯仰：前翼同向、后翼反向（力偶产生纯俯仰） */
    ca->B[1][0] = DART_B_PITCH_1;
    ca->B[1][1] = DART_B_PITCH_2;
    ca->B[1][2] = DART_B_PITCH_3;
    ca->B[1][3] = DART_B_PITCH_4;

    /* 偏航：对角翼面配对 (1,4)↔(2,3)，符号取反 */
    ca->B[2][0] = DART_B_YAW_1;
    ca->B[2][1] = DART_B_YAW_2;
    ca->B[2][2] = DART_B_YAW_3;
    ca->B[2][3] = DART_B_YAW_4;

    /* 标称动压 @ DART_Q_NOMINAL_SPEED_MPS, ρ=1.225 kg/m³
     * (15 m/s 时 ≈ 137.8 Pa)。该速度是 config 宏, 不再写死在这里。 */
    ca->q_nominal = 0.5f * 1.225f
                  * DART_Q_NOMINAL_SPEED_MPS * DART_Q_NOMINAL_SPEED_MPS;

    /* 舵面使用代价 (均等) */
    ca->Wu[0] = 0.1f;
    ca->Wu[1] = 0.1f;
    ca->Wu[2] = 0.1f;
    ca->Wu[3] = 0.1f;

    /* 正则化 */
    ca->lambda_reg = 0.01f;

    /* 约束 */
    ca->delta_min  = -DART_SERVO_DELTA_LIMIT;
    ca->delta_max  = +DART_SERVO_DELTA_LIMIT;
    ca->rate_limit = DART_SERVO_RATE_LIMIT_PER_S;

    /* 死区 */
    ca->deadband       = DART_SERVO_DEADBAND;
    ca->deadband_boost = DART_SERVO_DEADBAND_BOOST;

    ca->dt = dt;

    /* 初始舵面位置: 全部中位 */
    for (int j = 0; j < CA_NUM_SERVOS; j++) {
        ca->delta_last[j] = 0.0f;
    }

    ca->initialized = true;
}

void ca_set_B_matrix(control_allocator_t *ca,
                     const float B[CA_NUM_AXES][CA_NUM_SERVOS])
{
    for (int i = 0; i < CA_NUM_AXES; i++) {
        for (int j = 0; j < CA_NUM_SERVOS; j++) {
            ca->B[i][j] = B[i][j];
        }
    }
}

void ca_set_deadband(control_allocator_t *ca,
                     float deadband, float deadband_boost)
{
    ca->deadband       = deadband;
    ca->deadband_boost = deadband_boost;
}

/* ═══════════════════════════════════════════════════════════════
 *  核心分配
 * ═══════════════════════════════════════════════════════════════ */

void ca_allocate(control_allocator_t *ca,
                 float Mx_cmd, float My_cmd, float Mz_cmd,
                 float V_est, float T_go,
                 float delta_out[CA_NUM_SERVOS],
                 bool *saturated)
{
    *saturated = false;

    if (!ca->initialized) {
        for (int j = 0; j < CA_NUM_SERVOS; j++) delta_out[j] = 0.0f;
        return;
    }

    /* ═══ 1. 动压缩放 ═══ */
    float q_est = 0.5f * 1.225f * V_est * V_est;
    float q_scale = q_est / ca->q_nominal;
    if (q_scale < 0.2f) q_scale = 0.2f;     /* 防止低速时增益为 0 */
    if (q_scale > 4.0f) q_scale = 4.0f;     /* 防止高速时指令过小 */

    float B_scaled[CA_NUM_AXES][CA_NUM_SERVOS];
    for (int i = 0; i < CA_NUM_AXES; i++) {
        for (int j = 0; j < CA_NUM_SERVOS; j++) {
            B_scaled[i][j] = ca->B[i][j] * q_scale;
        }
    }

    /* ═══ 2. T_go → Wm ═══ */
    ca_weights_t w;
    ca_get_T_go_weights(T_go, &w);

    float sqrt_wm[CA_NUM_AXES];
    sqrt_wm[0] = sqrtf(w.w_roll);
    sqrt_wm[1] = sqrtf(w.w_pitch);
    sqrt_wm[2] = sqrtf(w.w_yaw);

    /* ═══ 3. 构造 B̃ = Wm^{½} · B_scaled ═══ */
    float B_tilde[CA_NUM_AXES][CA_NUM_SERVOS];
    for (int i = 0; i < CA_NUM_AXES; i++) {
        for (int j = 0; j < CA_NUM_SERVOS; j++) {
            B_tilde[i][j] = sqrt_wm[i] * B_scaled[i][j];
        }
    }

    /* ═══ 4. M = B̃ B̃^T + λI ═══ */
    float M[CA_NUM_AXES][CA_NUM_AXES] = {{0}};
    for (int i = 0; i < CA_NUM_AXES; i++) {
        for (int k = 0; k < CA_NUM_AXES; k++) {
            for (int j = 0; j < CA_NUM_SERVOS; j++) {
                M[i][k] += B_tilde[i][j] * B_tilde[k][j];
            }
        }
    }

    /* 正则化 */
    M[0][0] += ca->lambda_reg;
    M[1][1] += ca->lambda_reg;
    M[2][2] += ca->lambda_reg;

    /* ═══ 5. z = M^{-1} · (Wm^{½} · M_cmd) ═══ */
    float M_inv[CA_NUM_AXES][CA_NUM_AXES];
    if (!mat3_inv(M, M_inv)) {
        /* 矩阵不可逆 → 回中 */
        for (int j = 0; j < CA_NUM_SERVOS; j++) {
            delta_out[j] = 0.0f;
            ca->delta_last[j] = 0.0f;
        }
        return;
    }

    float Mc_weighted[CA_NUM_AXES];
    Mc_weighted[0] = sqrt_wm[0] * Mx_cmd;
    Mc_weighted[1] = sqrt_wm[1] * My_cmd;
    Mc_weighted[2] = sqrt_wm[2] * Mz_cmd;

    float z[CA_NUM_AXES] = {0};
    for (int i = 0; i < CA_NUM_AXES; i++) {
        for (int k = 0; k < CA_NUM_AXES; k++) {
            z[i] += M_inv[i][k] * Mc_weighted[k];
        }
    }

    /* ═══ 6. δ = B̃^T · z ═══ */
    float delta_unc[CA_NUM_SERVOS];
    for (int j = 0; j < CA_NUM_SERVOS; j++) {
        float s = 0.0f;
        for (int i = 0; i < CA_NUM_AXES; i++) {
            s += B_tilde[i][j] * z[i];
        }
        delta_unc[j] = s;
    }

    /* ═══ 7. 死区逆补偿 ═══ */
    for (int j = 0; j < CA_NUM_SERVOS; j++) {
        delta_unc[j] = deadband_inverse(delta_unc[j],
                                        ca->deadband, ca->deadband_boost);
    }

    /* ═══ 8. 位置限幅 + 速率限幅 ═══ */
    for (int j = 0; j < CA_NUM_SERVOS; j++) {
        /* 位置限幅 */
        if (delta_unc[j] > ca->delta_max) {
            delta_unc[j] = ca->delta_max;
            *saturated = true;
        } else if (delta_unc[j] < ca->delta_min) {
            delta_unc[j] = ca->delta_min;
            *saturated = true;
        }

        /* 速率限幅 */
        float max_step = ca->rate_limit * ca->dt;
        float diff = delta_unc[j] - ca->delta_last[j];

        if (diff > max_step) {
            delta_unc[j] = ca->delta_last[j] + max_step;
            *saturated = true;
        } else if (diff < -max_step) {
            delta_unc[j] = ca->delta_last[j] - max_step;
            *saturated = true;
        }
    }

    /* 输出 + 保存状态 */
    for (int j = 0; j < CA_NUM_SERVOS; j++) {
        delta_out[j]       = delta_unc[j];
        ca->delta_last[j] = delta_out[j];
    }
}
