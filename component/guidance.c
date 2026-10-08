#include "guidance.h"
#include "dart_config.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/* 弱追踪项权重: 随 T_go 增大而增大。远距保留以把目标拉回视场中心,
 * 末端归零让位给纯比例导引, 避免追踪法在末端产生过载陡增。 */
static float capture_weight(float tgo)
{
    const float span = DART_GUIDANCE_FAR_TGO_S - DART_GUIDANCE_NEAR_TGO_S;
    const float t = (span > 1.0e-3f)
                  ? (tgo - DART_GUIDANCE_NEAR_TGO_S) / span : 1.0f;
    return clampf(t, 0.0f, 1.0f);
}

static float fallback_tgo(float speed, float flight_time)
{
    const float tau = DART_V_TAU_S;
    const float travelled = DART_V_LAUNCH_MPS * tau
                          * (1.0f - expf(-flight_time / tau));
    const float remaining = fmaxf(0.0f, DART_TARGET_RANGE_M - travelled);
    return clampf(remaining / fmaxf(speed, 1.0f), 0.20f, 3.0f);
}

static float estimate_tgo(guidance_t *g, const vision_frame_t *f,
                          float speed, float flight_time, float dt)
{
    float radius = fmaxf((float)f->radius_px, 1.0f);
    if (!g->target_initialized) {
        g->radius_est = radius;
        g->radius_rate = 0.0f;
    } else {
        const float raw_rate = (radius - g->last_radius) / fmaxf(dt, 0.005f);
        g->radius_est += 0.25f * (radius - g->radius_est);
        g->radius_rate += 0.18f * (raw_rate - g->radius_rate);
    }
    g->last_radius = radius;

    float tgo = fallback_tgo(speed, flight_time);
    if (f->distance_dm > 0U && f->distance_dm < 1000U) {
        tgo = ((float)f->distance_dm * 0.1f) / fmaxf(speed, 1.0f);
    } else if (g->radius_rate > 1.0f && g->radius_est > 2.0f) {
        const float optical_tgo = g->radius_est / g->radius_rate;
        if (optical_tgo >= 0.20f && optical_tgo <= 3.0f) {
            tgo = 0.65f * g->last_tgo_s + 0.35f * optical_tgo;
        }
    }
    return clampf(tgo, 0.20f, 3.5f);
}

void guidance_init(guidance_t *g)
{
    memset(g, 0, sizeof(*g));
    ab_filter_init(&g->yaw_filter, DART_GUIDANCE_FILTER_ALPHA,
                   DART_GUIDANCE_FILTER_BETA, 0.02f);
    ab_filter_init(&g->pitch_filter, DART_GUIDANCE_FILTER_ALPHA,
                   DART_GUIDANCE_FILTER_BETA, 0.02f);
    ab_filter_set_limits(&g->yaw_filter, 0.75f, 3.0f);
    ab_filter_set_limits(&g->pitch_filter, 0.75f, 3.0f);
    /* λ̈ 改为视觉 λ̇ 帧间差分, 不再用 γ 通道 (gamma=0 退化为 α-β)。 */
    ab_filter_set_gamma(&g->yaw_filter, DART_GUIDANCE_FILTER_GAMMA);
    ab_filter_set_gamma(&g->pitch_filter, DART_GUIDANCE_FILTER_GAMMA);
    g->last_tgo_s = 1.5f;
    g->initialized = true;
}

void guidance_reset_target(guidance_t *g)
{
    ab_filter_reset(&g->yaw_filter);
    ab_filter_reset(&g->pitch_filter);
    g->radius_est = 0.0f;
    g->radius_rate = 0.0f;
    g->last_radius = 0.0f;
    g->held_q_cmd = 0.0f;
    g->held_r_cmd = 0.0f;
    g->yaw_los_integral = 0.0f;
    g->pitch_los_integral = 0.0f;
    g->yaw_rho = DART_GUIDANCE_ISMCG_RHO_MIN;
    g->pitch_rho = DART_GUIDANCE_ISMCG_RHO_MIN;
    g->yaw_lam_d_prev = 0.0f;
    g->pitch_lam_d_prev = 0.0f;
    g->yaw_lam_dd = 0.0f;
    g->pitch_lam_dd = 0.0f;
    g->target_initialized = false;
}

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
                     guidance_output_t *out)
{
    if (g == NULL || frame == NULL || out == NULL || !g->initialized
        || !vision_frame_is_valid(frame)) return false;

    memset(out, 0, sizeof(*out));
    const float dt = clampf(frame_dt_s, 0.008f, 0.080f);
    const float total_delay = DART_CAMERA_PIPELINE_DELAY_S
                            + DART_SERVO_DELAY_S
                            + (float)frame_age_ms * 0.001f;
    float roll_predict = roll_rad;
#if DART_ROLL_PHASE_LEAD_ENABLE
    roll_predict += roll_rate_rad_s * total_delay;
#endif

    /* 图像采集时刻的归一化图像坐标及其变化率。 */
    const float x = ((float)frame->cx - DART_CAMERA_CX_PX)
                  / DART_CAMERA_FX_PX;
    const float y = ((float)frame->cy - DART_CAMERA_CY_PX)
                  / DART_CAMERA_FY_PX;
    const float xd = (float)frame->vx_px_s / DART_CAMERA_FX_PX;
    const float yd = (float)frame->vy_px_s / DART_CAMERA_FY_PX;

    /* 对 LOS 位置和速度同时进行滚转补偿，转换出相机滚转坐标系。 */
    const float c = cosf(roll_predict);
    const float s = sinf(roll_predict);
    const float xb = c * x - s * y;
    const float yb = s * x + c * y;
    const float xdb = c * xd - s * yd;
    const float ydb = s * xd + c * yd;

    const float yaw_meas = atan2f(xb, 1.0f) + DART_CAMERA_YAW_OFFSET_RAD;
    const float pitch_meas = atan2f(yb, 1.0f) + DART_CAMERA_PITCH_OFFSET_RAD;
    /* 从视觉角速度中扣除镖体自身转动，得到惯性系 LOS 角速度。
     * p/q/r 必须是完成轴映射和陀螺零偏校准后的机体系角速度。 */
    const float yaw_rate_meas = xdb / (1.0f + xb * xb)
                              + r_rad_s - p_rad_s * yb;
    const float pitch_rate_meas = ydb / (1.0f + yb * yb)
                                - q_rad_s + p_rad_s * xb;

    ab_filter_set_dt(&g->yaw_filter, dt);
    ab_filter_set_dt(&g->pitch_filter, dt);
    if (!g->target_initialized) {
        ab_filter_set_state(&g->yaw_filter, yaw_meas, yaw_rate_meas);
        ab_filter_set_state(&g->pitch_filter, pitch_meas, pitch_rate_meas);
        /* λ̈ 差分初值: 首帧 λ̇ 即视觉 λ̇, λ̈ 暂记 0 */
        g->yaw_lam_d_prev = yaw_rate_meas;
        g->pitch_lam_d_prev = pitch_rate_meas;
        g->yaw_lam_dd = 0.0f;
        g->pitch_lam_dd = 0.0f;
    } else {
        /* 低通系数按帧间隔换算, 使滤波带宽不随帧率漂移 (15~60 fps)。 */
        const float rate_alpha = 1.0f
                               - expf(-dt / DART_GUIDANCE_FILTER_RATE_TAU_S);
        const float dd_alpha   = 1.0f
                               - expf(-dt / DART_GUIDANCE_FILTER_DD_TAU_S);

        ab_filter_update(&g->yaw_filter, yaw_meas);
        ab_filter_update(&g->pitch_filter, pitch_meas);
        /* 使用视觉端提供的当前帧像素速度作为变化率观测量。 */
        g->yaw_filter.rate += rate_alpha * (yaw_rate_meas - g->yaw_filter.rate);
        g->pitch_filter.rate += rate_alpha * (pitch_rate_meas - g->pitch_filter.rate);

        /* λ̈ 与 λ̇ 同源: 对融合后 λ̇ 帧间差分, 低通抑制抖动并限幅 */
        const float raw_yaw_dd = (g->yaw_filter.rate - g->yaw_lam_d_prev) / dt;
        const float raw_pitch_dd = (g->pitch_filter.rate - g->pitch_lam_d_prev) / dt;
        g->yaw_lam_dd += dd_alpha * (raw_yaw_dd - g->yaw_lam_dd);
        g->pitch_lam_dd += dd_alpha * (raw_pitch_dd - g->pitch_lam_dd);
        g->yaw_lam_dd = clampf(g->yaw_lam_dd, -DART_GUIDANCE_FILTER_ACC_LIMIT,
                                             DART_GUIDANCE_FILTER_ACC_LIMIT);
        g->pitch_lam_dd = clampf(g->pitch_lam_dd, -DART_GUIDANCE_FILTER_ACC_LIMIT,
                                               DART_GUIDANCE_FILTER_ACC_LIMIT);
        g->yaw_lam_d_prev = g->yaw_filter.rate;
        g->pitch_lam_d_prev = g->pitch_filter.rate;
    }

    const float tgo = estimate_tgo(g, frame, speed_m_s, flight_time_s, dt);

    /* ── 制导指令 (两模式共享主体) ──
     * λ   : 平滑视线角 (est)
     * λ̇   : 视线角速度 (rate)
     * 指令 = 弱追踪项 (远距拉回视场) + N·λ̇ (碰撞航线) + 机动处理项
     * 机动处理项按模式不同:
     *   APN   : K_ff·λ̈       前馈 (依赖 λ̈ 质量)
     *   ISMCG : ρ·sat(s/ε)    鲁棒抑制 (不依赖 λ̈) */
    const float yaw_lam   = g->yaw_filter.est;
    const float yaw_lam_d = g->yaw_filter.rate;
    const float pitch_lam   = g->pitch_filter.est;
    const float pitch_lam_d = g->pitch_filter.rate;

    /* 弱追踪项: 随 T_go 衰减, 远距把目标拉回视场中心, 末端让位给纯比例导引 */
    const float cap_w = capture_weight(tgo);
    const float yaw_capture = DART_GUIDANCE_K_CAPTURE * cap_w
                            * (yaw_lam + yaw_lam_d * total_delay);
    const float pitch_capture = DART_GUIDANCE_K_CAPTURE * cap_w
                              * (pitch_lam + pitch_lam_d * total_delay);

    /* 比例导引主导项: r_cmd ≈ N·λ̇ (镖接近慢速目标时 V_c≈V) */
    const float yaw_pn = DART_GUIDANCE_NAV_RATIO * yaw_lam_d;
    const float pitch_pn = DART_GUIDANCE_NAV_RATIO * pitch_lam_d;

    float yaw_feed;
    float pitch_feed;
#if DART_GUIDANCE_MODE_ISMCG
    /* ── 自适应积分滑模 (ISMCG) ──
     * 滑模面 s = λ̇ + k_i·∫λ̇ dt, 目标把 λ̇ 收敛到零;
     * 积分项让系统起始即在滑模面上, 消除趋近段抖振;
     * 自适应上界 ρ 用 |s| (一阶量) 在线估计, 不引入 λ̈ 噪声;
     * 边界层 sat(s/ε) 替代符号函数, 抑制离散抖振。 */
    g->yaw_los_integral   += yaw_lam_d * dt;
    g->pitch_los_integral += pitch_lam_d * dt;
    g->yaw_los_integral   = clampf(g->yaw_los_integral,
                                   -DART_GUIDANCE_ISMCG_INT_LIMIT,
                                    DART_GUIDANCE_ISMCG_INT_LIMIT);
    g->pitch_los_integral = clampf(g->pitch_los_integral,
                                   -DART_GUIDANCE_ISMCG_INT_LIMIT,
                                    DART_GUIDANCE_ISMCG_INT_LIMIT);
    const float yaw_s   = yaw_lam_d + DART_GUIDANCE_ISMCG_K_I * g->yaw_los_integral;
    const float pitch_s = pitch_lam_d + DART_GUIDANCE_ISMCG_K_I * g->pitch_los_integral;

    g->yaw_rho += (DART_GUIDANCE_ISMCG_GAMMA_RHO * fabsf(yaw_s)
                   - DART_GUIDANCE_ISMCG_DECAY * g->yaw_rho) * dt;
    g->pitch_rho += (DART_GUIDANCE_ISMCG_GAMMA_RHO * fabsf(pitch_s)
                     - DART_GUIDANCE_ISMCG_DECAY * g->pitch_rho) * dt;
    g->yaw_rho   = clampf(g->yaw_rho,
                          DART_GUIDANCE_ISMCG_RHO_MIN, DART_GUIDANCE_ISMCG_RHO_MAX);
    g->pitch_rho = clampf(g->pitch_rho,
                          DART_GUIDANCE_ISMCG_RHO_MIN, DART_GUIDANCE_ISMCG_RHO_MAX);

    yaw_feed   = g->yaw_rho   * clampf(yaw_s   / DART_GUIDANCE_ISMCG_EPS, -1.0f, 1.0f);
    pitch_feed = g->pitch_rho * clampf(pitch_s / DART_GUIDANCE_ISMCG_EPS, -1.0f, 1.0f);
#else
    /* ── 增广比例导引 (APN): 目标机动前馈, λ̈ 由视觉 λ̇ 差分得到 ── */
    const float yaw_lam_dd   = g->yaw_lam_dd;
    const float pitch_lam_dd = g->pitch_lam_dd;
    yaw_feed   = DART_GUIDANCE_K_ACCEL_FEED * yaw_lam_dd;
    pitch_feed = DART_GUIDANCE_K_ACCEL_FEED * pitch_lam_dd;
#endif

    /* 置信度缩放。下限受 DART_GUIDANCE_CONF_SCALE_MIN 约束: 该乘子作用在整条
     * 指令上, 等价于缩放导航比, 过低会使 N 掉出比例导引稳定区间。 */
    const float confidence_scale = clampf(((float)frame->confidence - 35.0f) / 45.0f,
                                          DART_GUIDANCE_CONF_SCALE_MIN, 1.0f);

    out->p_cmd = clampf(-DART_GUIDANCE_ROLL_KP * roll_rad,
                        -DART_GUIDANCE_ROLL_RATE_LIMIT,
                         DART_GUIDANCE_ROLL_RATE_LIMIT);
    out->r_cmd = DART_GUIDANCE_YAW_SIGN * confidence_scale
               * (yaw_capture + yaw_pn + yaw_feed);
    out->q_cmd = DART_GUIDANCE_PITCH_SIGN * confidence_scale
               * (pitch_capture + pitch_pn + pitch_feed)
               /* 俯仰配平偏置: 独立叠加, 不随 T_go 衰减、不被置信度缩放 */
               + DART_GUIDANCE_PITCH_SIGN * DART_GUIDANCE_PITCH_BIAS_RATE;
    out->q_cmd = clampf(out->q_cmd, -DART_GUIDANCE_RATE_LIMIT,
                                      DART_GUIDANCE_RATE_LIMIT);
    out->r_cmd = clampf(out->r_cmd, -DART_GUIDANCE_RATE_LIMIT,
                                      DART_GUIDANCE_RATE_LIMIT);
    out->los_yaw = yaw_lam;
    out->los_pitch = pitch_lam;
    out->los_yaw_rate = yaw_lam_d;
    out->los_pitch_rate = pitch_lam_d;
    out->t_go_s = tgo;
    out->total_delay_s = total_delay;
    out->authority = confidence_scale;

    g->held_q_cmd = out->q_cmd;
    g->held_r_cmd = out->r_cmd;
    g->last_tgo_s = tgo;
    g->last_confidence = frame->confidence;
    g->target_initialized = true;
    return true;
}

void guidance_hold(const guidance_t *g,
                   float roll_rad,
                   uint32_t frame_age_ms,
                   guidance_output_t *out)
{
    memset(out, 0, sizeof(*out));
    out->p_cmd = clampf(-DART_GUIDANCE_ROLL_KP * roll_rad,
                        -DART_GUIDANCE_ROLL_RATE_LIMIT,
                         DART_GUIDANCE_ROLL_RATE_LIMIT);
    float fade = 1.0f;
    if (frame_age_ms > DART_VISION_FRESH_MS) {
        const float span = (float)(DART_VISION_COAST_MS - DART_VISION_FRESH_MS);
        fade = 1.0f - ((float)frame_age_ms - DART_VISION_FRESH_MS) / span;
        fade = clampf(fade, 0.0f, 1.0f);
    }
    out->q_cmd = g->held_q_cmd * fade;
    out->r_cmd = g->held_r_cmd * fade;
    out->los_yaw = g->yaw_filter.est;
    out->los_pitch = g->pitch_filter.est;
    out->los_yaw_rate = g->yaw_filter.rate;
    out->los_pitch_rate = g->pitch_filter.rate;
    /* t_go 在保持期间继续递减, 不能冻结。
     * d(R/V)/dt ≈ -1 (镖以 ≈V 接近近乎静止的目标), 即 t_go 每秒减 1 s。
     * 冻结它会让 cap_w 和 ca_get_T_go_weights 停在过期值上最长达
     * DART_VISION_COAST_MS —— 末端正是这两者对 t_go 最敏感的地方。 */
    out->t_go_s = fmaxf(g->last_tgo_s - (float)frame_age_ms * 0.001f,
                        DART_GUIDANCE_NEAR_TGO_S);
    out->authority = fade;
}

void guidance_failsafe(float roll_rad, guidance_output_t *out)
{
    memset(out, 0, sizeof(*out));
    out->p_cmd = clampf(-0.8f * roll_rad, -1.0f, 1.0f);
    out->t_go_s = 1.0f;
}
