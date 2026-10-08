#include "dart_control.h"
#include "dart_config.h"
#include "ahrs.h"
#include "imu.h"
#include "rate_control.h"
#include "ballistic_estimator.h"
#include "control_allocation.h"
#include "servo.h"
#include "flight_state.h"
#include "guidance.h"
#include "vision_link.h"
#include "lora_telemetry.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#include <string.h>

static ahrs_t ahrs;
static rate_controller_t rate_controller;
static control_allocator_t allocator;
static flight_state_t flight_state;
static guidance_t guidance;
static ballistic_estimator_t ballistic_estimator;
static dart_data_t data;
static guidance_output_t last_guidance;

/* 捕获时刻锁存 (本次飞行只记一次, GUIDE/COAST 往返不重记)。
 * 用于回填 DART_GUIDANCE_NEAR_TGO_S / FAR_TGO_S 和 ca_get_T_go_weights 阈值。
 * 两个锁存分开: TA 对应该帧制导律实际用的 t_go, DF 记录第一个真正量到
 * 距离的帧 —— 捕获首帧目标太小, 距离通常是不可用的。 */
static bool tgo_acquire_latched;
static bool dist_first_latched;

static volatile uint8_t control_pending;
static volatile uint32_t overrun_count;
static bool ready;
static dart_status_t status;
static uint32_t last_vision_generation;
static uint16_t last_capture_time_ms;
static bool have_capture_time;
static bool have_valid_target;
static uint32_t last_valid_target_tick_ms;
static uint16_t last_vision_fps_capture_ms;
static uint32_t last_vision_fps_rx_tick_ms;
static bool have_vision_fps_sample;

static void update_vision_fps(const vision_frame_t *vision,
                              bool new_frame)
{
  const uint32_t now = HAL_GetTick();

  if (new_frame && vision != NULL) {
    if (have_vision_fps_sample) {
      /* uint16_t 相减天然支持视觉时间戳回绕。 */
      const uint16_t delta_ms = (uint16_t)(vision->capture_time_ms
                                         - last_vision_fps_capture_ms);
      if (delta_ms >= 5U && delta_ms <= DART_VISION_FPS_TIMEOUT_MS) {
        const float instant_fps = 1000.0f / (float)delta_ms;
        if (data.vision_fps <= 0.0f) {
          data.vision_fps = instant_fps;
        } else {
          data.vision_fps += DART_VISION_FPS_LPF_ALPHA
                           * (instant_fps - data.vision_fps);
        }
      }
    }
    last_vision_fps_capture_ms = vision->capture_time_ms;
    last_vision_fps_rx_tick_ms = now;
    have_vision_fps_sample = true;
  } else if (have_vision_fps_sample
             && now - last_vision_fps_rx_tick_ms
                    > DART_VISION_FPS_TIMEOUT_MS) {
    data.vision_fps = 0.0f;
  }
}

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

static float estimate_speed(float time_s)
{
#if DART_SPEED_DECAY_ENABLE
    return fmaxf(DART_V_MIN_MPS,
                 DART_V_LAUNCH_MPS * expf(-time_s / DART_V_TAU_S));
#else
    (void)time_s;
    return DART_V_LAUNCH_MPS;
#endif
}

static void map_sensor_to_body(const imu_data_t *imu,
                               float *p, float *q, float *r,
                               float *ax, float *ay, float *az)
{
    *p = DART_BODY_P_FROM_GX * imu->gx
       + DART_BODY_P_FROM_GY * imu->gy
       + DART_BODY_P_FROM_GZ * imu->gz;
    *q = DART_BODY_Q_FROM_GX * imu->gx
       + DART_BODY_Q_FROM_GY * imu->gy
       + DART_BODY_Q_FROM_GZ * imu->gz;
    *r = DART_BODY_R_FROM_GX * imu->gx
       + DART_BODY_R_FROM_GY * imu->gy
       + DART_BODY_R_FROM_GZ * imu->gz;
    *ax = DART_BODY_P_FROM_GX * imu->ax
        + DART_BODY_P_FROM_GY * imu->ay
        + DART_BODY_P_FROM_GZ * imu->az;
    *ay = DART_BODY_Q_FROM_GX * imu->ax
        + DART_BODY_Q_FROM_GY * imu->ay
        + DART_BODY_Q_FROM_GZ * imu->az;
    *az = DART_BODY_R_FROM_GX * imu->ax
        + DART_BODY_R_FROM_GY * imu->ay
        + DART_BODY_R_FROM_GZ * imu->az;
}

static float acceleration_correction_weight(float accel_mag,
                                            flight_phase_t phase)
{
    const float accel_g = accel_mag / 9.80665f;
    if (accel_g < DART_AHRS_ACCEL_MIN_G || accel_g > DART_AHRS_ACCEL_MAX_G) {
        return 0.0f;
    }
    if (phase == PHASE_IDLE || phase == PHASE_ARMED) return 1.0f;
    if (phase == PHASE_LAUNCH) return 0.0f;
    return DART_AHRS_INFLIGHT_ACCEL_WEIGHT;
}

static void capture_flight_reference(void)
{
    data.roll_ref = data.roll;
    data.pitch_ref = data.pitch;
    data.yaw_ref = data.yaw;
    data.ref_valid = true;
}

static void stabilization_command(float *p_cmd, float *q_cmd, float *r_cmd)
{
    *p_cmd = clampf(-1.2f * data.flight_roll, -2.0f, 2.0f);
    *q_cmd = 0.0f;
    *r_cmd = clampf(-0.35f * data.r, -1.0f, 1.0f);
}

dart_status_t dart_control_init(void)
{
    memset(&data, 0, sizeof(data));
    memset(&last_guidance, 0, sizeof(last_guidance));
    servo_init();
    imu_init();
    imu_calibrate_gyro_bias();
    ahrs_init(&ahrs, DART_AHRS_KP_GROUND, DART_AHRS_KI_GROUND,
              DART_CONTROL_DT);
    ballistic_estimator_init(&ballistic_estimator);

    /* 解锁前先在台架上等待姿态估计稳定。 */
    for (uint32_t i = 0; i < 300U; ++i) {
        imu_data_t imu;
        if (imu_read(&imu)) {
            float p, q, r, ax, ay, az;
            map_sensor_to_body(&imu, &p, &q, &r, &ax, &ay, &az);
            ahrs_update_weighted(&ahrs, p, q, r, ax, ay, az, 1.0f);
        }
        HAL_Delay(1U);
    }

    rate_control_init(&rate_controller, DART_CONTROL_DT,
                      DART_RATE_KP_P, DART_RATE_KP_Q, DART_RATE_KP_R,
                      DART_RATE_KD_P, DART_RATE_KD_Q, DART_RATE_KD_R);
    rate_control_set_limit(&rate_controller, DART_MOMENT_LIMIT);
    ca_init(&allocator, DART_CONTROL_DT);
    guidance_init(&guidance);
    flight_state_init(&flight_state);
    flight_state_arm(&flight_state);
    vision_link_init();

    control_pending = 0U;
    overrun_count = 0U;
    last_vision_generation = 0U;
    have_capture_time = false;
    have_valid_target = false;
    last_valid_target_tick_ms = 0U;
    last_vision_fps_capture_ms = 0U;
    last_vision_fps_rx_tick_ms = 0U;
    have_vision_fps_sample = false;
    ready = true;
    status = DART_OK;
    HAL_TIM_Base_Start_IT(&htim2);
    return status;
}

void dart_control_timer_isr(void)
{
    if (!ready) return;
    if (control_pending != 0U) {
        overrun_count++;
        status = DART_ERR_CONTROL_OVERRUN;
    } else {
        control_pending = 1U;
    }
}

void dart_control_tick(void)
{
    dart_control_timer_isr();
}

/* ── IMU 读取失败时的降级输出 ──
 *
 * 不复位舵面。原实现调用 servo_all_center() 后直接 return, 等于在整个飞行
 * 期间只要 SPI 偶发失败一拍, 就同时丢掉滚转姿态保持和全部阻尼。
 *
 * 改为: **俯仰/偏航力矩强制置零, 只保留滚转姿态保持。**
 *
 * 依据是数据陈旧时两类力矩的退化方向完全不同:
 *   - 横向 (俯仰/偏航): 角速度冻结后, 反馈项 Kq·(0−q) 不再"阻尼", 而是
 *     退化成一个**常值力矩**, 而常值横向力矩会持续偏转弹道 —— 最坏情况
 *     是把镖压向地面。
 *   - 滚转: 同样退化成常值力矩, 但滚转力矩绕纵轴, **不改变弹道方向**,
 *     只让弹体自转。而滚转姿态又是俯仰/偏航舵面作用方向的前提, 值得用
 *     一拍前的数据维持。
 *
 * 所以宁可让镖"滚着飞" —— 那只是自转; 不能让它"带着错误的横向力矩飞"。
 * 未离架 (IDLE/ARMED/LAUNCH) 和已结束 (END) 阶段照旧回中: 这些阶段本就
 * 要求舵面中立, 不该施加载荷, 更不该对抗发射机构。 */
static void imu_fallback_output(void)
{
    switch (flight_state.phase) {
    case PHASE_IDLE:
    case PHASE_ARMED:
    case PHASE_LAUNCH:
    case PHASE_END:
        servo_all_center();
        return;
    default:
        break;
    }

    /* 复用 guidance_failsafe 的滚转律 —— 与 PHASE_FAILSAFE 同一套, 保证
     * "只保留滚转"在全工程只有一种定义。写入局部变量而不是 last_guidance:
     * IMU 恢复后制导状态 (held_q_cmd/held_r_cmd/t_go_s) 必须能接着用。 */
    guidance_output_t fallback;
    guidance_failsafe(data.flight_roll, &fallback);

    float mx, my, mz;
    rate_control_update(&rate_controller,
                        fallback.p_cmd, 0.0f, 0.0f,
                        data.p, data.q, data.r,
                        &mx, &my, &mz);

    /* 本策略的核心, 理由见上方注释。 */
    my = 0.0f;
    mz = 0.0f;

    /* t_go 沿用上一帧制导值: 它只影响滚转力矩在四片舵面间的分配
     * (ca_get_T_go_weights), 不影响弹道。 */
    const float tgo_s = last_guidance.t_go_s > 0.0f
                      ? last_guidance.t_go_s : 1.5f;

    float delta[CA_NUM_SERVOS];
    bool saturated;
    ca_allocate(&allocator, mx, my, mz, data.V_est, tgo_s, delta, &saturated);
    servo_set_all_delta(delta);
    (void)saturated;
}

void dart_control_process(void)
{
    /* 接收回调只置位重启请求，DMA 重启在主循环完成。 */
    vision_link_poll();
    if (!ready || control_pending == 0U) return;
    __disable_irq();
    control_pending = 0U;
    __enable_irq();

    imu_data_t imu;
    if (!imu_read(&imu)) {
        /* 单次 SPI 失败最常见于发射瞬间的 EMI 毛刺。不复位舵面, 走
         * "零横向力矩 + 保留滚转"的降级输出, 详见 imu_fallback_output()。 */
        status = DART_ERR_IMU_READ;
        imu_fallback_output();
        return;
    }

    map_sensor_to_body(&imu, &data.p, &data.q, &data.r,
                       &data.ax, &data.ay, &data.az);
    data.accel_mag = sqrtf(data.ax * data.ax + data.ay * data.ay
                         + data.az * data.az);

    flight_phase_t phase_before = flight_state.phase;
    flight_state_update(&flight_state, data.accel_mag, DART_CONTROL_DT);
    ahrs_update_weighted(&ahrs, data.p, data.q, data.r,
                         data.ax, data.ay, data.az,
                         acceleration_correction_weight(data.accel_mag,
                                                        flight_state.phase));
    data.roll = ahrs_get_roll(&ahrs);
    data.pitch = ahrs_get_pitch(&ahrs);
    data.yaw = ahrs_get_yaw(&ahrs);

    if (!data.ref_valid && phase_before == PHASE_LAUNCH
        && flight_state.phase == PHASE_STABILIZE) {
        capture_flight_reference();
        rate_control_reset(&rate_controller);
        guidance_reset_target(&guidance);
        ballistic_estimator_reset(&ballistic_estimator, &ahrs.q);
        /* 每次发射重置捕获锁存: -1 表示本次飞行尚未进入制导 */
        tgo_acquire_latched = false;
        dist_first_latched = false;
        data.tgo_acquire_s = -1.0f;
        data.dist_first_dm = 0U;
        data.conf_max = 0U;
    }
    if (data.ref_valid) {
        data.flight_roll = wrap_pi(data.roll - data.roll_ref);
        data.flight_pitch = wrap_pi(data.pitch - data.pitch_ref);
        data.flight_yaw = wrap_pi(data.yaw - data.yaw_ref);
    }

    vision_frame_t vision;
    uint32_t vision_age_ms = UINT32_MAX;
    uint32_t vision_generation = last_vision_generation;
    const bool have_vision = vision_link_snapshot(&vision, &vision_age_ms,
                                                   &vision_generation);
    const bool new_frame = have_vision
                        && vision_generation != last_vision_generation;
    if (new_frame) last_vision_generation = vision_generation;
    update_vision_fps(&vision, new_frame);
    const bool target_valid = have_vision && vision_frame_is_valid(&vision);
    if (new_frame && target_valid
        && vision.confidence >= DART_VISION_CONF_HOLD) {
        last_valid_target_tick_ms = HAL_GetTick();
        have_valid_target = true;
    }
    const uint32_t target_age_ms = have_valid_target
                                 ? HAL_GetTick() - last_valid_target_tick_ms
                                 : UINT32_MAX;
    flight_state_vision_event(&flight_state, new_frame, target_valid,
                              have_vision ? vision.confidence : 0U,
                              target_age_ms);

    float frame_dt_s = 0.020f;
    if (new_frame) {
        if (have_capture_time) {
            const uint16_t delta_ms = (uint16_t)(vision.capture_time_ms
                                               - last_capture_time_ms);
            if (delta_ms >= 5U && delta_ms <= 100U) frame_dt_s = delta_ms * 0.001f;
        }
        last_capture_time_ms = vision.capture_time_ms;
        have_capture_time = true;
    }

    data.prev_phase = data.phase;
    data.phase = flight_state.phase;
    data.impact_detected = flight_state.impact_detected;
    data.flight_time_s = flight_state.flight_time_s;
    data.V_est = estimate_speed(data.flight_time_s);

#if DART_BALLISTIC_ESTIMATOR_ENABLE
    if (flight_state_is_flying(&flight_state)) {
        if (!ballistic_estimator.initialized) {
            ballistic_estimator_reset(&ballistic_estimator, &ahrs.q);
        }
        data.ballistic_valid = ballistic_estimator_update(
            &ballistic_estimator, &ahrs.q,
            data.ax, data.ay, data.az,
            data.V_est, DART_CONTROL_DT);
        data.velocity_yaw_rad = ballistic_estimator.velocity_yaw_rad;
        data.velocity_pitch_rad = ballistic_estimator.velocity_pitch_rad;
        data.track_yaw_rad = ballistic_estimator.track_yaw_rad;
        data.track_pitch_rad = ballistic_estimator.track_pitch_rad;
        data.track_yaw_rate_rad_s = ballistic_estimator.track_yaw_rate_rad_s;
        data.track_pitch_rate_rad_s = ballistic_estimator.track_pitch_rate_rad_s;
    } else {
        data.ballistic_valid = false;
    }
#else
    data.ballistic_valid = false;
#endif

    float p_cmd = 0.0f, q_cmd = 0.0f, r_cmd = 0.0f;
    float tgo_s = last_guidance.t_go_s > 0.0f ? last_guidance.t_go_s : 1.5f;
    bool guidance_fresh = false;   /* 本周期是否刚重新解算过制导 */
    switch (flight_state.phase) {
    case PHASE_IDLE:
    case PHASE_ARMED:
        servo_all_center();
        return;
    case PHASE_LAUNCH:
        /* 发射阶段保持舵面中立，避免对抗发射机构。 */
        servo_all_center();
        return;
    case PHASE_STABILIZE:
    case PHASE_ACQUIRE:
        stabilization_command(&p_cmd, &q_cmd, &r_cmd);
        break;
    case PHASE_GUIDE:
        if (new_frame && target_valid) {
            (void)guidance_update(&guidance, &vision,
                                  data.flight_roll, data.p, /* 滚转角和滚转角速度 */
                                  data.p, data.q, data.r,
                                  data.V_est, data.flight_time_s,
                                  frame_dt_s, vision_age_ms,
                                  &last_guidance);
            guidance_fresh = true;
        } else {
            guidance_hold(&guidance, data.flight_roll, target_age_ms,
                          &last_guidance);
        }
        p_cmd = last_guidance.p_cmd;
        q_cmd = last_guidance.q_cmd;
        r_cmd = last_guidance.r_cmd;
        tgo_s = last_guidance.t_go_s;
        break;
    case PHASE_COAST:
        guidance_hold(&guidance, data.flight_roll, target_age_ms,
                      &last_guidance);
        p_cmd = last_guidance.p_cmd;
        q_cmd = last_guidance.q_cmd;
        r_cmd = last_guidance.r_cmd;
        tgo_s = last_guidance.t_go_s;
        break;
    case PHASE_FAILSAFE:
        guidance_failsafe(data.flight_roll, &last_guidance);
        p_cmd = last_guidance.p_cmd;
        break;
    case PHASE_END:
    default:
        servo_all_center();
        return;
    }

    data.tgo_s = tgo_s;
    data.los_yaw = last_guidance.los_yaw;
    data.los_pitch = last_guidance.los_pitch;
    data.los_yaw_rate = last_guidance.los_yaw_rate;
    data.los_pitch_rate = last_guidance.los_pitch_rate;
    /* 捕获瞬间锁存: 只有刚跑过 guidance_update 的周期值才是新鲜的,
     * 保证记下的就是制导窗口起点, 而不是 COAST 里的保持值。 */
    if (guidance_fresh && !tgo_acquire_latched) {
        data.tgo_acquire_s = tgo_s;
        tgo_acquire_latched = true;
    }
    /* 首个有效距离独立锁存: 用来反查真实捕获距离 (预期 ≈12.5 m)。
     * 不要求 guidance_fresh —— 距离可用与否由视觉端决定。 */
    if (!dist_first_latched && new_frame && vision.distance_dm > 0U) {
        data.dist_first_dm = vision.distance_dm;
        dist_first_latched = true;
    }
    /* 置信度峰值: 定 CONF_STRONG / CONF_REACQUIRE 的唯一实测依据。
     * 只统计有效帧 —— 无效帧的 confidence 无意义。 */
    if (target_valid && vision.confidence > data.conf_max) {
        data.conf_max = vision.confidence;
    }

    float mx, my, mz;
    rate_control_update(&rate_controller,
                        p_cmd, q_cmd, r_cmd,
                        data.p, data.q, data.r,
                        &mx, &my, &mz);

    if (flight_state.phase == PHASE_STABILIZE
    || flight_state.phase == PHASE_ACQUIRE) {
    my = 0.0f;
    }
    
    float delta[CA_NUM_SERVOS];
    bool saturated;
    ca_allocate(&allocator, mx, my, mz, data.V_est, tgo_s,
                delta, &saturated);
    servo_set_all_delta(delta);
    lora_telemetry_send();
    (void)saturated;
}

uint8_t dart_get_flight_phase(void)
{
    return (uint8_t)flight_state.phase;
}

dart_status_t dart_get_status(void)
{
    return status;
}

void dart_emergency_stop(void)
{
    ready = false;
    HAL_TIM_Base_Stop_IT(&htim2);
    servo_all_center();
}

void dart_set_phase(uint8_t phase)
{
    if (phase > (uint8_t)PHASE_END) return;
    flight_state.previous_phase = flight_state.phase;
    flight_state.phase = (flight_phase_t)phase;
    flight_state.phase_time_s = 0.0f;
    if (flight_state.phase == PHASE_STABILIZE && !data.ref_valid) {
        capture_flight_reference();
    }
}

const dart_data_t *dart_data_get(void)
{
    return &data;
}
