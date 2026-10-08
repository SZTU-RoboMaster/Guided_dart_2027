#include "flight_state.h"
#include "dart_config.h"
#include <string.h>

static void set_phase(flight_state_t *fs, flight_phase_t phase)
{
    if (fs->phase == phase) return;
    fs->previous_phase = fs->phase;
    fs->phase = phase;
    fs->phase_time_s = 0.0f;
    if (phase == PHASE_FAILSAFE) {
        fs->consecutive_valid_frames = 0U;
    }
}

void flight_state_init(flight_state_t *fs)
{
    memset(fs, 0, sizeof(*fs));
    fs->phase = PHASE_IDLE;
    fs->previous_phase = PHASE_IDLE;
    fs->initialized = true;
}

void flight_state_arm(flight_state_t *fs)
{
    if (!fs->initialized) return;
    fs->flight_time_s = 0.0f;
    fs->impact_detected = false;
    fs->launch_confirm_s = 0.0f;
    set_phase(fs, PHASE_ARMED);
}

void flight_state_update(flight_state_t *fs, float accel_mag, float dt)
{
    if (!fs->initialized) return;
    fs->phase_time_s += dt;
    if (flight_state_is_flying(fs)) fs->flight_time_s += dt;

    if (flight_state_is_flying(fs)
        && fs->flight_time_s > DART_MAX_FLIGHT_TIME_S) {
        set_phase(fs, PHASE_END);
        return;
    }
    if (flight_state_is_flying(fs)
        && fs->flight_time_s > 2.0f
        && accel_mag > DART_IMPACT_ACCEL_MPS2) {
        fs->impact_detected = true;
        set_phase(fs, PHASE_END);
        return;
    }

    switch (fs->phase) {
    case PHASE_ARMED:
        if (accel_mag >= DART_LAUNCH_ACCEL_MPS2) {
            fs->launch_confirm_s += dt;
            if (fs->launch_confirm_s >= DART_LAUNCH_CONFIRM_S) {
                set_phase(fs, PHASE_LAUNCH);
            }
        } else {
            fs->launch_confirm_s = 0.0f;
        }
        break;
    case PHASE_LAUNCH:
        if (fs->phase_time_s >= DART_LAUNCH_HOLDOFF_S) {
            set_phase(fs, PHASE_STABILIZE);
        }
        break;
    default:
        break;
    }
}

void flight_state_vision_event(flight_state_t *fs,
                               bool new_frame,
                               bool target_valid,
                               uint8_t confidence,
                               uint32_t frame_age_ms)
{
    if (!fs->initialized) return;
    const bool acquire_quality = target_valid
                              && confidence >= DART_VISION_CONF_ACQUIRE;
    const bool hold_quality = target_valid
                           && confidence >= DART_VISION_CONF_HOLD;

    switch (fs->phase) {
    case PHASE_STABILIZE:
        if (new_frame && acquire_quality) {
            set_phase(fs, PHASE_ACQUIRE);
            fs->consecutive_valid_frames = 1U;
        }
        break;
    case PHASE_ACQUIRE:
        if (new_frame) {
            if (acquire_quality) {
                if (++fs->consecutive_valid_frames >= DART_VISION_LOCK_FRAMES) {
                    set_phase(fs, PHASE_GUIDE);
                }
            } else {
                fs->consecutive_valid_frames = 0U;
            }
        }
        if (frame_age_ms > DART_VISION_FAILSAFE_MS) set_phase(fs, PHASE_FAILSAFE);
        break;
    case PHASE_GUIDE:
        if (!hold_quality || frame_age_ms > DART_VISION_FRESH_MS) {
            set_phase(fs, PHASE_COAST);
        }
        break;
    case PHASE_COAST:
        if (new_frame && acquire_quality) {
            set_phase(fs, PHASE_GUIDE);
        } else if (frame_age_ms > DART_VISION_FAILSAFE_MS) {
            set_phase(fs, PHASE_FAILSAFE);
        }
        break;
    case PHASE_FAILSAFE:
        if (new_frame && acquire_quality) {
            set_phase(fs, PHASE_ACQUIRE);
            fs->consecutive_valid_frames = 1U;
        }
        break;
    default:
        break;
    }
}

flight_phase_t flight_state_get_phase(const flight_state_t *fs)
{
    return fs->phase;
}

bool flight_state_is_flying(const flight_state_t *fs)
{
    return fs->phase >= PHASE_LAUNCH && fs->phase < PHASE_END;
}

const char *flight_phase_name(flight_phase_t phase)
{
    static const char *names[] = {
        "IDLE", "ARMED", "LAUNCH", "STABILIZE", "ACQUIRE",
        "GUIDE", "COAST", "FAILSAFE", "END"
    };
    return (phase <= PHASE_END) ? names[phase] : "?";
}
