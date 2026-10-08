#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PHASE_IDLE = 0,
    PHASE_ARMED,
    PHASE_LAUNCH,
    PHASE_STABILIZE,
    PHASE_ACQUIRE,
    PHASE_GUIDE,
    PHASE_COAST,
    PHASE_FAILSAFE,
    PHASE_END
} flight_phase_t;

typedef struct {
    flight_phase_t phase;
    flight_phase_t previous_phase;
    float flight_time_s;
    float phase_time_s;
    float launch_confirm_s;
    uint8_t consecutive_valid_frames;
    bool impact_detected;
    bool initialized;
} flight_state_t;

void flight_state_init(flight_state_t *fs);
void flight_state_arm(flight_state_t *fs);
void flight_state_update(flight_state_t *fs, float accel_mag, float dt);
void flight_state_vision_event(flight_state_t *fs,
                               bool new_frame,
                               bool target_valid,
                               uint8_t confidence,
                               uint32_t frame_age_ms);
flight_phase_t flight_state_get_phase(const flight_state_t *fs);
bool flight_state_is_flying(const flight_state_t *fs);
const char *flight_phase_name(flight_phase_t phase);

#endif
