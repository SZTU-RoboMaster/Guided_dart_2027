#ifndef DART_CONTROL_H
#define DART_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "dart_data.h"

typedef enum {
    DART_OK = 0,
    DART_ERR_IMU_READ,
    DART_ERR_CONTROL_OVERRUN
} dart_status_t;



dart_status_t dart_control_init(void);

/* TIM2 中断服务入口：仅执行固定耗时的标志更新。 */
void dart_control_timer_isr(void);

/* 在 main() 中持续调用；每次执行一个待处理的 200 Hz 控制周期。 */
void dart_control_process(void);

/* 为旧版 main.c 保留的兼容封装；当前仅设置待处理标志。 */
void dart_control_tick(void);

uint8_t dart_get_flight_phase(void);
dart_status_t dart_get_status(void);
void dart_emergency_stop(void);
void dart_set_phase(uint8_t phase);
const dart_data_t *dart_data_get(void);

#endif
