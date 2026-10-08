/**
 * @file    imu.h
 * @brief   IMU 驱动 — ICM-45686（SPI1，模式 3，PA4 片选）
 *
 *          参考: Guided_DartV1.0/component/imu.c
 *          芯片: ICM-45686, WHO_AM_I = 0xE9
 *          陀螺: ±1000dps, ODR 1600Hz
 *          加计: ±4g, ODR 1600Hz
 */

#ifndef IMU_H__
#define IMU_H__

#include <stdbool.h>

typedef struct {
    float gx, gy, gz;      /* 陀螺角速度 (rad/s) */
    float ax, ay, az;      /* 加速度 (m/s^2) */
    float temp_c;          /* 温度 (°C) */
    bool  data_ready;      /* 数据有效 */
} imu_data_t;

void imu_init(void);
bool imu_read(imu_data_t *out);
void imu_calibrate_gyro_bias(void);
void imu_get_gyro_bias(float *bx, float *by, float *bz);

#endif /* IMU_H__ */
