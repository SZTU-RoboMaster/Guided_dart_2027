/**
 * @file    imu.c
 * @brief   ICM-45686 IMU 驱动 (SPI1)
 *
 *          完全对齐 Guided_DartV1.0/component/imu.c 的配置
 *          硬件: SPI1 — PA5(SCK), PA6(MISO), PA7(MOSI), PA4(CS)
 * 
 * ══════════════════════════════════════════════════════════════════
第 1 层: ICM-45686 原始寄存器 (imu.c:249-255)
══════════════════════════════════════════════════════════════════
raw[14]  ──→  raw_ax    int16_t   加计 X 原始 (LSB)
             raw_ay    int16_t   加计 Y 原始
             raw_az    int16_t   加计 Z 原始
             raw_gx    int16_t   陀螺 X 原始 (LSB)
             raw_gy    int16_t   陀螺 Y 原始
             raw_gz    int16_t   陀螺 Z 原始
             raw_t     int16_t   温度原始

常量: GYRO_SCALE  = π/180/32.768    (LSB → rad/s)
      ACCEL_SCALE = 9.81/8192.0     (LSB → m/s²)
      gyro_bias_x/y/z               (校准零偏, 初值 0)

══════════════════════════════════════════════════════════════════
第 2 层: 物理量转换 (imu.c:258-263)
══════════════════════════════════════════════════════════════════
ax = raw_ax × ACCEL_SCALE           → 加计 X (m/s²)
ay = raw_ay × ACCEL_SCALE           → 加计 Y (m/s²)
az = raw_az × ACCEL_SCALE           → 加计 Z (m/s²)
gx = raw_gx × GYRO_SCALE − bias_x   → 陀螺 X (rad/s)
gy = raw_gy × GYRO_SCALE − bias_y   → 陀螺 Y (rad/s)
gz = raw_gz × GYRO_SCALE − bias_z   → 陀螺 Z (rad/s)

══════════════════════════════════════════════════════════════════
第 3 层: 软件低通滤波 (imu.c:273-284) → imu_data_t (imu.c:286-293)
══════════════════════════════════════════════════════════════════
系数: IMU_LPF_ALPHA_GYRO  = 0.30
      IMU_LPF_ALPHA_ACCEL = 0.20

accel_lpf[0] += 0.20×(ax − accel_lpf[0])   →  out->ax
accel_lpf[1] += 0.20×(ay − accel_lpf[1])   →  out->ay
accel_lpf[2] += 0.20×(az − accel_lpf[2])   →  out->az
gyro_lpf[0]  += 0.30×(gx − gyro_lpf[0])    →  out->gx
gyro_lpf[1]  += 0.30×(gy − gyro_lpf[1])    →  out->gy
gyro_lpf[2]  += 0.30×(gz − gyro_lpf[2])    →  out->gz
out->temp_c = 25.0 + raw_t/128.0

══════════════════════════════════════════════════════════════════
第 4 层: dart_control_tick() 取出 (dart_control.c:140-159)
══════════════════════════════════════════════════════════════════
imu_data_t imu
  ├─ gx=imu.gx  gy=imu.gy  gz=imu.gz   (陀螺低通值)
  └─ ax=imu.ax  ay=imu.ay  az=imu.az   (加计低通值)

ahrs_update(&ahrs, gx, gy, gz, ax, ay, az)  ← Mahony 互补滤波
  │  输出:
  ├─ phi   = ahrs_get_roll(&ahrs)    (rad)  滚转角
  ├─ theta = ahrs_get_pitch(&ahrs)   (rad)  俯仰角
  └─ (yaw 无磁力计, 仅积分, 不用于控制)

body rate:
  p = gx    (rad/s)  roll 角速度
  q = gy    (rad/s)  pitch 角速度
  r = gz    (rad/s)  yaw 角速度

accel_mag = √(ax²+ay²+az²)   (m/s²)  发射/撞击检测

══════════════════════════════════════════════════════════════════
第 5 层: att_stabilize() → 角速率指令 (dart_control.c:193)
══════════════════════════════════════════════════════════════════
输入:          增益:
  phi   ──→ K_roll  = 4.0
  theta ──→ K_pitch = 4.0
  p     ──→ Kd_p    = 0.3
  q     ──→ Kd_q    = 0.8
  r     ──→ Kd_r    = 1.5

公式:
  p_cmd = −4.0×phi  − 0.3×p          (限幅 ±8.0)
  q_cmd = −4.0×theta − 0.8×q         (限幅 ±8.0)
  r_cmd = −1.5×r                      (限幅 ±8.0)

══════════════════════════════════════════════════════════════════
第 6 层: rate_control → 归一化力矩 (dart_control.c:210-213)
══════════════════════════════════════════════════════════════════
输入:                      增益:
  p_cmd, p   (期望 vs 实际)  Kp_p=0.3  Kd_p=0.1
  q_cmd, q                   Kp_q=0.8  Kd_q=0.15
  r_cmd, r                   Kp_r=0.8  Kd_r=0.15

公式:
  ep = p_cmd − p,  eq = q_cmd − q,  er = r_cmd − r
  Mx = 0.3×ep + 0.1×d(ep)/dt      (限幅 ±0.8)
  My = 0.8×eq + 0.15×d(eq)/dt     (限幅 ±0.8)
  Mz = 0.8×er + 0.15×d(er)/dt     (限幅 ±0.8)

══════════════════════════════════════════════════════════════════
第 7 层: control_allocation → 舵面偏角 (dart_control.c:228-230)
══════════════════════════════════════════════════════════════════
输入:
  Mx, My, Mz            期望力矩 (归一化, ±0.8)
  V_est = 18×e^(−t/4)   估计速度 (≥8 m/s)
  T_go  = 2.0           固定值 (无视觉)

B 矩阵 (X 翼面):
           Roll  Pitch  Yaw
  舵面1:  −0.5   +0.6   +0.6    (左前翼)
  舵面2:  +0.5   +0.6   −0.6    (右前翼)
  舵面3:  −0.5   −0.6   −0.6    (左后翼)
  舵面4:  +0.5   −0.6   +0.6    (右后翼)

动压缩放: B_q = B × (½ρV² / q_nominal)
加权伪逆: δ = B̃^T(B̃B̃^T+λI)^−1 × M_cmd
死区补偿 + 限幅限速

输出: delta[0..3]  (归一化, −1.0 ~ +1.0)

══════════════════════════════════════════════════════════════════
第 8 层: servo → PWM 脉宽 (dart_control.c:236)
══════════════════════════════════════════════════════════════════
delta[0] → 1500+δ×500 → TIM3_CH1 → PB4 → 舵机1 (左前翼)
delta[1] → 1500+δ×500 → TIM3_CH2 → PB5 → 舵机2 (右前翼)
delta[2] → 1500+δ×500 → TIM4_CH1 → PB6 → 舵机3 (左后翼)
delta[3] → 1500+δ×500 → TIM4_CH2 → PB7 → 舵机4 (右后翼)

    PWM：200Hz，0 → 1500μs（中位），±1.0 → 1000~2000μs

 * 
 */

#include "imu.h"
#include "main.h"
#include "spi.h"
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════
 *  ICM-45686 寄存器（寄存器组 0）
 * ═══════════════════════════════════════════════════════════════ */

#define ICM_REG_ACCEL_DATA_X1     0x00
#define ICM_REG_PWR_MGMT0         0x10
#define ICM_REG_GYRO_CONFIG0      0x1C
#define ICM_REG_ACCEL_CONFIG0     0x1B
#define ICM_REG_DRIVE_CONFIG0     0x32
#define ICM_REG_WHO_AM_I          0x72
#define ICM_REG_INT1_CONFIG0      0x16
#define ICM_REG_INT1_CONFIG2      0x18
#define ICM_REG_INTF_CONFIG1_OVRD 0x2D
#define ICM_REG_MISC2             0x7F
#define ICM_REG_IREG_ADDR_15_8    0x7C
#define ICM_REG_IREG_DATA         0x7E

/* ─── 间接寻址寄存器 (> 0xFF) ─── */
#define ICM_REG_SMC_CONTROL_0         0xA258
#define ICM_REG_IPREG_SYS1_REG_172    0xA4AC
#define ICM_REG_IPREG_SYS2_REG_131    0xA583
#define ICM_REG_SREG_CTRL             0xA267

#define ICM_WHO_AM_I_VAL            0xE9

/* ═══════════════════════════════════════════════════════════════
 *  传感器常量
 * ═══════════════════════════════════════════════════════════════ */

/* 陀螺: ±1000dps → LSB/°/s = 2^16 / 2000 = 32.768 */
#define GYRO_SCALE   ((float)(3.14159265359 / 180.0 / 32.768))

/* 加计: ±4g → LSB/g = 2^16 / 8 = 8192 */
#define ACCEL_SCALE  ((float)(9.81 / 8192.0))

/* 软件低通滤波系数 */
#define IMU_LPF_ALPHA_GYRO   0.33f
#define IMU_LPF_ALPHA_ACCEL  0.28f

/* ═══════════════════════════════════════════════════════════════
 *  内部状态
 * ═══════════════════════════════════════════════════════════════ */

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static bool  icm_data_big_endian = false;
static bool  imu_lpf_ready = false;
static float gyro_lpf[3]  = {0.0f, 0.0f, 0.0f};
static float accel_lpf[3] = {0.0f, 0.0f, 0.0f};

/* ═══════════════════════════════════════════════════════════════
 *  CS 引脚控制 — PA4
 * ═══════════════════════════════════════════════════════════════ */

#define ICM_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define ICM_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/* ═══════════════════════════════════════════════════════════════
 *  SPI 单字节收发
 * ═══════════════════════════════════════════════════════════════ */

static uint8_t spi_rw_byte(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
    return rx;
}

/* ═══════════════════════════════════════════════════════════════
 *  寄存器读写
 * ═══════════════════════════════════════════════════════════════ */

static void icm_write_reg(uint8_t reg, uint8_t val)
{
    ICM_CS_LOW();
    spi_rw_byte(reg);
    spi_rw_byte(val);
    ICM_CS_HIGH();
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t rx;
    ICM_CS_LOW();
    spi_rw_byte(reg | 0x80);
    rx = spi_rw_byte(0x00);
    ICM_CS_HIGH();
    return rx;
}

/* ─── 间接寄存器访问 (地址 > 0xFF) ─── */

static void icm_write_reg16(uint16_t reg, uint8_t val)
{
    ICM_CS_LOW();
    spi_rw_byte(ICM_REG_IREG_ADDR_15_8);
    spi_rw_byte((uint8_t)(reg >> 8));
    spi_rw_byte((uint8_t)(reg & 0xFF));
    spi_rw_byte(val);
    ICM_CS_HIGH();
}

static uint8_t icm_read_reg16(uint16_t reg)
{
    uint8_t rx;
    ICM_CS_LOW();
    spi_rw_byte(ICM_REG_IREG_ADDR_15_8);
    spi_rw_byte((uint8_t)(reg >> 8));
    spi_rw_byte((uint8_t)(reg & 0xFF));
    ICM_CS_HIGH();

    ICM_CS_LOW();
    spi_rw_byte(ICM_REG_IREG_DATA | 0x80);
    rx = spi_rw_byte(0x00);
    ICM_CS_HIGH();
    return rx;
}

/* ═══════════════════════════════════════════════════════════════
 *  数据解析
 * ═══════════════════════════════════════════════════════════════ */

static int16_t icm_parse_s16(uint8_t b0, uint8_t b1)
{
    if (icm_data_big_endian) {
        return (int16_t)(((int16_t)b0 << 8) | b1);
    }
    return (int16_t)(((int16_t)b1 << 8) | b0);
}

static void icm_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    ICM_CS_LOW();
    spi_rw_byte(reg | 0x80);
    while (len) {
        *buf = spi_rw_byte(0x00);
        buf++;
        len--;
    }
    ICM_CS_HIGH();
}

/* ═══════════════════════════════════════════════════════════════
 *  初始化
 * ═══════════════════════════════════════════════════════════════ */

void imu_init(void)
{
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;

    /* ── 1. 重新初始化 SPI 并唤醒总线 ── */
    HAL_SPI_Init(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);
    spi_rw_byte(0xFF);     /* 唤醒 SPI 总线 */
    ICM_CS_HIGH();
    HAL_Delay(3);

    /* ── 2. 配置 SPI 边沿速率，提升总线稳定性 ── */
    icm_write_reg(ICM_REG_DRIVE_CONFIG0, 0x04);
    HAL_Delay(1);

    /* ── 3. 读取 WHO_AM_I ── */
    uint8_t whoami = icm_read_reg(ICM_REG_WHO_AM_I);
    if (whoami != ICM_WHO_AM_I_VAL) {
        /* 通信失败 — 死循环 */
        while (1);
    }

    /* ── 4. 软复位 ── */
    uint8_t saved_drive = icm_read_reg(ICM_REG_DRIVE_CONFIG0);
    uint8_t saved_intf  = icm_read_reg(ICM_REG_INTF_CONFIG1_OVRD);

    icm_write_reg(ICM_REG_MISC2, 0x01);   /* REG_MISC2.soft_rst = 1 */
    HAL_Delay(1);

    /* 恢复被复位的寄存器 */
    icm_write_reg(ICM_REG_DRIVE_CONFIG0, saved_drive);
    icm_write_reg(ICM_REG_INTF_CONFIG1_OVRD, saved_intf);
    HAL_Delay(1);

    /* ── 5. 读取数据字节序 ── */
    uint8_t sreg_ctrl = icm_read_reg16(ICM_REG_SREG_CTRL);
    icm_data_big_endian = ((sreg_ctrl >> 1) & 0x01) != 0;
    imu_lpf_ready = false;

    /* ── 6. INT1: 高电平/脉冲/推挽 + 数据就绪中断 ── */
    icm_write_reg(ICM_REG_INT1_CONFIG2, 0x01);
    icm_write_reg(ICM_REG_INT1_CONFIG0, 0x04);

    /* ── 7. 陀螺: ±1000dps, ODR 1600Hz ── */
    icm_write_reg(ICM_REG_GYRO_CONFIG0, 0x25);

    /* ── 8. 加速度: ±4g, ODR 1600Hz ── */
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, 0x35);

    /* ── 9. LPF: ODR/4 ── */
    uint8_t reg = icm_read_reg16(ICM_REG_IPREG_SYS2_REG_131);
    reg = (reg & 0xF8) | 0x01;
    icm_write_reg16(ICM_REG_IPREG_SYS2_REG_131, reg);

    reg = icm_read_reg16(ICM_REG_IPREG_SYS1_REG_172);
    reg = (reg & 0xF8) | 0x03;
    icm_write_reg16(ICM_REG_IPREG_SYS1_REG_172, reg);

    /* ── 10. 电源模式：陀螺仪+加速度计 → LN（低噪声）── */
    icm_write_reg(ICM_REG_PWR_MGMT0, 0x0F);
    HAL_Delay(30);

    /* ── 11. 低功耗加速度时钟: RCOSC ── */
    reg = icm_read_reg16(ICM_REG_SMC_CONTROL_0);
    reg |= (1u << 4);
    icm_write_reg16(ICM_REG_SMC_CONTROL_0, reg);
}

/* ═══════════════════════════════════════════════════════════════
 *  读取传感器数据
 * ═══════════════════════════════════════════════════════════════ */

bool imu_read(imu_data_t *out)
{
    uint8_t raw[14];
    icm_read_regs(ICM_REG_ACCEL_DATA_X1, raw, 14);

    /* ── 解析 16 位原始值 ── */
    int16_t raw_ax = icm_parse_s16(raw[0], raw[1]);
    int16_t raw_ay = icm_parse_s16(raw[2], raw[3]);
    int16_t raw_az = icm_parse_s16(raw[4], raw[5]);
    int16_t raw_gx = icm_parse_s16(raw[6], raw[7]);
    int16_t raw_gy = icm_parse_s16(raw[8], raw[9]);
    int16_t raw_gz = icm_parse_s16(raw[10], raw[11]);
    int16_t raw_t  = icm_parse_s16(raw[12], raw[13]);

    /* ── 转换为物理量 ── */
    float ax = (float)raw_ax * ACCEL_SCALE;
    float ay = (float)raw_ay * ACCEL_SCALE;
    float az = (float)raw_az * ACCEL_SCALE;
    float gx = (float)raw_gx * GYRO_SCALE - gyro_bias_x;
    float gy = (float)raw_gy * GYRO_SCALE - gyro_bias_y;
    float gz = (float)raw_gz * GYRO_SCALE - gyro_bias_z;

    /* ── 软件低通滤波 ── */
    if (!imu_lpf_ready) {
        accel_lpf[0] = ax; accel_lpf[1] = ay; accel_lpf[2] = az;
        gyro_lpf[0]  = gx; gyro_lpf[1]  = gy; gyro_lpf[2]  = gz;
        imu_lpf_ready = true;
    } else {
        accel_lpf[0] += IMU_LPF_ALPHA_ACCEL * (ax - accel_lpf[0]);
        accel_lpf[1] += IMU_LPF_ALPHA_ACCEL * (ay - accel_lpf[1]);
        accel_lpf[2] += IMU_LPF_ALPHA_ACCEL * (az - accel_lpf[2]);
        gyro_lpf[0]  += IMU_LPF_ALPHA_GYRO  * (gx - gyro_lpf[0]);
        gyro_lpf[1]  += IMU_LPF_ALPHA_GYRO  * (gy - gyro_lpf[1]);
        gyro_lpf[2]  += IMU_LPF_ALPHA_GYRO  * (gz - gyro_lpf[2]);

    }

    out->ax = accel_lpf[0];
    out->ay = accel_lpf[1];
    out->az = accel_lpf[2];
    out->gx = gyro_lpf[0];
    out->gy = gyro_lpf[1];
    out->gz = gyro_lpf[2];
    out->temp_c     = 25.0f + (float)raw_t / 128.0f;
    out->data_ready = true;

    return true;
}

/* ═══════════════════════════════════════════════════════════════
 *  上电陀螺零偏校准 (静止检测 + 自动重试)
 *
 *  流程:
 *    1. 等待 1 秒让传感器内部稳定
 *    2. 采集 2000 个原始陀螺样本 (间隔 1ms, 共约 2 秒)
 *    3. 计算三轴峰峰值噪声 (dps)
 *    4. 峰峰值 < 5.0 dps → 静止, 取均值作为零偏
 *    5. 噪声过大 → 等待 200ms 后重试 (最多 10 次)
 *
 *  注意: 直接读取原始寄存器, 不经过软件 LPF,
 *        否则 LPF 会掩盖真实噪声导致误判。
 * ═══════════════════════════════════════════════════════════════ */

#define CALIB_SAMPLES     2000
#define CALIB_NOISE_DPS   5.0f
#define CALIB_RETRY_MS    200
#define CALIB_MAX_RETRIES 10

void imu_calibrate_gyro_bias(void)
{
    /* ── 1. 等待传感器内部稳定 ── */
    HAL_Delay(1000);

    int retry = 0;
    bool done = false;

    while (!done && retry < CALIB_MAX_RETRIES) {

        float gx_min = 1e9f, gx_max = -1e9f;
        float gy_min = 1e9f, gy_max = -1e9f;
        float gz_min = 1e9f, gz_max = -1e9f;
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

        /* ── 2. 采集 2000 个样本 (间隔 1ms, 约 2 秒) ── */
        for (int i = 0; i < CALIB_SAMPLES; i++) {
            uint8_t raw[14];
            icm_read_regs(ICM_REG_ACCEL_DATA_X1, raw, 14);

            int16_t raw_gx = icm_parse_s16(raw[6], raw[7]);
            int16_t raw_gy = icm_parse_s16(raw[8], raw[9]);
            int16_t raw_gz = icm_parse_s16(raw[10], raw[11]);

            /* 直接转换为 rad/s — 不校正零偏，不经过低通滤波器 */
            float gx = (float)raw_gx * GYRO_SCALE;
            float gy = (float)raw_gy * GYRO_SCALE;
            float gz = (float)raw_gz * GYRO_SCALE;

            /* 跟踪峰峰值 */
            if (gx < gx_min) gx_min = gx;
            if (gx > gx_max) gx_max = gx;
            if (gy < gy_min) gy_min = gy;
            if (gy > gy_max) gy_max = gy;
            if (gz < gz_min) gz_min = gz;
            if (gz > gz_max) gz_max = gz;

            sum_x += (double)gx;
            sum_y += (double)gy;
            sum_z += (double)gz;

            HAL_Delay(1);
        }

        /* ── 3. 计算峰峰值 (rad/s → dps) ── */
        float p2p_x = (gx_max - gx_min) * (180.0f / 3.14159265359f);
        float p2p_y = (gy_max - gy_min) * (180.0f / 3.14159265359f);
        float p2p_z = (gz_max - gz_min) * (180.0f / 3.14159265359f);

        /* ── 4. 判定静止: 三轴峰峰值均 < 5.0 dps ── */
        if (p2p_x < CALIB_NOISE_DPS && p2p_y < CALIB_NOISE_DPS && p2p_z < CALIB_NOISE_DPS) {
            gyro_bias_x = (float)(sum_x / CALIB_SAMPLES);
            gyro_bias_y = (float)(sum_y / CALIB_SAMPLES);
            gyro_bias_z = (float)(sum_z / CALIB_SAMPLES);
            done = true;
        } else {
            /* ── 5. 噪声过大 → 等待 200ms 后重试 ── */
            retry++;
            HAL_Delay(CALIB_RETRY_MS);
        }
    }

    /* 所有重试均失败 → 零偏保持 0 (不补偿) */
    if (!done) {
        gyro_bias_x = 0.0f;
        gyro_bias_y = 0.0f;
        gyro_bias_z = 0.0f;
    }
}

void imu_get_gyro_bias(float *bx, float *by, float *bz)
{
    *bx = gyro_bias_x;
    *by = gyro_bias_y;
    *bz = gyro_bias_z;
}
