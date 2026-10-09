/* 用缩小的日志区模拟扇区跨越、循环覆盖、重启和旧数据隔离。 */
#include "imu_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_FLASH_SIZE 0x90000U
#define TEST_SECTOR_SIZE 0x1000U

static uint8_t flash_memory[TEST_FLASH_SIZE];
static uint32_t tick_ms;
static uint32_t pending_erase;
static int erase_polls;

uint32_t HAL_GetTick(void) { return tick_ms; }
bool external_flash_init(void) { return true; }

bool external_flash_read(uint32_t address, void *data, uint32_t length)
{
    if (address + length > TEST_FLASH_SIZE || erase_polls != 0) return false;
    memcpy(data, flash_memory + address, length);
    return true;
}

bool external_flash_write(uint32_t address, const void *data, uint32_t length)
{
    if (address + length > TEST_FLASH_SIZE || erase_polls != 0) return false;
    const uint8_t *source = data;
    for (uint32_t i = 0; i < length; ++i) {
        assert((flash_memory[address + i] & source[i]) == source[i]);
        flash_memory[address + i] = source[i];
    }
    return true;
}

bool external_flash_erase_sector_async(uint32_t address)
{
    if (address % TEST_SECTOR_SIZE != 0 || address >= TEST_FLASH_SIZE
        || erase_polls != 0) return false;
    pending_erase = address;
    erase_polls = 3;
    return true;
}

bool external_flash_get_busy(bool *busy)
{
    if (erase_polls > 0 && --erase_polls == 0) {
        memset(flash_memory + pending_erase, 0xFF, TEST_SECTOR_SIZE);
    }
    *busy = erase_polls != 0;
    return true;
}

bool external_flash_wait_ready(void)
{
    bool busy;
    do {
        if (!external_flash_get_busy(&busy)) return false;
    } while (busy);
    return true;
}

bool external_flash_erase_sector(uint32_t address)
{
    return external_flash_erase_sector_async(address) && external_flash_wait_ready();
}

static void capture_samples(uint32_t count)
{
    dart_data_t data = {0};
    data.phase = PHASE_END;
    data.az = -9.80665f;
    for (uint32_t i = 0; i < count; ++i) {
        tick_ms += 5U;
        imu_log_capture(&data);
        imu_log_process();
    }
}

static void finish_log(void)
{
    imu_log_info_t info;
    assert(imu_log_stop());
    imu_log_get_info(&info);
    assert(info.state == IMU_LOG_FINALIZING);
    for (uint32_t i = 0U; i < 512U; ++i) {
        tick_ms += 5U;
        imu_log_process();
        imu_log_get_info(&info);
        if (info.state == IMU_LOG_READY) return;
        assert(info.state == IMU_LOG_FINALIZING);
    }
    assert(false && "日志异步提交超时");
}

static uint32_t timestamp_at(uint32_t offset)
{
    uint8_t bytes[4];
    assert(imu_log_read(offset * IMU_LOG_RECORD_SIZE, bytes, sizeof(bytes)));
    uint32_t timestamp;
    memcpy(&timestamp, bytes, sizeof(timestamp));
    return timestamp;
}

int main(void)
{
    memset(flash_memory, 0xFF, sizeof(flash_memory));
    imu_log_init();
    assert(imu_log_start(IMU_LOG_SAMPLE_RATE_HZ));
    capture_samples(300U);
    const uint32_t first_last_tick = tick_ms;
    finish_log();
    imu_log_info_t info;
    imu_log_get_info(&info);
    assert(info.state == IMU_LOG_READY && info.record_count == 172U);
    assert(timestamp_at(0U) == first_last_tick - 171U * 5U
           && timestamp_at(171U) == first_last_tick);

    imu_log_init();
    imu_log_get_info(&info);
    assert(info.record_count == 172U && timestamp_at(0U) == 645U);

    assert(imu_log_start(IMU_LOG_SAMPLE_RATE_HZ));
    capture_samples(300U);
    const uint32_t second_last_tick = tick_ms;
    imu_log_init();
    imu_log_get_info(&info);
    /* 掉电前尚在 RAM 的末尾四帧不能伪装成已经持久化。 */
    assert(info.state == IMU_LOG_READY && info.record_count == 168U);
    assert(timestamp_at(0U) == second_last_tick - 171U * 5U
           && timestamp_at(167U) == second_last_tick - 4U * 5U);

    assert(imu_log_start(IMU_LOG_SAMPLE_RATE_HZ));
    capture_samples(600U);
    const uint32_t third_last_tick = tick_ms;
    finish_log();
    imu_log_get_info(&info);
    assert(info.record_count == 216U);
    assert(timestamp_at(0U) == third_last_tick - 215U * 5U
           && timestamp_at(215U) == third_last_tick);

    assert(imu_log_start(IMU_LOG_SAMPLE_RATE_HZ));
    capture_samples(8U);
    const uint32_t fourth_last_tick = tick_ms;
    finish_log();
    imu_log_get_info(&info);
    assert(info.record_count == 8U && timestamp_at(7U) == fourth_last_tick);

    assert(imu_log_start(IMU_LOG_SAMPLE_RATE_HZ));
    /* 主循环暂时没有服务 Flash 时，256 条缓冲不能因 uint8_t 溢出卡住。 */
    dart_data_t data = {0};
    data.phase = PHASE_GUIDE;
    for (uint32_t i = 0U; i < 256U; ++i) {
        tick_ms += 5U;
        imu_log_capture(&data);
    }
    finish_log();
    imu_log_get_info(&info);
    assert(info.record_count == 256U);
    puts("IMU circular log tests passed");
    return 0;
}
