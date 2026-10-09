/**
 * @file    boot_external_storage.h
 * @brief   外挂 W25Q128 OTA 槽位布局与持久化接口。
 * @details 定义 A/B 镜像槽、活动槽元数据和断点续传日志的位置与结构。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 * @note    槽位 A/B 用于保存镜像；STM32 内部始终只执行单个 App 分区。
 */
#ifndef BOOT_EXTERNAL_STORAGE_H
#define BOOT_EXTERNAL_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

/* 这里的地址均为外挂 Flash 芯片内偏移，不是 STM32 绝对地址。 */
#define BOOT_STORAGE_SLOT_SIZE 0x20000UL
#define BOOT_STORAGE_SLOT_A 0x000000UL
#define BOOT_STORAGE_SLOT_B 0x020000UL
#define BOOT_STORAGE_METADATA 0x040000UL
#define BOOT_STORAGE_SESSION  0x060000UL
#define BOOT_STORAGE_SESSION_SECTOR_SIZE 0x1000UL
#define BOOT_STORAGE_SESSION_CHECKPOINT_INTERVAL 64U
#define BOOT_STORAGE_SESSION_MAGIC  0x53534553UL
#define BOOT_STORAGE_SESSION_COMMIT 0xA55AA55AUL

typedef struct {
    /* 头部位于槽位起始处，镜像字节从 32 字节偏移开始。 */
    uint32_t magic;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t image_version;
    uint32_t commit;
    uint32_t reserved[3];
} boot_storage_header_t;

/**
 * @brief  断点续传检查点的持久化格式。
 * @details running_crc32 是未按位取反的累积状态；next_chunk 指向下一
 *          个待发送块。record_crc32 只校验其之前的字段。
 */
typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t image_version;
    uint32_t storage_slot;
    uint32_t received_size;
    uint32_t next_chunk;
    uint32_t running_crc32;
    uint32_t record_crc32;
    uint32_t commit;
    uint32_t reserved[1];
} boot_update_session_t;

/** @brief 探测外挂 Flash；失败时所有槽位操作均返回 false。 */
bool boot_storage_init(void);
/** @brief 读取槽位头；成功不代表镜像内容 CRC 已核验。 */
bool boot_storage_header_read(uint32_t slot, boot_storage_header_t *header);
/** @brief 擦除待写镜像覆盖的扇区；调用方必须保证 slot 非活动槽。 */
bool boot_storage_prepare_slot(uint32_t slot, uint32_t image_size);
/** @brief 在槽位数据区按偏移写入，禁止覆盖槽位头。 */
bool boot_storage_write(uint32_t slot, uint32_t offset, const void *data, uint32_t length);
/** @brief 从槽位数据区按偏移读取。 */
bool boot_storage_read(uint32_t slot, uint32_t offset, void *data, uint32_t length);
/** @brief 写入通过接收校验的镜像头；不切换活动槽位。 */
bool boot_storage_finalize(uint32_t slot, uint32_t image_size, uint32_t image_crc32, uint32_t image_version);
/** @brief 读取活动槽指针；不校验槽位内容。 */
bool boot_storage_read_active(uint32_t *slot);
/** @brief 重写活动槽指针；仅在内部 App 最终校验通过后调用。 */
bool boot_storage_write_active(uint32_t slot);
/** @brief 载入序号最大的有效会话检查点。 */
bool boot_storage_session_load(boot_update_session_t *session);
/** @brief 擦除旧日志并追加新会话的起始记录。 */
bool boot_storage_session_start(const boot_update_session_t *session);
/** @brief 追加检查点，不擦除已有日志。 */
bool boot_storage_session_checkpoint(const boot_update_session_t *session);
/** @brief 清除已经提交的会话记录。 */
bool boot_storage_session_clear(void);

#endif
