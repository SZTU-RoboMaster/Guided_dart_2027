/**
 * @file    boot_config.h
 * @brief   内部 Flash 分区、OTA 数据块和 Bootloader 启动窗口常量。
 * @details App 链接地址与元数据页必须和链接脚本、镜像制作工具保持一致。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 * @note    地址和页数属于协议兼容边界；修改时须同步更新上位机校验。
 */
#ifndef BOOT_CONFIG_H__
#define BOOT_CONFIG_H__

#include <stdint.h>

/* Boot 占 8 页；App 占 55 页；最后 1 页保存 App 元数据。 */
#define BOOT_APP_ADDRESS       0x08004000UL
#define BOOT_METADATA_ADDRESS  0x0801F800UL
#define BOOT_APP_MAX_SIZE      (BOOT_METADATA_ADDRESS - BOOT_APP_ADDRESS)
#define BOOT_FLASH_PAGE_SIZE   0x800UL
#define BOOT_APP_FIRST_PAGE    8U
#define BOOT_APP_PAGE_COUNT    55U
#define BOOT_METADATA_PAGE     63U
#define BOOT_UPDATE_CHUNK_SIZE 42U
#define BOOT_UPDATE_ACK_INTERVAL 4U
#define BOOT_WAIT_WINDOW_MS    3000U
#define BOOT_REQUEST_ADDRESS   0x20007FF0UL
#define BOOT_REQUEST_MAGIC     0x42555044UL

#define BOOT_METADATA_MAGIC    0x31505041UL
#define BOOT_METADATA_COMMIT   0xA55AA55AUL
#define BOOT_STORAGE_HEADER_SIZE 32U

typedef struct {
    /* 仅在 magic、commit 和整包 CRC 同时匹配时，内部 App 才可启动。 */
    uint32_t magic;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t image_version;
    uint32_t commit;
    uint32_t reserved;
} boot_metadata_t;

#endif /* BOOT_CONFIG_H__ */
