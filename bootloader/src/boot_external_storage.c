/**
 * @file    boot_external_storage.c
 * @brief   W25Q128 上的 OTA A/B 槽位、活动指针和续传日志实现。
 * @details 槽位头标识完整镜像；独立元数据扇区标识当前活动槽；会话日志
 *          以追加方式保存检查点。各区域只通过 external_flash 接口访问。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 * @note    写入槽位头不等于切换活动槽；只有 Bootloader 校验内部 App 后
 *          才调用 boot_storage_write_active 提交新活动槽位。
 */
#include "boot_external_storage.h"
#include "boot_config.h"
#include "external_flash.h"
#include <stddef.h>
#include <string.h>

#define STORAGE_HEADER_MAGIC 0x534C4F54UL
#define STORAGE_COMMIT 0xA55AA55AUL
#define STORAGE_SECTOR_SIZE 0x1000UL
#define STORAGE_HEADER_SIZE ((uint32_t)sizeof(boot_storage_header_t))
#define SESSION_RECORD_SIZE ((uint32_t)sizeof(boot_update_session_t))

typedef struct {
    uint32_t magic;
    uint32_t active_slot;
    uint32_t generation;
    uint32_t commit;
} storage_metadata_t;

static bool initialized;

static uint32_t session_crc(const boot_update_session_t *session)
{
    /* 只计算 record_crc32 之前的字段，避免校验值参与自身计算。 */
    const uint8_t *bytes = (const uint8_t *)session;
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0U; i < offsetof(boot_update_session_t, record_crc32); ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320UL : crc >> 1U;
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static bool session_valid(const boot_update_session_t *session)
{
    /* 半写记录可能有 magic，但 commit 或整条记录 CRC 不会同时有效。 */
    return session->magic == BOOT_STORAGE_SESSION_MAGIC
        && session->commit == BOOT_STORAGE_SESSION_COMMIT
        && session->record_crc32 == session_crc(session)
        && session->storage_slot <= 1U;
}

/**
 * @brief  把一个已生成的会话状态追加到首个空白日志位置。
 * @return true 表示记录已写入；扇区已满或读写失败时返回 false。
 * @note   本函数不自动擦除旧日志；新会话由 session_start 显式擦除。
 */
static bool session_append(const boot_update_session_t *source)
{
    /* 追加日志只寻找全擦除记录，不在每个数据块后擦除扇区，
     * 这样既减少 W25Q 擦写次数，也让断电时至少保留上一条完整检查点。 */
    boot_update_session_t session = *source;
    session.magic = BOOT_STORAGE_SESSION_MAGIC;
    session.commit = BOOT_STORAGE_SESSION_COMMIT;
    session.record_crc32 = session_crc(&session);
    for (uint32_t offset = 0U; offset + SESSION_RECORD_SIZE <= BOOT_STORAGE_SESSION_SECTOR_SIZE;
         offset += SESSION_RECORD_SIZE) {
        uint32_t marker = 0U;
        if (!external_flash_read(BOOT_STORAGE_SESSION + offset, &marker, sizeof(marker))) return false;
        if (marker == UINT32_MAX) {
            return external_flash_write(BOOT_STORAGE_SESSION + offset,
                                        &session, sizeof(session));
        }
    }
    return false;
}

static uint32_t slot_address(uint32_t slot)
{
    return slot == 0U ? BOOT_STORAGE_SLOT_A : BOOT_STORAGE_SLOT_B;
}

static bool valid_slot(uint32_t slot)
{
    return slot <= 1U;
}

bool boot_storage_init(void)
{
    initialized = external_flash_init();
    return initialized;
}

/**
 * @brief  读取并检查槽位头的边界字段。
 * @return true 仅表示槽位头结构有效；镜像数据 CRC 由安装前另行校验。
 */
bool boot_storage_header_read(uint32_t slot, boot_storage_header_t *header)
{
    if (!initialized || !valid_slot(slot) || header == 0) return false;
    if (!external_flash_read(slot_address(slot), header, sizeof(*header))) return false;
    return header->magic == STORAGE_HEADER_MAGIC
        && header->commit == STORAGE_COMMIT
        && header->image_size > 0U
        && header->image_size <= BOOT_APP_MAX_SIZE;
}

/**
 * @brief  按镜像实际长度擦除备用槽的槽位头和数据扇区。
 * @note   仅应作用于非活动槽；上层 BEGIN 负责选择槽位。
 */
bool boot_storage_prepare_slot(uint32_t slot, uint32_t image_size)
{
    /* 槽位头也在擦除范围内，必须先擦除再接收；未完成的槽位没有有效 commit。 */
    if (!initialized || !valid_slot(slot) || image_size == 0U || image_size > BOOT_APP_MAX_SIZE) return false;
    const uint32_t base = slot_address(slot);
    const uint32_t erase_size = (STORAGE_HEADER_SIZE + image_size + STORAGE_SECTOR_SIZE - 1U) & ~(STORAGE_SECTOR_SIZE - 1U);
    for (uint32_t offset = 0U; offset < erase_size; offset += STORAGE_SECTOR_SIZE) {
        if (!external_flash_erase_sector(base + offset)) return false;
    }
    return true;
}

bool boot_storage_write(uint32_t slot, uint32_t offset, const void *data, uint32_t length)
{
    if (!initialized || !valid_slot(slot) || data == 0 || offset < STORAGE_HEADER_SIZE
        || length == 0U || offset > BOOT_STORAGE_SLOT_SIZE
        || length > BOOT_STORAGE_SLOT_SIZE - offset) return false;
    return external_flash_write(slot_address(slot) + offset, data, length);
}

bool boot_storage_read(uint32_t slot, uint32_t offset, void *data, uint32_t length)
{
    if (!initialized || !valid_slot(slot) || data == 0 || offset < STORAGE_HEADER_SIZE
        || length == 0U || offset > BOOT_STORAGE_SLOT_SIZE
        || length > BOOT_STORAGE_SLOT_SIZE - offset) return false;
    return external_flash_read(slot_address(slot) + offset, data, length);
}

/**
 * @brief  数据写完且运行 CRC 核对成功后写入槽位头。
 * @note   commit 字段随槽位头写入；槽位数据尚需由安装流程重新读出核验。
 */
bool boot_storage_finalize(uint32_t slot, uint32_t image_size, uint32_t image_crc32, uint32_t image_version)
{
    /* 接收完成后才写含 commit 的槽位头；写入中途掉电仍需依靠
     * 安装前的整包 CRC 校验，不能只凭槽位头判断镜像完整。 */
    if (!initialized || !valid_slot(slot) || image_size == 0U || image_size > BOOT_APP_MAX_SIZE) return false;
    const boot_storage_header_t header = {
        STORAGE_HEADER_MAGIC, image_size, image_crc32, image_version, STORAGE_COMMIT, {0U, 0U, 0U}
    };
    return external_flash_write(slot_address(slot), &header, sizeof(header));
}

/**
 * @brief  从独立元数据扇区读取当前活动槽编号。
 * @return true 表示指针结构有效；不验证槽位头或镜像内容。
 */
bool boot_storage_read_active(uint32_t *slot)
{
    if (!initialized || slot == 0) return false;
    storage_metadata_t metadata = {0};
    if (!external_flash_read(BOOT_STORAGE_METADATA, &metadata, sizeof(metadata))) return false;
    if (metadata.magic != STORAGE_HEADER_MAGIC || metadata.commit != STORAGE_COMMIT || !valid_slot(metadata.active_slot)) {
        return false;
    }
    *slot = metadata.active_slot;
    return true;
}

/**
 * @brief  擦除并重写活动槽位元数据扇区。
 * @note   必须在内部 App 安装及最终 CRC 校验成功后调用；此操作不是
 *         断电原子的，写入中断时启动代码会按内部 App 校验结果处理。
 */
bool boot_storage_write_active(uint32_t slot)
{
    if (!initialized || !valid_slot(slot)) return false;
    if (!external_flash_erase_sector(BOOT_STORAGE_METADATA)) return false;
    const storage_metadata_t metadata = {STORAGE_HEADER_MAGIC, slot, 1U, STORAGE_COMMIT};
    return external_flash_write(BOOT_STORAGE_METADATA, &metadata, sizeof(metadata));
}

/**
 * @brief  扫描会话扇区，选取序号最大的完整有效检查点。
 * @return true 表示找到了可供上层匹配镜像标识的记录。
 */
bool boot_storage_session_load(boot_update_session_t *session)
{
    /* 扫描整个 4 KB 日志，选择序号最大的完整记录；半写入记录会因
     * commit 或 CRC 不匹配被忽略，不会污染可恢复状态。 */
    if (!initialized || session == 0) return false;
    bool found = false;
    boot_update_session_t candidate = {0};
    for (uint32_t offset = 0U; offset + SESSION_RECORD_SIZE <= BOOT_STORAGE_SESSION_SECTOR_SIZE;
         offset += SESSION_RECORD_SIZE) {
        if (!external_flash_read(BOOT_STORAGE_SESSION + offset, &candidate, sizeof(candidate))) return false;
        if (session_valid(&candidate)
            && (!found || candidate.sequence >= session->sequence)) {
            *session = candidate;
            found = true;
        }
    }
    return found;
}

bool boot_storage_session_start(const boot_update_session_t *session)
{
    /* 新镜像或镜像标识变化时才清空会话日志，避免误续传旧镜像。 */
    if (!initialized || session == 0) return false;
    if (!external_flash_erase_sector(BOOT_STORAGE_SESSION)) return false;
    return session_append(session);
}

bool boot_storage_session_checkpoint(const boot_update_session_t *session)
{
    /* 检查点写入失败由上层转成 DATA 错误，防止“数据已写但无法恢复”的隐性状态。 */
    if (!initialized || session == 0) return false;
    return session_append(session);
}

bool boot_storage_session_clear(void)
{
    return initialized && external_flash_erase_sector(BOOT_STORAGE_SESSION);
}
