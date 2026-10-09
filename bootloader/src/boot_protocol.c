/**
 * @file    boot_protocol.c
 * @brief   Bootloader 的 LoRa OTA 接收、校验、安装与 App 跳转实现。
 * @details 使用 USART1 接收统一二进制帧，将镜像顺序写入 W25Q128 备用槽；
 *          END 校验通过后才复制到 STM32 内部 App 区。会话检查点支持断电后
 *          从最近的持久化块号续传，活动槽位只在内部 App 校验通过后切换。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 * @note    外挂 Flash 为 A/B 镜像存储，内部 Flash 只有一个可执行 App 区；
 *          CRC 能验证字节完整性，不能证明新 App 运行健康或提供自动回滚。
 */
#include "boot_protocol.h"
#include "boot_config.h"
#include "boot_external_storage.h"
#include "lora_protocol.h"
#include "stm32g4xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/* RAM 中只保存本次接收的即时状态；断电续传依据外挂 Flash 会话日志。 */
typedef struct {
    bool active;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t image_version;
    uint32_t received_size;
    uint32_t running_crc32;
    uint16_t next_chunk;
    uint32_t storage_slot;
    uint32_t active_slot;
    uint32_t session_sequence;
} update_state_t;

static bool erase_app_area(void);
static bool flash_write_bytes(uint32_t address, const uint8_t *data, uint32_t length);
bool boot_app_is_valid(void);

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    /* 与上位机 zlib.crc32 对应：初值全 1，结束时再按位取反。 */
    for (uint32_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U
                ? (crc >> 1U) ^ 0xEDB88320UL
                : crc >> 1U;
        }
    }
    return crc;
}

static uint32_t image_crc32(const uint8_t *image, uint32_t length)
{
    return crc32_update(0xFFFFFFFFUL, image, length) ^ 0xFFFFFFFFUL;
}

static bool uart_read_byte(uint8_t *value, uint32_t timeout_ms)
{
    return HAL_UART_Receive(&huart1, value, 1U, timeout_ms) == HAL_OK;
}

/**
 * @brief  从透明串口读取并验证一帧 LoRa 协议数据。
 * @param  frame        长度至少为 LORA_PROTOCOL_MAX_FRAME 的接收缓冲区。
 * @param  frame_length 成功时写入完整帧长度。
 * @param  timeout_ms   各次 UART 读取的超时时间，单位毫秒。
 * @return true 表示帧头、长度和 CRC16 均有效；false 表示超时或帧损坏。
 */
static bool receive_frame(uint8_t *frame, size_t *frame_length, uint32_t timeout_ms)
{
    /* 先寻找双字节 magic，再读取固定头、payload 和 CRC，避免错位数据
     * 让后续升级状态机误把普通串口噪声当成固件块。 */
    uint8_t byte;
    do {
        if (!uart_read_byte(&byte, timeout_ms)) return false;
    } while (byte != LORA_PROTOCOL_MAGIC0);

    if (!uart_read_byte(&byte, timeout_ms) || byte != LORA_PROTOCOL_MAGIC1) {
        return false;
    }
    frame[0] = LORA_PROTOCOL_MAGIC0;
    frame[1] = LORA_PROTOCOL_MAGIC1;
    if (HAL_UART_Receive(&huart1, &frame[2], 9U, timeout_ms) != HAL_OK) return false;
    if (frame[2] != LORA_PROTOCOL_VERSION || frame[10] > LORA_PROTOCOL_MAX_PAYLOAD) {
        return false;
    }

    const uint8_t payload_length = frame[10];
    const size_t tail_length = (size_t)payload_length + 2U;
    if (HAL_UART_Receive(&huart1, &frame[11], (uint16_t)tail_length, timeout_ms) != HAL_OK) {
        return false;
    }

    const uint16_t received_crc = (uint16_t)frame[11U + payload_length]
                                 | ((uint16_t)frame[12U + payload_length] << 8U);
    const uint16_t calculated_crc = lora_protocol_crc16(&frame[2], 9U + payload_length);
    if (received_crc != calculated_crc) return false;
    *frame_length = 13U + payload_length;
    return true;
}

static void send_ack(uint8_t message_type, uint8_t status, uint16_t value)
{
    /* ACK 载荷顺序为 next_chunk(2)、原消息类型(1)、状态码(1)。 */
    uint8_t payload[4];
    uint8_t frame[LORA_PROTOCOL_MAX_FRAME];
    put_u16(&payload[0], value);
    payload[2] = message_type;
    payload[3] = status;
    const size_t length = lora_protocol_build(
        LORA_MSG_ACK, 0U, HAL_GetTick(), payload, sizeof(payload),
        frame, sizeof(frame));
    if (length != 0U) {
        (void)HAL_UART_Transmit(&huart1, frame, (uint16_t)length, 2000U);
    }
}

/**
 * @brief  擦除内部 App 页及紧随其后的元数据页。
 * @return true 表示所有目标页擦除成功。
 * @note   只有外挂槽位整包 CRC 已通过时才能调用；中途掉电由活动槽恢复。
 */
static bool erase_app_area(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = BOOT_APP_FIRST_PAGE;
    erase.NbPages = BOOT_APP_PAGE_COUNT + 1U;

    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
    return status == HAL_OK;
}

static bool flash_write_bytes(uint32_t address, const uint8_t *data, uint32_t length)
{
    /* STM32G4 最小编程单位为 8 字节；末尾不足 8 字节以 0xFF 补齐。 */
    HAL_FLASH_Unlock();
    for (uint32_t offset = 0U; offset < length; offset += 8U) {
        uint64_t double_word = UINT64_MAX;
        const uint32_t remaining = length - offset;
        const uint32_t count = remaining > 8U ? 8U : remaining;
        memcpy(&double_word, &data[offset], count);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              address + offset, double_word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

static bool write_metadata(const update_state_t *state)
{
    /* 元数据和 App 位于不同页；写入后 boot_app_is_valid 还会重新算 CRC。 */
    const boot_metadata_t metadata = {
        BOOT_METADATA_MAGIC,
        state->image_size,
        state->image_crc32,
        state->image_version,
        BOOT_METADATA_COMMIT,
        0U
    };
    return flash_write_bytes(BOOT_METADATA_ADDRESS,
                             (const uint8_t *)&metadata,
                             sizeof(metadata));
}

static bool copy_internal_to_slot(uint32_t slot)
{
    /* 首次启用外挂 A/B 时，把当前有效内部 App 建成活动槽基线。 */
    const boot_metadata_t *metadata = (const boot_metadata_t *)BOOT_METADATA_ADDRESS;
    if (metadata->image_size == 0U || metadata->image_size > BOOT_APP_MAX_SIZE) return false;
    if (!boot_storage_prepare_slot(slot, metadata->image_size)) return false;
    uint8_t buffer[256];
    uint32_t offset = 0U;
    while (offset < metadata->image_size) {
        const uint32_t count = (metadata->image_size - offset) > sizeof(buffer)
            ? sizeof(buffer) : metadata->image_size - offset;
        memcpy(buffer, (const void *)(BOOT_APP_ADDRESS + offset), count);
        if (!boot_storage_write(slot, BOOT_STORAGE_HEADER_SIZE + offset, buffer, count)) return false;
        offset += count;
    }
    return boot_storage_finalize(slot, metadata->image_size,
                                 metadata->image_crc32, metadata->image_version);
}

/**
 * @brief  校验外挂槽位并安装到内部可执行 App 区。
 * @param  slot 槽位编号，0 为 A，1 为 B。
 * @return true 表示外部 CRC、内部写入、元数据和最终 App CRC 全部通过。
 * @note   擦除内部 App 以后若发生掉电，下一次启动仍会尝试从活动外部槽
 *         恢复旧镜像；此函数本身不会切换活动槽位。
 */
static bool copy_slot_to_internal(uint32_t slot)
{
    /* 复制前先完整读取并计算外挂槽位 CRC。只有外部副本可信时才擦除
     * 内部 App 区，避免掉电或无线错误留下半个新镜像。 */
    boot_storage_header_t header_storage;
    const boot_storage_header_t *header = &header_storage;
    if (!boot_storage_header_read(slot, &header_storage)) return false;
    if (header == 0 || header->image_size == 0U || header->image_size > BOOT_APP_MAX_SIZE) return false;
    uint8_t buffer[256];
    uint32_t checked_crc = 0xFFFFFFFFUL;
    uint32_t offset = 0U;
    while (offset < header->image_size) {
        const uint32_t count = (header->image_size - offset) > sizeof(buffer)
            ? sizeof(buffer) : header->image_size - offset;
        if (!boot_storage_read(slot, BOOT_STORAGE_HEADER_SIZE + offset, buffer, count)) return false;
        checked_crc = crc32_update(checked_crc, buffer, count);
        offset += count;
    }
    if ((checked_crc ^ 0xFFFFFFFFUL) != header->image_crc32) return false;
    if (!erase_app_area()) return false;
    offset = 0U;
    while (offset < header->image_size) {
        const uint32_t count = (header->image_size - offset) > sizeof(buffer)
            ? sizeof(buffer) : header->image_size - offset;
        if (!boot_storage_read(slot, BOOT_STORAGE_HEADER_SIZE + offset, buffer, count)
            || !flash_write_bytes(BOOT_APP_ADDRESS + offset, buffer, count)) return false;
        offset += count;
    }
    update_state_t state = {0};
    state.image_size = header->image_size;
    state.image_crc32 = header->image_crc32;
    state.image_version = header->image_version;
    return write_metadata(&state) && boot_app_is_valid();
}

static bool ensure_active_slot(uint32_t *slot)
{
    /* 活动元数据不存在时，仅在内部 App 可用的前提下建立槽位 A 基线。 */
    if (boot_storage_read_active(slot)) {
        boot_storage_header_t header;
        if (boot_storage_header_read(*slot, &header)) return true;
    }
    *slot = 0U;
    return copy_internal_to_slot(*slot) && boot_storage_write_active(*slot);
}

/**
 * @brief  核验内部 App 元数据、向量表边界与整包 CRC32。
 * @return true 表示镜像可尝试跳转；不代表应用运行逻辑健康。
 */
bool boot_app_is_valid(void)
{
    const boot_metadata_t *metadata = (const boot_metadata_t *)BOOT_METADATA_ADDRESS;
    const uint32_t *vector = (const uint32_t *)BOOT_APP_ADDRESS;
    const uint32_t stack = vector[0];
    const uint32_t reset = vector[1];

    if (metadata->magic != BOOT_METADATA_MAGIC
        || metadata->commit != BOOT_METADATA_COMMIT
        || metadata->image_size == 0U
        || metadata->image_size > BOOT_APP_MAX_SIZE) return false;
    /* App 链接器的初始 MSP 正好位于保留区下边界，边界值本身合法。 */
    if (stack < 0x20000000UL || stack > 0x20007FE0UL
        || (stack & 0x7U) != 0U) return false;
    if (reset < BOOT_APP_ADDRESS || reset >= BOOT_METADATA_ADDRESS) return false;
    return image_crc32((const uint8_t *)BOOT_APP_ADDRESS,
                       metadata->image_size) == metadata->image_crc32;
}

/**
 * @brief  退出 Bootloader 外设上下文并从 App 向量表启动应用。
 * @note   调用前必须先通过 boot_app_is_valid；本函数不重复校验。
 */
void boot_jump_to_app(void)
{
    const uint32_t *vector = (const uint32_t *)BOOT_APP_ADDRESS;
    void (*app_reset_handler)(void) = (void (*)(void))vector[1];
    __disable_irq();
    HAL_DeInit();
    SCB->VTOR = BOOT_APP_ADDRESS;
    __set_MSP(vector[0]);
    /* App 需要依赖 SysTick、UART 和 DMA 中断，跳转前恢复全局中断。 */
    __enable_irq();
    app_reset_handler();
}

/**
 * @brief  执行 BEGIN、DATA、END 三阶段 OTA 状态转换。
 * @param  frame 已通过 CRC16 验证的完整帧。
 * @param  state 当前接收会话；BEGIN 会重建，DATA 会推进块号和 CRC。
 * @return true 表示本帧已被接受；false 表示格式、顺序或存储操作失败。
 */
static bool handle_frame(const uint8_t *frame, update_state_t *state)
{
    const uint8_t type = frame[3];
    const uint8_t payload_length = frame[10];
    const uint8_t *payload = &frame[11];

    if (type == LORA_MSG_UPDATE_BEGIN) {
        /* BEGIN 只建立会话和准备备用槽位；此时尚未改变内部 App。 */
        if (payload_length < 12U) return false;
        update_state_t next = {0};
        next.image_size = read_u32(&payload[0]);
        next.image_crc32 = read_u32(&payload[4]);
        next.image_version = read_u32(&payload[8]);
        if (next.image_size == 0U || next.image_size > BOOT_APP_MAX_SIZE
            || !ensure_active_slot(&next.active_slot)) {
            send_ack(type, 1U, 0U);
            return false;
        }
        boot_update_session_t saved = {0};
        const bool can_resume = boot_storage_session_load(&saved)
            && saved.image_size == next.image_size
            && saved.image_crc32 == next.image_crc32
            && saved.image_version == next.image_version
            && saved.storage_slot != next.active_slot
            && saved.received_size <= next.image_size
            && saved.received_size == ((saved.next_chunk * BOOT_UPDATE_CHUNK_SIZE < saved.image_size)
                                      ? saved.next_chunk * BOOT_UPDATE_CHUNK_SIZE
                                      : saved.image_size)
            && saved.next_chunk <= UINT16_MAX;
        if (can_resume) {
            /* 只有镜像标识、槽位和接收长度全部一致才能续传；检查点之后
             * 已写但未记录的字节可能会重写，W25Q 对相同数据重复编程有效。 */
            next.storage_slot = saved.storage_slot;
            next.received_size = saved.received_size;
            next.next_chunk = (uint16_t)saved.next_chunk;
            next.running_crc32 = saved.running_crc32;
            next.session_sequence = saved.sequence;
        } else {
            next.storage_slot = next.active_slot ^ 1U;
            if (!boot_storage_prepare_slot(next.storage_slot, next.image_size)) {
                send_ack(type, 1U, 0U);
                return false;
            }
            next.running_crc32 = 0xFFFFFFFFUL;
            next.session_sequence = 0U;
            boot_update_session_t session = {
                BOOT_STORAGE_SESSION_MAGIC, 0U, next.image_size, next.image_crc32,
                next.image_version, next.storage_slot, 0U, 0U,
                next.running_crc32, 0U, BOOT_STORAGE_SESSION_COMMIT, {0U}
            };
            if (!boot_storage_session_start(&session)) {
                send_ack(type, 1U, 0U);
                return false;
            }
        }
        next.active = true;
        state[0] = next;
        send_ack(type, 0U, next.next_chunk);
        return true;
    }

    if (type == LORA_MSG_UPDATE_DATA && state->active) {
        /* 每块都必须严格等于 next_chunk。允许上一块重复是为了覆盖
         * “数据已写入但 ACK 丢失”的无线场景。 */
        if (payload_length < 3U) return false;
        const uint16_t chunk = read_u16(&payload[0]);
        const uint8_t data_length = payload[2];
        if (data_length == 0U || data_length > BOOT_UPDATE_CHUNK_SIZE
            || data_length != payload_length - 3U
            || state->received_size + data_length > state->image_size) {
            send_ack(type, 2U, state->next_chunk);
            return false;
        }
        /* ACK 丢失时允许上位机重发上一块，避免因为无线链路丢一个 ACK
         * 就必须从头擦除并升级。重复块不再次写 Flash，只补发成功 ACK。 */
        if ((uint16_t)(chunk + 1U) == state->next_chunk) {
            send_ack(type, 0U, state->next_chunk);
            return true;
        }
        if (chunk != state->next_chunk) {
            send_ack(type, 2U, state->next_chunk);
            return false;
        }
        if (!boot_storage_write(state->storage_slot,
                                BOOT_STORAGE_HEADER_SIZE + state->received_size,
                                &payload[3], data_length)) {
            send_ack(type, 3U, state->next_chunk);
            return false;
        }
        state->running_crc32 = crc32_update(state->running_crc32,
                                            &payload[3], data_length);
        state->received_size += data_length;
        state->next_chunk++;
        if ((state->next_chunk % BOOT_STORAGE_SESSION_CHECKPOINT_INTERVAL) == 0U
            || state->received_size == state->image_size) {
            /* 检查点先于累计成功 ACK 落盘；断电时主机最多回退到上一条
             * 有效记录，而不会误把未持久化进度当成已完成。 */
            boot_update_session_t session = {
                BOOT_STORAGE_SESSION_MAGIC, ++state->session_sequence,
                state->image_size, state->image_crc32, state->image_version,
                state->storage_slot, state->received_size, state->next_chunk,
                state->running_crc32, 0U, BOOT_STORAGE_SESSION_COMMIT, {0U}
            };
            if (!boot_storage_session_checkpoint(&session)) {
                send_ack(type, 3U, state->next_chunk);
                return false;
            }
        }
        /* 累计 ACK 每四块发送一次，末块必须应答；错误和重复块仍立即应答。
         * 上位机按同一窗口边界等待，减少空口往返而不丢失块序号校验。 */
        if ((state->next_chunk % BOOT_UPDATE_ACK_INTERVAL) == 0U
            || state->received_size == state->image_size) {
            send_ack(type, 0U, state->next_chunk);
        }
        return true;
    }

    if (type == LORA_MSG_UPDATE_END && state->active) {
        /* END 是唯一允许提交新镜像的路径，长度和 CRC 任一不符都拒绝提交。 */
        const uint32_t crc = state->running_crc32 ^ 0xFFFFFFFFUL;
        if (state->received_size != state->image_size || crc != state->image_crc32) {
            send_ack(type, 4U, state->next_chunk);
            return false;
        }
        if (!boot_storage_finalize(state->storage_slot, state->image_size,
                                   state->image_crc32, state->image_version)
            || !copy_slot_to_internal(state->storage_slot)
            || !boot_storage_write_active(state->storage_slot)) {
            /* 活动槽位最后提交；安装中断时旧活动槽仍可用于下次启动恢复。 */
            send_ack(type, 5U, state->next_chunk);
            return false;
        }
        /* 清理失败不会否定已经校验并提交的新 App；下次 BEGIN 会因活动槽位
         * 已变化而放弃旧会话，避免把“清理日志”变成升级失败点。 */
        (void)boot_storage_session_clear();
        /* ACK 在复位前发送，给上位机一个明确的“镜像已提交”结果。 */
        send_ack(type, 0U, state->next_chunk);
        HAL_Delay(100U);
        NVIC_SystemReset();
    }
    return false;
}

/**
 * @brief  初始化 Bootloader 通信并持续处理 OTA 帧。
 * @details 启动时先校验内部 App；无效时尝试从活动外挂槽恢复。有效 App
 *          在无强制升级请求时保留 3 秒接收窗口，窗口到期才跳转。
 * @note   无有效 App、强制升级或已有接收会话时持续留在 Bootloader。
 */
void boot_protocol_run(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = LORA_UART_BAUDRATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

    update_state_t state = {0};
    uint8_t frame[LORA_PROTOCOL_MAX_FRAME];
    (void)boot_storage_init();
    bool valid_app = boot_app_is_valid();
    if (!valid_app) {
        uint32_t active_slot;
        boot_storage_header_t header;
        if (boot_storage_read_active(&active_slot)
            && boot_storage_header_read(active_slot, &header)
            && copy_slot_to_internal(active_slot)) {
            valid_app = boot_app_is_valid();
        }
    }
    volatile uint32_t *boot_request = (volatile uint32_t *)BOOT_REQUEST_ADDRESS;
    const bool forced_update = *boot_request == BOOT_REQUEST_MAGIC;
    *boot_request = 0U;
    /* 即使 RAM 中的升级标志因复位时序丢失，也给 LoRa 链路一个明确的
     * 接收窗口。这样上位机在收到 App 的过渡 ACK 后重发 BEGIN，仍能进入升级。
     * 窗口结束且没有 BEGIN 时才跳转 App，正常上电最多增加 3 秒启动等待。 */
    const uint32_t boot_wait_deadline = HAL_GetTick() + BOOT_WAIT_WINDOW_MS;
    while (1) {
        if (!state.active && valid_app && !forced_update
            && (int32_t)(HAL_GetTick() - boot_wait_deadline) >= 0) {
            boot_jump_to_app();
        }

        size_t frame_length = 0U;
        const uint32_t timeout = state.active ? 1000U : 50U;
        if (receive_frame(frame, &frame_length, timeout)) {
            (void)frame_length;
            (void)handle_frame(frame, &state);
        }
    }
}
