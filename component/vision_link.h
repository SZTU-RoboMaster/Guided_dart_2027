#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "vision_protocol.h"

typedef struct {
  uint32_t good_frames;
  uint32_t crc_errors;
  uint32_t uart_errors;
  uint32_t overruns;
  uint32_t rx_start_busy;
  uint32_t rx_start_errors;
  uint32_t rx_last_status;
  uint32_t last_uart_error;
  uint32_t last_dma_error;
} vision_link_stats_t;

void vision_link_init(void);

/* 在主循环中调用，负责启动或恢复视觉 DMA 接收。 */
void vision_link_poll(void);

/* 至少收到一帧 CRC 有效的数据后返回 true，其中也包括目标无效帧。
 * 每接收一个新的 UART 帧，generation 就变化一次。 */
bool vision_link_snapshot(vision_frame_t *out,
                          uint32_t *age_ms,
                          uint32_t *generation);

bool vision_link_get_latest(vision_frame_t *out);
uint32_t vision_link_get_age_ms(void);
bool vision_link_is_fresh(uint32_t max_age_ms);
void vision_link_get_stats(vision_link_stats_t *out);

#endif
