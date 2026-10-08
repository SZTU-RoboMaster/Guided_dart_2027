#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_SOF_0          0xAAU
#define VISION_SOF_1          0x55U
#define VISION_PROTOCOL_VER   0x05U
#define VISION_FRAME_SIZE     24U

#define VISION_FLAG_VALID     0x80U
#define VISION_FLAG_COAST     0x40U /* 兼容旧协议；当前视觉端不发送预测帧 */

typedef struct {
    int16_t cx;
    int16_t cy;
    int16_t vx_px_s;
    int16_t vy_px_s;
    uint16_t radius_px;
    uint16_t area_px;
    uint16_t distance_dm;     /* 为零表示距离未标定或不可用 */
    uint16_t capture_time_ms; /* OpenMV 图像采集时刻的低 16 位 */
    uint8_t frame_id;
    uint8_t confidence;
    uint8_t flags;
} vision_frame_t;

static inline int vision_frame_is_valid(const vision_frame_t *f)
{
    return f != 0 && (f->flags & VISION_FLAG_VALID) != 0U;
}

#endif
