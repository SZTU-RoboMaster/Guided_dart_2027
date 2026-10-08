#include "vision_link.h"
#include "usart.h"
#include "main.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_usart2_rx;

#define RX_DMA_BUF_SIZE 96U

static uint8_t rx_dma_buf[RX_DMA_BUF_SIZE];
static uint8_t frame_buf[VISION_FRAME_SIZE];
static uint8_t frame_index;
static vision_frame_t latest_frame;
static volatile bool have_frame;
static volatile uint32_t latest_tick_ms;
static volatile uint32_t generation_count;
static volatile bool rx_started;
static volatile bool rx_restart_pending;
static vision_link_stats_t stats;

static uint16_t read_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int16_t read_i16_be(const uint8_t *p)
{
    return (int16_t)read_u16_be(p);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t size)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint32_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void accept_frame(const uint8_t *b)
{
    vision_frame_t f;
    f.flags          = b[3];
    f.frame_id       = b[4];
    f.confidence     = b[5];
    f.cx             = read_i16_be(&b[6]);
    f.cy             = read_i16_be(&b[8]);
    f.vx_px_s        = read_i16_be(&b[10]);
    f.vy_px_s        = read_i16_be(&b[12]);
    f.radius_px      = read_u16_be(&b[14]);
    f.area_px        = read_u16_be(&b[16]);
    f.distance_dm    = read_u16_be(&b[18]);
    f.capture_time_ms = read_u16_be(&b[20]);

    __disable_irq();
    latest_frame = f;
    latest_tick_ms = HAL_GetTick();
    generation_count++;
    have_frame = true;
    __enable_irq();
    stats.good_frames++;
}

/* 滑动式定长解析器。再次遇到一对 SOF 字节时重新开始收集，因此丢失一个
 * 字节最多只会损失一帧。 */
static void parse_byte(uint8_t value)
{
    if (frame_index == 0U) {
        if (value == VISION_SOF_0) frame_buf[frame_index++] = value;
        return;
    }
    if (frame_index == 1U) {
        if (value == VISION_SOF_1) {
            frame_buf[frame_index++] = value;
        } else {
            frame_index = (value == VISION_SOF_0) ? 1U : 0U;
        }
        return;
    }

    frame_buf[frame_index++] = value;
    if (frame_index < VISION_FRAME_SIZE) return;

    const uint16_t received = read_u16_be(&frame_buf[22]);
    const uint16_t calculated = crc16_ccitt(frame_buf, 22U);
    if (frame_buf[2] == VISION_PROTOCOL_VER && received == calculated) {
        accept_frame(frame_buf);
    } else {
        stats.crc_errors++;
    }
    frame_index = 0U;
}

static bool start_receive(void)
{
  /* 已经有一个 DMA 接收时，不得重复调用启动接口。 */
  if (huart2.RxState == HAL_UART_STATE_BUSY_RX
      && hdma_usart2_rx.State == HAL_DMA_STATE_BUSY) {
    rx_started = true;
    rx_restart_pending = false;
    return true;
  }

  /* UART 仍处于接收状态时，等待当前 HAL 流程完成，不重复启动。 */
  if (huart2.RxState == HAL_UART_STATE_BUSY_RX) {
    return false;
  }

  /* UART 已回到 READY 但 DMA 状态残留为 BUSY 时，先回收 DMA。 */
  if (huart2.RxState == HAL_UART_STATE_READY
      && hdma_usart2_rx.State == HAL_DMA_STATE_BUSY) {
    ATOMIC_CLEAR_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
    (void)HAL_DMA_Abort(&hdma_usart2_rx);
  }

  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart2, rx_dma_buf, (uint16_t)sizeof(rx_dma_buf));
  stats.rx_last_status = (uint32_t)status;
  stats.last_uart_error = huart2.ErrorCode;
  stats.last_dma_error = hdma_usart2_rx.ErrorCode;
  if (status == HAL_OK) {
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    rx_started = true;
    rx_restart_pending = false;
    return true;
  }

  rx_started = false;
  stats.overruns++;
  if (status == HAL_BUSY) {
    stats.rx_start_busy++;
  } else {
    stats.rx_start_errors++;
  }
  return false;
}

void vision_link_poll(void)
{
  if (!rx_started || rx_restart_pending) {
    (void)start_receive();
  }
}

static void clear_uart_receive_error(void)
{
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_IDLEF | UART_CLEAR_OREF
                                   | UART_CLEAR_NEF | UART_CLEAR_PEF
                                   | UART_CLEAR_FEF);
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
}

void vision_link_init(void)
{
  frame_index = 0U;
  have_frame = false;
  latest_tick_ms = 0U;
  generation_count = 0U;
  rx_started = false;
  rx_restart_pending = false;
  memset(&latest_frame, 0, sizeof(latest_frame));
  memset(&stats, 0, sizeof(stats));

  /* 前面的初始化阶段可能产生 ORE，启动视觉 DMA 前清掉残留状态。 */
  (void)HAL_UART_AbortReceive(&huart2);
  clear_uart_receive_error();
  (void)start_receive();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance != USART2) return;
  rx_started = false;
  rx_restart_pending = true;
  if (size > RX_DMA_BUF_SIZE) size = RX_DMA_BUF_SIZE;
  for (uint16_t i = 0; i < size; ++i) parse_byte(rx_dma_buf[i]);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;
  stats.uart_errors++;
  stats.last_uart_error = huart->ErrorCode;
  stats.last_dma_error = hdma_usart2_rx.ErrorCode;
  rx_started = false;
  rx_restart_pending = true;
  (void)HAL_UART_AbortReceive(huart);
  frame_index = 0U;
  clear_uart_receive_error();
}

/*
 * 旧版代码在中断回调中直接重启 DMA，可能与 DMA/UART 状态清理交错，导致
 * HAL_BUSY 或 DMA 错误。重启现在统一由 vision_link_poll() 完成。
 */
bool vision_link_snapshot(vision_frame_t *out,
                          uint32_t *age_ms,
                          uint32_t *generation)
{
    if (!have_frame || out == NULL) return false;
    __disable_irq();
    *out = latest_frame;
    uint32_t tick = latest_tick_ms;
    uint32_t gen = generation_count;
    __enable_irq();
    if (age_ms != NULL) *age_ms = HAL_GetTick() - tick;
    if (generation != NULL) *generation = gen;
    return true;
}

bool vision_link_get_latest(vision_frame_t *out)
{
    return vision_link_snapshot(out, NULL, NULL);
}

uint32_t vision_link_get_age_ms(void)
{
    if (!have_frame) return UINT32_MAX;
    return HAL_GetTick() - latest_tick_ms;
}

bool vision_link_is_fresh(uint32_t max_age_ms)
{
    return have_frame && vision_link_get_age_ms() <= max_age_ms;
}

void vision_link_get_stats(vision_link_stats_t *out)
{
    if (out == NULL) return;
    __disable_irq();
    *out = stats;
    __enable_irq();
}
