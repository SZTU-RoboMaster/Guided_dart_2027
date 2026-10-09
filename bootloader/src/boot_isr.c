/**
 * @file    boot_isr.c
 * @brief   Bootloader 的 SysTick 中断入口。
 * @details 维持 HAL 毫秒计时，供 LoRa 接收超时和启动窗口判定使用。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 */
#include "stm32g4xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}
