/**
 * @file    boot_main.c
 * @brief   Bootloader 的 HAL、GPIO、SPI2 初始化入口。
 * @details 完成最小硬件初始化后交由 boot_protocol_run 管理恢复、升级和跳转。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 */
#include "boot_protocol.h"
#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "spi.h"

int main(void)
{
    HAL_Init();
    MX_GPIO_Init();
    MX_SPI2_Init();
    boot_protocol_run();
    while (1) HAL_Delay(1000U);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
