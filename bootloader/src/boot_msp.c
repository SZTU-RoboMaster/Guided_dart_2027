/**
 * @file    boot_msp.c
 * @brief   Bootloader 的 USART1 时钟与 PA9/PA10 复用引脚配置。
 * @details 仅在 Bootloader 中初始化 LoRa 串口硬件；App 使用自己的 CubeMX
 *          外设初始化配置，两侧的 UART 波特率必须保持一致。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 */
#include "stm32g4xx_hal.h"
#include "boot_protocol.h"

UART_HandleTypeDef huart1;

void HAL_UART_MspInit(UART_HandleTypeDef *uart_handle)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    if (uart_handle->Instance != USART1) return;

    clock.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    clock.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle->Instance != USART1) return;
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
}
