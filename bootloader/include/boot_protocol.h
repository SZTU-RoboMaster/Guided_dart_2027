/**
 * @file    boot_protocol.h
 * @brief   Bootloader OTA 服务与 App 跳转接口。
 * @author  Guided Dart 项目维护者
 * @date    2026-10-07
 * @version 1.0
 */
#ifndef BOOT_PROTOCOL_H__
#define BOOT_PROTOCOL_H__

#include <stdbool.h>

void Error_Handler(void);
/** @brief 启动 LoRa 接收循环；满足跳转条件时进入有效 App。 */
void boot_protocol_run(void);
/** @brief 核验内部 App 元数据、向量表和镜像 CRC32。 */
bool boot_app_is_valid(void);
/** @brief 调用前必须确认 App 有效；切换向量表及 MSP 后跳转。 */
void boot_jump_to_app(void);

#endif /* BOOT_PROTOCOL_H__ */
