/**
 * @file    lora_telemetry.h
 * @brief   LoRa 串口遥测 — USART1 115200bps
 *
 *          通过 USART1 (PA9 TX) 连接 LoRa 模块, 周期性发送:
 *            - 飞行阶段 PH (0~8) 与视觉目标坐标 CX/CY (px)
 *            - 四路舵机实际输出脉宽 S1~S4 (μs), 回读 TIM3/TIM4 的 CCR
 *
 *          报文形如: PH5 CX156 CY112 S1+1500 S2+1500 S3+1543 S4+1462
 *
 *          发送频率: 内部控制, ~20Hz (每 50ms 发一次)
 *
 *          注: 报文只含飞行阶段 PH、视觉目标坐标 CX/CY 和四路舵机脉宽
 *              S1~S4。制导量、帧率、IMU 量等不再发送, 但相关变量计算保留
 *              (见 lora_telemetry.c 内注释块, 需要时可恢复完整格式)。
 *
 *          ⚠ 本函数只在 PHASE_STABILIZE 及之后的阶段被调用。dart_control_process()
 *            在 IDLE/ARMED/LAUNCH 三处提前 return, 那条路径不会走到发送语句,
 *            所以地面收不到 PH0/1/2 —— PH2 (LAUNCH) 本身也只有 80ms。
 */
#ifndef LORA_TELEMETRY_H__
#define LORA_TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 尝试发送一帧遥测数据 (内部自动节流)
 *
 *        可在 200Hz 控制循环或主循环中调用,
 *        内部用时间间隔门控, 约 20Hz 实际发送。
 *
 *        使用 USART1 DMA 发送，非阻塞。
 *        若上次 DMA 未完成则跳过本帧。
 */
/* 完整遥测格式 (P/Q/R、LY/LP、LYR/LPR、TG、IM 等) 见实现文件内注释块。 */
/* 当前只发送 PH/CX/CY/S1~S4, 其余字段裁剪但计算保留。 */
void lora_telemetry_send(void);

#ifdef __cplusplus
}
#endif

#endif /* LORA_TELEMETRY_H__ */
