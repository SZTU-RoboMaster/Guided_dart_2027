# 兼容入口

旧教程已合并到 [BUILD_GUIDE.md](BUILD_GUIDE.md)，请以该文件和 `build_all.ps1` 为准。

当前固定连接：

```text
主控 PA9  USART1_TX -> LoRa RXD
主控 PA10 USART1_RX <- LoRa TXD
主控 LoRa UART：115200 8N1
电脑端 LoRa 串口：实际枚举的电脑端 LoRa 串口（以系统实际端口为准），115200 8N1
视觉 USART2：921600
```

电脑端模块参数读取/修改只能在 M0=M1=1 的配置模式执行（固定 9600 8N1）；通信测试必须恢复 M0=M1=0，并使用 115200 8N1。不要在透明模式下发送配置命令。
