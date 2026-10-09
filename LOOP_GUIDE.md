# Guided Dart 构建与 LoRa 闭环

## 工程入口

只在仓库根目录下执行构建。根目录源码就是 App，`bootloader/` 是启动工程，`groundstation/` 是 Windows 上位机，`tools/` 是命令行工具，`out/` 是正式产物。

## 固件编译

```powershell
cd D:\JetBrains\CLionworkspace\Guided_dart_2027
.\build_all.ps1 -Configuration Release -Version 1
```

该命令只构建 App、Bootloader 和固件产物，不打包上位机。Boot、App、App-SWD 和合并 BIN/ELF 全部输出到 `out\firmware`。

如需显式指定工具链：

```powershell
.\build_all.ps1 -Toolchain "D:\JetBrains\CLion 2023.3.5\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gcc.exe"
```

烧录方式：

```powershell
.\tools\flash_swd.ps1 -Mode Boot     # 仅 Boot
.\tools\flash_swd.ps1 -Mode App      # 仅 App
.\tools\flash_swd.ps1 -Mode BootApp  # 分别烧录 Boot + App
.\tools\flash_swd.ps1 -Mode All      # 合并整片
```

开始前可先运行一次工程检查：

```powershell
.\tools\doctor.ps1 -Port <COM口>
```

## 物理连接（保持现状）

主控 LoRa 使用 USART1：PA9 为 TX、PA10 为 RX、115200 8N1、无硬件流控，DMA 仍为 DMA1 Channel3/4。USART2 继续连接视觉模块，921600；IMU 继续使用 SPI1 PA5/PA6/PA7。外挂 W25Q128 使用原理图已有 SPI2：PB12 片选、PB13 时钟、PB14 MISO、PB15 MOSI。

电脑端 LoRa 仍接 <COM口>（端口号以系统实际枚举为准）。透明通信要求两端频段、信道、空口速率和模式一致；两端本地 UART 速率分别匹配各自模块即可。

## LoRa 配置闭环

电脑端 EWM32M：

```powershell
python .\tools\guided_dart_tool.py probe-lora <COM口> --profile ewm32 --local-baud 9600
python .\tools\guided_dart_tool.py configure-lora <COM口> --profile ewm32 --address 0 --channel 23 --air-rate 5 --uart-baud 115200
```

主控端 EWM220M-400T22S（配置模式 UART 固定 9600）：

```powershell
python .\tools\guided_dart_tool.py probe-lora <COM口> --profile ewm220
python .\tools\guided_dart_tool.py configure-lora <COM口> --profile ewm220 --uart-baud 115200 --air-rate 5 --channel 23
```

配置前把待配置模块 M0、M1 同时拉高，完成后恢复 M0=M1=0 并重新上电。信道 23 对应 433.125 MHz；`air-rate 5` 为 19.2 kbps。EWM32 与 EWM220 使用不同配置帧，工具和上位机共用根目录的 `lora_profiles.py`。

主控端模块没有引出 M0/M1 时，不能通过 SWD 读取其内部射频寄存器；SWD 只能确认 STM32 的 USART1、DMA 和收发缓存。此时电脑端只能按主控模块的已知参数配置，不能凭空反推出未知信道。

## 上位机

```powershell
cd .\groundstation
python -m pip install -r requirements.txt
.\build_windows.ps1
```

生成 `out\groundstation\GuidedDartGroundStation.exe`。选择实际枚举的串口后即可查看遥测、读写运行期参数和进入升级页。

OTA 页面必须选择 `out\firmware\Guided_dart_app.bin`（或 `out\build\app\Guided_dart_2027.bin`）。`out\firmware\Guided_dart_initial_full.bin` 只用于首次 ST-Link 整片烧录，不能作为无线升级输入。

## 发布前自检

```powershell
python .\tools\test_protocol.py
python .\tools\test_lora_profiles.py
python -m py_compile .\groundstation\main.py .\groundstation\protocol.py .\lora_profiles.py .\tools\guided_dart_tool.py
```

## OTA 当前实现边界

外挂 W25Q128 已划分为两个 128 KB 镜像槽：A=`0x000000`、B=`0x020000`，槽头和活动槽元数据独立保存。Bootloader 接收升级包时先写备用槽，不再先擦除正在运行的内部 App；整包 CRC 校验成功后才复制到内部 App。复制中断或掉电导致内部 App 无效时，Bootloader 会从上一次活动槽恢复。

应用运行期的看门狗级故障确认/自动回滚还没有启用，等 LoRa 链路和现场升级流程验证后再增加，避免把故障定位和无线链路问题混在一起。
