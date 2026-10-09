# Guided Dart 编译与烧录入口

本文件只维护固件编译和 ST-Link 烧录命令；上位机打包不属于本入口。

## 编译命令

```powershell
cd D:\JetBrains\CLionworkspace\Guided_dart_2027
.\build_all.ps1 -Configuration Release -Version 1
```

构建与烧录脚本均采用带 BOM 的 UTF-8 编码，可直接在 Windows PowerShell 5.1 或 PowerShell 7 中运行。`build_all.ps1` 只生成并校验固件，不会烧录主控；烧录需另行执行下方命令。

产物统一位于 `out\firmware`：

```text
Guided_dart_bootloader.bin / .elf   Boot 区，装载地址 0x08000000
Guided_dart_app.bin / .elf          App 原始镜像，供 OTA 和调试
Guided_dart_app_swd.bin             含 App 元数据，单独 SWD 烧录
Guided_dart_initial_full.bin / .elf 合并整片镜像，装载地址 0x08000000
```

无线 OTA 使用 `Guided_dart_app.bin`；单独 SWD 烧录 App 使用 `Guided_dart_app_swd.bin`。

## 烧录命令

确认 ST-Link 已连接后执行：

```powershell
# 只烧录 Bootloader 区
.\tools\flash_swd.ps1 -Mode Boot

# 只烧录 App 区（含元数据）
.\tools\flash_swd.ps1 -Mode App

# 分别烧录 Bootloader 区和 App 区
.\tools\flash_swd.ps1 -Mode BootApp

# 烧录合并整片镜像
.\tools\flash_swd.ps1 -Mode All
```

脚本会校验镜像大小、App 向量表、元数据和 CRC。固定地址为 Boot=`0x08000000`、App=`0x08004000`。需要调试时，OpenOCD 使用 `interface/stlink.cfg` 与 `target/stm32g4x.cfg`。

## 硬件约定（保持不变）

```text
LoRa：PA9/PA10 -> USART1，115200 8N1
视觉：USART2，921600
IMU：SPI1，PA5/PA6/PA7
外挂 W25Q128：SPI2，PB12 CS、PB13 SCK、PB14 MISO、PB15 MOSI
电脑端 LoRa：使用系统实际枚举的串口，上位机启动后从端口列表选择
```

不要在 App 或 Bootloader 源码目录单独创建 CMake 构建目录；统一使用仓库根目录的 `out`。
