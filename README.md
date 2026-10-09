# Guided Dart 工程总入口

当前工程由 App、Bootloader、LoRa 协议、Windows 上位机和工具链组成。

完整构建、接线、首次烧录和测试步骤见 [BUILD_GUIDE.md](BUILD_GUIDE.md)。

当前 LoRa 优先的统一操作说明见 [LOOP_GUIDE.md](LOOP_GUIDE.md)。

目录职责和模块边界见 [ARCHITECTURE.md](ARCHITECTURE.md)，OTA 升级操作与完整状态流程见 [docs/OTA_UPGRADE_FULL_FLOW.md](docs/OTA_UPGRADE_FULL_FLOW.md)，协议设计和故障处理见 [docs/OTA_DESIGN.md](docs/OTA_DESIGN.md)。

常用命令：

```powershell
cd D:\JetBrains\CLionworkspace\Guided_dart_2027
.\build_all.ps1 -Configuration Release -Version 1
.\tools\flash_swd.ps1 -Mode All
```

输出统一放在 `out\`：

```text
out\build\app\                 App 构建目录
out\build\bootloader\          Bootloader 构建目录
out\firmware\Guided_dart_bootloader.bin/.elf
out\firmware\Guided_dart_app.bin/.elf
out\firmware\Guided_dart_app_swd.bin
out\firmware\Guided_dart_initial_full.bin/.elf
```

Boot、App、Boot+App 和合并整片的烧录命令见 [BUILD_GUIDE.md](BUILD_GUIDE.md)。本固件构建入口不打包上位机。

当前仍保持 Bootloader 16KB、App 110KB、元数据 2KB 的内部布局；外挂 W25Q128 已提供 A/B 镜像槽，Bootloader 先写外部备用槽，校验通过后再替换内部 App。
