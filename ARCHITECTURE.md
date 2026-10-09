# Guided Dart 工程架构

## 顶层目录

```text
Guided_dart_2027/
├─ Core/                STM32G431 App 的启动、外设初始化和中断
├─ Drivers/             STM32 HAL、CMSIS 和芯片厂商代码
├─ USB_Device/          USB CDC 设备代码
├─ component/           App 业务代码
├─ bootloader/          独立 Bootloader 工程
│  ├─ include/          启动配置、协议和外挂存储接口
│  └─ src/              启动流程、升级协议和存储实现
├─ groundstation/       Windows 上位机源码和打包脚本
├─ tools/               镜像、模块配置、烧录和自检工具
├─ docs/                测试、布线和 OTA 方案文档
└─ out/                 唯一构建输出目录，不提交到 Git
```

## App 业务分层

```text
Core/Src/main.c
    ↓
component/app_runtime.c       启动顺序和主循环服务编排
    ├─ lora_update.c           LoRa 接收、协议帧和升级请求
    │   ├─ lora_protocol.c     帧格式和 CRC16
    │   └─ lora_param.c        设备信息、参数读写和 ACK
    ├─ external_flash.c        W25Q128 SPI2 驱动
    ├─ imu_log.c               200 Hz 原始 IMU 日志、批量写入和分块读取
    └─ dart_control.c          200 Hz 飞控控制链路
        ├─ imu / ahrs
        ├─ flight_state
        ├─ guidance
        ├─ rate_control
        ├─ control_allocation
        └─ servo
```

`app_runtime.c` 是 App 唯一的服务编排入口；驱动和业务模块不反向调用主循环。CubeMX 生成文件只负责硬件初始化，中断回调通过明确的模块接口转发。

## Bootloader 分层

- `boot_main.c`：启动入口和 HAL 初始化；
- `boot_protocol.c`：内部 App 校验、LoRa OTA 状态机、Flash 复制和跳转；
- `boot_external_storage.c`：外挂 Flash A/B 槽位和活动槽位元数据；
- `external_flash.c`：复用 App 的 SPI2/W25Q128 底层驱动。

Bootloader 与 App 共用 `lora_protocol.h` 和芯片 HAL，但拥有独立链接脚本和独立构建目录。

## 上位机分层

- `groundstation/protocol.py`：帧、CRC、镜像校验、参数消息、ZUPT 和连续时间 B 样条轨迹优化；
- `groundstation/main.py`：Tk UI、串口线程、队列和页面事件；
- `lora_profiles.py`：EWM32/EWM220 配置寄存器和项目默认参数；
- `tools/guided_dart_tool.py`：无界面镜像和模块配置命令；
- `groundstation/build_windows.ps1`：PyInstaller 单文件打包。

上位机串口读取线程只负责解析和入队，UI 线程只负责消费队列和更新控件；升级线程通过独立的 ACK 等待函数运行，不直接操作 Tk 控件。

轨迹处理分为实时观测和离线拟合两层：实时层保留遥测样本，离线层在用户点击拟合后运行 ZUPT 或连续时间 B 样条。带 `0x10` 速度方向字段时，三轴投影优先使用固件弹道方向；B 样条同时保留速度积分位移基线，避免平滑项压缩恒速段距离。原始 IMU 日志由 `imu_log.c` 在控制周期复制到 RAM，再由主循环批量写入外挂 Flash；回传使用独立日志消息和偏移重试，不与 OTA 数据帧复用。

## 规范约定

- 新增业务代码放入 `component/`、`bootloader/` 或 `groundstation/`，不把业务逻辑写进 CubeMX 生成驱动；
- 所有新注释使用简体中文；厂商代码和历史注释不批量改写；
- App BIN 用于 OTA，整片 BIN 只用于首次 ST-Link 烧录；
- 构建目录统一为 `out/`，不在源码目录生成临时构建目录；
- LoRa 工程默认信道 23、空口速率编号 5（19.2 kbps）、数据 UART 115200，配置 UART 9600；
- 上位机将飞行阶段枚举映射为中文：待机、已武装、发射、稳定、目标捕获、制导、滑翔、失效保护、结束；
- 修改协议字段时必须同步修改 C 端、Python 端和 `tools/test_protocol.py`。
