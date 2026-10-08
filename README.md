# Guided Dart 2027 制导镖工程

本工程包含 STM32G431 控制固件与 OpenMV 视觉前端程序，适配以下硬件：

- STM32G431CBU 主控
- 通过 SPI1 连接的 ICM-45686 IMU
- 通过 USART2 连接的 OpenMV H7 Plus 彩色相机，波特率为 921600
- PB4、PB5、PB6、PB7 上的四路 PWM 舵机

## 控制架构

软件采用级联控制架构：

1. OpenMV 使用 RGB565/LAB 识别绿色目标，仅输出当前帧原始位置、像素速度和置信度。
2. STM32 对视觉位置和速度进行唯一一次 α-β 滤波，根据 IMU 滚转角消除相机画面的滚转耦合，并结合系统延迟预测当前视线角。
3. 视觉外环根据视线角和视线角速度生成俯仰、偏航角速度指令。
4. IMU 角速度反馈构成 200 Hz 内环。
5. 可配置的 3x4 舵效矩阵将三轴控制量分配给四个舵面。

飞行状态机包含待发射、发射屏蔽、自稳、目标捕获、制导、短时丢失、失效保护和飞行结束等阶段。

## 重要文件

- `component/dart_config.h`：相机、IMU安装方向、控制增益、舵机和舵效矩阵等集中参数。
- `component/dart_control.c`：200 Hz 姿态估计、状态机和主控制流程。
- `component/guidance.c`：视线角/视线角速度制导、滚转解耦和延迟补偿。
- `component/rate_control.c`：IMU角速度内环。
- `component/control_allocation.c`：三轴控制量到四个舵面的控制分配。
- `component/vision_link.c`：带 CRC16 校验的 V5 视觉串口协议解析器。
- `openmv_vision_guidance_2027.py`：与 STM32 V5 协议配套的 OpenMV 彩色视觉程序。
- `docs/testing/`：按优先级整理的参数标定、台架调试、飞行验证和记录模板。
- `tools/protocol_selftest.py`：视觉协议字段与 CRC16 自检工具。

## 编译方法

确保 STM32CubeCLT 中的 `cmake`、`ninja` 和 `arm-none-eabi-gcc` 已加入 `PATH`，然后在工程根目录执行：

```powershell
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --parallel 8
```

生成的固件位于：

- `build-release/Guided_dart_2027.elf`
- `build-release/Guided_dart_2027.hex`
- `build-release/Guided_dart_2027.bin`

当前工程中的 USART2 已配置为 921600 baud，USART2 RX/TX DMA 均为 Normal 模式。使用 CubeMX 重新生成代码时，需要保留这些设置，否则视觉 DMA 接收逻辑可能无法正常工作。

## OpenMV 使用方法

使用 OpenMV IDE 打开并运行 `openmv_vision_guidance_2027.py`。完成台架验证后，可将其保存为相机上的 `main.py`，使其上电自动运行。视觉端不做 α-β 滤波和位置提前，相关滤波与延迟补偿统一由 STM32 完成。

为保证视觉帧率，默认设置如下：

- 图像分辨率为 QVGA RGB565。
- 串口波特率为 921600。
- OSD显示关闭。
- MJPEG录像关闭。
- 视觉端提前量为零，由 STM32 统一进行延迟补偿。

距离模型默认未启用，此时 OpenMV 在协议中发送的 `distance_dm` 为零。完成目标尺寸和距离的实测标定后，填写 `DIST_A`、`DIST_B`，再将 `DISTANCE_CALIBRATED` 设置为 `True`。

## 首次上机前必须完成的标定

默认参数只能作为工程起点，不能直接视为正确的实机参数。至少需要依次确认：

1. IMU 三轴与机体滚转、俯仰、偏航方向一致。
2. 四路舵机中位、行程和正负方向正确。
3. 舵效矩阵中每个舵面对三轴力矩的符号正确。
4. 相机焦距、主点和相机安装偏角已经标定。
5. 目标位于画面不同方向时，制导指令方向正确。
6. 角速度内环稳定后，再逐步增加视觉外环增益。
7. 实测并填写图像处理延迟和舵机响应延迟。

详细顺序见 `docs/testing/README.md`。其中 `01_PARAMETER_CALIBRATION.md` 明确区分了必须实测、闭环调参、现场调参和通常固定的参数。

## 安全说明

本工程是可编译的工程基线，并不是已经完成实机验证的飞行控制器。仅凭代码无法确定相机内参、IMU安装符号、舵机方向、连杆几何和实际气动舵效。

首次测试应使用软质训练头、刚性约束装置、可靠捕获网和封闭测试区域。应先完成无桨叶/无舵角、受约束小舵量和低能量测试，再逐步扩大控制权限；不要使用默认参数直接进行无约束发射。
