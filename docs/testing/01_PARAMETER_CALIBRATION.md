# 参数总表与标定清单

本文件是**参数侧的唯一索引**：每个可调参数是什么、当前值是多少、在哪个文件哪一行、
属于哪一层、调它的依据是什么、对应哪一步调试。

步骤侧见 [`02_TEST_PROCEDURE.md`](02_TEST_PROCEDURE.md)，
自稳层的展开见 [`05_STABILIZATION_TUNING.md`](05_STABILIZATION_TUNING.md)。

---

## 0. 怎么用这份表

1. **先定位层次。** §1 按"调试顺序"分层——执行器 → 姿态 → 内环 → 外环 →
   视觉 → 制导 → 状态机。**下层没对好时调上层没有意义**，所以层次顺序就是
   调试顺序。
2. **再看分类。** 分类决定这个值能不能"调出来"：

   | 分类 | 含义 |
   | --- | --- |
   | **必须实测** | 必须用当前实物测量或辨识，不能沿用默认值直接飞行 |
   | **闭环调参** | 没有唯一物理真值，依据阶跃响应、稳定性或命中误差调整 |
   | **数据调参** | 有真值，但要先采集数据（遥测/图像）才能算出该填多少 |
   | **现场调参** | 随光照、目标和场地变化，应使用现场图像数据调整 |
   | **必须核对** | 通常不变，但必须确认两端配置或设计约定一致 |
   | **通常固定** | 除非硬件或系统设计变化，否则不需要修改 |

3. **每次只改一行。** 一行里的参数若强相关（如同一通道的 `Kp`/`Kd`），
   算作一组，仍要逐个改。
4. **改完在 [`03_TEST_RECORD_TEMPLATE.md`](03_TEST_RECORD_TEMPLATE.md) 留记录。**

> ⚠ **遥测里已不含任何原始 IMU 量**（陀螺仪 `gx/gy/gz`、加速度 `ax/ay/az` 均已
> 从 `lora_telemetry.c` 报文移除）。凡是"看角速度响应""复核发射判定""看内环
> 振荡"的调参依据，**都不能再从遥测获得**，必须改用台架示波器看 PWM、调试器
> watch，或临时把字段加回 `snprintf`。详见 `05` 第 3 节。

---

## 1. 参数分层总表（按调试顺序）

`位置` 列的行号以本文件写作时的代码为准，改动后需同步更新。

### 1.1 执行器层 —— 最先，也最不该用增益去掩盖

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_SERVO_1..4_CENTER_US` | 1500 | `dart_config.h:204-207` | 必须实测 | 断开连杆测机械中立脉宽 |
| `DART_SERVO_1..4_DIRECTION` | 全 −1.0 | `dart_config.h:208-211` | 必须实测 | **方向反了改这里，不是改增益** |
| `DART_SERVO_SPAN_US` | 500.0 | `dart_config.h:212` | 必须实测 | 逐步加脉宽测实际舵角，不碰机械限位 |
| `DART_SERVO_DELTA_LIMIT` | 0.70 | `dart_config.h:213` | 必须实测 | 首次带电测试临时降 `0.20f`；全组合不卡滞 |
| `DART_SERVO_RATE_LIMIT_PER_S` | 6.0 | `dart_config.h:214` | 必须实测 | 满行程 0.33 s；**必须实测舵机带载能否达到** |
| `DART_SERVO_DEADBAND` / `_BOOST` | 0 / 0 | `dart_config.h:215-216` | 必须实测 | 两者必须同时改，见 `05` 第 7.6 节 |
| `DART_B_ROLL/PITCH/YAW_1..4` | ±0.5 / ±0.6 | `dart_config.h:219-230` | 必须实测 | 单舵偏转测力矩；符号+相对效能都要来自实测 |
| `DART_Q_NOMINAL_SPEED_MPS` | 15.0 | `dart_config.h:145` | 通常固定 | `q_scale = (V_est/V_nom)²`，改动整体缩放控制权限 |

### 1.2 姿态估计层 —— 自稳的隐式前置条件

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_BODY_*`（9 个映射系数） | 见文件 | `dart_config.h:193-201` | 必须实测 | 绕三轴正转，`p/q/r` 与姿态角符号；矩阵正交且行列式 +1 |
| `DART_AHRS_KP_GROUND` | 1.0 | `dart_config.h:149` | 闭环调参 | 地面静态收敛速度 vs 噪声 |
| `DART_AHRS_KI_GROUND` | 0.015 | `dart_config.h:150` | 闭环调参 | **陀螺漂移的唯一对手**；漂了先查它和零偏标定 |
| `DART_AHRS_ACCEL_MIN_G` / `MAX_G` | 0.86 / 1.14 | `dart_config.h:151-152` | 数据调参 | 静态与发射数据，区分可信重力与动态比力 |
| `DART_AHRS_INFLIGHT_ACCEL_WEIGHT` | 0.01 | `dart_config.h:153` | 数据调参 | 飞行姿态漂移 vs 动态加速度污染；**当前基本等于纯陀螺积分** |

> 飞行中姿态靠陀螺积分 → **滚转姿态会漂**，而滚转是俯仰/偏航舵面作用方向的
> 前提。这使地面零偏标定（`imu_calibrate_gyro_bias()`）成为自稳的隐式前置条件。

### 1.3 自稳内环层（速率环路）

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_RATE_KP_P/Q/R` | 0.20 / 0.22 / 0.26 | `dart_config.h:132-134` | 闭环调参 | 顶到 `MOMENT_LIMIT` 的速率误差 = 0.65/Kp（3.25/2.95/2.50 rad/s） |
| `DART_RATE_KD_P/Q/R` | 0.010 / 0.010 / 0.012 | `dart_config.h:135-137` | 闭环调参 | Kp 稳定后加，抑超调与高频振 |
| `DART_RATE_ACCEL_LPF_ALPHA` | 0.19 | `dart_config.h:138` | 闭环调参 | 200 Hz 下 τ ≈ 23.7 ms。**噪声大就降它，不是降 KD** |
| `DART_MOMENT_LIMIT` | 0.65 | `dart_config.h:139` | 闭环调参 | 三轴力矩限幅；滚转满力矩只用 46% 舵面位置余量 |

**调试方法见 [`05_STABILIZATION_TUNING.md`](05_STABILIZATION_TUNING.md) 第 4 节 S2/S3。**

### 1.4 自稳外环层 —— 硬编码在 `dart_control.c`，**不在 config 里**

| 参数（代码写法） | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `-1.2f * data.flight_roll` | 1.2 | `dart_control.c:113` | 闭环调参 | 滚转恢复时间 vs 过冲/摆头 |
| `clampf(..., -2.0f, 2.0f)` | ±2.0 rad/s | `dart_control.c:113` | 闭环调参 | 角误差 96° 即饱和 |
| `-0.35f * data.r` | 0.35 | `dart_control.c:115` | 闭环调参 | 偏航阻尼；**无角度基准，只是阻尼** |
| `clampf(..., -1.0f, 1.0f)` | ±1.0 rad/s | `dart_control.c:115` | 闭环调参 | r = 2.86 rad/s 饱和 |
| `my = 0.0f`（STABILIZE/ACQUIRE） | 强制零 | `dart_control.c:422-425` | **待确认** | 抹掉俯仰全部角速度阻尼，见 `05` 第 7.1 节 |

### 1.5 视觉链路层

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_VISION_BAUD` | 921600 | `dart_config.h:28` | 必须核对 | 与 OpenMV 端一致 |
| `DART_VISION_CONF_ACQUIRE` / `HOLD` | 58 / 52 | `dart_config.h:32-33` | 数据调参 | 按遥测 `CF`（置信度峰值）定：`ACQUIRE ≈ 0.85·CF` |
| OpenMV `CONF_STRONG` / `CONF_REACQUIRE` | 72 / 68 | `openmv_*.py` | 数据调参 | 同上，`≈ 0.85·CF` / `0.75·CF` |
| OpenMV `CONF_ACCEPT` | 52 | `openmv_*.py` | 数据调参 | **必须满足 `CONF_ACCEPT ≤ HOLD ≤ ACQUIRE`** |
| `DART_VISION_FRESH_MS/COAST_MS/FAILSAFE_MS` | 45 / 120 / 280 | `dart_config.h:34-36` | 数据调参 | 满足 `FRESH < COAST < FAILSAFE`；按实测帧率与抖动 |
| `DART_VISION_LOCK_FRAMES` | 3 | `dart_config.h:37` | 数据调参 | 首次锁定速度 vs 误锁率 |
| `DART_CAMERA_FX/FY/CX/CY_PX` | 320/240/160/120 | `dart_config.h:20-23` | 必须实测 | 当前镜头 + QVGA 下标定，**`CX/CY` 不能默认等于几何中心** |
| `DART_CAMERA_YAW/PITCH_OFFSET_RAD` | 0 / 0 | `dart_config.h:24-25` | 必须实测 | 固定镖体对准已知方位目标，LOS 应接近零 |
| `DART_CAMERA_PIPELINE_DELAY_S` | 0.005 | `dart_config.h:40` | 必须实测 | 采集 → OpenMV 检测 → STM32 收帧，三级合并 |
| `DART_SERVO_DELAY_S` | 0.002 | `dart_config.h:41` | 必须实测 | PWM 指令 → 带载舵面开始动作 |
| `DART_ROLL_PHASE_LEAD_ENABLE` | 1 | `dart_config.h:42` | 必须核对 | 滚转去旋相超前使能；依赖上一条延迟值的准确性 |
| `GREEN_THRESHOLD_*`、`EXPOSURE_US`、`GAIN_DB` | 见脚本 | `openmv_*.py` | 现场调参 | 12.5 m 实测距离下的 LAB 分布与曝光 |
| `BLOB_*`、`MIN_*`、`MAX_*` | 见脚本 | `openmv_*.py` | 现场调参 | 12.5 m 处目标约 7.2 px，门限过高会直接漏检 |
| `DIST_A` / `DIST_B` | 已标定 | `openmv_*.py` | 必须实测 | ≥6 个已知距离拟合 `Z_mm = A/r + B`；`DISTANCE_CALIBRATED` 已置 True |
| `ROI_HALF_TRACK`、`ROI_FULL_SCAN_EVERY` | 见脚本 | `openmv_*.py` | 数据调参 | 帧间位移、重捕获时间与帧率的折中 |
| `JUMP_*`（跳变门控三参数） | 见脚本 | `openmv_*.py` | 数据调参 | 依据真实目标运动与误检跳变样本 |

### 1.6 制导层

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_GUIDANCE_YAW_SIGN` / `PITCH_SIGN` | 1.0 / 1.0 | `dart_config.h:45-46` | 必须核对 | 移动目标看机体修正方向，**只允许取正确符号** |
| `DART_GUIDANCE_NAV_RATIO`（N） | 3.0 | `dart_config.h:66` | 闭环调参 | 从默认值 50% 起步，看碰撞航线收敛；经典上限约 5 |
| `DART_GUIDANCE_K_ACCEL_FEED` | 0.5 | `dart_config.h:67` | 闭环调参 | **首次闭环先置零**，λ̈ 遥测干净后从 0.1~0.2 起 |
| `DART_GUIDANCE_K_CAPTURE` | 0.6 | `dart_config.h:68` | 闭环调参 | 目标偏离视场中心时加；过大→末端过载陡增 |
| `DART_GUIDANCE_CONF_SCALE_MIN` | 0.7 | `dart_config.h:72` | 闭环调参 | 等价于缩放 N；**过低会让 N 掉出稳定区间（N ≥ 2）** |
| `DART_GUIDANCE_MODE_ISMCG` | 0（APN） | `dart_config.h:78` | 通常固定 | 本任务**不做 A/B 对比**，理由见 `04` 第 3 节 |
| `DART_GUIDANCE_FILTER_ALPHA` / `BETA` | 0.58 / 0.12 | `dart_config.h:90-91` | 数据调参 | 依据视线角噪声折中 |
| `DART_GUIDANCE_FILTER_GAMMA` | 0.0 | `dart_config.h:94` | 通常固定 | 已置零退化为 α-β；λ̈ 改由 λ̇ 差分得到 |
| `DART_GUIDANCE_FILTER_ACC_LIMIT` | 3.0 | `dart_config.h:103` | 数据调参 | λ̈ 限幅（rad/s²）。物理峰值 0.29，噪声 σ 0.05~0.13 |
| `DART_GUIDANCE_FILTER_RATE_TAU_S` | 0.093 | `dart_config.h:111` | 数据调参 | λ̇ 低通**时间常数**，帧率变化时带宽不变 |
| `DART_GUIDANCE_FILTER_DD_TAU_S` | 0.205 | `dart_config.h:115` | 数据调参 | λ̈ 差分低通时间常数。**停止瞬态反向过冲时增它** |
| `DART_GUIDANCE_NEAR_TGO_S` / `FAR_TGO_S` | 0.30 / 0.90 | `dart_config.h:121-122` | 飞行调参 | 按实测 `TA` 回填：`FAR ≈ TA`、`NEAR ≈ 0.3~0.4·TA` |
| `DART_GUIDANCE_RATE_LIMIT` | 3.5 | `dart_config.h:125` | 闭环调参 | 避免外环要求超过内环与舵机能力；本任务仅用到 12% |
| `DART_GUIDANCE_ROLL_KP` | 1.4 | `dart_config.h:128` | 闭环调参 | 制导段滚转保持。**与自稳段的 1.2 是两个旋钮** |
| `DART_GUIDANCE_ROLL_RATE_LIMIT` | 2.5 | `dart_config.h:129` | 闭环调参 | 制导段滚转指令限幅 |
| `DART_GUIDANCE_PITCH_BIAS_RATE` | 0.0 | `dart_config.h:59` | 命中调参 | **最后调**。初值 `dq ≈ −2·N·e/(V·t_go²)`；灵敏度含 `t_go²` |
| `DART_GUIDANCE_ISMCG_*`（7 个） | 见文件 | `dart_config.h:81-87` | 通常固定 | 仅 `MODE_ISMCG = 1` 时生效 |
| `ca_get_T_go_weights` 四档阈值 | `>3.0 / >1.5 / >0.5 / else` | `control_allocation.c:74-97` | 飞行调参 | 按 `TA` 归一化重映射；**因 B 正交，实际只通过正则项起作用** |

### 1.7 飞行状态机与发射判定层

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_LAUNCH_ACCEL_MPS2` | 20.4 | `dart_config.h:156` | 必须实测 | 装填/搬运振动 vs 真实发射加速度 |
| `DART_LAUNCH_CONFIRM_S` | 0.010 | `dart_config.h:157` | 必须实测 | 同上，避免误触发或漏触发 |
| `DART_LAUNCH_HOLDOFF_S` | 0.080 | `dart_config.h:158` | 必须实测 | 发射机构完全脱离、舵面可安全动作的时刻 |
| `DART_IMPACT_ACCEL_MPS2` | 34.0 | `dart_config.h:159` | 必须实测 | 发射冲击与落地冲击；避免把发射误判为撞击 |
| `DART_IMPACT_BLANKING_S` | 0.8 | `dart_config.h:160` | 必须实测 | 同上 |
| `DART_MAX_FLIGHT_TIME_S` | 60.0 | `dart_config.h:161` | 通常固定 | 安全配置，按试验区域和任务上限 |

### 1.8 速度模型与弹道估计层

| 参数 | 当前值 | 位置 | 分类 | 调它的依据 / 看什么 |
| --- | --- | --- | --- | --- |
| `DART_V_LAUNCH_MPS` | 18.0 | `dart_config.h:176` | 必须实测 | 测速仪/高速视频拟合；**约束 ∫₀² V dt = 25 m** |
| `DART_V_TAU_S` | 3.8 | `dart_config.h:177` | 必须辨识 | 与上一条**必须一起辨识**，单改一个拟合不出曲线形状 |
| `DART_V_MIN_MPS` | 5.0 | `dart_config.h:178` | 通常固定 | 速度衰减下限 |
| `DART_SPEED_DECAY_ENABLE` | 1 | `dart_config.h:175` | 通常固定 | 关闭则 `V_est` 恒为 `V_LAUNCH` |
| `DART_BALLISTIC_MIN_SPEED_MPS` | 5.0 | `dart_config.h:183` | 数据调参 | **仅影响遥测**，不改变制导指令 |
| `DART_BALLISTIC_ACCEL_LPF_ALPHA` | 0.12 | `dart_config.h:184` | 数据调参 | **仅影响遥测** |
| `DART_BALLISTIC_TRACK_RATE_LPF_ALPHA` | 0.15 | `dart_config.h:185` | 数据调参 | **仅影响遥测** |
| `DART_BALLISTIC_MAX_LATERAL_ACCEL_MPS2` | 30.0 | `dart_config.h:186` | 必须核对 | **仅影响遥测**；与气动能力和 IMU 量程一致即可 |
| `DART_BALLISTIC_GRAVITY_MPS2` | 9.80665 | `dart_config.h:182` | 通常固定 | 标准常数，不通过飞行调参 |

> 弹道估计器仍输出速度方向/航迹角供遥测，但**已不再作为制导前馈**
> （增广比例导引的 λ̇ 项已隐含速度方向失配）。因此这一层的任何取值都**不会
> 改变制导指令**，不需要为命中精度调它。

---

## 2. 必须实测的实物标定项（P0/P1）

这些参数直接取决于当前相机、安装方式、舵机、连杆和镖体，**不应直接使用默认值
进行无约束发射**。完成标定后应在测试记录中填写测量方法、原始数据和最终值。

| 参数 | 分类 | 标定方法 | 验收要求 |
| --- | --- | --- | --- |
| `DART_BODY_*` | 必须实测 | 分别绕机体三轴做正向旋转，对照 `p/q/r` 和姿态角符号 | 三轴满足右手定则，映射矩阵正交且行列式为 +1 |
| `DART_SERVO_1..4_CENTER_US` | 必须实测 | 断开连杆，测量舵面机械中立位置对应脉宽 | 上电回中无预载、无明显偏角 |
| `DART_SERVO_1..4_DIRECTION` | 必须实测 | 每路施加小幅正负指令，观察真实舵面方向 | 与舵面编号和控制坐标约定一致 |
| `DART_SERVO_SPAN_US` | 必须实测 | 逐步增加脉宽并测量实际舵角 | 覆盖所需舵角且不碰机械限位 |
| `DART_SERVO_DELTA_LIMIT` | 必须实测 | 从 `0.20f` 开始，检查全组合指令下的连杆余量 | 任意组合均不卡滞、不撞限位 |
| `DART_SERVO_RATE_LIMIT_PER_S` | 必须实测 | 记录带载舵机阶跃曲线 | 不高于实物可重复达到的变化率 |
| `DART_SERVO_DEADBAND`、`DART_SERVO_DEADBAND_BOOST` | 必须实测 | 缓慢双向扫动脉宽，记录开始动作和回差 | 小指令可重复动作且无来回抖动 |
| `DART_B_ROLL_1..4` | 必须实测 | 单舵偏转，测量滚转响应或力矩 | 符号和相对效能来自当前镖体 |
| `DART_B_PITCH_1..4` | 必须实测 | 单舵偏转，测量俯仰响应或力矩 | 同上 |
| `DART_B_YAW_1..4` | 必须实测 | 单舵偏转，测量偏航响应或力矩 | 同上 |
| `DART_CAMERA_FX_PX`、`DART_CAMERA_FY_PX` | 必须实测 | 使用当前镜头、分辨率完成相机标定 | 重投影误差满足视觉角度精度要求 |
| `DART_CAMERA_CX_PX`、`DART_CAMERA_CY_PX` | 必须实测 | 与相机内参同时标定 | 使用标定结果，不能默认等于图像几何中心 |
| `DART_CAMERA_YAW_OFFSET_RAD` | 必须实测 | 固定镖体，对准已知方位目标测量安装误差 | 目标在机头轴线上时 LOS 偏航接近零 |
| `DART_CAMERA_PITCH_OFFSET_RAD` | 必须实测 | 固定镖体，对准已知俯仰目标测量安装误差 | 目标在机头轴线上时 LOS 俯仰接近零 |
| `DART_CAMERA_PIPELINE_DELAY_S` | 必须实测 | 用 LED 跳变、逻辑分析仪或高速相机测量采集到 STM32 收帧时间 | 使用多次测量的稳定统计值 |
| `DART_SERVO_DELAY_S` | 必须实测 | 同步记录 PWM 指令和舵面开始响应时刻 | 必须包含带载情况下的执行延迟 |
| `GREEN_THRESHOLD_STRICT`、`GREEN_THRESHOLD_WIDE` | 现场调参 | 采集比赛场地不同距离和光照下的 LAB 分布 | 目标召回率可接受且背景误检受控 |
| `EXPOSURE_US`、`GAIN_DB` | 现场调参 | 对远近目标分别检查过曝、拖影、噪声和帧率 | 目标不饱和，运动时轮廓仍可检测 |
| `DIST_A`、`DIST_B` | 必须实测 | 至少六个已知距离采集 `radius_px`，拟合 `Z_mm=A/r+B` | 残差满足 `T_go` 使用要求后才能启用 |

### 本任务（25 m 剖面）新增标定项

| 项目 | 分类 | 标定方法 | 验收要求 |
| --- | --- | --- | --- |
| **12.5 m 处捕获率与捕获时刻** | 必须实测 | 在 12.5 m 实测距离统计首次捕获帧数、捕获时刻和误锁率 | 捕获时刻可重复，且 12.5 m 起能连续跟踪到命中不丢帧 |
| 12.5 m 处 `radius_px` | 必须实测 | 作为 `DIST_A` 距离反演在制导段起点的精度校验点（`r ≈ 2 px`） | 量化导致的 `t_go` 误差在可接受范围 |
| `BLOB_*`、`MIN_*`、`MAX_*` | 现场调参 | 12.5 m 处目标约 7.2 px；门限过高会直接漏检 | 捕获后不丢帧，且不误检场馆顶灯与背景绿 |
| **停止瞬态** | 必须验证 | 台架复现 1.2 s 目标停止，记录停止前后 0.3 s 的 `los_*_rate` 与指令 | 无阶跃、无反向过冲；见 `02` 第 16 节 |
| **自稳三轴响应** | 闭环调参 | 台架阶跃 + 示波器看 PWM；遥测已无 IMU 量 | 无持续振荡、无末端饱和；见 `05` 第 4 节 |

---

## 3. 任务剖面耦合参数（改一处要连带改）

`DART_TARGET_RANGE_M`（当前 25 m）是任务给定值。**改动靶距时以下数值全部耦合**，
宏替代不了：

| 连带项 | 耦合关系 |
| --- | --- |
| `DART_V_LAUNCH_MPS` / `DART_V_TAU_S` | 约束 `∫₀² V dt = DART_TARGET_RANGE_M` |
| `DART_GUIDANCE_NEAR_TGO_S` / `FAR_TGO_S` | 由捕获距离与 `V_est` 推算 |
| `ca_get_T_go_weights` 四档阈值 | 按实测 `TA` 归一化重映射 |
| `DART_GUIDANCE_PITCH_BIAS_RATE` | 灵敏度含 `t_go²`，**固定偏置只能适配一种交战几何** |
| `DART_Q_NOMINAL_SPEED_MPS` | 与 `V_est` 耦合，`q_scale = (V_est/15)²` |

详见 [`04_MISSION_PROFILE.md`](04_MISSION_PROFILE.md) 第 5 节。

---

## 4. 当前必须注意的默认值

按"会不会立刻造成错误"排序。

1. **`DART_V_LAUNCH_MPS` = 18.0 / `DART_V_TAU_S` = 3.8 与 25 m / 2 s 剖面不自洽**
   ——该模型预测 2 s 行程 28.0 m（偏高 3 m）。误差会污染 `t_go` 的**每一个**
   消费者（`cap_w`、`ca_get_T_go_weights`、`q_scale` 按平方放大）。在速度曲线
   实测前，临时取 `V_LAUNCH` = 16.1（保持 τ）或 τ = 2.56（保持 V₀）。
   **不要继续试凑 `V_LAUNCH`。** 见 `04` 第 5.1 节。
2. **`my = 0.0f` 在 STABILIZE/ACQUIRE 抹掉俯仰全部角速度阻尼**
   （`dart_control.c:422-425`）。偏航有阻尼、俯仰没有，两轴不对称，且无注释。
   **先确认是有意为之还是遗留。** 见 `05` 第 7.1 节。
3. **`DART_VISION_CONF_ACQUIRE`（58）/ `CONF_STRONG`（72）/ `CONF_REACQUIRE`（68）
   都还没按 12.5 m 实测标定。** 跑一次剖面读遥测 `CF`，再按 `0.85×CF` /
   `0.75×CF` 定。**若峰值低于这些门限，依赖它们的门控会静默失效、不报错。**
4. **`DART_CAMERA_PIPELINE_DELAY_S`（0.005）/ `DART_SERVO_DELAY_S`（0.002）是
   占位值**，必须实测。两者同时喂给滚转去旋的相位超前项
   （`DART_ROLL_PHASE_LEAD_ENABLE`），延迟偏大时超前量会过冲。
5. **所有 `DART_B_*` 都是经验矩阵**，必须经当前舵面布局和实物响应确认，
   不能只验证符号而保留随意的相对大小。
6. **`DART_GUIDANCE_K_ACCEL_FEED` 当前 0.5**，首次闭环应先置零（从 0.1~0.2 起步），
   λ̈ 遥测干净后再逐步增加。
7. **`DART_SERVO_DELTA_LIMIT` 当前 0.70**，首次带电约束测试前临时改为 `0.20f`。
8. **`DART_Q_NOMINAL_SPEED_MPS`（15）与速度模型耦合**：`q_scale = (V_est/15)²`。
   速度偏 10% → 控制权限偏 21%。**这是台架-飞行不一致的头号原因。**
9. **`DART_GUIDANCE_MODE_ISMCG` = 0（APN）**。台架对比两套方案时切 `1`，
   其余增益保持不变。
10. **`DISTANCE_CALIBRATED` 已置 `True`**，`DIST_A` 标定完成。但 12.5 m 处半径仅
    2 px 量级，距离反演的量化噪声需在台架上量化后再决定 `t_go` 的可信区间。
11. **遥测已不含任何原始 IMU 量**。发射判定、姿态估计、内环振荡都无法从遥测
    复核，必须用示波器看 PWM 或临时加回字段。见 `05` 第 3 节。
