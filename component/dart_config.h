#ifndef DART_CONFIG_H
#define DART_CONFIG_H

/* 所有首飞参数集中在此处。当前取值是较保守的初始值；飞行前须在带约束的
 * 台架上核对符号与限幅。 */

/* ── 任务剖面常量 ──
 * 本任务 (25 m 靶距 / 2 s 滞空 / 560 mm 横向机动) 的几何约束。
 * 改动靶距时必须同步重标: NEAR_TGO_S/FAR_TGO_S、ca_get_T_go_weights 阈值、
 * PITCH_BIAS_RATE (其灵敏度含 t_go²)，以及 V_LAUNCH/V_TAU。
 * 详见 docs/testing/04_MISSION_PROFILE.md。 */
#define DART_TARGET_RANGE_M             25.0f   /* 发射点到目标的距离 */
#define DART_TARGET_LATERAL_RANGE_M     0.56f   /* 目标横向移动范围 (任务给定) */
#define DART_MISSION_FLIGHT_TIME_S      2.0f    /* 标称滞空时间 */

#define DART_CONTROL_HZ                 200.0f
#define DART_CONTROL_DT                 (1.0f / DART_CONTROL_HZ)

/* 相机参数：使用 OpenMV 镜头标定结果替换。 */
#define DART_CAMERA_FX_PX               320.0f
#define DART_CAMERA_FY_PX               240.0f
#define DART_CAMERA_CX_PX               160.0f
#define DART_CAMERA_CY_PX               120.0f
#define DART_CAMERA_YAW_OFFSET_RAD      0.0f
#define DART_CAMERA_PITCH_OFFSET_RAD    0.0f

/* OpenMV 串口和帧有效性参数。USART2 必须使用相同波特率。 */
#define DART_VISION_BAUD                921600U
/* 置信度门限构成迟滞: 进入 GUIDE 用 ACQUIRE (严格), 留在 GUIDE 用 HOLD (宽松),
 * 必须满足 HOLD <= ACQUIRE, 否则会在 GUIDE/COAST 间以帧率反复横跳。
 * HOLD 低于视觉端 CONF_ACCEPT 时无意义 —— 视觉端不会发送该区间之外的帧。 */
#define DART_VISION_CONF_ACQUIRE        58U
#define DART_VISION_CONF_HOLD           52U
#define DART_VISION_FRESH_MS            45U
#define DART_VISION_COAST_MS            120U
#define DART_VISION_FAILSAFE_MS         280U
#define DART_VISION_LOCK_FRAMES         3U
/* 视觉帧率估计参数：使用视觉帧时间戳计算，并进行一阶低通。 */
#define DART_VISION_FPS_LPF_ALPHA       0.20f
#define DART_VISION_FPS_TIMEOUT_MS      1000U

/* 从图像采集到控制指令的延迟估计。舵机延迟通常占比最大。 */
#define DART_CAMERA_PIPELINE_DELAY_S    0.005f
#define DART_SERVO_DELAY_S              0.002f
#define DART_ROLL_PHASE_LEAD_ENABLE     1

/* LOS 角度/角速度外环。必须通过约束测试核对符号。 */
#define DART_GUIDANCE_YAW_SIGN          (1.0f) //yaw方向修正
#define DART_GUIDANCE_PITCH_SIGN        (1.0f) //pitch方向修正

/* 制导俯仰配平偏置 (rad/s, 角速度指令)。独立叠加在 q_cmd 上,
 * 不受 T_go 衰减 (cap_w) 和置信度缩放影响 —— 这两者都会在末端归零,
 * 若把偏置放在追踪项内部则永远起不到修正末端落点的作用。
 *
 * 用途: 抵消重复出现的系统性垂直落点偏差 (气动不对称、重力下垂等)。
 * 不能代替相机安装角标定 —— 先用 DART_CAMERA_PITCH_OFFSET_RAD 修正安装误差。
 * 正值与 LOS 俯仰误差同向 (经 DART_GUIDANCE_PITCH_SIGN 映射到机体)。
 *
 * 标定: 先跑多次落点记平均垂直偏差 e, 按下式估初值, 再按落点趋势微调。
 *   dq ≈ -2 · N · e / (V · t_go²)
 * 注意该灵敏度含 t_go², 固定偏置只能适配一种交战几何, 换靶距须重新标定。 */
#define DART_GUIDANCE_PITCH_BIAS_RATE   0.0f

/* ── 增广比例导引 (APN) ──
 * 角速度指令 = 弱追踪项 + N·λ̇ (比例导引) + K_ff·λ̈ (目标机动前馈)
 * 主导项 N·λ̇ 用于建立碰撞航线; λ̈ 前馈提前响应目标加减速;
 * 弱追踪项随 T_go 衰减, 远距用于把目标拉回视场, 末端让位给纯比例导引,
 * 避免追踪法在末端产生过载陡增。 */
#define DART_GUIDANCE_NAV_RATIO        2.8f    /* 导航比 N, 经典 3~5 */
#define DART_GUIDANCE_K_ACCEL_FEED     0.3f    /* 目标机动前馈增益 (λ̈ 项) */
#define DART_GUIDANCE_K_CAPTURE        0.5f    /* 弱追踪项系数 (远距保视场) */

/* 置信度缩放下限。该乘子作用在整条制导指令上, 等价于缩放导航比 N,
 * 下限设得过低会让 N 掉出比例导引的稳定区间 (N >= 2)。0.7 对应 N_eff >= 2.1。 */
#define DART_GUIDANCE_CONF_SCALE_MIN   0.7f

/* ── 制导律模式选择 ──
 * 0 = 增广比例导引 (APN):   N·λ̇ + K_ff·λ̈  (前馈目标机动, 依赖 λ̈ 质量)
 * 1 = 自适应积分滑模 (ISMCG): N·λ̇ + ρ·sat(s/ε) (鲁棒抑制目标机动, 不依赖 λ̈)
 * 两模式共享比例导引主导项与弱追踪项, 仅"目标机动处理项"不同。 */
#define DART_GUIDANCE_MODE_ISMCG       0

/* ISMCG 参数 (仅当 DART_GUIDANCE_MODE_ISMCG = 1 时生效) */
#define DART_GUIDANCE_ISMCG_K_I        1.0f    /* 滑模面积分系数 (1/s) */
#define DART_GUIDANCE_ISMCG_EPS        0.15f   /* 边界层宽度 (rad/s), 替代符号函数抑制抖振 */
#define DART_GUIDANCE_ISMCG_GAMMA_RHO  4.0f    /* 自适应增益增长率 (1/s) */
#define DART_GUIDANCE_ISMCG_DECAY      2.0f    /* 自适应增益衰减率 (1/s), 防 ρ 无限增长 */
#define DART_GUIDANCE_ISMCG_RHO_MIN    0.1f    /* ρ 下限 (rad/s) */
#define DART_GUIDANCE_ISMCG_RHO_MAX    2.5f    /* ρ 上限 (rad/s) */
#define DART_GUIDANCE_ISMCG_INT_LIMIT  1.5f    /* 滑模面积分限幅 (rad), 防积分饱和 */

/* 视线角 α-β 滤波。lambda 平滑位置, rate 融合视觉 λ̇ (见 RATE_TAU_S)。 */
#define DART_GUIDANCE_FILTER_ALPHA     0.58f
#define DART_GUIDANCE_FILTER_BETA      0.12f
/* γ 通道不再用于 λ̈: λ̈ 改为视觉 λ̇ 帧间差分 (与 λ̇ 同源)。gamma 置 0 退化为
 * α-β, 避免 γ 推得的 acc 残留在预测步污染 est/rate。 */
#define DART_GUIDANCE_FILTER_GAMMA     0.0f

/* λ̈ 限幅 (rad/s², 作用在已低通的 λ̈ 上)。
 * 按信号量级定, 不按"看起来安全"定:
 *   真实物理峰值 ≈ a/R = 1.33/4.6 ≈ 0.29 rad/s² (目标停止减速时)
 *   低通后噪声 σ ≈ 0.05~0.13 rad/s² (像素速度噪声差分 + DD 低通衰减)
 * 取 3.0 ≈ 物理峰值的 10 倍、噪声 σ 的 ~30 倍: 不碰真实机动, 只截掉
 * 会让 K_ff·λ̈ 单独打满指令的异常值。原值 25.0 是物理峰值的 86 倍,
 * 实际上等于没有限幅。 */
#define DART_GUIDANCE_FILTER_ACC_LIMIT 3.0f

/* 视觉 λ̇ 低通时间常数 (s)。λ̇ 主来源是视觉像素速度 (vx/vy 换算),
 * β 通道为位置辅助, 两者在同一个 rate 变量上叠加。
 *
 * 用时间常数而不是固定每帧系数: 系数按 α = 1 - exp(-dt/τ) 逐帧换算,
 * 保证帧率在 15~60 fps 之间波动时滤波带宽不变。原 RATE_MIX = 0.30 是
 * "每帧"系数, 帧率一变时间常数就跟着漂; 本值即 30 fps 下的等效 τ。 */
#define DART_GUIDANCE_FILTER_RATE_TAU_S 0.093f

/* λ̈ 差分低通时间常数 (s)。λ̈ = 对融合后 λ̇ 帧间差分, 再经本系数低通。
 * 原 DD_MIX = 0.15 在 30 fps 下的等效 τ。 */
#define DART_GUIDANCE_FILTER_DD_TAU_S   0.205f

/* 追踪项权重随 T_go 衰减的参考范围。
 * 按实测捕获距离 12.5 m 回填: 捕获时 tgo ≈ 12.5/V ≈ 0.75 s, 制导窗口仅为
 * tgo ∈ [0.2, 0.9] s。原 0.70/1.80 会让追踪项一出生就归零。台架实测捕获
 * 时刻后再复核本区间。 */
#define DART_GUIDANCE_NEAR_TGO_S       0.30f
#define DART_GUIDANCE_FAR_TGO_S        0.90f

//期望角速度限幅
#define DART_GUIDANCE_RATE_LIMIT       3.5f

//滚转通道稳定控制
#define DART_GUIDANCE_ROLL_KP           1.4f
#define DART_GUIDANCE_ROLL_RATE_LIMIT   2.5f

/* 角速度内环：微分项取自实测角速度，而非指令误差。 */
#define DART_RATE_KP_P                  0.20f
#define DART_RATE_KP_Q                  0.22f
#define DART_RATE_KP_R                  0.26f
#define DART_RATE_KD_P                  0.010f
#define DART_RATE_KD_Q                  0.010f
#define DART_RATE_KD_R                  0.012f
#define DART_RATE_ACCEL_LPF_ALPHA       0.19f
#define DART_MOMENT_LIMIT               0.65f

/* 控制分配的标称动压参考速度 (m/s)。q_scale = (V_est/V_nom)², 用于把舵效
 * 归一化到标称动压。原为 control_allocation.c 中的字面量 15.0。
 * 注意 V_est 来自上面那个尚未实测的速度模型 —— V 偏 10%, q_scale 偏 21%,
 * 即控制权限会跟着偏。 */
#define DART_Q_NOMINAL_SPEED_MPS        15.0f

/* 姿态估计器。发射阶段禁用加速度修正；飞行中因比力不等于重力，
 * 对加速度修正施加强门控。 */
#define DART_AHRS_KP_GROUND             1.0f
#define DART_AHRS_KI_GROUND             0.015f
#define DART_AHRS_ACCEL_MIN_G           0.86f
#define DART_AHRS_ACCEL_MAX_G           1.14f
#define DART_AHRS_INFLIGHT_ACCEL_WEIGHT 0.01f

/* 发射与状态判定阈值。 */
#define DART_LAUNCH_ACCEL_MPS2          20.4f
#define DART_LAUNCH_CONFIRM_S           0.008f
#define DART_LAUNCH_HOLDOFF_S           0.010f
#define DART_IMPACT_ACCEL_MPS2          34.0f /* ICM 配置量程为正负 4 g */
#define DART_IMPACT_BLANKING_S          0.8f
#define DART_MAX_FLIGHT_TIME_S          60.0f

/* 简化空速调度。用于缩放控制效能 (q_scale)、fallback_tgo() 和遥测。
 *
 * ⚠ 这两个值**尚未实测，且当前不自洽**：V(t) = 18·exp(-t/3.8) 在 2 s 内
 *   累计行程 28.0 m，而靶距是 25 m (偏高 3 m)。
 *
 * 这个误差不是小事 —— t_go 的每个消费者都会被污染：cap_w、
 * ca_get_T_go_weights、q_scale (误差按平方放大)。而且 t_go 的源头
 * t = R/V_est 是**除法**，V_est 错 10%，t_go 就错 10%。
 *
 * 必须用实测速度曲线重新拟合，约束 ∫₀²ᐧ⁰ V dt = DART_TARGET_RANGE_M。
 * 在实测数据出来之前不要继续试凑 V_LAUNCH —— 单改初速拟合不了曲线的
 * 形状 (τ)，两者必须一起辨识。步骤见 docs/testing/02_TEST_PROCEDURE.md 第 12 节。 */
#define DART_SPEED_DECAY_ENABLE         1
#define DART_V_LAUNCH_MPS               18.0f   /* 待实测 */
#define DART_V_TAU_S                    3.8f    /* 待实测 */
#define DART_V_MIN_MPS                  5.0f

/* 基于 IMU 的速度方向估计。输出航迹角仅用于遥测，不再作为制导前馈。 */
#define DART_BALLISTIC_ESTIMATOR_ENABLE       1
#define DART_BALLISTIC_GRAVITY_MPS2           9.80665f
#define DART_BALLISTIC_MIN_SPEED_MPS          5.0f
#define DART_BALLISTIC_ACCEL_LPF_ALPHA        0.12f
#define DART_BALLISTIC_TRACK_RATE_LPF_ALPHA   0.15f
#define DART_BALLISTIC_MAX_LATERAL_ACCEL_MPS2 30.0f

/* 弹道估计器仍计算速度方向/航迹角用于遥测，但不再作为制导前馈：
 * 增广比例导引的 λ̇ 项已隐含速度方向失配，显式航迹反馈是冗余的。 */

/* IMU 安装映射，依据旧工程中芯片 X/Y 轴互换的说明。完成轴向测试后只需在此
 * 修改符号，无须改动控制代码。 */
#define DART_BODY_P_FROM_GX             0.0f
#define DART_BODY_P_FROM_GY             1.0f
#define DART_BODY_P_FROM_GZ             0.0f
#define DART_BODY_Q_FROM_GX             1.0f
#define DART_BODY_Q_FROM_GY             0.0f
#define DART_BODY_Q_FROM_GZ             0.0f
#define DART_BODY_R_FROM_GX             0.0f
#define DART_BODY_R_FROM_GY             0.0f
#define DART_BODY_R_FROM_GZ             (-1.0f)

/* 舵机标定参数。控制分配完成后再应用方向系数。 */
#define DART_SERVO_1_CENTER_US          1500U
#define DART_SERVO_2_CENTER_US          1500U
#define DART_SERVO_3_CENTER_US          1500U
#define DART_SERVO_4_CENTER_US          1500U
#define DART_SERVO_1_DIRECTION          (-1.0f)
#define DART_SERVO_2_DIRECTION          (-1.0f)
#define DART_SERVO_3_DIRECTION          (-1.0f)
#define DART_SERVO_4_DIRECTION          (-1.0f)
#define DART_SERVO_SPAN_US              500.0f
#define DART_SERVO_DELTA_LIMIT          0.70f
#define DART_SERVO_RATE_LIMIT_PER_S     6.0f
#define DART_SERVO_DEADBAND             0.006f
#define DART_SERVO_DEADBAND_BOOST       0.0f

/* X 尾翼效能矩阵。行依次为滚转、俯仰、偏航，列依次为舵机 1 至 4。 */
#define DART_B_ROLL_1  (0.50f)
#define DART_B_ROLL_2  (0.50f)
#define DART_B_ROLL_3  (0.50f)
#define DART_B_ROLL_4  (0.50f)
#define DART_B_PITCH_1 (+0.60f)
#define DART_B_PITCH_2 (-0.60f)
#define DART_B_PITCH_3 (+0.60f)
#define DART_B_PITCH_4 (-0.60f)
#define DART_B_YAW_1   (+0.60f)
#define DART_B_YAW_2   (+0.60f)
#define DART_B_YAW_3   (-0.60f)
#define DART_B_YAW_4   (-0.60f)

#endif
