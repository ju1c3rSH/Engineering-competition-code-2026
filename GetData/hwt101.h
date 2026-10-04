/**
 * @file    hwt101.h
 * @brief   维特 HWT101 姿态传感器驱动（采样层）
 *
 * 收敛原先散在三处的 HWT101 代码：帧解析(原 Uart3_task 扫描段)、寄存器读写与上电配置
 * (原 Control_Car.c)、帧计数与观测字段(原 YawSeq/GyroWz_Seq + RateHz)。
 *
 * 依赖方向恒为 hwt101 <- Car_Sense.c / Control_Car.c / car_obs.c（同级）。
 * 绝不允许反向 include Control_Car.h 或任何 App/ 头 —— 会成环。
 * 本库只 include: main.h（HAL 类型）、stdint.h；不含 mf_*、不含 protocol.h。
 * 帧解析层为纯逻辑（无 HAL/Delay/printf）⇒ 可在 PC 单测，见 build_verify/hwt101_test/。
 *
 * 协议依据与寄存器表：<https://wit-motion.yuque.com/wumwnr/ltst03/seuiba>
 * 含 HAL_Delay 的配置层函数只能在 main() 上电阶段调用，禁止进中断。
 */
#ifndef __HWT101_H__
#define __HWT101_H__

#include <stdint.h>

/* ================= 帧类型（官方 TYPE 字节） ================= */
#define HWT101_TYPE_WZ        0x52u   /* 角速度帧: [55][52][00][00][RWzL][RWzH][WzL][WzH][00][00][SUM] */
#define HWT101_TYPE_YAW       0x53u   /* 角度帧:   [55][53][00][00][00][00][YawL][YawH][VL][VH][SUM]  */
#define HWT101_TYPE_REG       0x5Fu   /* 回读应答: [55][5F][d0][d1][d2][d3][d4][d5][d6][d7][SUM]     */

/* 数据帧固定 11 字节；写帧固定 5 字节 */
#define HWT101_FRAME_LEN      11u
#define HWT101_WRITE_LEN      5u
#define HWT101_HEAD           0x55u
#define HWT101_WRITE_HEAD0    0xFFu
#define HWT101_WRITE_HEAD1    0xAAu

/* ================= 寄存器地址（官方表，勿凭记忆改） ================= */
#define HWT101_REG_SAVE       0x00u   /* 保存 0x0000 / 重启 0x00FF / 恢复出厂 0x0001 */
#define HWT101_REG_CALSE      0x01u   /* 手动获取零偏（旧称 CALSE） */
#define HWT101_REG_RRATE      0x03u   /* 输出速率 */
#define HWT101_REG_BAUD       0x04u   /* 波特率（**出厂默认 0x0002 = 9600**） */
#define HWT101_REG_LEDOFF     0x1Bu   /* 关闭 LED */
#define HWT101_REG_READADDR   0x27u   /* 读寄存器 */
#define HWT101_REG_VERSION    0x2Eu   /* 版本号（0x2E~0x2F，R） */
#define HWT101_REG_GYROBAIS   0x4Cu   /* Z 轴零偏值（R/W，/1000 得 °/s） */
#define HWT101_REG_GYROCALITHR 0x61u  /* 陀螺仪静止阈值（配合 0x63） */
#define HWT101_REG_GYROCALTIME 0x63u  /* 陀螺仪自动校准时间 */
#define HWT101_REG_WERROR     0x6Au   /* 陀螺仪变化值（R） */
#define HWT101_REG_WZTIME     0x6Eu   /* 角速度连续静止时间（配合 0x6F） */
#define HWT101_REG_WZSTATIC   0x6Fu   /* 角速度积分阈值（/1000 得 °/s） */
#define HWT101_REG_KEY        0x69u   /* 解锁（写 0xB588 才有效，其他值无效） */
#define HWT101_REG_CALIYAW    0x76u   /* Z 轴角度归零 */
#define HWT101_REG_MANUALCALI 0xA6u   /* 手动零偏开关（**免解锁免保存**） */
#define HWT101_REG_NOAUTOCALI 0xA7u   /* 关闭自动零偏（**免解锁免保存**） */
/* 出厂标定，官方明确「请勿修改」——本库刻意不提供写接口 */
#define HWT101_REG_PGSCALE    0x50u   /* Z 轴标定因素 P */
#define HWT101_REG_GSCALERANGE 0x52u  /* Z 轴校准角度 */

/* ---- RRATE 取值（官方枚举） ---- */
#define HWT101_RRATE_10HZ     0x06u   /* 出厂默认 */
#define HWT101_RRATE_100HZ    0x09u
#define HWT101_RRATE_200HZ    0x0Bu   /* 本工程目标值 */
#define HWT101_RRATE_500HZ    0x0Cu   /* 500Hz 在 115200 下占用率过高，勿用（见 docs §8.11） */

/* ---- BAUD 取值 ---- */
#define HWT101_BAUD_115200    0x06u   /* 本工程 USART3 固定值；模块出厂是 0x02=9600 */

/* ---- 解锁 / 保存 ---- */
#define HWT101_KEY_UNLOCK     0xB588u /* 写 0x69 只认这个值，其他无效 */
#define HWT101_KEY_SAVE       0x0000u

/* ================= 量纲换算（官方给法，勿改系数） ================= */
#define HWT101_WZ_FULLSCALE   2000.0f /* 角速度 ±2000°/s */
#define HWT101_YAW_FULLSCALE  180.0f  /* 航向角 ±180° */

/* ================= 观测/诊断结构 ================= */
/* rate_hz 是真实帧率(旧版有 yaw_got 守卫, 抽库时已删)⇒ 跨固件版本比较 BOOT/RRATE 前
 * 须先对齐口径: 旧固件日志的值是突发下界, 同物理帧率下新值更大。见 docs §8.11 */
typedef struct
{
	uint32_t yaw_seq;        /* 0x53 帧计数（新鲜度判据：与「车是否在转」解耦） */
	uint32_t wz_seq;         /* 0x52 帧计数（交叉校验丢弃时**不**增长） */
	uint32_t bad_sum;        /* sum8 校验失败帧数 */
	uint32_t tx_fail;        /* 配置帧发送失败次数（HAL_UART_Transmit != HAL_OK）。
	                            **非 0 就是"配置没写进去"**：写寄存器单向无 ACK，
	                            失败唯一可见现象就是这个计数 */
	uint32_t version;        /* 回读版本号原始 32 位（未回读时为 0）。
	                            官方: (VHH<<24)|(VHL<<16)|(VLH<<8)|VLL */
	uint16_t rate_hz;        /* 最近一个窗口的 0x53 帧率（主循环刷新） */
	uint16_t rate_boot;      /* 上电那一次采样：判「要不要写模块 Flash」的依据 */
	uint8_t  has_version;    /* 1 = 版本号已成功回读 */
	uint8_t  xchk_ok;        /* 交叉校验通过/未判次数（低 8 位滚动，会回绕） */
	uint8_t  xchk_discard;   /* 交叉校验丢弃次数（低 8 位滚动，会回绕） */
} hwt101_health_t;

/* ================= 对外接口 ================= */

/* ---- 生命周期 ---- */
void Hwt101_Init(void);      /* 上电调用：状态机清零 + 测帧率 + 必要时配 RRATE */

/* ---- 帧解析（纯逻辑，无 HAL 依赖，可在 PC 单测） ---- */
/* USART3 IDLE 中断里调，喂本次突发的实际字节。O(n)，无阻塞，无 printf */
void Hwt101_FeedBytes(const uint8_t *buf, uint16_t len);

/* ---- 读数（采样层读，Car_Sense / 上层用） ---- */
float Hwt101_GetYawDeg(void);   /* 航向角，±180 */
float Hwt101_GetWzDps(void);    /* 0x52 直给角速度，±2000 */

/* ---- 帧计数（新鲜度判据） ---- */
uint32_t Hwt101_YawSeq(void);
uint32_t Hwt101_WzSeq(void);

/* 声明"下一条 0x5F 应答是要解释哪个寄存器"。Hwt101_ReadVersion() 内部已调;
 * 单独暴露是为将来加 RRATE/BAUD 回读时不必改解析层。0x5F 是通用应答头,
 * 不设这个上下文就会把任意回读当成版本号(评审 P1-6) */
void Hwt101_ExpectReg(uint8_t reg);

/* ---- 诊断 ---- */
const hwt101_health_t *Hwt101_Health(void);
void Hwt101_RateTask(void);     /* 主循环每圈调：刷新 rate_hz 滚动窗口 */

/* ---- 配置层：**含 HAL_Delay 阻塞，只许 main() 上电阶段调用** ---- */
void Hwt101_ConfigRate(uint8_t rrate);  /* 写 RRATE 并保存（解锁→写→保存） */
void Hwt101_ZeroYaw(void);              /* CALIYAW 归零：解锁→0x76 写 0→保存 */
void Hwt101_ReadVersion(void);          /* 回读 0x2E 版本号（阻塞，结果进 Health） */
void Hwt101_SetAutoCalib(uint8_t off);  /* NOAUTOCALI=0xA7：1 关自动零偏（免解锁） */
void Hwt101_ManualBias(uint8_t start); /* MANUALCALI=0xA6：1 进/0 退零偏采集（免解锁） */

/* ---- 编译期开关（放这里是因为它们是"本库行为的一部分"） ---- */
/* CALIYAW 归零会阻塞约 1.2s。单基准航向架构(MF_HEADING_FRAME_EN=1)下 GYRO_ZERO
 * 改用实测航向钉基准，物理归零只影响读数是否从 0 起。默认 0 = 不做这次归零。 */
#ifndef HWT101_CFG_ZEROWAW_EN
#define HWT101_CFG_ZEROWAW_EN       0
#endif
/* 帧率判定阈值：远高于出厂 10Hz 即认为 RRATE 已生效, 从而不写模块 Flash
 * (官方模块 Flash 有擦写寿命)。已达标就跳过写入。
 * ⚠ 该值同时**必须保证 HWT101_FRAME_DT_S 成立**: 交叉校验的分母按 200Hz 折算,
 *   若放行 100Hz, 真实 Δt=10ms 而分母按 5ms ⇒ yaw_rate 是真值的 2 倍 ⇒ 弧段
 *   每帧都被判不一致(代码评审 P1-1)。取 180: 只有真正接近 200Hz 才放行。 */
#ifndef HWT101_RATE_TARGET_HZ
#define HWT101_RATE_TARGET_HZ       180u
#endif
/* 帧率统计窗口(ms) */
#ifndef HWT101_RATE_WIN_MS
#define HWT101_RATE_WIN_MS          500u
#endif
/* 交叉校验：|ψ̇_0x52 − Δψ/Δt| 超过此值即判该 Wz 不可信(deg/s)。0 = 关闭。
 * 取 12 而非更大: WzL 单字节全翻转的最大偏差 = 255×(2000/32768) ≈ 15.6dps,
 * 容差必须显著小于它才有意义(评审 P1-2); 同时远大于两路正常时的量化噪声
 * (航向 1LSB ≈ 0.0055°, /5ms ≈ 1.1dps), 不会误报。 */
#ifndef HWT101_XCHK_EN
#define HWT101_XCHK_EN              1
#endif
#ifndef HWT101_XCHK_TOL_DPS
#define HWT101_XCHK_TOL_DPS         12.0f
#endif
/* 交叉校验的**死区**: |Wz| 与 |上一帧 Wz| 都低于它就不判。
 * 理由: 静止/过零时航向差分只剩量化噪声(±0.5~1.4dps), 与 Wz 的零偏同量级,
 * 此时比值全是误报。取 40dps: 远低于实车工作点(弧段 ~43dps)、高于噪声。
 * ⚠ 必须与 TOL 解耦(早期版本两者都取 40, 恰好使保护在刚起步处最弱)。 */
#ifndef HWT101_XCHK_MIN_DPS
#define HWT101_XCHK_MIN_DPS         40.0f
#endif
/* 0x52/0x53 的帧间隔(s): 模块每周期各发一帧, RRATE=200Hz ⇒ 5ms。
 * ⚠ 必须与 HWT101_RATE_TARGET_HZ 一致(那个阈值的作用之一就是保证这点):
 *   改速率时同步改这里, 否则交叉校验的 Δψ/Δt 分母错, 表现为"好好的链路被
 *   大量丢弃"。HWT: 行可核对实际 RRATE。 */
#ifndef HWT101_FRAME_DT_S
#define HWT101_FRAME_DT_S           0.005f
#endif

#endif /* __HWT101_H__ */
