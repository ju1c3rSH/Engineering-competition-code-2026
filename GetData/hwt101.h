/**
 * @file    hwt101.h
 * @brief   维特 HWT101 姿态传感器驱动（采样层）
 *
 * 依赖方向恒为 hwt101 <- Car_Sense.c / Control_Car.c / car_obs.c（同级）；绝不允许反向
 * include Control_Car.h 或任何 App/ 头（会成环）。本库只 include main.h（HAL 类型）与
 * stdint.h，不含 mf_*.h 与 protocol.h。帧解析层为纯逻辑(无 HAL/Delay/printf)⇒ 可 PC 单测,
 * 见 build_verify/hwt101_test/。含 HAL_Delay 的配置层函数只能在 main() 上电阶段调, 禁止进中断。
 * 协议依据与寄存器表：<https://wit-motion.yuque.com/wumwnr/ltst03/seuiba>
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
#define HWT101_REG_WORKMODE   0x48u   /* 工作模式（R，0x00=普通；非 0 说明在校准态） */
#define HWT101_REG_OPTPE      0x4Au   /* Z 轴静止峰峰值（R）：判履带振动是否被当成转动 */
#define HWT101_REG_GYROBAIS   0x4Cu   /* Z 轴零偏值（R/W，/1000 得 °/s）。**本工程零偏观测的核心** */
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

/* ---- RRATE 取值（官方枚举, 唯一权威: 语雀 HWT101 TTL 协议页 wit-motion.yuque.com/wumwnr/ltst03/seuiba） ----
 * 2026-10-10 复核官方协议: 0x0B=200Hz, 0x0C=500Hz, 0x0D=1000Hz —— **协议里没有 250Hz 档**。
 * (同日曾误"勘误"为 0x0C=250Hz, 已回退 —— 教训: 动码表前先查官方协议原文, 见坑文档 §8.14)
 * 现场这台模块实测稳定出 250Hz(HWT:RRATE=250): 属该机固件的非标行为, 配置写 0x0B 实出 250Hz。
 * 码表以协议为准, **实际帧率以 HWT:RRATE / YF 实测为准**(FRAME_DT_S 跟实测走)。 */
#define HWT101_RRATE_10HZ     0x06u   /* 出厂默认 */
#define HWT101_RRATE_100HZ    0x09u
#define HWT101_RRATE_200HZ    0x0Bu   /* 本工程写入目标(本机实出 250Hz, 见上) */
#define HWT101_RRATE_500HZ    0x0Cu   /* 500Hz: 22B/帧*500≈11kB/s 顶满 115200 链路, 勿用（见 docs §8.11） */
#define HWT101_RRATE_1000HZ   0x0Du   /* 1000Hz: 同上更甚, 勿用 */

/* ---- BAUD 取值 ---- */
#define HWT101_BAUD_115200    0x06u   /* 本工程 USART3 固定值；模块出厂是 0x02=9600 */

/* ---- 解锁 / 保存 ---- */
#define HWT101_KEY_UNLOCK     0xB588u /* 写 0x69 只认这个值，其他无效 */
#define HWT101_KEY_SAVE       0x0000u

/* ================= 量纲换算（官方给法，勿改系数） ================= */
#define HWT101_WZ_FULLSCALE   2000.0f /* 角速度 ±2000°/s */
#define HWT101_YAW_FULLSCALE  180.0f  /* 航向角 ±180° */

/* ================= 观测/诊断结构 ================= */
/* 通用寄存器回读的槽位。**按寄存器分开存, 不做"单槽 + 上次读的是谁"**:
 * 应答是异步回来的, 单槽会让"某次回读没应答"表现为"另一个寄存器的旧值
 * 被当成本次结果" —— 对一个用来判断零偏的观测行, 这种误判不可接受。
 * 槽位顺序即 IMU: 行的打印顺序, 改这里要同步改 car_obs.c 的 rb 掩码位序。 */
#define HWT101_RB_BIAS   0u   /* 0x4C GYROBAIS  零偏账面值, raw/1000 = °/s */
#define HWT101_RB_NOCAL  1u   /* 0xA7 NOAUTOCALI 自动零偏开关, 0=开着(出厂) */
#define HWT101_RB_OPTPE  2u   /* 0x4A OPTPE     静止峰峰值 */
#define HWT101_RB_MODE   3u   /* 0x48 WORKMODE  工作模式, 0x00=普通 */
#define HWT101_RB_COUNT  4u

/* rate_hz 是真实帧率 ⇒ 跨固件版本比较 BOOT/RRATE 前须先对齐口径: 旧固件日志的值是
 * 突发下界, 同物理帧率下新值更大。见 docs/坑-编码器与里程.md §8.11 */
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
	int16_t  reg_val[HWT101_RB_COUNT]; /* 各槽最近一次回读的原始 16 位值。
	                             **BIAS 槽必须按有符号读**(官方读格式全局约定),
	                             负偏编码在 d0/d1: -100 → 9C FF */
	uint16_t reg_fresh;      /* 槽位有效位掩码(1<<槽号)。0 = 该槽还没回读成功过,
	                             观测层靠它区分"值是 0"与"根本没读到" */
	uint32_t reg_seq;        /* 每解释掉一条 0x5F 应答 +1（判"这次回读有更新"） */
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
 * 不设这个上下文就会把任意回读当成版本号 */
void Hwt101_ExpectReg(uint8_t reg);

/* ---- 配置寄存器回读（纯只读：官方明确读寄存器**不需要解锁**、不写 Flash） ----
 * 发一条 FF AA 27 <reg> 00 并登记待解释寄存器；**不等应答**，结果由解析层
 * 异步填入 health.reg_val[槽] 并置 reg_fresh 位。主循环按几拍轮读即可。 */
void Hwt101_ReadReg(uint8_t reg);
/* 槽位 → 值的换算: 0x4C/0x4A 按 /1000 得 °/s, 0xA7/0x48 原值。返回 0 表示该槽未回读过 */
int16_t Hwt101_RegRaw(uint8_t slot);
uint8_t Hwt101_RegFresh(uint8_t slot);   /* 该槽是否已成功回读过 */
void Hwt101_ClearRegFresh(void);          /* 清有效位（观测窗口开始时调） */

/* ---- 诊断 ---- */
const hwt101_health_t *Hwt101_Health(void);
void Hwt101_RateTask(void);     /* 主循环每圈调：刷新 rate_hz 滚动窗口 */

/* ---- 配置层：**含 HAL_Delay 阻塞，只许 main() 上电阶段调用** ---- */
void Hwt101_ConfigRate(uint8_t rrate);  /* 写 RRATE 并保存（解锁→写→保存） */
void Hwt101_ZeroYaw(void);              /* CALIYAW 归零：解锁→0x76 写 0→保存 */
void Hwt101_ReadVersion(void);          /* 回读 0x2E 版本号（阻塞，结果进 Health） */
void Hwt101_SetAutoCalib(uint8_t off);  /* NOAUTOCALI=0xA7：1 关自动零偏（免解锁） */
void Hwt101_ManualBias(uint8_t start); /* MANUALCALI=0xA6：非0 进/0 退零偏采集（免解锁） */
/* 上电零偏校准一站式封装: 关自动校准 → 进零偏 → 静置 seconds → 退零偏 → 回读 0x4C。
 * 含 HAL_Delay(seconds*1000) 的阻塞, **只许 main() 上电静置期调用**; 其间车必须完全
 * 静止(轮速=0 且电机未使能), 否则会把运动数据当零偏。返回回读到的 BIAS 原始码
 * (int16, /1000 = °/s), 未回读到返回 0 且不置 fresh 位。 */
int16_t Hwt101_BootCalib(uint16_t seconds);

/* ---- 编译期开关（放这里是因为它们是"本库行为的一部分"） ---- */
/* CALIYAW 归零会阻塞约 1.2s。单基准航向架构(MF_HEADING_FRAME_EN=1)下 GYRO_ZERO
 * 改用实测航向钉基准，物理归零只影响读数是否从 0 起。默认 0 = 不做这次归零。 */
#ifndef HWT101_CFG_ZEROWAW_EN
#define HWT101_CFG_ZEROWAW_EN       0
#endif
/* 上电手动零偏采集: 1 = 开(默认)。官方 AGV 流程的落地——车静止后 0xA6=1 进、
 * 运动前 0xA6=4 退, 免解锁免保存、不写 Flash。上电静置期本就不动, 顺手做掉。
 * 走手动零偏必须先关自动校准(见 Hwt101_BootCalib), 否则两者会互相打架。
 * 置 0 可整体回退到"只有 Hwt101_Init + 2s 静置"的老行为, 便于 A/B。 */
#ifndef HWT101_CFG_MANUALCALI_EN
#define HWT101_CFG_MANUALCALI_EN    1
#endif
/* 上电手动零偏的静止窗口(s)。协议对 MANUALCALI 只写"时长由用户决定", 但
 * **模块内部按 GBAISTIME(0x4D, 出厂默认 0x000A = 10s) 计算零偏** —— 窗口短于它,
 * 退出时零偏可能还没算完(见 docs/坑-IMU零偏与长期漂移.md §6 阻塞项 5)。
 * 10s 与 GBAISTIME 默认值对齐, 保证模块算得完; 再想更稳可加到 12~15s。
 * 改这一个数字即改窗口(上电总耗时随之变化, 看门狗在其后启动、不受影响)。 */
#ifndef HWT101_CFG_CALIB_SECONDS
#define HWT101_CFG_CALIB_SECONDS    10u
#endif
/* 帧率判定阈值：达到目标帧率附近即认为 RRATE 已生效, 从而不写模块 Flash
 * (官方模块 Flash 有擦写寿命)。已达标就跳过写入。
 * 该值同时**必须保证 HWT101_FRAME_DT_S 成立**: 交叉校验的分母按 250Hz(实测)折算,
 *   若放行 100Hz, 真实 Δt=10ms 而分母按 4ms ⇒ yaw_rate 是真值的 2.5 倍 ⇒ 弧段
 *   每帧都被判不一致。取 230: 本机 250Hz 放行(免写 Flash); 若掉回出厂 10Hz 则补写
 *   0x0B(本机实出 250Hz, 见 RRATE 枚举注)。注意: 若哪天换上一台**标准**模块(0x0B 实出
 *   200Hz), 它 <230 每次上电都会补写一次且 FRAME_DT_S 失配 —— 那是换硬件的信号,
 *   到时按新机实测帧率改 FRAME_DT_S 与控制拍, 别硬凑这组数。 */
#ifndef HWT101_RATE_TARGET_HZ
#define HWT101_RATE_TARGET_HZ       230u
#endif
/* 帧率统计窗口(ms) */
#ifndef HWT101_RATE_WIN_MS
#define HWT101_RATE_WIN_MS          500u
#endif
/* 交叉校验：|ψ̇_0x52 − Δψ/Δt| 超过此值即判该 Wz 不可信(deg/s)。0 = 关闭。
 * 取 12 而非更大: WzL 单字节全翻转的最大偏差 = 255×(2000/32768) ≈ 15.6dps,
 * 容差必须显著小于它才有意义; 同时远大于两路正常时的量化噪声
 * (航向 1LSB ≈ 0.0055°, /4ms ≈ 1.4dps), 不会误报。 */
#ifndef HWT101_XCHK_EN
#define HWT101_XCHK_EN              1
#endif
#ifndef HWT101_XCHK_TOL_DPS
#define HWT101_XCHK_TOL_DPS         12.0f
#endif
/* 交叉校验的**死区**: |Wz| 与 |上一帧 Wz| 都低于它就不判。
 * 理由: 静止/过零时航向差分只剩量化噪声(±0.5~1.4dps), 与 Wz 的零偏同量级,
 * 此时比值全是误报。取 40dps: 远低于实车工作点(弧段 ~43dps)、高于噪声。
 * 必须与 TOL 解耦(早期版本两者都取 40, 恰好使保护在刚起步处最弱)。 */
#ifndef HWT101_XCHK_MIN_DPS
#define HWT101_XCHK_MIN_DPS         40.0f
#endif
/* 0x52/0x53 的帧间隔(s): 模块每周期各发一帧, 实测 RRATE=250Hz ⇒ 4ms。
 * 2026-10-10 从 0.005f 改: 模块实出 250Hz(旧文档"200Hz 已配"前提有误), 分母不改则交叉校验
 * 的 Δψ/Δt 低读 0.8x, 弧上大批丢样(XBAD 随弧爬升 ~20/条)。
 * 注意它按**模块实际帧率**填, 不随控制拍自动变; HWT: 行可核对实际 RRATE。 */
#ifndef HWT101_FRAME_DT_S
#define HWT101_FRAME_DT_S           0.004f
#endif

#endif /* __HWT101_H__ */
