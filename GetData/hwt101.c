/**
 * @file    hwt101.c
 * @brief   维特 HWT101 驱动实现
 *
 * 依赖层级见 hwt101.h 顶部。
 *
 * ---- 为什么拆成 .inc ----
 * 帧解析层必须是**无 HAL 依赖**的纯逻辑, 否则它无法在 PC 上单测（本工程的
 * mf_path/maixcam 都用这个办法）。因此把 [1][2] 两段放进 hwt101_testable.inc:
 *   - 固件: 本文件 include 它 → 同一份代码编进目标
 *   - 单测: test_hwt101.c 直接 include 它 → 不需要 stm32f4xx_hal.h
 * 若在 .inc 里 include HAL 头, 单测就编不过了 —— 保持它的 include 只有 hwt101.h。
 *
 * 分工:
 *   [1] 帧解析层（.inc）纯逻辑, O(n), 无 printf/无 Delay, 跑在 USART3 IDLE 中断
 *   [2] 寄存器层（.inc）11 字节帧编解码, 纯函数
 *   [3] 配置层（本文件）上电阻塞序列, **含 HAL_Delay, 禁进中断**
 */
#include "hwt101.h"
#include "hwt101_testable.inc"   /* [1] 帧解析层 + [2] 寄存器层（纯逻辑, PC 可单测） */

#include "usart.h"
#include "main.h"

/* ================= [3] 配置层（**含 HAL_Delay，禁进中断**） ================= */

/* 帧率滚动统计: 主循环(Car_DebugTask)每圈调, 每满窗口用 0x53 帧计数增量算一次。
 * 与上电那次一次性采样互补 —— 它反映"现在", 任何时候查都作数 */
void Hwt101_RateTask(void)
{
	static uint32_t prev_seq = 0u;
	static uint32_t prev_ms  = 0u;
	uint32_t now_ms  = HAL_GetTick();
	uint32_t now_seq = health.yaw_seq;
	uint32_t span    = now_ms - prev_ms;

	if (span >= HWT101_RATE_WIN_MS)
	{
		health.rate_hz = (uint16_t)(((now_seq - prev_seq) * 1000u) / span);
		prev_seq = now_seq;
		prev_ms  = now_ms;
	}
}

/* 发一帧并等它出去。
 *
 * ⚠ 用**轮询阻塞**而非 DMA: USART3 IDLE 中断每突发都调 HAL_UART_DMAStop(),而它在
 *   gState==BUSY_TX 时会 HAL_DMA_Abort(hdmatx) —— 顺带把配置帧从中间掐断。模块 200Hz
 *   出流 ⇒ IDLE 每~5ms 一次,5 字节 TX 窗口仅 0.43ms,三帧序列至少一帧被截 ≈23%。
 *   被腰斩的帧模块直接忽略 ⇒ 解锁失败 ⇒ RRATE 写不进去 ⇒ 停在出厂 10Hz(评审 P0-3)。
 *   轮询天然抗抢占; 副作用是消掉了"DMA payload 不得是栈数组"的隐患。 */
static HAL_StatusTypeDef Hwt101_Tx(const uint8_t *cmd)
{
	return HAL_UART_Transmit(&huart3, (uint8_t *)cmd, HWT101_WRITE_LEN, 50u);
}

/* 官方四步: 解锁(KEY 0x69 写 0xB588) → 延时 → 写寄存器 → 延时 → 保存(SAVE 0x00 写 0)。
 * 旧码只发中间那条: 缺解锁被静默拒写、缺保存掉电不保留 ⇒ 从未生效过。
 * ⚠ 官方要求解锁后 10s 内完成写与保存,本函数三步远小于该窗口; 延时不得合并。
 * ⚠ 失败必须计数: 写寄存器单向无 ACK,静默丢弃 = 下次排查完全没有线索。 */
static void Hwt101_WriteRegSaved(uint8_t reg, uint16_t val,
                                 uint16_t d1_ms, uint16_t d2_ms)
{
	static const uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; /* KEY = 0xB588 */
	static const uint8_t save[]   = {0xFF, 0xAA, 0x00, 0x00, 0x00}; /* SAVE = 0x0000 */
	uint8_t wbuf[HWT101_WRITE_LEN];

	if (Hwt101_Tx(unlock) != HAL_OK) { health.tx_fail++; return; }
	HAL_Delay(d1_ms);
	Hwt101_MakeWrite(reg, val, wbuf);
	if (Hwt101_Tx(wbuf) != HAL_OK) { health.tx_fail++; return; }
	HAL_Delay(d2_ms);
	if (Hwt101_Tx(save) != HAL_OK) { health.tx_fail++; }
	HAL_Delay(20u);
}

/* 测 0x53 帧率。**必须在模块出流之后调** —— 它上电约 200ms 内一帧不发,
 * 采早了会把启动期低读成速率, 而那个读数正是"要不要写模块 Flash"的判据
 * (docs/坑-编码器与里程.md §7.8) */
static uint16_t Hwt101_MeasureYawHz(void)
{
	uint32_t f0 = health.yaw_seq;
	uint16_t t_ms = 0u;

	while (t_ms < HWT101_RATE_WIN_MS)
	{
		HAL_Delay(10u);
		t_ms += 10u;
	}
	return (uint16_t)(((health.yaw_seq - f0) * 1000u) / HWT101_RATE_WIN_MS);
}

void Hwt101_Init(void)
{
	/* 清零必须关中断: main() 里 UART_Task_Init() 已开优先级 0 的 IDLE 中断,
	 * 能在下面逐条赋值之间插进来写 ps/health, 最坏得到 has_prev_yaw=1 而 yaw_seq=0
	 * 的组合(靠下一帧 0x53 自愈, 属巧合而非设计)。评审 P1-4 */
	__disable_irq();
	ps.cnt = 0u;
	ps.has_wz = 0u;
	ps.has_prev_yaw = 0u;
	ps.reg_pending = 0u;
	ps.yaw_deg = 0.0f;
	ps.prev_yaw_deg = 0.0f;
	ps.last_wz_dps = 0.0f;
	health.yaw_seq = 0u;
	health.wz_seq = 0u;
	health.bad_sum = 0u;
	health.tx_fail = 0u;
	health.xchk_ok = 0u;
	health.xchk_discard = 0u;
	__enable_irq();

	health.rate_boot = Hwt101_MeasureYawHz();
	health.rate_hz   = health.rate_boot;
	if (health.rate_boot >= HWT101_RATE_TARGET_HZ)
	{
		return;   /* 已达标: 别再写模块 Flash(它有擦写寿命) */
	}
	Hwt101_ConfigRate(HWT101_RRATE_200HZ);
	health.rate_boot = Hwt101_MeasureYawHz();   /* 复验: 不达标会经 HWT: 行暴露 */
	health.rate_hz   = health.rate_boot;
}

void Hwt101_ConfigRate(uint8_t rrate)
{
	/* 官方 RRATE 示例的延时是 100/100ms; 这里取 200/200 更保守。
	 * ⚠ RRATE 写模块 Flash, 只在上电测得未达标时才走这里 */
	Hwt101_WriteRegSaved(HWT101_REG_RRATE, (uint16_t)rrate, 200u, 200u);
}

void Hwt101_ZeroYaw(void)
{
	/* 官方 CALIYAW 示例: unlock → 200ms → 0x76 写 0 → 500ms → save。
	 * 整段阻塞约 1.2s。⚠ 它与 GYROBAIS(0x4C 零偏) 是**两个独立寄存器**:
	 * CALIYAW 只改"角度读数从哪起", 不动陀螺零偏, 所以不会破坏自动零偏成果 */
	Hwt101_WriteRegSaved(HWT101_REG_CALIYAW, 0x0000u, 200u, 500u);
}

void Hwt101_ReadVersion(void)
{
	/* 读命令: [FF AA 27 2E 00], 官方注明读寄存器**不需要解锁**。
	 * 模块回 [55 5F d0..d7 SUM](11B), 由解析层 0x5F 分支填进 health。
	 * 这里只等应答到达, 不阻塞等结果 —— 结果由 Hwt101_Health() 读。
	 * ⚠ 必须先 Hwt101_ExpectReg(0x2E): 0x5F 是通用应答头, 不声明上下文的话
	 *   解析层无法判断这条应答是不是版本号(评审 P1-6)。
	 * ⚠ tx_fail 会被累加: 回读属"配置面"操作, 失败同样没有 ACK 可查。 */
	uint8_t rd[HWT101_WRITE_LEN];

	Hwt101_MakeWrite(HWT101_REG_READADDR, (uint16_t)HWT101_REG_VERSION, rd);
	/* MakeWrite 已把 rd[4] 写成 (0x2E>>8)&0xFF = 0x00, 即官方示例的末字节 00,
	 * **不要**再补一次赋值 —— 那看着像在修字节序, 实际是多余的 */
	if (Hwt101_Tx(rd) != HAL_OK)
	{
		health.tx_fail++;
		return;
	}
	Hwt101_ExpectReg((uint8_t)HWT101_REG_VERSION);
	HAL_Delay(30u);   /* 等应答; 结果由解析层填, 这里不阻塞等结果 */
}

/* NOAUTOCALI(0xA7) / MANUALCALI(0xA6): 官方注明**不需要解锁也不需要保存**,
 * 因此不套 Hwt101_WriteRegSaved —— 套了反而多两次无谓动作, 且这两条命令
 * 走的是运行期开关, 每次上电重发即可, 不必落 Flash */
void Hwt101_SetAutoCalib(uint8_t off)
{
	uint8_t cmd[HWT101_WRITE_LEN];

	Hwt101_MakeWrite(HWT101_REG_NOAUTOCALI, (uint16_t)(off ? 1u : 0u), cmd);
	if (Hwt101_Tx(cmd) != HAL_OK)
	{
		health.tx_fail++;
		return;
	}
	HAL_Delay(10u);
}

void Hwt101_ManualBias(uint8_t start)
{
	uint8_t cmd[HWT101_WRITE_LEN];

	/* 官方: 1 = 进入获取零偏(务必车完全静止), 4 = 退出。其它值无效 */
	Hwt101_MakeWrite(HWT101_REG_MANUALCALI, (uint16_t)(start ? 1u : 4u), cmd);
	if (Hwt101_Tx(cmd) != HAL_OK)
	{
		health.tx_fail++;
		return;
	}
	HAL_Delay(10u);
}
