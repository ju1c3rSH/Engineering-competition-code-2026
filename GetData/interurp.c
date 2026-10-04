#include "interurp.h"
#include "protocol.h"
#include "selftest.h"
#include "hwt101.h"

/* HWT101 帧接收计数与读数已移入 GetData/hwt101.c（独立驱动库）。
 * 帧计数经 Hwt101_YawSeq()/Hwt101_WzSeq() 暴露, 读数经
 * Hwt101_GetYawDeg()/Hwt101_GetWzDps()。原来在这里的 Yaw_struct 全局量
 * 与 interurp.c 的扫描式解析已删除(见 docs/坑-编码器与里程.md §9)。 */

uint8_t Rxbuffer1[RXbuffer1_size] = {0};
uint8_t Rxbuffer2[RXbuffer2_size] = {0};
uint8_t Rxbuffer3[RXbuffer3_size] = {0};
uint8_t Rxbuffer6[RXbuffer6_size] = {0};
uint8_t Rxbuffer4[RXbuffer4_size] = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)      //10ms定时器
	{
		Car_Control();
	}
	else if(htim->Instance == TIM5)
	{
		Emm_Pos_Control();   /* 内部带 tt_arm 门控: 首条 TURNTABLE 前/急停后不自走 */
	}
}

/* ================== UART4 ================== */
#define CAR_ACK_EN   1        /* 收到包后是否返回ACK帧 */

static Protocol_Parser_t uart4_parser;
static uint8_t uart4_body[16];
static uint8_t evt_seq = 1;

/* ================== UART4 上行发送队列 ==================
 * 生产者两个: UART4 中断(ACK/NACK) 与 TIM4 中断(事件上报), 优先级不同可互相抢占;
 * 消费者唯一: TIM4 10ms 拍里的 Car_TxQueue_Pump。
 * 取代旧"static txbuf + 直接 Transmit_DMA"写法: 那样在 DMA 在途时返回 HAL_BUSY
 * 会静默丢事件帧, 且同函数重入会覆写 static 缓冲把在途帧发坏 */
#define CAR_TXQ_DEPTH   8u          /* 幂 2, 便于取模 */
#define CAR_TXQ_FRAME   16u         /* 最大帧: DATA(payload10) = 16B */

static uint8_t txq_buf[CAR_TXQ_DEPTH][CAR_TXQ_FRAME];
static uint8_t txq_len[CAR_TXQ_DEPTH];
static volatile uint8_t txq_head = 0u;   /* 写位置(仅生产者, 关中断内更新) */
static volatile uint8_t txq_tail = 0u;   /* 读位置(仅 Pump 更新) */

static void Car_Dbg2_EvtRecord(uint8_t event, uint8_t echo);   /* 定义在 dbg2 区, 前置声明 */

/* 入队: 两个 ISR 上下文都会调, 关中断保护头指针与缓冲拷贝 */
static uint8_t Car_TxEnqueue(uint8_t type, const uint8_t *payload, uint8_t plen)
{
	uint8_t ok = 0u;
	uint16_t n;

	__disable_irq();
	if ((uint8_t)((txq_head + 1u) & (CAR_TXQ_DEPTH - 1u)) != txq_tail)
	{
		n = Protocol_EncodeFrame(txq_buf[txq_head], CAR_TXQ_FRAME, type, payload, plen);
		if (n != 0u)
		{
			txq_len[txq_head] = (uint8_t)n;
			txq_head = (uint8_t)((txq_head + 1u) & (CAR_TXQ_DEPTH - 1u));
			ok = 1u;
		}
	}
	__enable_irq();
	/* 队列满时丢最新一帧(保持入队者不被阻塞); 上行均为通知类, 由上层超时兜底 */
	return ok;
}

/* 单点泵出: 仅在 TIM4 10ms 拍里调用; DMA 在途则留到下拍续发 */
void Car_TxQueue_Pump(void)
{
	if (txq_tail == txq_head)
	{
		return;
	}
	if (huart4.gState != HAL_UART_STATE_READY)
	{
		return;
	}
	HAL_UART_Transmit_DMA(&huart4, txq_buf[txq_tail], txq_len[txq_tail]);
	txq_tail = (uint8_t)((txq_tail + 1u) & (CAR_TXQ_DEPTH - 1u));
}

void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason)
{
	uint8_t p[10];
	uint8_t i;

	p[0] = evt_seq++;
	p[1] = CAR_DATA_ID_EVT;
	p[2] = event;
	p[3] = echo;
	p[4] = reason;
	for (i = 5u; i < 10u; i++)
	{
		p[i] = 0u;
	}
	(void)Car_TxEnqueue(FRAME_TYPE_DATA, p, 10u);
	Car_Dbg2_EvtRecord(event, echo);
}

/* ---- 调试口观测记录 (USART2, 详 interurp.h) -------------------------------
 * 生产者: UART4 ISR(ACK/NACK 见帧即记) 与 TIM4 ISR(事件帧同路); 消费者: 主循环
 * Car_DebugTask() 唯一发出口——所有 printf/阻塞发送只在主循环, 四个 ISR 零打印 ---- */
static volatile struct
{
	uint16_t ack_cnt;
	uint16_t nack_cnt;
	uint16_t evt_cnt;
	uint8_t  last_sub;
	uint8_t  last_code;    /* ACK=status / NACK=reason */
	uint8_t  last_evt;     /* EVT=事件码 */
	uint8_t  last_evt_echo;
	volatile uint8_t ack_fresh;
	volatile uint8_t nack_fresh;
	volatile uint8_t evt_fresh;
	volatile uint8_t query_fresh;
	volatile uint8_t periodic_fresh;   /* 调试口 'p': 翻转周期状态输出 */
	volatile uint8_t trace_fresh;      /* 调试口 't': 武装"段内瞬时跟踪"(见 car_obs.c) */
	/* 转台事件登记 + 台架命令寄存 */
	uint8_t  tt_kind;                  /* 登记 kind: 0=停止 1=打码 2=微调 3=上电零点对齐 */
	uint8_t  tt_code;                  /* 本次登记的码 */
	float    tt_ang;                   /* 登记时刻目标角(deg) */
	volatile uint8_t tt_fresh;
	volatile uint8_t bench_fresh;
	uint8_t  bench_kind;               /* 台架: 0=停转 1=+5° 2=-5° 3=直接打码 */
	uint8_t  bench_arg;                /* bench_kind==3: 码 1~7 */
} dbg2;

/* 入队后的观测登记: ISR 与主循环双上下文都会调, 关中断保护计数字段 */
static void Car_Dbg2_Record(uint8_t ack, uint8_t sub, uint8_t code)
{
	__disable_irq();
	if (ack != 0u)
	{
		dbg2.ack_cnt++;
		dbg2.ack_fresh = 1u;
	}
	else
	{
		dbg2.nack_cnt++;
		dbg2.nack_fresh = 1u;
	}
	dbg2.last_sub = sub;
	dbg2.last_code = code;
	__enable_irq();
}

/* EVT 上行回显登记: 生产者 TIM4 ISR(Car_Event_Send), 消费者主循环 Car_DebugTask */
static void Car_Dbg2_EvtRecord(uint8_t event, uint8_t echo)
{
	__disable_irq();
	dbg2.last_evt = event;
	dbg2.last_evt_echo = echo;
	dbg2.evt_cnt++;
	dbg2.evt_fresh = 1u;
	__enable_irq();
}

/* TT 转台事件登记(kind: 0=停止 1=打码 2=微调)。生产者: TIM4 ISR(Nav_Turntable/
 * Nav_DoStop)与主循环台架命令, 消费者 Car_DebugTask */
void Car_Dbg2_TtRecord(uint8_t kind, uint8_t code, float angle_deg)
{
	__disable_irq();
	dbg2.tt_kind  = kind;
	dbg2.tt_code  = code;
	dbg2.tt_ang   = angle_deg;
	dbg2.tt_fresh = 1u;
	__enable_irq();
}

/* 作废上电静置期攒下的台架转台命令。UART_Task_Init 一到静置结束(2.5s)之间 USART2
 * 是活的而 TIM5 还没起, 这期间任何裸字节('1'~'7'/'w'/'e'/'x')都只攒成 bench_fresh,
 * 留到主循环第一圈就会拿噪声驱动转台。调用点: Car_Turntable_BootZero() */
void Car_Dbg2_BenchDrop(void)
{
	__disable_irq();
	dbg2.bench_fresh = 0u;
	dbg2.bench_kind  = 0u;
	dbg2.bench_arg   = 0u;
	__enable_irq();
}

static void Car_Ack_Send(uint8_t sub_echo, uint8_t status)
{
#if CAR_ACK_EN
	uint8_t p[3];

	p[0] = CAR_DATA_ID_CTRL;
	p[1] = sub_echo;
	p[2] = status;
	if (Car_TxEnqueue(FRAME_TYPE_ACK, p, 3u) != 0u)
	{
		Car_Dbg2_Record(1u, sub_echo, status);
	}
#else
	(void)sub_echo;
	(void)status;
#endif
}

/* NACK payload 3B [data_id, sub_echo, reason] (v2.2 §1.4): sub_echo=被拒 sub_cmd 原值,
 * 帧类型/data_id 层被拒时填 0; master(Brain) 按长度判别, legacy 2B 仍须被其接受 */
static void Car_Nack_Send(uint8_t sub_echo, uint8_t reason)
{
	uint8_t p[3];

	p[0] = CAR_DATA_ID_CTRL;
	p[1] = sub_echo;
	p[2] = reason;
	if (Car_TxEnqueue(FRAME_TYPE_NACK, p, 3u) != 0u)
	{
		Car_Dbg2_Record(0u, sub_echo, reason);
	}
}

static void Uart4_OnFrame(void *callback_context, uint8_t type, const uint8_t *payload, uint8_t payload_length)
{
	/* callback_context 恒为 NULL(Protocol_ParserInit 内写死), 仅为凑框架签名保留 */
	(void)callback_context;

	if (type == FRAME_TYPE_STOP)                 /* CMD_STOP(通用) -> 急停 */
	{
		Car_Ack_Send(CAR_SUB_REQ_STOP, 0x01u);
		Car_Nav_MailPush(CAR_SUB_REQ_STOP, 0u, 0u);
		return;
	}
	if (type != FRAME_TYPE_REQUEST || payload_length < 1u)
	{
		Car_Nack_Send(0u, CAR_NACK_FORMAT);      /* 非 REQUEST 突发/过短: 无法定位 sub */
		return;
	}
	if (payload[0] != CAR_DATA_ID_CTRL)
	{
		Car_Nack_Send(payload[0], CAR_NACK_FORMAT);   /* data_id 不支持: 回显原 data_id */
		return;
	}
	if (payload_length < 2u)
	{
		Car_Nack_Send(0u, CAR_NACK_FORMAT);
		return;
	}

	switch (payload[1])
	{
	case CAR_SUB_REQ_NAVIGATION:       /* arg: navigation_target_id 0~8(6/7/8=重锚伪路线), speed_gear 0~3 */
		if (payload_length >= 4u && payload[2] < CAR_NAV_ID_COUNT && payload[3] <= 3u)
		{
			Car_Ack_Send(CAR_SUB_REQ_NAV, 0x01u);
			Car_Nav_MailPush(CAR_SUB_REQ_NAV, payload[2], payload[3]);
		}
		else
		{
			Car_Nack_Send(payload[1],
			              (payload_length >= 4u) ? CAR_NACK_PARAM : CAR_NACK_FORMAT);
		}
		break;

	case CAR_SUB_REQ_STOP:             /* 急停: 参数忽略 */
		Car_Ack_Send(CAR_SUB_REQ_STOP, 0x01u);
		Car_Nav_MailPush(CAR_SUB_REQ_STOP, 0u, 0u);
		break;

	case CAR_SUB_REQ_TURNTABLE:        /* arg: 转台码 0~7(1~3对坑 4~6对桶 7回正, 0=不动作) */
		if (payload_length >= 3u && payload[2] <= 7u)
		{
			Car_Ack_Send(CAR_SUB_REQ_TURNTABLE, 0x01u);
			Car_Nav_MailPush(CAR_SUB_REQ_TURNTABLE, payload[2], 0u);
		}
		else
		{
			Car_Nack_Send(payload[1],
			              (payload_length >= 3u) ? CAR_NACK_PARAM : CAR_NACK_FORMAT);
		}
		break;

	case CAR_SUB_GYRO_ZERO:            /* 陀螺仪置零: 须车静止(原点上电流程) */
		Car_Ack_Send(CAR_SUB_GYRO_ZERO, 0x01u);
		Car_Nav_MailPush(CAR_SUB_GYRO_ZERO, 0u, 0u);
		break;

	case CAR_SUB_TRACKING_START:
		/* payload 4B [data_id, sub, speed_level, aim_axis]; 两参仅做范围校验
		 * (底盘不再分轴/不设低速档), 越界 → PARAM */
		if (payload_length >= 4u && payload[2] <= 1u && payload[3] <= 1u)
		{
			Car_Ack_Send(CAR_SUB_TRACKING_START, 0x01u);
			Car_Nav_MailPush(CAR_SUB_TRACKING_START, payload[2], payload[3]);
		}
		else
		{
			Car_Nack_Send(payload[1],
			              (payload_length >= 3u) ? CAR_NACK_PARAM : CAR_NACK_FORMAT);
		}
		break;

	/* 误差流无 ACK(20~50Hz 高频, 回 ACK 会淹没链路); 短帧静默丢弃 */
	case CAR_SUB_TRACKING_ERROR:
		if (payload_length >= 5u)
		{
			int16_t horizontal_error_fixed =
			    (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
			Car_Tracking_ErrorPush(horizontal_error_fixed, payload[4]);
		}
		break;

	case CAR_SUB_TRACKING_STOP:
		if (payload_length >= 3u && payload[2] <= 1u)
		{
			Car_Ack_Send(CAR_SUB_TRACKING_STOP, 0x01u);
			Car_Nav_MailPush(CAR_SUB_TRACKING_STOP, payload[2], 0u);
		}
		else
		{
			Car_Nack_Send(payload[1],
			              (payload_length >= 3u) ? CAR_NACK_PARAM : CAR_NACK_FORMAT);
		}
		break;

	default:
		Car_Nack_Send(payload[1], CAR_NACK_UNKNOWNSUB);   /* 回显原 sub 定位 */
		break;
	}
}

void Uart4_task()
{
	/* IDLE 中断时 Rxbuffer4[0..UART4_RX_STA-1] 为本次突发，整块喂入解析器 */
	Protocol_ParserFeedBlock(&uart4_parser, Rxbuffer4, (uint8_t)UART4_RX_STA);
}

void UART_Task_Init(void)
{
	  HAL_UART_Receive_DMA(&huart2, Rxbuffer2, RXbuffer2_size);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	  HAL_UART_Receive_DMA(&huart3, Rxbuffer3, RXbuffer3_size);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	  HAL_UART_Receive_DMA(&huart6, Rxbuffer6, RXbuffer6_size);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart4, Rxbuffer4, RXbuffer4_size);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

	Protocol_ParserInit(&uart4_parser, uart4_body, sizeof(uart4_body), 0);
	uart4_parser.on_frame = Uart4_OnFrame;
}
void Uart3_task()
{
	/* HWT101 帧解析已收敛到独立库(GetData/hwt101.c): 状态机 + 0x52/0x53/0x5F 三帧 + 交叉校验。
	 * 这里只按本次 IDLE 突发的**实际长度**喂字节: 必须传实际长度而非常量 22(连续流, 一个突发帧数不固定)。
	 * 解析 O(n) 无循环上界, 跑在优先级 0 的 USART3 中断里, 无 printf/无 Delay。 */
	uint8_t sta = (uint8_t)UART3_RX_STA;

	if (sta > RXbuffer3_size)
	{
		sta = RXbuffer3_size;
	}
	if (sta == 0u)
	{
		return;
	}
	Hwt101_FeedBytes(Rxbuffer3, (uint16_t)sta);
}
void Uart2_task()
{
	uint8_t i, sta = (uint8_t)(UART2_RX_STA & 0x7FFFu);

	/* 消费 0x8000 完成位约定: 突发内扫命令, 命中的置标志供 Car_DebugTask 处理。
	 * '?'=状态块 / 'p'=翻 1Hz 周期块 / 't'=武装段内瞬时跟踪(20Hz×4s, 见 car_obs.c)
	 * 'x''w''e''1'~'7'=台架直转转台 (ISR 只置标志, 打印与执行全在主循环) */
	if (sta == 0u)
	{
		return;
	}
	if (sta > RXbuffer2_size)
	{
		sta = RXbuffer2_size;
	}
	for (i = 0u; i < sta; i++)
	{
		if (Rxbuffer2[i] == '?')
		{
			dbg2.query_fresh = 1u;
		}
		else if (Rxbuffer2[i] == 'p')
		{
			dbg2.periodic_fresh = 1u;   /* 翻转 1Hz 周期状态输出开关 */
		}
		else if (Rxbuffer2[i] == 't')
		{
			dbg2.trace_fresh = 1u;      /* 武装段内瞬时跟踪(20Hz×4s, 命中弧段自动开窗) */
		}
		/* 台架直转命令: ISR 只置标志, 命令执行/打印全在 Car_DebugTask 主循环 */
		else if (Rxbuffer2[i] == 'x')
		{
			dbg2.bench_kind = 0u;
			dbg2.bench_fresh = 1u;
		}
		else if (Rxbuffer2[i] == 'w')
		{
			dbg2.bench_kind = 1u;
			dbg2.bench_fresh = 1u;
		}
		else if (Rxbuffer2[i] == 'e')
		{
			dbg2.bench_kind = 2u;
			dbg2.bench_fresh = 1u;
		}
		else if ((Rxbuffer2[i] >= '1') && (Rxbuffer2[i] <= '7'))
		{
			dbg2.bench_kind = 3u;
			dbg2.bench_arg = (uint8_t)(Rxbuffer2[i] - '0');
			dbg2.bench_fresh = 1u;
		}
	}
}

/* ================== USART2 调试口(115200): 事件回显 + '?' 查询 ==================
 * 主循环每圈调用: 唯一的 printf/阻塞发送点; ISR 从不发送。
 * 查询回两行: 调度状态(NAV/TRK/误差) + 里程位姿, 详见 Car_Debug_Snapshot */
void Car_DebugTask(void)
{
	static uint32_t stat_last_tick = 0u;
	/* 自测形态由 car_config.h 切到 1: 跑全程不用手动敲 'p' 就能录下每段停等的 X/Y/YAW */
	static uint8_t  periodic_en = (uint8_t)DBG_PERIODIC_DEFAULT_EN;
	uint32_t now;
	uint16_t ack_n, nack_n, evt_n;
	uint8_t  sub, code, evt, evt_echo;
	uint8_t  tt_kind, tt_code, bench_kind, bench_arg;
	float    tt_ang;
	uint8_t  ack_f = 0u, nack_f = 0u, evt_f = 0u, query_f = 0u, periodic_f = 0u;
	uint8_t  tt_f = 0u, bench_f = 0u, trace_f = 0u;

	Hwt101_RateTask();   /* 0x53 帧率滚动统计: 每圈刷新, 与有没有敲 '?' 无关 */
	__disable_irq();
	ack_n  = dbg2.ack_cnt;
	nack_n = dbg2.nack_cnt;
	evt_n  = dbg2.evt_cnt;
	sub    = dbg2.last_sub;
	code   = dbg2.last_code;
	evt    = dbg2.last_evt;
	evt_echo = dbg2.last_evt_echo;
	tt_kind  = dbg2.tt_kind;
	tt_code  = dbg2.tt_code;
	bench_kind  = dbg2.bench_kind;
	bench_arg   = dbg2.bench_arg;
	tt_ang   = dbg2.tt_ang;
	if (dbg2.ack_fresh != 0u)      { dbg2.ack_fresh = 0u;      ack_f = 1u; }
	if (dbg2.nack_fresh != 0u)     { dbg2.nack_fresh = 0u;     nack_f = 1u; }
	if (dbg2.evt_fresh != 0u)      { dbg2.evt_fresh = 0u;      evt_f = 1u; }
	if (dbg2.query_fresh != 0u)    { dbg2.query_fresh = 0u;    query_f = 1u; }
	if (dbg2.periodic_fresh != 0u) { dbg2.periodic_fresh = 0u; periodic_f = 1u; }
	if (dbg2.trace_fresh != 0u)    { dbg2.trace_fresh = 0u;    trace_f = 1u; }
	if (dbg2.tt_fresh != 0u)       { dbg2.tt_fresh = 0u;       tt_f = 1u; }
	if (dbg2.bench_fresh != 0u)    { dbg2.bench_fresh = 0u;    bench_f = 1u; }
	__enable_irq();

	if (ack_f != 0u)
	{
		printf("ACK:%02X/%02X #%u\r\n", sub, code, (unsigned)ack_n);
	}
	if (nack_f != 0u)
	{
		printf("NACK:%02X/%02X #%u\r\n", sub, code, (unsigned)nack_n);
	}
	if (evt_f != 0u)   /* NAV_DONE/NAV_FAULT/TURN_DONE/TRACKING_DONE 上行回显 */
	{
		printf("EVT:%02X:%02X #%u\r\n", evt, evt_echo, (unsigned)evt_n);
	}
	if (tt_f != 0u)   /* 转台事件: 0=停止 1=收到协议码 2=台架微调 */
	{
		if (tt_kind == 0u)
		{
			printf("TT:STOP CODE=%u @%d\r\n", (unsigned)tt_code, (int)(tt_ang * 10.0f));
		}
		else if (tt_kind == 1u)
		{
			printf("TT:CODE=%u ARM=1 @%d\r\n", (unsigned)tt_code, (int)(tt_ang * 10.0f));
		}
		else if (tt_kind == 2u)
		{
			printf("TT:NUDGE @%d\r\n", (int)(tt_ang * 10.0f));
		}
		else
		{
			printf("TT:BOOT ZERO @%d\r\n", (int)(tt_ang * 10.0f));   /* 3=上电零点对齐 */
		}
	}
	if (bench_f != 0u)   /* 台架直转执行(命令打印走 TT/WTBL 行) */
	{
		if (bench_kind == 0u)
		{
			Car_Turntable_BenchStop();
		}
		else if (bench_kind == 1u)
		{
			Car_Turntable_BenchNudge(5.0f);
		}
		else if (bench_kind == 2u)
		{
			Car_Turntable_BenchNudge(-5.0f);
		}
		else if ((bench_kind == 3u) && (bench_arg >= 1u) && (bench_arg <= 7u))
		{
			Car_Turntable_BenchCode(bench_arg);
		}
		dbg2.query_fresh = 1u;   /* 回一份完整状态块供 WTBL 核对 */
	}
	if (query_f != 0u)
	{
		/* 回读 HWT101 版本号(0x27 读 0x2E): 换模块/批次时确认能否依赖官方零偏流程; 只在 '?' 时读, 不进上电静置路径。
		 * 应答由 Hwt101_FeedBytes 解析进 health.version, **下一拍** HWT: 行才带上(~30ms 延迟)。
		 * 唯一代价: 这条 11B 应答占一帧窗口, 交叉校验可能丢一个 Wz —— 仅人工敲 '?' 时发生, 不影响行驶。 */
		Hwt101_ReadVersion();
		Car_Debug_Stat_Print();
	}
	if (periodic_f != 0u)   /* 'p': 翻转 1Hz 周期状态块, 翻转回执确认 */
	{
		periodic_en = (uint8_t)!periodic_en;
		stat_last_tick = HAL_GetTick();
		printf("PERIODIC:%u\r\n", (unsigned)periodic_en);
	}
	if (periodic_en != 0u)
	{
		now = HAL_GetTick();
		if ((uint32_t)(now - stat_last_tick) >= 1000u)
		{
			stat_last_tick = now;
			Car_Debug_Stat_Print();
		}
	}
	Car_SegChange_Log();   /* 段切换: 变化才打, 主循环上下文(唯一 printf 点) */
	Car_SegExit_Log();     /* 段出口质量: ARC 出 EY/YE, 直线出 LN:IE/YE/EY */
	Car_YawZero_Log();     /* 整表重锚: 置零那拍打一行 YAWZERO(基点/开关) */
	Car_TrkChange_Log();   /* 跟瞄状态迁移: 四态机换态才打一行 TRK:旧>新 */
	Car_TrkDetail_Log();   /* ARMED/STOPPED 期 10Hz 打点 TRKD: 判"误差卡在哪一环" */
	if (trace_f != 0u)
	{
		Car_Obs_TraceArm();   /* 't': 武装段内跟踪, 命中弧段自动开窗(见 car_obs.c) */
	}
	Car_Obs_TraceTask();      /* 段内瞬时跟踪: 未武装/未命中时立即返回 */

	/* RT:FINISH / PD: 漂移行; 推进本身在 10ms 定时里, 打印只能在主循环 */
	SELFTEST_DEBUG_POLL();
}
void Uart6_task()
{
	//步进电机数据处理
	/* 台架观测('?' 的 EMM 行): OK = 认"回显功能码 0xFD + 帧尾 0x6B"的位置帧应答次数(不认地址),
	 * RX = USART6 应答字节数; 上电清零帧(0x0A)的应答不计入 OK */
	uint8_t sta = (uint8_t)(UART6_RX_STA & 0x0FFFu);

	if ((sta > 0u) && (sta <= RXbuffer6_size))
	{
		Emm_rx_cnt += sta;
		if ((sta >= 4u) && (Rxbuffer6[1] == 0xFDu) && (Rxbuffer6[3] == 0x6Bu))
		{
			Emm_reply_ok_cnt++;
		}
	}
}
