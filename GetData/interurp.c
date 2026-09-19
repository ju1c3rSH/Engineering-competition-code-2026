#include "interurp.h"
#include "protocol.h"

int16_t YawL = 0;
int16_t YawH = 0;
int VL = 0;
int VH = 0;

int16_t RWzL = 0;
int16_t RWzH = 0;
int16_t WzL = 0;
int16_t WzH = 0;

uint8_t Rxbuffer1[RXbuffer1_size] = {0};
uint8_t Rxbuffer2[RXbuffer2_size] = {0};
uint8_t Rxbuffer3[RXbuffer3_size] = {0};
uint8_t Rxbuffer6[RXbuffer6_size] = {0};
uint8_t Rxbuffer4[RXbuffer4_size] = {0};

uint8_t Rxbuffer1_2[RXbuffer1_size] = {0};
uint8_t Rxbuffer2_2[RXbuffer2_size] = {0};
uint8_t Rxbuffer3_2[RXbuffer3_size] = {0};
uint8_t Rxbuffer6_2[RXbuffer6_size] = {0};
uint8_t Rxbuffer4_2[RXbuffer4_size] = {0};

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
	/* 转台事件登记 + 台架命令寄存 */
	uint8_t  tt_kind;                  /* 登记 kind: 0=停止 1=打码 2=微调 */
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
	uint16_t n;

	__disable_irq();
	if (ack != 0u)
	{
		dbg2.ack_cnt++;
		n = dbg2.ack_cnt;
		dbg2.ack_fresh = 1u;
	}
	else
	{
		dbg2.nack_cnt++;
		n = dbg2.nack_cnt;
		dbg2.nack_fresh = 1u;
	}
	dbg2.last_sub = sub;
	dbg2.last_code = code;
	(void)n;
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
	case CAR_SUB_REQ_NAVIGATION:       /* arg: navigation_target_id 0~7(6/7=重锚伪路线), speed_gear 0~3 */
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
		if (payload_length >= 3u && payload[2] <= 1u)
		{
			Car_Ack_Send(CAR_SUB_TRACKING_START, 0x01u);
			Car_Nav_MailPush(CAR_SUB_TRACKING_START, payload[2], 0u);
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
	/*检索并解包hwt101: 按本次突发实际长度扫描, sum8 校验通过才采用。
	 * 旧实现固定扫前 22 字节且不校验——多包输出时 0x53 角度包可能落在
	 * 扫描窗外(航向冻结), 数据字节恰为 55 53 时会假同步(航向跳变)。*/
	uint8_t i, j;
	uint16_t sum;
	uint8_t sta = (uint8_t)UART3_RX_STA;

	if (sta > RXbuffer3_size)
	{
		sta = RXbuffer3_size;
	}
	if (sta < 11u)
	{
		return;   /* 不足以容纳一个完整角度包 */
	}
	for (i = 0u; i <= sta - 11u; i++)
	{
		if ((Rxbuffer3[i] == 0x55u) && (Rxbuffer3[i + 1u] == 0x53u))
		{
			sum = 0u;
			for (j = 0u; j < 10u; j++)
			{
				sum += Rxbuffer3[i + j];
			}
			if ((uint8_t)sum == Rxbuffer3[i + 10u])
			{
				YawL = Rxbuffer3[i + 6u];
				YawH = Rxbuffer3[i + 7u];
				VL = Rxbuffer3[i + 8u];
				VH = Rxbuffer3[i + 9u];
				Yaw_struct.Yaw = (float)((YawH<<8)|YawL)/32768.0f*180.0f;
				if(Yaw_struct.Yaw > 180)
				{
					Yaw_struct.Yaw = -(360.0f - Yaw_struct.Yaw);
				}
				break;   /* 取突发内第一个通过校验的角度包 */
			}
		}
	}
}
void Uart2_task()
{
	uint8_t i, sta = (uint8_t)(UART2_RX_STA & 0x7FFFu);

	/* 消费 0x8000 完成位约定: 突发内扫 '?', 命中置查询标志供 Car_DebugTask 回报
	 * (历史 Uart2_task 恒空壳, 此处首次实现; ISR 只置标志, 打印全在主循环) */
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
	static uint8_t  periodic_en = 0u;
	uint32_t now;
	uint16_t ack_n, nack_n, evt_n;
	uint8_t  sub, code, evt, evt_echo;
	uint8_t  tt_kind, tt_code, bench_kind, bench_arg;
	float    tt_ang;
	uint8_t  ack_f = 0u, nack_f = 0u, evt_f = 0u, query_f = 0u, periodic_f = 0u;
	uint8_t  tt_f = 0u, bench_f = 0u;

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
		else
		{
			printf("TT:NUDGE @%d\r\n", (int)(tt_ang * 10.0f));
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
}
void Uart6_task()
{
	//步进电机数据处理
	/* 台架观测(USART2 '?' 回显): 统计 USART6 应答字节数与首字节 0x11
	 * (张大头驱动器对合法 FD 位置帧回应答)——区分"MCU 发了但驱动器没收到"
	 * 与"驱动器收到了已在动" */
	uint8_t sta = (uint8_t)(UART6_RX_STA & 0x0FFFu);

	if ((sta > 0u) && (sta <= RXbuffer6_size))
	{
		Emm_rx_cnt += sta;
		if (Rxbuffer6[0] == 0x11u)
		{
			Emm_reply_ok_cnt++;
		}
	}
}
