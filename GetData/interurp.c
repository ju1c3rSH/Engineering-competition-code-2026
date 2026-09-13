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
		Emm_Pos_Control(Car_data.Motor_Angle);
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
}

static void Car_Ack_Send(uint8_t sub_echo, uint8_t status)
{
#if CAR_ACK_EN
	uint8_t p[3];

	p[0] = CAR_DATA_ID_CTRL;
	p[1] = sub_echo;
	p[2] = status;
	(void)Car_TxEnqueue(FRAME_TYPE_ACK, p, 3u);
#else
	(void)sub_echo;
	(void)status;
#endif
}

static void Car_Nack_Send(uint8_t reason)
{
	uint8_t p[2];

	p[0] = CAR_DATA_ID_CTRL;
	p[1] = reason;
	(void)Car_TxEnqueue(FRAME_TYPE_NACK, p, 2u);
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
	if (type != FRAME_TYPE_REQUEST || payload_length < 2u || payload[0] != CAR_DATA_ID_CTRL)
	{
		Car_Nack_Send(0x01u);
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
			Car_Nack_Send(0x01u);
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
			Car_Nack_Send(0x01u);
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
			Car_Nack_Send(0x01u);
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
			Car_Nack_Send(0x01u);
		}
		break;

	default:
		Car_Nack_Send(0x01u);
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
		/*检索并解包hwt101*/
		for(uint8_t i = 0; i<22; i++)
		{
			if(Rxbuffer3[i] == 0x55 && Rxbuffer3[(i+1)%22] == 0x53)
			{
				YawL = Rxbuffer3[(i + 6)%22];
				YawH = Rxbuffer3[(i + 7)%22];
				VL = Rxbuffer3[(i + 8)%22];
				VH = Rxbuffer3[(i + 9)%22];
				Yaw_struct.Yaw = (float)((YawH<<8)|YawL)/32768*180;
				if(Yaw_struct.Yaw > 180)
				{
					Yaw_struct.Yaw = -(360.0f - Yaw_struct.Yaw);
				}
				break;
			}
		}
}
void Uart2_task()
{
		//调试数据处理
}
void Uart6_task()
{
		//步进电机数据处理
}
