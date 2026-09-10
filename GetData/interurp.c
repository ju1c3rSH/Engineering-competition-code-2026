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

void Car_Event_Send(uint8_t event, uint8_t echo, uint8_t reason)
{
	static uint8_t txbuf[16];
	uint8_t p[10];
	uint16_t n;
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
	n = Protocol_EncodeFrame(txbuf, sizeof(txbuf), 0x24u, p, 10u);
	if (n != 0u)
	{
		HAL_UART_Transmit_DMA(&huart4, txbuf, n);
	}
}

static void Car_Ack_Send(uint8_t sub_echo, uint8_t status)
{
#if CAR_ACK_EN
	static uint8_t txbuf[16];
	uint8_t p[3];
	uint16_t n;

	p[0] = CAR_DATA_ID_CTRL;
	p[1] = sub_echo;
	p[2] = status;
	n = Protocol_EncodeFrame(txbuf, sizeof(txbuf), 0x21u, p, 3u);
	if (n != 0u)
	{
		HAL_UART_Transmit_DMA(&huart4, txbuf, n);
	}
#else
	(void)sub_echo;
	(void)status;
#endif
}

static void Car_Nack_Send(uint8_t reason)
{
	static uint8_t txbuf[16];
	uint8_t p[2];
	uint16_t n;

	p[0] = CAR_DATA_ID_CTRL;
	p[1] = reason;
	n = Protocol_EncodeFrame(txbuf, sizeof(txbuf), 0x22u, p, 2u);
	if (n != 0u)
	{
		HAL_UART_Transmit_DMA(&huart4, txbuf, n);
	}
}

static void Uart4_OnFrame(void *ctx, uint8_t type, const uint8_t *payload, uint8_t plen)
{
	(void)ctx;

	if (type == 0x23u)                 /* CMD_STOP(通用) -> 急停 */
	{
		Car_Ack_Send(CAR_SUB_REQ_STOP, 0x01u);
		Car_Nav_MailPush(CAR_SUB_REQ_STOP, 0u, 0u);
		return;
	}
	if (type != 0x20u || plen < 2u || payload[0] != CAR_DATA_ID_CTRL)
	{
		Car_Nack_Send(0x01u);
		return;
	}

	switch (payload[1])
	{
	case CAR_SUB_REQ_NAV:              /* arg: nav_id, gear */
		if (plen >= 4u && payload[2] < CAR_NAV_ID_COUNT && payload[3] <= 3u)
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

	case CAR_SUB_REQ_TURNTABLE:        /* arg: pit 0~3 */
		if (plen >= 3u && payload[2] <= 3u)
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
