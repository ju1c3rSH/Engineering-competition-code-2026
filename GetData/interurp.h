#ifndef __INTERURP_H__
#define __INTERURP_H__

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "PID.h"
#include "Encode.h"
#include "usart.h"
#include "stdio.h"
#include "Control_Car.h"

//串口数据存储数组
#define RXbuffer1_size     8
#define RXbuffer2_size     10
#define RXbuffer3_size     44
#define RXbuffer6_size     30
#define RXbuffer4_size     16

extern uint8_t Rxbuffer1[RXbuffer1_size];
extern uint8_t Rxbuffer2[RXbuffer2_size];
extern uint8_t Rxbuffer3[RXbuffer3_size];
extern uint8_t Rxbuffer6[RXbuffer6_size];
extern uint8_t Rxbuffer4[RXbuffer4_size];

extern uint8_t Rxbuffer1_2[RXbuffer1_size];
extern uint8_t Rxbuffer2_2[RXbuffer2_size];
extern uint8_t Rxbuffer3_2[RXbuffer3_size];
extern uint8_t Rxbuffer6_2[RXbuffer6_size];
extern uint8_t Rxbuffer4_2[RXbuffer4_size];

extern __IO uint16_t UART4_RX_STA;    /* stm32f4xx_it.c: 本次 IDLE 突发字节数 */



void UART_Task_Init(void);
void Uart3_task(void);
void Uart4_task(void);
void Uart6_task(void);
void Uart2_task(void);
void Car_TxQueue_Pump(void);         /* TIM4 拍内调用: 泵出 ACK/EVT 发送队列 */



#endif
