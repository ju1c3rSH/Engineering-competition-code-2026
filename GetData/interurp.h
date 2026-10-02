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
/* 128: RRATE=200Hz 时模块连续出流(0x52+0x53 各 11B), 一个 IDLE 突发实测多为单帧 11B、
 * 两帧粘连时 22B; 旧值 44 只容 2 个突发就写满触发 DMA 停摆(丢帧, 靠下一拍 RxRearm 恢复),
 * 128 ≈ 数个突发的余量。解析按**实际突发长度**扫描, 不依赖 22B 这个经验值 */
#define RXbuffer3_size     128
#define RXbuffer6_size     30
/* 32B: 旧值 16 只容 1 帧, Brain 连续两帧一个 IDLE 突发(NAV 重发+其他)时 DMA 回绕
 * 令 UART4_RX_STA 算出错值、parser 收错乱尾段(随机性 ACK 丢失根因之一) */
#define RXbuffer4_size     32

extern uint8_t Rxbuffer1[RXbuffer1_size];
extern uint8_t Rxbuffer2[RXbuffer2_size];
extern uint8_t Rxbuffer3[RXbuffer3_size];
extern uint8_t Rxbuffer6[RXbuffer6_size];
extern uint8_t Rxbuffer4[RXbuffer4_size];

extern __IO uint16_t UART2_RX_STA;    /* stm32f4xx_it.c: 本次 IDLE 突发字节数(0x8000=完成) */
extern __IO uint16_t UART3_RX_STA;    /* stm32f4xx_it.c: 本次 IDLE 突发字节数 */
extern __IO uint16_t UART4_RX_STA;    /* stm32f4xx_it.c: 本次 IDLE 突发字节数 */
extern __IO uint16_t UART6_RX_STA;    /* stm32f4xx_it.c: 张大头 USART6 IDLE 突发字节数 */

/* HWT101 的帧计数已移入驱动库 GetData/hwt101.c:
 *   0x53 角度帧计数 -> Hwt101_YawSeq()   (与"车是否在转"解耦的新鲜度判据)
 *   0x52 角速度帧计数 -> Hwt101_WzSeq()
 * 读数 -> Car_GetYawDeg()/Car_GetGyroWzDps()(接缝声明在 App/car_link.h)
 * 诊断字段 -> Hwt101_Health()。此处不再 extern 任何 HWT101 全量。 */

void UART_Task_Init(void);
void Uart3_task(void);
void Uart4_task(void);
void Uart6_task(void);
void Uart2_task(void);
void Car_TxQueue_Pump(void);         /* TIM4 拍内调用: 泵出 ACK/EVT 发送队列 */

/* ---- USART2 调试口(115200): 主循环唯一 printf 点, ISR 零打印 ---- */
void Car_DebugTask(void);            /* main while(1) 调用: ACK/NACK/EVT 回显 + '?'/'p' 状态输出 */



#endif
