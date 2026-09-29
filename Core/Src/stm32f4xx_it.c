/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include "stdio.h"
#include "interurp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

__IO uint16_t UART2_RX_STA = 0;
__IO uint16_t UART3_RX_STA = 0;
__IO uint16_t UART4_RX_STA = 0;
__IO uint16_t UART6_RX_STA = 0;

/* 调试口(USART2)链路健康计数 —— 用于把"日志空档/串口没反应"定位到具体一层。
 * 生产者: fputc(主循环) 与 HAL_UART_ErrorCallback(ISR); 消费者: Car_Debug_Stat_Print
 * 的 LINK 行(主循环, 1Hz)。均为单字读写, 无需临界区。 */
volatile uint32_t Dbg2_TxFail_Cnt = 0u;   /* HAL_UART_Transmit 非 OK = 该字符没发出去 */
volatile uint32_t Dbg2_RxErr_Cnt  = 0u;   /* USART2 RX 错误回调次数(ORE/FE/NE/PE) */


/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============ 故障勘查 (cnblogs ljbguanli 法落地): HardFault 时从栈帧挖回 PC/LR ============
 * 现场 PC/LR/CFSR 同时写RTC 备份寄存器: IWDG/PIN 复位不清备份域,下次上电调试口打出来,
 * 再用 fromelf -c panzer.axf / Keil Disassembly 把 PC 对回源码行。
 * RAM 快照供挂调试器时随时看(调试器态 IWDG 冻结, g_fault_buf 新鲜)。 */
volatile uint32_t g_fault_buf[6] = {0, 0, 0, 0, 0, 0};

void Fault_Capture(uint32_t *sp)
{
  g_fault_buf[0] = sp[6];      /* PC    */
  g_fault_buf[1] = sp[5];      /* LR    */
  g_fault_buf[2] = SCB->CFSR;
  g_fault_buf[3] = SCB->HFSR;
  g_fault_buf[4] = SCB->MMFAR;
  g_fault_buf[5] = SCB->BFAR;

  /* RTC 备份域: 仅需 PWR 时钟 + DBP, 与 RTC 本体是否启用无关(IWDG 复位不清) */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
  (void)RCC->APB1ENR;
  SET_BIT(PWR->CR, PWR_CR_DBP);
  RTC->BKP0R = 0xFA17FA17u;
  RTC->BKP1R = g_fault_buf[0];
  RTC->BKP2R = g_fault_buf[1];
  RTC->BKP3R = g_fault_buf[2];
  RTC->BKP4R = g_fault_buf[3];
  RTC->BKP5R = g_fault_buf[4];
  RTC->BKP6R = g_fault_buf[5];

  TIM3->CCR1 = 0u;   /* 故障态先清履带 PWM, 防失控滑跑(喂狗停摆后 IWDG 复位前窗口) */
  TIM3->CCR2 = 0u;
  for (;;) { }       /* 使 IWDG 复位重启; 挂调试器时被冻结死机现场可查 */
}

void Fault_Capture_Auto(void)
{
  uint32_t *__sp;

  __asm volatile (
      "tst   lr, #4     \n"
      "ite   eq         \n"
      "mrseq %0, msp    \n"
      "mrsne %0, psp    \n"
      : "=r"(__sp) :: );
  Fault_Capture(__sp);
}

void HardFault_Handler(void);   /* 定义在 file 主体(接 Fault_Capture), 见下 */

/* ---- RX DMA 重装修复: IDLE/Error 路径在 HAL 锁竞争(HAL_BUSY)下的失败必须可恢复 ----
 * UART4 IDLE(flash 优先级0)抢占 TIM4 发送(DMA Tx 持 huart4 锁)时,
 * HAL_UART_Receive_DMA 返回 HAL_BUSY; 旧代码丢掉返回值 → RX DMA 已被 DMAStop
 * 裸停而不再装回 → IDLE 从此不产生, 控制链路永久哑掉。
 * 现改: 失败置标志,Car_Control 的 TIM4 拍调用 Uart_RxRearm_Task 重试(堵住死口)。 */
static volatile uint8_t rx_rearm_req = 0u;

static void RxRearm(UART_HandleTypeDef *hu, uint8_t *buf, uint16_t size, uint8_t bit)
{
  if (HAL_UART_Receive_DMA(hu, buf, size) != HAL_OK)
  {
    rx_rearm_req |= bit;     /* HAL 持锁中: 下一 TIM4 拍重试 */
  }
  else
  {
    rx_rearm_req &= (uint8_t)(~bit);
  }
}

void Uart_RxRearm_Task(void)
{
  if ((rx_rearm_req & 1u) != 0u) { RxRearm(&huart4, Rxbuffer4, RXbuffer4_size, 1u); }
  if ((rx_rearm_req & 2u) != 0u) { RxRearm(&huart3, Rxbuffer3, RXbuffer3_size, 2u); }
  if ((rx_rearm_req & 4u) != 0u) { RxRearm(&huart2, Rxbuffer2, RXbuffer2_size, 4u); }
  if ((rx_rearm_req & 8u) != 0u) { RxRearm(&huart6, Rxbuffer6, RXbuffer6_size, 8u); }
}



void Fault_Boot_Report(void)
{
  if (RTC->BKP0R == 0xFA17FA17u)
  {
    printf("FAULT-LAST: PC=%08lX LR=%08lX CFSR=%08lX HFSR=%08lX MMFAR=%08lX BFAR=%08lX\r\n",
           (unsigned long)RTC->BKP1R, (unsigned long)RTC->BKP2R,
           (unsigned long)RTC->BKP3R, (unsigned long)RTC->BKP4R,
           (unsigned long)RTC->BKP5R, (unsigned long)RTC->BKP6R);
    RTC->BKP0R = 0u;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)     { printf("RST=IWDG\r\n"); }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) { printf("RST=BOR\r\n"); }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) { printf("RST=SOFTR\r\n"); }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) { printf("RST=POR\r\n"); }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) { printf("RST=PIN\r\n"); }
  __HAL_RCC_CLEAR_RESET_FLAGS();
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  TIM3->CCR1 = 0u;   /* 故障态先清履带 PWM(占空比=CCR/ARR, 低有效驱动), 防失控滑跑 */
  TIM3->CCR2 = 0u;
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  {
    uint32_t *__sp;
    __asm volatile (
        "tst   lr, #4  \n"
        "ite   eq      \n"
        "mrseq %0, msp \n"
        "mrsne %0, psp \n"
        : "=r"(__sp) :: );
    Fault_Capture(__sp);   /* 挖回栈帧 PC/LR, 写 RTC 备份, 不返回 */
  }
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  Fault_Capture_Auto();   /* 同故障勘查路径: PC/LR/故障状态落 RTC 备份, 不返回 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  Fault_Capture_Auto();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  Fault_Capture_Auto();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
  {
      __HAL_UART_CLEAR_IDLEFLAG(&huart2); 
      HAL_UART_DMAStop(&huart2);   
      UART2_RX_STA = RXbuffer2_size - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
      Uart2_task();
      UART2_RX_STA |= 0X8000; 
			memset(Rxbuffer2,0,RXbuffer2_size);
      RxRearm(&huart2, Rxbuffer2, RXbuffer2_size, 4u);
  }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
	{
	    __HAL_UART_CLEAR_IDLEFLAG(&huart3); 
	    HAL_UART_DMAStop(&huart3);   
	    UART3_RX_STA = RXbuffer3_size - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
		Uart3_task();
			UART3_RX_STA |= 0X8000; 
			memset(Rxbuffer3,0,RXbuffer3_size);
	    RxRearm(&huart3, Rxbuffer3, RXbuffer3_size, 2u);
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET)
  {
      __HAL_UART_CLEAR_IDLEFLAG(&huart4); 
      HAL_UART_DMAStop(&huart4);   
      UART4_RX_STA = RXbuffer4_size - __HAL_DMA_GET_COUNTER(huart4.hdmarx);  
      Uart4_task();
      UART4_RX_STA |= 0X8000; 
			memset(Rxbuffer4,0,RXbuffer4_size);
      RxRearm(&huart4, Rxbuffer4, RXbuffer4_size, 1u);
  }
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
  {
      __HAL_UART_CLEAR_IDLEFLAG(&huart6);
      /* TX 在途(Emm 位置帧 ~1.1ms)时 DMAStop 会连同停止 TX DMA → 线上出半帧,
       * 驱动器收不到完整指令且 Emm_tx_fail_cnt 观测不到 —— 此分支只终止 RX */
      if (huart6.gState == HAL_UART_STATE_BUSY_TX)
      {
          (void)HAL_DMA_Abort(huart6.hdmarx);
      }
      else
      {
          HAL_UART_DMAStop(&huart6);
      }
      UART6_RX_STA = RXbuffer6_size - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      Uart6_task();
      UART6_RX_STA |= 0X8000;
			memset(Rxbuffer6,0,RXbuffer6_size);
      RxRearm(&huart6, Rxbuffer6, RXbuffer6_size, 8u);
  }
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USART2_RX DMA (DMA1_Stream5, 详 usart.c MspInit): 与 Stream1~4 同款生成模板 */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}


/* UART 错误恢复: DMA 接收模式下 FE/NE/ORE/PE 会走 UART_EndRxTransfer 停收,
 * 不重装则 IDLE 中断从此不再产生, 该链路永久哑掉(比赛中无人工复位机会)。
 * 与 IDLE 分支同样整缓冲重装; HAL 在进本回调前已清错误标志, 直接重开即可 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    RxRearm(&huart4, Rxbuffer4, RXbuffer4_size, 1u);
  }
  else if (huart == &huart3)
  {
    RxRearm(&huart3, Rxbuffer3, RXbuffer3_size, 2u);
  }
  else if (huart == &huart2)
  {
    Dbg2_RxErr_Cnt++;   /* 调试口 RX 帧错/噪声/溢出: 持续增长=RX 线接触不良或受扰 */
    RxRearm(&huart2, Rxbuffer2, RXbuffer2_size, 4u);
  }
  else if (huart == &huart6)
  {
    RxRearm(&huart6, Rxbuffer6, RXbuffer6_size, 8u);
  }
}


/* 调试口逐字符轮询发送(不关中断 ⇒ 只拖主循环, 不拖 TIM4 控制拍)。
 * 返回值必须接上 Dbg2_TxFail_Cnt: LINK 行的 TF= 是"日志空档到底是发不出去、还是压根没
 * 生成"的唯一依据 —— 丢了它这个诊断口等于不存在(2026-09-29 修复)。 */
int fputc(int ch , FILE *f)
{
	if (HAL_UART_Transmit(&huart2 , (uint8_t *)&ch , 1 , 10) != HAL_OK)
	{
		Dbg2_TxFail_Cnt++;
	}
	return ch;
}


/* USER CODE END 1 */
