/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Encode.h"
#include "interurp.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	

  UART_Task_Init();
  {
    extern void Fault_Boot_Report(void);   /* stm32f4xx_it.c: 读 RTC 复位源 + 上次 HardFault 现场 */
    Fault_Boot_Report();
  }
  
	Yaw_pid_Init();
  Motor_Init();
	ConfigHWT101();       
  HAL_Delay(500);
  ResetHWT101();
	HAL_Delay(2000);
	{
		/* 转台上电零点对齐: 把驱动器"当前位置"清零(电机不动)并把 MCU 记账角对齐到同一
		 * 物理姿态 —— 修"一条命令转好几圈"(绝对定位 + 从不回零, MCU 与驱动器零点脱节)。
		 * 必须在 htim5 启动前调用: TIM5 一起就开始周期下发绝对位置了。
		 * 放在 2.5s 静置之后, 等张大头驱动器上电自检完成。 */
		extern void Car_Turntable_BootZero(void);   /* Control_Car.c */
		Car_Turntable_BootZero();
	}
	HAL_TIM_Base_Start_IT(&GAP_TIM);		
	HAL_TIM_Base_Start_IT(&htim5);

  /* 独立看门狗: LSI 32kHz/64 分频/重载 99 ≈ 198ms; Car_Control(TIM4 10ms) 末尾喂狗。
   * 控制环停摆(HardFault/跑飞/时钟失效)即复位, 防故障态履带保持最后占空比失控;
   * 放在 2s 静置之后启动, 避免 HAL_Delay 期间被复位。调试器挂起时冻结不喂狗复位。
   * 直接寄存器操作(HAL IWDG 模块未启用): prescaler=4×2^PR, PR=4 → /64 */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  IWDG->KR = 0xCCCCU;                                   /* 启动 */
  IWDG->KR = 0x5555U;                                   /* 解除 PR/RLR 写保护 */
  IWDG->PR = 0x4U;
  while ((IWDG->SR & IWDG_SR_PVU) != 0U) { }            /* 等分频生效 */
  IWDG->RLR = 99U;
  while ((IWDG->SR & IWDG_SR_RVU) != 0U) { }            /* 等重载值生效 */
  IWDG->KR = 0xAAAAU;                                   /* 首喂 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Car_DebugTask();                     /* USART2 调试口: 事件回显 + '?' 查询(唯一 printf 点) */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
