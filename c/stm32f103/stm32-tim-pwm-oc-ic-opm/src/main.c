/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

 /**
 * @file main.c
 * 
 * 这里pwm dma中断和定时器更新中断不能同时打开，很奇怪还没有找到原因。
 * 目标：理解并实现定时器的基础定时、输入捕获、输出比较、PWM、单脉冲输出以及中断的使用
 * 中断：更新、触发、输入捕获、输出比较
 * 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include<stdio.h>
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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


uint32_t pre_tick1=0;
uint32_t pre_tick100=0;
uint32_t pre_tick500=0;
uint32_t curr_ticks=0;
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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef PWM_MODE
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
#endif
#ifdef OC_MODE
  HAL_TIM_OC_Start(&htim4,TIM_CHANNEL_2);
#endif
#ifdef IC_MODE  
#ifdef PWM_INPUT
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
#else
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);  
#endif
#endif
#ifdef ONE_PULSE_MODE
  HAL_TIM_OnePulse_Start(&htim4,TIM_CHANNEL_2);
#endif
    
    //__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,5);
    // LL_TIM_GenerateEvent_UPDATE(TIM4); //apply new compare value immediately，below same
    //    TIM4->EGR = TIM_EGR_UG;   
    pre_tick1=HAL_GetTick();
    pre_tick100=pre_tick1;
    pre_tick500=pre_tick1;
    curr_ticks=pre_tick500;
    int8_t i=1;
  while (1)
  {
    /* USER CODE END WHILE */
#if defined(PWM_MODE)||defined(OC_MODE)
    if(curr_ticks-pre_tick1>=1){
      pre_tick1=curr_ticks;
      t4pwm_value+=i;
      if(t4pwm_value>=1000||t4pwm_value<=0)
      {
        i=-i;
      }
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,t4pwm_value);
    }
#endif
    if(curr_ticks-pre_tick100>=100){
        pre_tick100=curr_ticks;
    }
    if(curr_ticks-pre_tick500>=500){
      pre_tick500=curr_ticks;
      HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
       //HAL_TIM_OC_Start(&htim4,TIM_CHANNEL_2);  //内部会检查通道的状态HAL_TIM_CHANNEL_STATE_READY，start后就是busy，
#ifdef ONE_PULSE_MODE                                                  //在stop、init或者dma回调函数中才会设置ready。
      __HAL_TIM_ENABLE(&htim4);     //test opm
#endif
#ifdef IC_MODE
      #ifdef PWM_INPUT
      printf("ic get period %d %d\r\n",HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1),t4period);
      printf("ic get pwmval %d %d\r\n",HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2),t4pwm_value);
      #else
      printf("ic get value %d\r\n",t4ic_value);
      #endif
#endif
    }
    curr_ticks=HAL_GetTick();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
