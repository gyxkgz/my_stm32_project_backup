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
/* USER CODE END Header */
/*
tips:
中断与事件的比较
从外部激励信号来看,中断和事件的产生源都可以是一样的.之所以分成2个部分,由于中断是需要CPU参与的,
需要软件的中断服务函数才能完成中断后产生的结果;

但是事件,是靠脉冲发生器产生一个脉冲,进而由硬件自动完成这个事件产生的结果,当然相应的联动部件需要\
先设置好,比如引起DMA操作,AD转换等;

简单举例：外部I/O触发AD转换,来测量外部物品的重量;如果使用传统的中断通道,需要I/O触发产生外部中断,
外部中断服务程序启动AD转换,AD转换完成中断服务程序提交最后结果;要是使用事件通道,I/O触发产生事件,
然后联动触发AD转换,AD转换完成中断服务程序提交最后结果;相比之下,后者不要软件参与AD触发,并且响应速\
度也更块;要是使用事件触发DMA操作,就完全不用软件参与就可以完成某些联动任务了。

总结
可以这样简单的认为,事件机制提供了一个完全有硬件自动完成的触发到产生结果的通道,不要软件的参与,降低\
了CPU的负荷,节省了中断资源，提高了响应速度(硬件总快于软件)，是利用硬件来提升CPU芯片处理事件能力的
一个有效方法;

睡眠
HAL_PWR_EnterSLEEPMode(NULL,PWR_SLEEPENTRY_WFE);PWR_SLEEPENTRY_WFI
进入睡眠，只有cpu停止工作。
WFI (Wait For Interrupt) or WFE (Wait for Event)

由于一般情况下会打开systick中断，所以系统会1ms启动一次，感觉好像没有睡眠，实际上睡眠了。关掉systick
中断就可以验证。

外部中断或事件可以唤醒cpu。

停止模式
HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_STOPENTRY_WFI);
该模式进一步降低功耗，但是唤醒时间变长了。唤醒时间有点奇怪，比理论长很多，还没搞清楚原因。
另外唤醒后时钟变成了hsi rc，如果之前是hse的话，系统整体频率会下降，所以需要重新配置一下clock

待机模式
HAL_PWR_EnterSTANDBYMode();
最省电的模式，系统几乎停滞，除了rtc、iwdg等等。唤醒后类似复位，从最开始执行程序。
使用wakeup引脚，需要清除wakeup标志，不然无法再次进入待机模式。
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "rtc.h"
#include "iwdg.h"
#include "wwdg.h"

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
#ifdef BLOCKING_DELAY
void HAL_Delay( uint32_t ms)   //72Mhz时钟，大概1ms延时，不是很准确
{
  for ( uint32_t j=0;j<ms;j++)
    for (volatile uint32_t i=0;i<4000;i++){  //如果函数体没东西这个volatile必不可少，如果函数体有东西，这里没有volatile感觉时间变快了
      __asm__ volatile ("nop"); 
    }; 
}
#endif
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
#ifdef BLOCKING_DELAY
  //关闭systick必须在SystemClock_Config之后，因为这个里边也初始化了systick。
   SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
  // SysTick->CTRL = 0;           // 禁用SysTick
  //HAL_SuspendTick();   
#endif   
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //quick blink indicate reset
  HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
   HAL_Delay(50);
  HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
  HAL_Delay(50);
  HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
  HAL_Delay(50);
  HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
  HAL_Delay(50);
  //end blink

#ifdef RTC_ENABLE
  MX_RTC_Init();
#endif
#ifdef IWDG_ENABLE
  MX_IWDG_Init();
#endif
#ifdef WWDG_ENABLE
  MX_WWDG_Init();
#endif
  

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //
  //
//   if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB))
//   {
//        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
//   }
  //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#ifdef ENTER_STANDBY_MODE
   HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
#endif
  uint32_t pre_tick_10=HAL_GetTick();
  uint32_t pre_tick_20=pre_tick_10;
  uint32_t pre_tick_100=pre_tick_10;
  uint32_t pre_tick_500=pre_tick_100;
  uint32_t pre_tick_1000=pre_tick_100;
  uint32_t pre_tick_2000=pre_tick_100;
  uint32_t curr_tick=pre_tick_100;
  while (1)
  {
    if(curr_tick-pre_tick_10>=10) //100Hz
    {
        pre_tick_10=curr_tick;
#ifdef WWDG_ENABLE
        //HAL_WWDG_Refresh(&hwwdg);  //too early for wwdg
#endif
    }
    if(curr_tick-pre_tick_20>=20) //50Hz
    {
        pre_tick_20=curr_tick;
#ifdef WWDG_ENABLE
        HAL_WWDG_Refresh(&hwwdg);  //wwdg windows 15.5ms-24.5ms
#endif
    }
    if(curr_tick-pre_tick_100>=100) //10Hz
    {
#ifdef WWDG_ENABLE
   //     HAL_WWDG_Refresh(&hwwdg);     //too late for wwdg
#endif
        pre_tick_100=curr_tick;
    }
    if(curr_tick-pre_tick_500>=500) //2Hz
    {
        pre_tick_500=curr_tick;
        HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
    }
    if(curr_tick-pre_tick_1000>=1000) //1Hz
    {
        pre_tick_1000=curr_tick;
    }   
    if(curr_tick-pre_tick_2000>=2000) //0.5Hz
    {
        pre_tick_2000=curr_tick;
#ifdef IWDG_ENABLE
        HAL_IWDG_Refresh(&hiwdg);
#endif
    }   
    //press key1 enter low power mode
    if( GPIO_PIN_RESET == HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)){
       HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
#ifdef ENTER_SLEEP_MODE
      //HAL_PWR_EnterSLEEPMode(0,PWR_SLEEPENTRY_WFE);
      HAL_PWR_EnterSLEEPMode(0,PWR_SLEEPENTRY_WFI);
#endif
#ifdef ENTER_STOP_MODE
      //HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_STOPENTRY_WFI);
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
      //退出后会自动选择HSI，如果使用HSE需要重新配置时钟
       SystemClock_Config();
       SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;  //同样得关掉systick，不然会一直被systick唤醒
#endif
#ifdef ENTER_STANDBY_MODE
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
      HAL_PWR_EnterSTANDBYMode();
#endif

    }
#ifdef BLOCKING_DELAY
    curr_tick++;
    HAL_Delay(1);
#else
    curr_tick=HAL_GetTick();
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
