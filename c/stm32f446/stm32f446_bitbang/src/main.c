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
#include "dshot.h"
#include "dshot_bitbang.h"
#include "bb_uart.h"

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
uint32_t tick1,tick10,tick50,tick100,tick500,tick1000,tick2000,curr_tick;
/* USER CODE END 0 */
dshotProtocolControl_t pcb;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
#ifdef BB_UART
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
  bb_uart_init(bb_uart_txbuff,GPIO_PIN_5);
  bb_uart_data_set(bb_uart_txbuff,GPIO_PIN_5,'A');
  for(int i=0;i<10*SAMPLERATE;i++)
  {
    printf("%.8lx\r\n",bb_uart_txbuff[i]);
  }
#endif
  /* USER CODE END 2 */
#ifdef USE_DSHOT_BITBANG
  bbOutputDataInit(bbOutputBuffer,GPIO_PIN_5);
  pcb.requestTelemetry=false;
  pcb.value=48;//0x5dc
  uint16_t v=prepareDshotPacket(&pcb);
  bbOutputDataSet(bbOutputBuffer,GPIO_PIN_5,v);
  for(int i=0;i<16;i++){
    printf("%.8lx %.8lx %.8lx\r\n",bbOutputBuffer[0+i*3],bbOutputBuffer[1+i*3],bbOutputBuffer[2+i*3]);
    /*
      1500 -> bb88 -> 1011 1011 1000 1000
      00000020 00000000 00200000
      00000020 00200000 00200000
      00000020 00000000 00200000
      00000020 00000000 00200000
      00000020 00000000 00200000
      00000020 00200000 00200000
      00000020 00000000 00200000
      00000020 00000000 00200000
      00000020 00000000 00200000
      00000020 00200000 00200000
      00000020 00200000 00200000
      00000020 00200000 00200000
      00000020 00000000 00200000
      00000020 00200000 00200000
      00000020 00200000 00200000
      00000020 00200000 00200000
    */
  }
  #endif
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  tick1=tick10=tick50=tick100=tick500=tick1000=tick2000=curr_tick=HAL_GetTick();
  //HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,rxbuff,0xffff);
  hdma_tim1_ch1.XferCpltCallback=tim1_dma_callback;
  hdma_tim1_ch2.XferCpltCallback=tim1_ch2_dma_callback;
  __HAL_TIM_ENABLE(&htim1);
  while (1)
  {
    /* USER CODE END WHILE */
    if(curr_tick-tick1>=1){
      tick1=curr_tick;
#ifdef USE_DSHOT_BITBANG
       if(rx_flag)
      {
       rx_flag=false;
       pcb.value=atoi(rxbuff);//
        printf("uart get %d\r\n",pcb.value);
        uint16_t v=prepareDshotPacket(&pcb);
        bbOutputDataClear(bbOutputBuffer);
        bbOutputDataSet(bbOutputBuffer,GPIO_PIN_5,v);
      }
     // printf("tim sr before %x %x\r\n",TIM1->SR,TIM1->CCER);
     // __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1);
     // printf("tim sr after %x\r\n",TIM1->SR);
      //TIM1->EGR|=1;
      
       HAL_DMA_Start_IT(&hdma_tim1_ch1,(uint32_t)bbOutputBuffer,(uint32_t)&GPIOC->BSRR,48);
      __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
       //__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1);        
#endif
    }
    if(curr_tick-tick10>=10){
      tick10=curr_tick;
    }
    if(curr_tick-tick50>=50){
      tick50=curr_tick;
    }
    if(curr_tick-tick100>=100){
      tick100=curr_tick;
    }
    if(curr_tick-tick500>=500){
      tick500=curr_tick;
     // printf("test\r\n");
      HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);  //heartbeat
    }
    if(curr_tick-tick1000>=1000){
      tick1000=curr_tick;
#ifdef BB_UART
      if(rx_flag)
      {
        rx_flag=false;
        bb_uart_data_clear(bb_uart_txbuff,GPIO_PIN_5);
        bb_uart_data_set(bb_uart_txbuff,GPIO_PIN_5,rxbuff[0]);
      }

      extern uint8_t rx[10],rcnt;
      if(rcnt){
        for(int i=0;i<rcnt;i++){
            bb_uart_data_clear(bb_uart_txbuff,GPIO_PIN_5);
            bb_uart_data_set(bb_uart_txbuff,GPIO_PIN_5,rx[i]);
            bb_uart_tx();
            HAL_Delay(1);
        }
        rcnt=0;
      }
       
#endif
    }
    if(curr_tick-tick2000>=2000){
      tick2000=curr_tick;
    }
    curr_tick=HAL_GetTick();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
