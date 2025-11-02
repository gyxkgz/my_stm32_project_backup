/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "rtc_bkp.h"

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
//复位后拉低usb引脚让电脑重新枚举，这样不需要断电。
void soft_reconnect_usb(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_Delay(200);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
#define APPLICATION_ADDRESS_START      USBD_DFU_APP_DEFAULT_ADD

typedef void (*pFunction)(void) ;//__attribute__((noreturn));
pFunction JumpToApplication;
uint32_t JumpAddresss;
void jump_to_APP(void)
{
   //printf("Jump to APP1\r\n");
	if (((*(__IO uint32_t*)APPLICATION_ADDRESS_START) & 0x2FFE0000 ) == 0x20000000)   //这里之后研究下为啥，貌似固件开头是0x20010000
	{
    //__disable_irq();
		JumpAddresss = *(__IO uint32_t*)(APPLICATION_ADDRESS_START+4);
   // printf("add %p\r\n",JumpAddresss);
		JumpToApplication = (pFunction)JumpAddresss;
		__set_MSP(*(__IO uint32_t*)APPLICATION_ADDRESS_START);
		JumpToApplication();
	}
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t magic_value=0;
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
  soft_reconnect_usb();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
   // uint8_t sftrst= __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)>>RCC_CSR_SFTRSTF_Pos;
    uint8_t sftrst= __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)&&RCC_CSR_SFTRSTF_Msk;
   __HAL_RCC_CLEAR_RESET_FLAGS();
  if(sftrst){
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKP_CLK_ENABLE() ;
    magic_value|=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);
    magic_value|=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR2)<<16;
    printf("%lx\r\n",magic_value);
    if(magic_value==MAGIC_VALUE)
    {
      uint8_t tmp=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR3);
      HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR3,0);
      if(tmp!=RESET_TO_DFU){
            jump_to_APP();
      }
    }
  }else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))
  {
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))
    {
      printf("jump to APP\n");
      jump_to_APP();
    }
  }

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 

  MX_USB_DEVICE_Init();
  printf(" dfu upgrade\n\r");
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_14);
    HAL_Delay(100);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
