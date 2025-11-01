/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key2_Pin GPIO_PIN_13
#define key2_GPIO_Port GPIOC
#define key2_EXTI_IRQn EXTI15_10_IRQn
#define led_Pin GPIO_PIN_14   //led D5
#define led_GPIO_Port GPIOG
#define key1_Pin GPIO_PIN_0
#define key1_GPIO_Port GPIOE
#define led2_Pin GPIO_PIN_13  //led D2
#define led2_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

//#define ENTER_SLEEP_MODE        //cpu clock off  唤醒：中断或事件、reset
#define ENTER_STOP_MODE       //all clock off  唤醒：exti line、reset
//#define ENTER_STANDBY_MODE    //1.8V区域掉电    唤醒：wakeup pin（PA0）、RTC alarm、reset
#if defined(ENTER_SLEEP_MODE)||defined(ENTER_STOP_MODE)  //systick中断会唤醒系统
#define BLOCKING_DELAY    //use blocking delay other than systick delay
#endif

//#define IWDG_ENABLE
//#define WWDG_ENABLE
//#define RTC_ENABLE
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
