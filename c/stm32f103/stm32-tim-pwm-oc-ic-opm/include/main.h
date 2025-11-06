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
#include "stm32f1xx_ll_tim.h"
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
#define pwm1_Pin GPIO_PIN_9
#define pwm1_GPIO_Port GPIOE
#define pwm2_Pin GPIO_PIN_11
#define pwm2_GPIO_Port GPIOE
#define pwm3_Pin GPIO_PIN_13
#define pwm3_GPIO_Port GPIOE
#define pwm4_Pin GPIO_PIN_14
#define pwm4_GPIO_Port GPIOE
#define led_Pin GPIO_PIN_14
#define led_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */
//#define USE_ETR2
//#define USE_ETR1
#if defined(USE_ETR2)||defined(USE_ETR1)
#define USE_ETR
#endif
//#define PWM_MODE    //PD13 as output
//#define OC_MODE
//#define IC_MODE  //PD12 as input 得到的值通过printf输出，并且作为pwm的输入
//#define PWM_INPUT
#ifdef PWM_INPUT
#define IC_MODE
#endif
#define ONE_PULSE_MODE  //单脉冲模式需要配合PWM/OC模式，本质上就是在溢出的时候停止计数器（CEN=0），
                        //任何让CEN=1的方式都可以触发它。

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
