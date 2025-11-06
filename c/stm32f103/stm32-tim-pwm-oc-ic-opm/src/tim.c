/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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

  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim4;
//1kHz;  
#ifdef USE_ETR
uint16_t prescaler=1;  //ETR2 2M;    fCK_PSC / (PSC[15:0] + 1)
#else
uint16_t prescaler=71; //interal clock 36M
#endif
#ifdef IC_MODE
uint16_t t4period=65535;    //输入捕获模式，周期开最大
#else
uint16_t t4period=999;
#endif

uint16_t t4pwm_value=0;
uint16_t t4ic_value=0;
/* TIM4 init function */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
   TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = prescaler;                     //分频系数d-1（2分频：1，4分频：3）here we want get 1Mhz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = t4period;                      //1ms
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    //数字滤波采样时钟分频
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;//TIM_AUTORELOAD_PRELOAD_DISABLE;  //不开启的话，ARR的新值会立即生效，开启后ARR的新值会在下一个更新事件后生效
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  #ifdef USE_ETR //PE0 as clock 
  #ifdef USE_ETR2
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;   
  #else
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
  #endif
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  #else
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;    //驱动定时器计数的来源
  #endif
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #ifdef ONE_PULSE_MODE
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  t4pwm_value=500;
  #endif
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = t4pwm_value;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
#if defined(PWM_MODE)||defined(ONE_PULSE_MODE)   //会设置TIMx_CCR1的preload
  sConfigOC.OCMode = TIM_OCMODE_PWM1;     //PWM1：CCR大于CNT输出OCPolarity，PWM2：CCR小于CNT输出OCPolarity
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
#endif
#ifdef OC_MODE
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE; 
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
#endif


  /* USER CODE BEGIN TIM4_Init 2 */
#ifdef IC_MODE
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
#ifdef PWM_INPUT
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro_IT(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
#endif
#endif
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
#ifdef USE_ETR
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PE0     ------> TIM4_ETR
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
#endif
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM4 GPIO Configuration
      *PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    __HAL_AFIO_REMAP_TIM4_ENABLE();

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);

    /* TIM4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  printf("update callback\r\n");
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  //printf("ic callback\r\n");
  static uint32_t pre;
    uint32_t time = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
    if(htim->Instance->CCER&TIM_CCER_CC1P)
    {
      if(time>=pre)
      {

        t4ic_value =  time-pre;
      }
      else
      {
          t4ic_value = time+1000-pre;
      }

        __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
    }
    else
    {
      pre=time;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
  printf("oc callback\r\n");
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  printf("pwm callback\r\n");
}
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim){
 // printf("trigger callback\r\n");
  t4pwm_value=__HAL_TIM_GetCompare(htim,TIM_CHANNEL_2);
  t4period=__HAL_TIM_GetCompare(htim,TIM_CHANNEL_1);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
