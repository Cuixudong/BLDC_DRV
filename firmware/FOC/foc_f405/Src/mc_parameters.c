
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"
#include "r3_2_f4xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
  .Tw                       = MAX_TWAIT,
  .bFreqRatio               = FREQ_RATIO,
  .bIsHigherFreqTim         = FREQ_RELATION,

/* Current reading A/D Conversions initialization ----------------------------*/
  .ADCx_1                  = ADC1,
  .ADCx_2                  = ADC2,

/* PWM generation parameters --------------------------------------------------*/
  .TIMx                       =	TIM1,
  .RepetitionCounter         =	REP_COUNTER,
  .hTafter                    =	TW_AFTER,
  .hTbefore                   =	TW_BEFORE,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs             =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,

  .ADCConfig1 = {   MC_ADC_CHANNEL_9<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_8<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_8<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_8<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_8<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_9<<ADC_JSQR_JSQ4_Pos
                  },

  .ADCConfig2 = {   MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_9<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_9<<ADC_JSQR_JSQ4_Pos
                   ,MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ4_Pos
                  },

  .ADCDataReg1 = {
                   &ADC1->JDR1,
                   &ADC1->JDR1,
                   &ADC1->JDR1,
                   &ADC1->JDR1,
                   &ADC1->JDR1,
                   &ADC1->JDR1,
  },
  .ADCDataReg2 = {
                   &ADC2->JDR1,
                   &ADC2->JDR1,
                   &ADC2->JDR1,
                   &ADC2->JDR1,
                   &ADC2->JDR1,
                   &ADC2->JDR1,
  },

/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                =	(FunctionalState) ENABLE,
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
