/**
  ******************************************************************************
  * @file    parameters_conversion_f4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the Parameter conversion on the base
  *          of stdlib F4xx for the first drive
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F4XX_H
#define __PARAMETERS_CONVERSION_F4XX_H

#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"
#include "drive_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/

#define SYSCLK_FREQ      168000000uL
#define TIM_CLOCK_DIVIDER  1
#define TIMAUX_CLOCK_DIVIDER (TIM_CLOCK_DIVIDER)
#define ADV_TIM_CLK_MHz  168/TIM_CLOCK_DIVIDER
#define ADC_CLK_MHz     21
#define HALL_TIM_CLK    84000000uL

#define ADC1_2  ADC1

/*************************  IRQ Handler Mapping  *********************/
#define TIMx_UP_M1_IRQHandler TIM1_UP_TIM10_IRQHandler
#define DMAx_R1_M1_IRQHandler DMA2_Stream4_IRQHandler
#define DMAx_R1_M1_Stream     DMA2_Stream4
#define TIMx_BRK_M1_IRQHandler TIM1_BRK_TIM9_IRQHandler

/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
/* Defined here for legacy purposes */
#define R1_PWM_AUX_TIM                  TIM4

/*************************  ADC Physical characteristics  ************/
#define ADC_TRIG_CONV_LATENCY_CYCLES 3
#define ADC_SAR_CYCLES 12

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

#endif /*__PARAMETERS_CONVERSION_F4XX_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
