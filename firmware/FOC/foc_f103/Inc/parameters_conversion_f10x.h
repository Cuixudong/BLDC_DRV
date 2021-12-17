
/**
  ******************************************************************************
  * @file    parameters_conversion_f10x.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F1 Family.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F10X_H
#define __PARAMETERS_CONVERSION_F10X_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "mc_math.h"

#define SYSCLK_FREQ    72000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz    72
#define ADC_CLK_MHz        12
#define HALL_TIM_CLK       72000000uL

#define ADC1_2 ADC1

/*************************  IRQ Handler Mapping  *********************/

#define TIMx_UP_M1_IRQHandler TIM1_UP_IRQHandler
#define DMAx_R1_M1_IRQHandler DMA1_Channel4_IRQHandler
#define TIMx_BRK_M1_IRQHandler TIM1_BRK_IRQHandler

/*******************  ADC Physical characteristics  ************/
#define ADC_TRIG_CONV_LATENCY_CYCLES 3
#define ADC_SAR_CYCLES 13

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

#endif /*__PARAMETERS_CONVERSION_F10X_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
