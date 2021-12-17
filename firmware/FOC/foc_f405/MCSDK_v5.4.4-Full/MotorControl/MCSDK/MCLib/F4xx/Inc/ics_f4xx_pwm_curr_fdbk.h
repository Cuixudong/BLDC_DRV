/**
******************************************************************************
* @file    ics_f4xx_pwm_curr_fdbk.h
* @author  Motor Control SDK Team, ST Microelectronics
* @brief   This file contains all definitions and functions prototypes for the
*          ICS PWM current feedback component for F4xx of the Motor Control SDK.
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
  * @ingroup ics_f4xx_pwm_curr_fdbk
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_F4XX_PWMNCURRFDBK_H
#define __ICS_F4XX_PWMNCURRFDBK_H

#include "pwm_curr_fdbk.h"
#include "ics_dd_pwmncurrfdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup ics_f4xx_pwm_curr_fdbk
* @{
*/

#define SOFOC 0x0008u /**< This flag is reset at the beginning of FOC
                           and it is set in the TIM UP IRQ. If at the end of
                           FOC this flag is set, it means that FOC rate is too 
                           high and thus an error is generated */

/**
* @brief  PWMnCurrFdbk ICS F4xx handle
*/
typedef struct
{
  PWMC_Handle_t _Super;     /**< PWMC handler */
  uint32_t PhaseAOffset;   /**< Offset of Phase A current sensing network  */
  uint32_t PhaseBOffset;   /**< Offset of Phase B current sensing network  */
  uint16_t Half_PWMPeriod;     /**< Half PWM Period in timer clock counts */
  uint8_t  PolarizationCounter;
  uint32_t ADCTriggerSet;  /**< Store the value for ADC CR2 to proper configure
                                 current sampling during the context switching*/
  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  ICS_Params_t const *pParams_str;
} PWMC_ICS_Handle_t;

/* Exported functions ------------------------------------------------------- */

void ICS_Init( PWMC_ICS_Handle_t * pHandle );

void ICS_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

void ICS_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * pStator_Currents );

void ICS_TurnOnLowSides( PWMC_Handle_t * pHandle );

void ICS_SwitchOnPWM( PWMC_Handle_t * pHandle );

void ICS_SwitchOffPWM( PWMC_Handle_t * pHandle );

uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHandle );

uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHandle );

void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle );

void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHdl );


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ICS_F4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
