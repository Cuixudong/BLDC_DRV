/**
  ******************************************************************************
  * @file    r3_2_f4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the three shunts current sensing topology.
  *           It is specifically designed for STM32F1xx microcontrollers and
  *          implements the successive sampling of current using two ADCs.
  *           + MCU peripheral and handle initialization function
  *           + three shunt current sensing
  *           + space vector modulation function
  *           + ADC sampling function
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
#include "r3_2_f4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r3_2_f4xx_pwm_curr_fdbk PWM & Current Feedback
 *
 * @brief STM32F4, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F4 MCU
 * and using a three shunt resistors current sensing topology.
 *
 * @todo: TODO: complete documentation.
  * @{
  */

/* Private defines -----------------------------------------------------------*/

#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE uint16_t R3_2_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg );
void R3_2_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
void R3_2_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void R3_2_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void R3_2_RLTurnOnLowSides( PWMC_Handle_t * pHdl );
static void R3_2_RLSwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * @brief  It initializes TIM, ADC, GPIO, DMA and NVIC for current reading and
  *         PWM generation in three shunt configuration using STM32F4XX
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_2_Init( PWMC_R3_2_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  LL_TIM_DisableCounter( TIMx );

  /* BKIN, if enabled */
  if ( (pHandle->pParams_str->EmergencyStop) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK(TIMx);
    LL_TIM_EnableIT_BRK(TIMx);
  }

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
  if ( pHandle->pParams_str->bFreqRatio == 2u )
  {
    if ( pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->RepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1u );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );
      }
    }

    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  }
  else /* FreqRatio equal to 1 or 3 */
  {
    if ( pHandle->_Super.Motor == M1 )
    {
      LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
    }
  }

  /* Enable PWM channel */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  if ( TIMx == TIM1 )
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
    pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
    pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM8_CH4;
  }

  /* ADCs registers configuration ---------------------------------*/
  /* Enable ADCx_1 and ADCx_2 */
  LL_ADC_Enable( ADCx_1 );
  LL_ADC_Enable( ADCx_2 );

  /* ADCx_1 Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(ADCx_1);
  LL_ADC_EnableIT_JEOS( ADCx_1 );

  /* reset regular conversion sequencer length set by cubeMX */
  LL_ADC_REG_SetSequencerLength( ADCx_1, LL_ADC_REG_SEQ_SCAN_DISABLE );
  LL_ADC_INJ_SetSequencerLength( ADCx_1, LL_ADC_INJ_SEQ_SCAN_DISABLE );
  LL_ADC_INJ_SetSequencerLength( ADCx_2, LL_ADC_INJ_SEQ_SCAN_DISABLE );

  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;

  pHandle->_Super.DTTest = 0u;

}

/**
  * @brief  It stores into the component's handle the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor
  * @param  pHdl handler of the current instance of the PWM component
  */
__weak void R3_2_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->PhaseAOffset = 0u;
  pHandle->PhaseBOffset = 0u;
  pHandle->PhaseCOffset = 0u;

  pHandle->PolarizationCounter = 0u;

  LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_HFCurrentsCalibrationAB;
  pHandle->_Super.pFctSetADCSampPointSectX = &R3_2_SetADCSampPointCalibration;

  pHandle->CalibSector = SECTOR_4;
  /* Required to force first polarization conversion on SECTOR_4*/
  pHandle->_Super.Sector = SECTOR_4;   

  R3_2_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd( TIMx,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->PolarizationCounter );

  R3_2_SwitchOffPWM( &pHandle->_Super );
  /* Offset calibration for C phase */
  /* Reset PolarizationCounter */
  pHandle->PolarizationCounter = 0u;

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_HFCurrentsCalibrationC;

  pHandle->CalibSector = SECTOR_1;
  /* Required to force first polarization conversion on SECTOR_1*/
  pHandle->_Super.Sector = SECTOR_1;   
  R3_2_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd( TIMx,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->PolarizationCounter );

  R3_2_SwitchOffPWM( &pHandle->_Super );

  pHandle->PhaseAOffset >>= 3;
  pHandle->PhaseBOffset >>= 3;
  pHandle->PhaseCOffset >>= 3;

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSectX = &R3_2_SetADCSampPointSectX;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverter legs */
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH3 );

  LL_TIM_OC_SetCompareCH1(TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH2(TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH3(TIMx,pHandle->Half_PWMPeriod);

  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );

  /* sector and phase sequence for the switch on phase */
  pHandle->_Super.Sector = SECTOR_4;

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);
}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in ab_t format
  */
__weak void R3_2_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t* pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;
  uint8_t bSector;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  bSector = pHandle->_Super.Sector;
  hReg1 = *pHandle->pParams_str->ADCDataReg1[bSector] * 2;
  hReg2 = *pHandle->pParams_str->ADCDataReg2[bSector] * 2;

  switch ( bSector )
  {
    case SECTOR_4:
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( hReg1 );
      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->a = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->a = INT16_MAX;
      }
      else
      {
        pStator_Currents->a = ( int16_t )wAux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
        wAux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( hReg2 );

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->b = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->b = INT16_MAX;
      }
      else
      {
        pStator_Currents->b = ( int16_t )wAux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( hReg1 );
      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->b = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->b = INT16_MAX;
      }
      else
      {
        pStator_Currents->b = ( int16_t )wAux;
      }

      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ia = -Ic -Ib */
      wAux = ( int32_t )( pHandle->PhaseCOffset ) - ( int32_t )( hReg2 );
      wAux = -wAux - ( int32_t )pStator_Currents->b;

      /* Saturation of Ia */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->a = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->a = -INT16_MAX;
      }
      else
      {
        pStator_Currents->a = ( int16_t )wAux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( hReg1 );
      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->a = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->a = INT16_MAX;
      }
      else
      {
        pStator_Currents->a = ( int16_t )wAux;
      }

      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ib = -Ic -Ia */
      wAux = ( int32_t )( pHandle->PhaseCOffset ) - ( int32_t )( hReg2 );
      wAux = -wAux -  ( int32_t )pStator_Currents->a;

      /* Saturation of Ib */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->b = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->b = -INT16_MAX;
      }
      else
      {
        pStator_Currents->b = ( int16_t )wAux;
      }
      break;

    default:
      break;
  }

  pHandle->_Super.Ia = pStator_Currents->a;
  pHandle->_Super.Ib = pStator_Currents->b;
  pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into PhaseAOffset and
  *         PhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC inputs before to enable
  *         the offset computation.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in ab_t format
  */
__weak void R3_2_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseAOffset += *pHandle->pParams_str->ADCDataReg1[pHandle->CalibSector];
    pHandle->PhaseBOffset += *pHandle->pParams_str->ADCDataReg2[pHandle->CalibSector];
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into PhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC input before to enable
  *         the offset computation.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in ab_t format
  */
__weak void R3_2_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;  

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseCOffset += *pHandle->pParams_str->ADCDataReg2[pHandle->CalibSector];
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_2_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,0);
  LL_TIM_OC_SetCompareCH2(TIMx,0);
  LL_TIM_OC_SetCompareCH3(TIMx,0);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET)
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return;
}

/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_2_SwitchOnPWM( PWMC_Handle_t * pHdl )
{  
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 5u));

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( LL_TIM_CC_IsEnabledChannel(TIMx,TIMxCCER_MASK_CH123) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE(TIMx);

  return;
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_2_SwitchOffPWM( PWMC_Handle_t * pHdl )
{ 
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIMx );

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIMx );
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  
  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);

  return;
}

/**
  * @brief  writes into peripheral registers the new duty cycles and
  *        sampling point
  * @param  pHdl: handler of the current instance of the PWM component
  * @param hCCR4Reg: new capture/compare register value.
  * @retval Error status
  */
__STATIC_INLINE uint16_t R3_2_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint16_t hAux;

  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->_Super.CntPhC );
  LL_TIM_OC_SetCompareCH4( TIMx, hCCR4Reg );

  /* Limit for update event */
  /* Check the if TIMx CH4 is enabled. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if (LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

/**
 * @brief  Configure the ADC for the current sampling during calibration.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param pHdl: handler of the current instance of the PWM component
 * @retval none
 */
__weak uint16_t R3_2_SetADCSampPointCalibration( PWMC_Handle_t * pHdl)
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;

  /* Set rising edge trigger (default) */
  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;
  pHandle->_Super.Sector = pHandle->CalibSector;

  return R3_2_WriteTIMRegisters( &pHandle->_Super, (uint32_t)(pHandle->Half_PWMPeriod - 1u) );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 1.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak uint16_t R3_2_SetADCSampPointSectX( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;

  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  register uint16_t lowDuty = pHdl->lowDuty;
  register uint16_t midDuty = pHdl->midDuty;

  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - lowDuty ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled wIch corresponds
     * to sector 4 */
    pHandle->_Super.Sector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;
  }
  else
  {
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases
    with variable complementary duty and with maximum duty are converted and the first will be always
    the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( lowDuty - midDuty );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - lowDuty ) * 2u )
    {
      /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC */
      hCntSmp = lowDuty - pHandle->pParams_str->hTbefore;
    }
    else
    {
      /* hTafter = DT + max(Trise, Tnoise) */
      hCntSmp = lowDuty + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
          /* It must be changed the trigger direction from positive to negative
               to sample after middle of PWM*/
        pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  }

  return R3_2_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void * R3_2_TIMx_UP_IRQHandler( PWMC_R3_2_Handle_t * pHandle)
{
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;
  uint32_t ADCInjFlags;
  
  /* dual drive check */
  ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
  if ( ADCInjFlags == LL_ADC_FLAG_JSTRT )
  {
    /* ADC conversion is on going on the second motor */
    do
    {
      /* wait for end of conversion */
      ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
    } 
    while ( ADCInjFlags != (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS) );
  }
  else if ( ADCInjFlags == 0 )
  {
    /* ADC conversion on the second motor is not yet started */
    while ( ( TIMx->CNT ) < ( pHandle->pParams_str->Tw ) )
    {
      /* wait for a maximum delay */
    }
    
    ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
    if ( ADCInjFlags == LL_ADC_FLAG_JSTRT )
    {
      /* ADC conversion is on going on the second motor */
      do
      {
        /* wait for end of conversion */
        ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
      }
      while ( ADCInjFlags != (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS) );
    }
  }
  else
  {
    /* ADC conversion on the second motor is done */
  }
  
  /* Disabling trigger to avoid unwanted conversion */
  LL_ADC_INJ_StopConversionExtTrig(ADCx_1);
  LL_ADC_INJ_StopConversionExtTrig(ADCx_2);

  /* Set next current channel according to sector  */
  ADCx_1->JSQR = pHandle->pParams_str->ADCConfig1[pHandle->_Super.Sector];
  ADCx_2->JSQR = pHandle->pParams_str->ADCConfig2[pHandle->_Super.Sector];

  LL_ADC_INJ_SetTriggerSource(ADCx_1,pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource(ADCx_2,pHandle->ADC_ExternalTriggerInjected);
    
  /* enable ADC trigger source */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1, pHandle->ADCTriggerEdge);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2, pHandle->ADCTriggerEdge);

  /* reset default edge detection trigger */
  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;

  return &( pHandle->_Super.Motor );
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void *R3_2_BRK_IRQHandler(PWMC_R3_2_Handle_t *pHandle)
{

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  pHandle->OverCurrentFlag = true;

  return &(pHandle->_Super.Motor);
}

/**
  * @brief  It is used to check if an over current occurred since last call.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an over current has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
__weak uint16_t R3_2_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  uint16_t retVal = MC_NO_FAULTS;
  if (pHandle->OverCurrentFlag == true )
  {
    retVal = MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retVal;
}

/**
  * @brief  It is used to set the PWM mode for R/L detection.
  * @param  pHandle: handler of the current instance of the PWM component
  * @param  hDuty to be applied in uint16_t
  * @retval none
  */
void R3_2_RLDetectionModeEnable( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == false )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    LL_TIM_OC_SetCompareCH1( TIMx, 0u );

    /*  Channel2 configuration */
    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );

  }

  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_2_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_2_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_2_SwitchOffPWM;

  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  It is used to disable the PWM mode in 6-step.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_RLDetectionModeDisable( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == true )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH1( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH2( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3 );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH3( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_2_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_2_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_2_SwitchOffPWM;

    pHandle->_Super.RLDetectionMode = false;
  }
}

/**
  * @brief  It is used to set the PWM dutycycle during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @param  hDuty: duty cycle to be applied in uint16_t
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t R3_2_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  uint32_t val;
  uint16_t hAux;

  val = ( ( uint32_t )( pHandle->Half_PWMPeriod ) * ( uint32_t )( hDuty ) ) >> 16;
  pHandle->_Super.CntPhA = ( uint16_t )( val );

  /* TIM1 Channel 1 Duty Cycle configuration.
   * In RL Detection mode only the Up-side device of Phase A are controlled*/
  LL_TIM_OC_SetCompareCH1(TIMx, ( uint32_t )pHandle->_Super.CntPhA);

  /* set the sector that correspond to Phase A and B sampling */
  pHdl->Sector = SECTOR_4;

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  *         during RL detection phase
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in ab_t format
  */
void R3_2_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  int32_t wAux;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  wAux = (int32_t)( pHandle->PhaseBOffset ) - (int32_t)*pHandle->pParams_str->ADCDataReg2[pHandle->_Super.Sector]*2;

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  pStator_Currents->a = (int16_t)wAux;
  pStator_Currents->b = (int16_t)wAux;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_2_RLTurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1 ( TIMx, 0u );

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  return;
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral
  *         This function is specific for RL detection phase.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_2_RLSwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;


  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1( TIMx, 1u );
  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod ) - 5u );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );
  

  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }
  
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* enable TIMx update interrupt*/
  LL_TIM_EnableIT_UPDATE( TIMx );

  return;
}

/**
 * @brief  It turns on low sides switches and start ADC triggering.
 *         This function is specific for MP phase.
 * @param  pHandle Pointer on the target component instance
 * @retval none
 */
void RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH2 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH3 ( TIMx, 0x0u );

  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod - 5u));

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  pHdl->Sector = SECTOR_4;
  LL_ADC_INJ_SetTriggerSource( ADCx_1, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource( ADCx_2, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1,LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2,LL_ADC_INJ_TRIG_EXT_RISING);

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  LL_TIM_EnableIT_UPDATE( TIMx );

  return;
}


/**
 * @brief  It sets ADC sampling points.
 *         This function is specific for MP phase.
 * @param  pHandle Pointer on the target component instance
 * @retval none
 */
void RLSetADCSampPoint( PWMC_Handle_t * pHdl )
{
  /* dummy sector setting to get correct Ia value */
  pHdl->Sector = SECTOR_4;

  return;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
