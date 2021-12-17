/**
  ******************************************************************************
  * @file    ics_f4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the ICS
  *          PWM current feedback component for F4xx of the Motor Control SDK.
  ********************************************************************************
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
#include "ics_f4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup ics_f4xx_pwm_curr_fdbk ICS F4xx PWM & Current Feedback
 *
 * @brief STM32F4, ICS PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F4 MCU
 * and using an Insulated Current Sensors topology.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

#define TIMxCCER_MASK_CH123         (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2| LL_TIM_CHANNEL_CH2N |\
                                     LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)
#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))
#define ADC_SR_MASK                ((uint32_t) (0xC))

void ICS_HFCurrentsCalibration( PWMC_Handle_t * pHandle, ab_t * pStator_Currents );

/**
* @brief  It initializes TIMx, ADC, GPIO and NVIC for current reading
*         in ICS configuration using STM32F4XX
* @param  ICS F4xx PWM Current Feedback Handle
* @retval none
*/
__weak void ICS_Init( PWMC_ICS_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter( TIMx );

    /* BKIN, if enabled */
    if ( ( pHandle->pParams_str->EmergencyStop ) != DISABLE )
    {
      LL_TIM_ClearFlag_BRK( TIMx );
      LL_TIM_EnableIT_BRK( TIMx );
    }

    /* Prepare timer for synchronization */
    LL_TIM_GenerateEvent_UPDATE( TIMx );

    if ( pHandle->pParams_str->FreqRatio == 2u )
    {
      if ( pHandle->pParams_str->IsHigherFreqTim == HIGHER_FREQ )
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
      if ( pHandle->pParams_str->InstanceNbr == 1u )
      {
        LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
      }
    }

    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

    if ( pHandle->pParams_str->TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
      pHandle->ADCTriggerSet = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
    }
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
      pHandle->ADCTriggerSet = LL_ADC_INJ_TRIG_EXT_TIM8_CH4;
    }

    /* ADC1 and ADC2 registers configuration ---------------------------------*/
    /* Enable ADC1 and ADC2 */
    LL_ADC_Enable( ADC1 );
    LL_ADC_Enable( ADC2 );

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength( ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE );

    /* ADC1 Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS( ADC1 );
    LL_ADC_EnableIT_JEOS( ADC1 );


    pHandle->OverCurrentFlag = false;
    pHandle->_Super.DTTest = 0u;

  }
}

/**
* @brief  Stores into the component's handle the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowing into the
*         motor
* @param pHandle ICS F4xx PWM Current Feedback Handle
*/
__weak void ICS_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->PhaseAOffset = 0u;
  pHandle->PhaseBOffset = 0u;
  pHandle->PolarizationCounter = 0u;

  /* Force inactive level on TIMx CHy and TIMx CHyN */
  LL_TIM_CC_DisableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_HFCurrentsCalibration;
  ICS_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd( TIMx,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->PolarizationCounter );

  ICS_SwitchOffPWM( &pHandle->_Super );

  pHandle->PhaseAOffset >>= 3;
  pHandle->PhaseBOffset >>= 3;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
   force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH3 );

  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->Half_PWMPeriod );

  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );

  /* Set back TIMx CCER register */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_GetPhaseCurrents;
}

/**
* @brief Computes and return latest converted motor phase currents motor
* @param pHandle ICS F4xx PWM Current Feedback Handle
* @retval Ia and Ib current in ab_t format
*/
__weak void ICS_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  int32_t aux;
  uint16_t reg;
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* disable ADC trigger */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  */
  reg = ( uint16_t )( ( ADC1->JDR1 ) << 1 );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseAOffset );

  /* Saturation of Ia */
  if ( aux < -INT16_MAX )
  {
    pStator_Currents->a = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    pStator_Currents->a = INT16_MAX;
  }
  else
  {
    pStator_Currents->a = ( int16_t )aux;
  }

  /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = ( uint16_t )( ( ADC2->JDR1 ) << 1 );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseBOffset );

  /* Saturation of Ib */
  if ( aux < -INT16_MAX )
  {
    pStator_Currents->b = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    pStator_Currents->b = INT16_MAX;
  }
  else
  {
    pStator_Currents->b = ( int16_t )aux;
  }

  pHandle->_Super.Ia = pStator_Currents->a;
  pHandle->_Super.Ib = pStator_Currents->b;
  pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;

}


/**
* @brief Sums up injected conversion data into wPhaseXOffset. It is called
*         only during current calibration
* @param pHdl ICS F4xx PWM Current Feedback Handle
* @retval Always returns {0,0} in ab_t format
*/
__weak void ICS_HFCurrentsCalibration( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* disable ADC trigger */
  LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH4 );

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseAOffset += ADC1->JDR1;
    pHandle->PhaseBOffset += ADC2->JDR1;
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/**
  * @brief Turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param pHdl ICS F4xx PWM Current Feedback Handle
  */
__weak void ICS_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1( TIMx, 0 );
  LL_TIM_OC_SetCompareCH2( TIMx, 0 );
  LL_TIM_OC_SetCompareCH3( TIMx, 0 );

  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET ) ;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
}


/**
* @brief Enables PWM generation on the proper Timer peripheral acting on MOE bit
* @param pHdl ICS F4xx PWM Current Feedback Handle
*/
__weak void ICS_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 5u));

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( LL_TIM_CC_IsEnabledChannel(TIMx, TIMxCCER_MASK_CH123) != 0u )
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
  LL_TIM_EnableIT_UPDATE( TIMx );

}


/**
* @brief  Disables PWM generation on the proper Timer peripheral acting on
*         MOE bit
* @param pHdl ICS F4xx PWM Current Feedback Handle
*/
__weak void ICS_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIMx );

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  return;
}

/**
* @brief Stores into 'this' object variables the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param pHandle ICS F4xx PWM Current Feedback Handle
* @retval none
*/
__weak uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
  uint16_t aux;
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->_Super.CntPhC );

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if ( LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    aux = MC_FOC_DURATION;
  }
  else
  {
    aux = MC_NO_ERROR;
  }
  return aux;
}



/**
* @brief Contains the TIMx Update event interrupt
* @param pHandle ICS F4xx PWM Current Feedback Handle
* @retval none
*/
__weak void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle )
{
  uint32_t adcinjflags;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  adcinjflags = ( ADC1->SR ) & ADC_SR_MASK;

  if ( adcinjflags == CONV_STARTED )
  {
    do
    {
      adcinjflags = ( ADC1-> SR ) & ADC_SR_MASK;
    }
    while ( adcinjflags != CONV_FINISHED );
  }
  else if ( adcinjflags == FLAGS_CLEARED )
  {
    while ( ( TIMx->CNT ) < ( pHandle->pParams_str->Tw ) )
    {}
    adcinjflags = ( ADC1-> SR ) & ADC_SR_MASK;

    if ( adcinjflags == CONV_STARTED )
    {
      do
      {
        adcinjflags = ( ADC1-> SR ) & ADC_SR_MASK;
      }
      while ( adcinjflags != CONV_FINISHED );
    }
  }
  else {}

  /* Switch Context */
  /* It re-initilize AD converter in run time when using dual MC */
  LL_ADC_INJ_SetTriggerSource(ADC1, pHandle->ADCTriggerSet);
  LL_ADC_INJ_SetTriggerSource(ADC2, pHandle->ADCTriggerSet);

  /* Change channels keeping equal to 1 element the sequencer lenght */
  ADC1->JSQR = ( uint32_t )( pHandle->pParams_str->IaChannel ) << 15;
  ADC2->JSQR = ( uint32_t )( pHandle->pParams_str->IbChannel ) << 15;

  LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH4 );

  return &( pHandle->_Super.Motor );
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHandle )
{

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  pHandle->OverCurrentFlag = true;

  return &( pHandle->_Super.Motor );
}

/**
* @brief Used to check if an overcurrent occurred since last call.
* @param pHandle ICS F4xx PWM Current Feedback Handle
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
__weak uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  uint16_t retval = MC_NO_FAULTS;

  if ( pHandle->OverCurrentFlag == true )
  {
    retval = MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retval;
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

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
