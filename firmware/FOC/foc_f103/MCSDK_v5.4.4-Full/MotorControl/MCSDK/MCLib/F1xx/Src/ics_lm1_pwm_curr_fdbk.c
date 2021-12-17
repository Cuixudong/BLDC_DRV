/**
  ******************************************************************************
  * @file    ics_lm1_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the ICS
  *          LM1 PWM Current Feedback component of the Motor Control SDK.
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
#include "pwm_curr_fdbk.h"
#include "ics_lm1_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
* @{
*/

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup ics_lm1_pwm_curr_fdbk ICS LM1 PWM & Current Feedback
 *
 * @brief STM32F1 Low & Medium Density, ICS PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F103 Low & Medium Density MCU
 * and using an Insulated Current Sensors topology.
 *
 * *STM32F1 Low & Medium Density MCUs* refers to STM32F103x4, STM32F103x6, STM32F103x8 and
 * STM32F103xB MCUs.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

#define TIMxCCER_MASK_CH123         (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2| LL_TIM_CHANNEL_CH2N |\
                                    LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)
void ICS_HFCurrentsCalibration(PWMC_Handle_t *pHdl, ab_t* pStator_Currents);
/**
* @brief  It initializes TIM1, ADC and DMA1 for current reading
*         in ICS configuration using STM32F103x Low/Medium Density
* @param  ICS LM1 PWM Current Feedback Handle
* @retval none
*/
__weak void ICS_Init(PWMC_ICS_Handle_t *pHandle)
{

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {
    pHandle->_Super.TurnOnLowSidesAction = false;

    RCC->AHBENR |= LL_AHB1_GRP1_PERIPH_CRC;

    /* BKIN, if enabled */
    if ( (pHandle->pParams_str->EmergencyStop) != DISABLE )
    {
      LL_TIM_ClearFlag_BRK(TIM1);
      LL_TIM_EnableIT_BRK(TIM1);
    }

    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

    /* TIM1 counter enable */
    LL_TIM_EnableCounter(TIM1);

    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);

    /* Enable ADC1 and ADC2 */
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);

    /* ADC1 Injected conversions configuration */
    LL_ADC_INJ_SetSequencerLength( ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE );
    LL_ADC_INJ_SetSequencerLength( ADC2, LL_ADC_INJ_SEQ_SCAN_DISABLE );

    /* ADC trigger source */
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_CH4);
    LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_CH4);
    
    /* ADC start injected conversion */
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    /* Start calibration of ADC1 and ADC2 */
    LL_ADC_StartCalibration(ADC1);
    LL_ADC_StartCalibration(ADC2);

    /* Wait for the end of ADCs calibration */
    while ( LL_ADC_IsCalibrationOnGoing(ADC1))
    {}
    while ( LL_ADC_IsCalibrationOnGoing(ADC2))
    {}

    /* Enable external trigger fo injected conv of ADC2 */
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_EnableIT_JEOS(ADC1);
    pHandle->OverCurrentFlag = false;
    pHandle->_Super.TurnOnLowSidesAction = false;
    pHandle->_Super.DTTest = 0u;

  }
}

/**
* @brief  Offset computation for both current phases Ia and Ib. It is called
*         only during current calibration.
* @param pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
__weak void ICS_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;
  
  pHandle->PhaseAOffset = 0u;
  pHandle->PhaseBOffset = 0u;
  pHandle->PolarizationCounter = 0u;

  /* Force inactive level on TIM1 CHy and TIM1 CHyN */
  LL_TIM_CC_DisableChannel(TIM1, TIMxCCER_MASK_CH123);

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_HFCurrentsCalibration;

  ICS_SwitchOnPWM(&pHandle->_Super);

  waitForPolarizationEnd( TIM1,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->PolarizationCounter );

  ICS_SwitchOffPWM(&pHandle->_Super);

  pHandle->PhaseAOffset >>= 3;
  pHandle->PhaseBOffset >>= 3;

  /* It over write TIM1 CCRy wrongly written by FOC during calibration so as to
   force 50% duty cycle on the three inverer legs */
  /* Disable TIM1 preload */
  LL_TIM_OC_DisablePreload( TIM1, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_DisablePreload( TIM1, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_DisablePreload( TIM1, LL_TIM_CHANNEL_CH3 );

  LL_TIM_OC_SetCompareCH1( TIM1, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH2( TIM1, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH3( TIM1, pHandle->Half_PWMPeriod );

  LL_TIM_OC_EnablePreload( TIM1, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_EnablePreload( TIM1, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_EnablePreload( TIM1, LL_TIM_CHANNEL_CH3 );

  /* Set back TIM1 CCER register */
  LL_TIM_CC_EnableChannel( TIM1, TIMxCCER_MASK_CH123 );

  /* ADC1 Injected conversions end interrupt enabling */
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_GetPhaseCurrents;

}

/**
* @brief Computes and return latest converted motor phase currents motor
* @param pHandle ICS LM1 PWM Current Feedback Handle
* @retval Ia and Ib current in ab_t format
*/
__weak void ICS_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t* pStator_Currents)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;
  int32_t aux;
  uint16_t reg;

  /* Clear TIM1 Update Flag necessary to detect FOC duration SW error */
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);

  /* Ia = (PhaseAOffset)-(PHASE_A_ADC_CHANNEL value)  */
  reg = (uint16_t)((ADC1->JDR1) << 1);
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseAOffset );

  /* Saturation of Ia */
  if (aux < -INT16_MAX)
  {
    pStator_Currents->a = -INT16_MAX;
  }
  else  if (aux > INT16_MAX)
  {
    pStator_Currents->a = INT16_MAX;
  }
  else
  {
    pStator_Currents->a = (int16_t)aux;
  }

  /* Ib = (PhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = (uint16_t)((ADC2->JDR1) << 1);
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseBOffset );

  /* Saturation of Ib */
  if (aux < -INT16_MAX)
  {
    pStator_Currents->b = -INT16_MAX;
  }
  else  if (aux > INT16_MAX)
  {
    pStator_Currents->b = INT16_MAX;
  }
  else
  {
    pStator_Currents->b = (int16_t)aux;
  }

  pHandle->_Super.Ia = pStator_Currents->a;
  pHandle->_Super.Ib = pStator_Currents->b;
  pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;

}

/**
* @brief  Sums up injected conversion data into wPhaseXOffset. It is called
*         only during current calibration
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval Always returns {0,0} in ab_t format
*/
__weak void ICS_HFCurrentsCalibration(PWMC_Handle_t *pHdl, ab_t* pStator_Currents)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;

  /* disable ADC trigger */
  LL_TIM_CC_DisableChannel( TIM1, LL_TIM_CHANNEL_CH4 );

  if (pHandle->PolarizationCounter < NB_CONVERSIONS)
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
  * @brief  Turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHandle ICS LM1 PWM Current Feedback Handle
  * @retval none
  */
__weak void ICS_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIM1,0);
  LL_TIM_OC_SetCompareCH2(TIM1,0);
  LL_TIM_OC_SetCompareCH3(TIM1,0);

  LL_TIM_ClearFlag_UPDATE(TIM1);
  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET ) ;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIM1);
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
}


/**
* @brief  Enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
__weak void ICS_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(pHandle->Half_PWMPeriod - 5u));

  /* wait for a neew PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIM1);

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIM1);
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( LL_TIM_CC_IsEnabledChannel(TIM1, TIMxCCER_MASK_CH123) != 0u )
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
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE( TIM1 );

}


/**
* @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit
* @param  pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
__weak void ICS_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIM1 );

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE(TIM1);

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIM1);

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  /* wait for a neew PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIM1);

  return;
}

/**
* @brief  Stores into the component's instance handle the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowing into the motor
* @param  pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
__weak uint16_t ICS_WriteTIMRegisters(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;
  uint16_t aux;

  LL_TIM_OC_SetCompareCH1( TIM1, pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2( TIM1, pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3( TIM1, pHandle->_Super.CntPhC );

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if ( LL_TIM_CC_IsEnabledChannel(TIM1, LL_TIM_CHANNEL_CH4))
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
* @brief Contains the TIM1 Update event interrupt
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
__weak void *ICS_TIMx_IRQHandler(PWMC_ICS_Handle_t *pHandle)
{
  LL_TIM_CC_EnableChannel( TIM1, LL_TIM_CHANNEL_CH4 );
  return &( pHandle->_Super.Motor );
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void *ICS_BRK_IRQHandler(PWMC_ICS_Handle_t *pHandle)
{
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  pHandle->OverCurrentFlag = true;

  return &(pHandle->_Super.Motor);
}

/**
* @brief Used to check if an overcurrent occurred since last call.
* @param pHandle pointer on the target component instance handle
* @retval Returns MC_BREAK_IN whether an overcurrent has been
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
__weak uint16_t ICS_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_Handle_t * pHandle = (PWMC_ICS_Handle_t *) pHdl;
  uint16_t retVal = MC_NO_FAULTS;
  if (pHandle->OverCurrentFlag == true )
  {
    retVal = MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retVal;
}

/**
* @}
*/

/**
* @}
*/

/** @} */


/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
