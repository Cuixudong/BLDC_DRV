/**
 ******************************************************************************
 * @file    r3_1_f4xx_pwm_curr_fdbk.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement current sensor
 *          class to be stantiated when the three shunts current sensing
 *          topology is used. It is specifically designed for STM32F401x8
 *          microcontrollers.
 *           + MCU peripheral and handle initialization function
 *           + three shunt current sesnsing
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
  * @ingroup r3_1_f4xx_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r3_1_f4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r3_1_f4XX_pwm_curr_fdbk R3 1 ADC PWM & Current Feedback
 *
 * @brief STM32F3, 1 ADC, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU, using a three
 * shunt resistors current sensing topology and only one ADC to acquire the current
 * values. This is typically the implementation to use with STM32F301xx and STM32F302xx
 * MCUs that only have one ADC peripheral.
 *
 * It computes the PWM duty cycles on each PWM period, applies them to the Motor phases and reads
 * the current flowing through the Motor phases. It is built on the @ref pwm_curr_fdbk component.
 *
 * Instances of this component are managed by a PWMC_R3_1_Handle_t Handle structure that needs
 * to be initialized with the R3_1_Init() prior to being used.
 *
 * Usually, the R3_1_Init() function i the only function of this component that needs to be
 * called directly. Its other functions are usually invoked by functions of the @ref pwm_curr_fdbk
 * base component.
 *
 * @section r3_1_f4xx_periph_usage Peripheral usage
 * The PWMC_R3_1_F4 uses the following IPs of the STM32 MCU....
 *
 * * 1 Advanced Timer: 3 PWM channels, their output pins and optionally their complemented output
 * * 1 ADC: Three channels of the ADC are used. The ADC to choose can be triggered by the Timer....
 * * ...
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

/* DIR bits of TIM1 CR1 register identification for correct check of Counting direction detection*/
#define DIR_MASK 0x0010u       /* binary value: 0000000000010000 */

/* Private typedef -----------------------------------------------------------*/


/**
 * @brief  ADC Init structure definition
 */

/* Private function prototypes -----------------------------------------------*/
void R3_1_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,ab_t* pStator_Currents);
void R3_1_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl,ab_t* pStator_Currents);
uint16_t R3_1_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg);

/* Private functions ---------------------------------------------------------*/

/* Local redefinition of both LL_TIM_OC_EnablePreload & LL_TIM_OC_DisablePreload */
__STATIC_INLINE void __LL_TIM_OC_EnablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

__STATIC_INLINE void __LL_TIM_OC_DisablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}


/**
  * @brief  It initializes peripherals for current reading and PWM generation
  *         in three shunts configuration using STM32F302x8
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_1_Init(PWMC_R3_1_Handle_t *pHandle)
{
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx  = pHandle->pParams_str->ADCx;

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOCS(ADCx);
    LL_ADC_ClearFlag_EOCS(ADCx);
    LL_ADC_DisableIT_JEOS(ADCx);
    LL_ADC_ClearFlag_JEOS(ADCx);

    /* disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter(TIMx);
    if ( TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
    }
#if defined(TIM8)
    else if ( TIMx == TIM8 )
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
    }
#endif

    LL_TIM_ClearFlag_BRK(TIMx);

    if ( ( pHandle->pParams_str->EmergencyStop ) != DISABLE )
    {
      LL_TIM_EnableIT_BRK( TIMx );
    }

    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

    /* ADC Enable (must be done after calibration) */
    LL_ADC_Enable( ADCx );

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength( ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE );
    
    /* ADCx Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS( ADCx );
    LL_ADC_EnableIT_JEOS( ADCx );

    pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;
    /* reset injected conversion sequencer length set by cubeMX */
    LL_ADC_INJ_SetSequencerLength( ADCx, LL_ADC_INJ_SEQ_SCAN_DISABLE );

    /* Clear the flags */
    pHandle->OverCurrentFlag = false;
    pHandle->_Super.DTTest = 0u;
  }
}

/**
  * @brief  It measures and stores into handler component variables the offset voltage on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_1_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->PhaseAOffset = 0u;
  pHandle->PhaseBOffset = 0u;
  pHandle->PhaseCOffset = 0u;

  pHandle->PolarizationCounter = 0u;

  /* It forces inactive level on TIMx CHy and CHyN */
  LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_HFCurrentsCalibrationAB;
  pHandle->_Super.pFctSetADCSampPointSectX = &R3_1_SetADCSampPointCalibration;

  pHandle->CalibSector = SECTOR_5;
  /* Required to force first polarization conversion on SECTOR_5*/
  pHandle->_Super.Sector = SECTOR_5; 

  R3_1_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd( TIMx,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->PolarizationCounter );

  R3_1_SwitchOffPWM( &pHandle->_Super );

  /* Offset calibration for C phase */
  /* Reset PolarizationCounter */
  pHandle->PolarizationCounter = 0u;

  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_HFCurrentsCalibrationC;

  /* "Phase C current calibration to verify"    */
  pHandle->CalibSector = SECTOR_1;
  /* Required to force first polarization conversion on SECTOR_1*/
  pHandle->_Super.Sector = SECTOR_1; 

  R3_1_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd( TIMx,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->PolarizationCounter );

  R3_1_SwitchOffPWM( &pHandle->_Super );

  /* Shift of N bits to divide for the NB_ CONVERSIONS = 16= 2^N with N = 4 */
  pHandle->PhaseAOffset >>= 3;
  pHandle->PhaseBOffset >>= 3;
  pHandle->PhaseCOffset >>= 3;

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSectX = &R3_1_SetADCSampPointSectX;
  
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */  
  __LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  __LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  __LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  
  LL_TIM_OC_SetCompareCH1 (TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH2 (TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH3 (TIMx,pHandle->Half_PWMPeriod);
  
  /* Enable TIMx preload */
  __LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  __LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  __LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  
  /* sector and phase sequence for the switch on phase */
  pHandle->_Super.Sector = SECTOR_5;

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  pHandle->BrakeActionLock = false;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in ab_t format
  */
__weak void R3_1_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t* pStator_Currents)
{
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;
  uint8_t bSector;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  bSector = ( uint8_t )( pHandle->_Super.Sector );

  hReg1 =  *pHandle->pParams_str->ADCDataReg1[bSector]*2;
  hReg2 =  *pHandle->pParams_str->ADCDataReg2[bSector]*2;

  switch ( bSector )
  {
    case SECTOR_4:
    case SECTOR_5:
    {
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
    }
    break;

    case SECTOR_6:
    case SECTOR_1:
    {
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
    }
    break;

    case SECTOR_2:
    case SECTOR_3:
    {
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
    }
    break;

    default:
    {
    }
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
 * @param pHdl: handler of the current instance of the PWM component
 * @retval It always returns {0,0} in ab_t format
 */
void R3_1_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,ab_t* pStator_Currents)
{  
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
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
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into PhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC input before to enable
  *         the offset computation.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in ab_t format
  */
__weak void R3_1_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl, ab_t* pStator_Currents)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R3_1_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1( TIMx, 0 );
  LL_TIM_OC_SetCompareCH2( TIMx, 0 );
  LL_TIM_OC_SetCompareCH3( TIMx, 0 );

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Enable signals activation */
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return; 
}

/**
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit
  * @param pHdl handler of the current instance of the PWM component
  */
__weak void R3_1_SwitchOnPWM(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;
  
  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 5u));

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE( TIMx );

  return; 
}

/**
  * @brief  Disables PWM generation on the proper Timer peripheral acting on  MOE bit
  * @param pHdl handler of the current instance of the PWM component
  */
__weak void R3_1_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIMx );

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }

  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);

  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  writes into peripheral registers the new duty cycles and
  *        sampling point
  * @param pHdl: handler of the current instance of the PWM component
  * @param hCCR4Reg: new capture/compare register value.
  * @retval none
  */
__weak uint16_t R3_1_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg)
{
  uint16_t hAux;
      
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  LL_TIM_OC_SetCompareCH1 (TIMx,pHandle->_Super.CntPhA);
  LL_TIM_OC_SetCompareCH2 (TIMx,pHandle->_Super.CntPhB);
  LL_TIM_OC_SetCompareCH3 (TIMx,pHandle->_Super.CntPhC);
  
  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if ( LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
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

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling during calibration.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak uint16_t R3_1_SetADCSampPointCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;
  pHandle->_Super.Sector = pHandle->CalibSector;
  
  return R3_1_WriteTIMRegisters(&pHandle->_Super, (uint16_t)(pHandle->Half_PWMPeriod) - 1u);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector x.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak uint16_t R3_1_SetADCSampPointSectX( PWMC_Handle_t * pHdl )
{

  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
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

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 5 */
    pHandle->_Super.Sector = SECTOR_5;

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

  return R3_1_WriteTIMRegisters( &pHandle->_Super, hCntSmp);
}


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Update event interrupt
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void *R3_1_TIMx_UP_IRQHandler(PWMC_R3_1_Handle_t *pHandle)
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  /* reset ADC external trigger edge */
  LL_ADC_INJ_StopConversionExtTrig(ADCx);
  
  ADCx->JSQR = pHandle->pParams_str->ADCConfig[pHandle->_Super.Sector];

  /* enable ADC trigger source */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  
  /* set ADC external trigger edge */
  LL_ADC_INJ_StartConversionExtTrig(ADCx, pHandle->ADCTriggerEdge);

  /* reset default edge detection trigger */
  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;

  return &( pHandle->_Super.Motor );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Break1 event interrupt
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
__weak void *R3_1_BRK_IRQHandler(PWMC_R3_1_Handle_t *pHandle)
{
  if ( pHandle->BrakeActionLock == false )
  {
    if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }
  pHandle->OverCurrentFlag = true;

  return &( pHandle->_Super.Motor );
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
__weak uint16_t R3_1_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  uint16_t retVal = MC_NO_FAULTS;

  if (pHandle->OverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  
  return retVal;
}


/**
  * @brief  It is used to disable the PWM mode during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_RLDetectionModeEnable(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  
  if (pHandle->_Super.RLDetectionMode == false)
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    
    /*  Channel2 configuration */
    if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {
    }
    
    /*  Channel3 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);

  }
  
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_1_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_1_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_1_SwitchOffPWM;

  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  It is used to disable the PWM mode during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_RLDetectionModeDisable(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == true)
  {
    /* Repetition Counter of TIM1 User value reactivation BEGIN*/
    
    /* The folowing while cycles ensure the identification of the positive counting mode of TIM1 
     * for correct reactivation of Repetition Counter value of TIM1.*/
    
    /* Wait the change of Counter Direction of TIM1 from Up-Direction to Down-Direction*/
    while ((TIMx->CR1 & DIR_MASK) == 0u)
    {
    }
    /* Wait the change of Counter Direction of TIM1 from Down-Direction to Up-Direction.*/
    while ((TIMx->CR1 & DIR_MASK) == DIR_MASK)
    {
    }
    
    /* TIM1 Repetition Counter reactivation to the User Value */
    TIMx->RCR = pHandle->pParams_str->RepetitionCounter;
    /* Repetition Counter of TIM1 User value reactivation END*/
    
    
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
    
    /*  Channel2 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
    
    /*  Channel3 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
	
    /* ADCx Injected discontinuous mode disable */
    LL_ADC_INJ_SetSequencerDiscont(pHandle->pParams_str->ADCx,
                                   LL_ADC_INJ_SEQ_DISCONT_DISABLE);
       
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_1_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_1_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_1_SwitchOffPWM;
    
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
uint16_t R3_1_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint32_t val;
  uint16_t hAux;

  val = ( ( uint32_t )( pHandle->Half_PWMPeriod ) * ( uint32_t )( hDuty ) ) >> 16;
  pHandle->_Super.CntPhA = ( uint16_t )( val );

  /* set sector in order to sample phase B */
  pHandle->_Super.Sector = SECTOR_4;
  
  /* TIM1 Channel 1 Duty Cycle configuration.
   * In RL Detection mode only the Up-side device of Phase A are controlled*/
  LL_TIM_OC_SetCompareCH1(TIMx, ( uint32_t )pHandle->_Super.CntPhA);
  
  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if ( LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if (pHandle->_Super.SWerror == 1u)
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
	  }
  return hAux;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It computes and return latest converted motor phase currents motor
  *         during RL detection phase
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in ab_t format
  */
void R3_1_RLGetPhaseCurrents(PWMC_Handle_t *pHdl,ab_t* pStator_Currents)
{

  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  int32_t wAux;
  
  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  
  wAux = (int32_t)( pHandle->PhaseBOffset ) - (int32_t)*pHandle->pParams_str->ADCDataReg2[pHandle->_Super.Sector]*2;
  
  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
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
  /* First value read of Phase B*/
  pStator_Currents->a = ( int16_t )( wAux );
  pStator_Currents->b = ( int16_t )( wAux );

}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_RLTurnOnLowSides(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1(TIMx, 0u);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return; 
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_RLSwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
  
  pHandle->_Super.TurnOnLowSidesAction = false;
  /* The following while cycles ensure the identification of the nergative counting mode of TIM1
   * for correct modification of Repetition Counter value of TIM1.*/

  /* Wait the change of Counter Direction of TIM1 from Down-Direction to Up-Direction*/

  while ( ( TIMx->CR1 & DIR_MASK ) == DIR_MASK )
  {
  }
  /* Wait the change of Counter Direction of TIM1 from Up-Direction to Down-Direction*/
  while ( ( TIMx->CR1 & DIR_MASK ) == 0u )
  {
  }

  /* Set channel 1 Compare/Capture register to 1 */
  LL_TIM_OC_SetCompareCH1(TIMx, 1u);

  /* Set channel 4 Compare/Capture register to trig ADC in the middle 
     of the PWM period */
  LL_TIM_OC_SetCompareCH4(TIMx,(( uint32_t )( pHandle->Half_PWMPeriod ) - 5u));
  

  LL_TIM_ClearFlag_UPDATE( TIMx ); /* Clear flag to wait next update */

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs(TIMx);

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  
  /* Clear JEOC Flag */
  LL_ADC_ClearFlag_JEOS(ADCx);
    
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* enable TIMx update interrupt*/
  LL_TIM_EnableIT_UPDATE( TIMx );

  return; 
}

/**
 * @brief  It turns on low sides switches and start ADC triggering.
 *         This function is specific for MP phase.
 * @param  pHdl Pointer on the target component instance
 * @retval none
 */
void RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

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

//  ADCx->JSQR = pHandle->wADC_JSQR_phAB;
  LL_ADC_INJ_StartConversionExtTrig(ADCx,LL_ADC_INJ_TRIG_EXT_RISING);

  return;
}


/**
 * @brief  It sets ADC sampling points.
 *         This function is specific for MP phase.
 * @param  pHdl Pointer on the target component instance
 * @retval none
 */
void RLSetADCSampPoint( PWMC_Handle_t * pHdl )
{
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  /* dummy sector setting to get correct Ia value */
  pHdl->Sector = SECTOR_5;

//  ADCx->JSQR = pHandle->wADC_JSQR_phAB;
  LL_ADC_INJ_StartConversionExtTrig(ADCx,LL_ADC_INJ_TRIG_EXT_RISING);

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
