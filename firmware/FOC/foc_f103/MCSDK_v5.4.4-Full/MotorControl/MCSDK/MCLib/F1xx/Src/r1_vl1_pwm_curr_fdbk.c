/**
  ******************************************************************************
  * @file    r1_vl1_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the r1_vl1_pwm_curr_fdbk component of the Motor Control SDK.
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
#include "r1_vl1_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup r1_vl1_pwm_curr_fdbk R1 VL1 PWM & Current Feedback
 *
 * @brief STM32F1 Value Line, 1-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F103 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))
/* Direct address of the registers used by DMA */
#define TIM1_CCR1_Address   0x40012C34u
#define TIM1_CCR2_Address   0x40012C38u
#define TIM1_CCR3_Address   0x40012C3Cu
#define TIM3_CCR4_Address   0x40000440u
#define TIM4_CCR3_Address   0x4000083Cu

/** @{ */
#define REGULAR         ((uint8_t)0u) /**< Regular */
#define BOUNDARY_1      ((uint8_t)1u) /**< Two small, one big. */
#define BOUNDARY_2      ((uint8_t)2u) /**< Two big, one small. */
#define BOUNDARY_3      ((uint8_t)3u) /**< Three equal. */
/** @} */

/** @{ */
/* Possible types for PWM periods */
/** @brief Regular PWM period */
#define INVERT_NONE 0u
/** @brief Distort PHA */
#define INVERT_A 1u
/** @brief Distort PHB */
#define INVERT_B 2u
/** @brief Distort PHC */
#define INVERT_C 3u

#define SAMP_NO 0u
#define SAMP_IA 1u
#define SAMP_IB 2u
#define SAMP_IC 3u
#define SAMP_NIA 4u
#define SAMP_NIB 5u
#define SAMP_NIC 6u
#define SAMP_OLDA 7u
#define SAMP_OLDB 8u
#define SAMP_OLDC 9u

#define CCMR1_PRELOAD_DISABLE_MASK 0xF7F7u
#define CCMR2_PRELOAD_DISABLE_MASK 0xFFF7u
#define CCMR1_PRELOAD_ENABLE_MASK 0x0808u
#define CCMR2_PRELOAD_ENABLE_MASK 0x0008u
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

/* Private Constants ---------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

/* Private function prototypes -----------------------------------------------*/
void R1VL1_TIMxInit( TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, PWMC_R1_VL1_Handle_t * pHandle );
void R1VL1_1ShuntMotorVarsInit( PWMC_R1_VL1_Handle_t * pHandle );
void R1VL1_1ShuntMotorVarsRestart( PWMC_R1_VL1_Handle_t * pHandle );

/* Global functions ---------------------------------------------------------*/

/**
  * @brief  It initializes TIM, ADC and DMA for single shunt current
  *         reading configuration using STM32 F10x Medium or Low Density.
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */

__weak void R1VL1_Init( PWMC_R1_VL1_Handle_t * pHandle )
{
  TIM_TypeDef* AuxTIM;
  
  pHandle->_Super.TurnOnLowSidesAction = false;

  AuxTIM = pHandle->pParams_str->TIMx_2;

  R1VL1_1ShuntMotorVarsInit( pHandle );

  RCC->AHBENR |= LL_AHB1_GRP1_PERIPH_CRC;

  /* Enable AUX_TIM clock and Debug MODE */
  if ( AuxTIM == TIM3 )
  {
    LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_TIM3_STOP);
  }
  else
  {
#if (defined(TIM4))
    LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_TIM4_STOP);
#endif
  }
  
  /* Set timer in Debug MODE */
  /* TIM1 Counter Clock stopped when the core is halted */
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);

  R1VL1_TIMxInit( pHandle->pParams_str->TIMx, AuxTIM, pHandle );
  
  /* DMA Settings */

  /* DMA Event related to TIM1 Channel 4 */
  /* DMA1 Channel4 configuration ----------------------------------------------*/
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) TIM1_CCR1_Address);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) (pHandle->hDmaBuff));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, 2u);
  /* Enable DMA1 Channel4 */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

  /* DMA Event related to AUX_TIM */
  /* DMA channel configuration */
  if ( AuxTIM == TIM3 )
  {
   LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t) TIM3_CCR4_Address);
   LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t) (pHandle->hCCDmaBuffCh4));
   LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 3u);
     /* Enable DMA Channel3 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  }
  else
  {
   LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t) TIM4_CCR3_Address);
   LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)(pHandle->hCCDmaBuffCh4));
   LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 3u);
    /* Enable DMA1 Channel5 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
  }
  
  if ( pHandle->pParams_str->RepetitionCounter > 1u )
  {
    /* Enable DMA1 CH4 TC IRQ */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
    pHandle->bDMATot = (pHandle->pParams_str->RepetitionCounter + 1u) / 2u;
  }
  else
  {
    /* REP RATE = 1 */
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_4);
    pHandle->bDMATot = 0u;
  }
        
  /* Enable ADC */
  LL_ADC_Enable(pHandle->pParams_str->ADCx_Inj);
 
  /* Start calibration of ADC regular conversions */
  LL_ADC_StartCalibration(pHandle->pParams_str->ADCx_Inj);

  /* Wait for the end of ADC calibration */
  while ( LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->ADCx_Inj) )
  {
  }
  
  /* Enable discontinuous mode */
  LL_ADC_INJ_SetSequencerDiscont(pHandle->pParams_str->ADCx_Inj, LL_ADC_INJ_SEQ_DISCONT_1RANK);

  /* Disable sequencers scan mode */
  LL_ADC_SetSequencersScanMode(pHandle->pParams_str->ADCx_Inj, LL_ADC_SEQ_SCAN_DISABLE);

  R1VL1_1ShuntMotorVarsRestart( pHandle );

  /*  Set AUX_TIM channel start value and enable DMA */
  if ( AuxTIM == TIM3 )
  {
    LL_TIM_OC_SetCompareCH4(AuxTIM, (pHandle->Half_PWMPeriod >> 1) - pHandle->pParams_str->hTbefore);
    LL_TIM_EnableDMAReq_CC4(AuxTIM);
  }
  else
  {
    LL_TIM_OC_SetCompareCH3(AuxTIM, (pHandle->Half_PWMPeriod >> 1) - pHandle->pParams_str->hTbefore);
    LL_TIM_EnableDMAReq_CC3(AuxTIM);
  }
  
  LL_ADC_EnableIT_JEOS(pHandle->pParams_str->ADCx_Inj);

  pHandle->OverCurrentFlag = false;
  pHandle->_Super.DTTest = 0u;

}

/**
 * @brief  Initializes TIMx and TIMx_2 peripherals for PWM generation,
 *         active vector insertion and ADC triggering.
 *
 * @param  TIMx Timer to be initialized
 * @param  TIMx_2 Auxiliary timer to be initialized used for ADC triggering
 * @param  pHandle Handle of the component being initialized
 */
__weak void R1VL1_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, PWMC_R1_VL1_Handle_t * pHandle )
{
  
  /* Channel 1, 2,3 Configuration in PWM mode */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* disable main and auxiliary TIM counters to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter(TIMx);
  LL_TIM_DisableCounter(TIMx_2);
  
  /* BKIN, if enabled */
  if ( (pHandle->pParams_str->EmergencyStop) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK(TIMx);
    LL_TIM_EnableIT_BRK(TIMx);
  }
  
  /* Disable update interrupt */
  LL_TIM_DisableIT_UPDATE(TIMx);

  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* TIMx_2 channel Init */
  if (TIMx_2 == TIM3)
  {
    LL_TIM_OC_DisablePreload(TIMx_2, LL_TIM_CHANNEL_CH4);
    LL_TIM_CC_EnableChannel(TIMx_2, LL_TIM_CHANNEL_CH4);
  }
  else
  {
    LL_TIM_OC_DisablePreload(TIMx_2, LL_TIM_CHANNEL_CH3);
  } 
  
  /* Prepare timers for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
  LL_TIM_GenerateEvent_UPDATE(TIMx_2);
}


/**
 * @brief  Calibrates the ADC used for reading current
 *
 *  This function stores the voltage measured on the current feedback analog channel
 *  when no current is flowing into the motor in the handle of the component.
 *
  * @param  pHandle handler of the current instance of the PWM component
 */
__weak void R1VL1_CurrentReadingCalibration( PWMC_Handle_t * pHandle )
{
  uint8_t bIndex = 0u;
  uint32_t wPhaseOffset = 0u;
  PWMC_R1_VL1_Handle_t * pH = (PWMC_R1_VL1_Handle_t *) pHandle;
  R1_VL1Params_t const * pDParams_str = pH->pParams_str;
  
  /* Set the CALIB flags to indicate the ADC calibration phase*/
  pH->hFlags |= CALIB;

  /* ADC Channel used for current reading are read 
  in order to get zero currents ADC values*/   
  while (bIndex< NB_CONVERSIONS)
  {
	  pDParams_str->ADCx_Inj->SQR3 = pDParams_str->hIChannel;
           
    /* It starts software triggered regular conversion
    through bit banding access. It is equivalent to 
    ADC1->CR2 |= EXTTRIG_SWSTART_Set;    */
    BB_REG_BIT_SET ( &pH->pParams_str->ADCx_Inj->CR2, ADC_CR2_SWSTART_Pos );
    
    /* Wait until end of regular conversion */
    while ( LL_ADC_IsActiveFlag_EOS (pDParams_str->ADCx_Inj ) == 0u) {}
    {}    
        
    wPhaseOffset += LL_ADC_REG_ReadConversionData12( pDParams_str->ADCx_Inj );
    bIndex++;
  }
  
  pH->wPhaseOffset = (uint16_t)(wPhaseOffset/NB_CONVERSIONS);
  
  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pH->hFlags &= (~CALIB);
  
}

/**
 * @brief  Initializes motor variables of the component pointed by @p pHandle
 */
__weak void R1VL1_1ShuntMotorVarsInit( PWMC_R1_VL1_Handle_t * pHandle )
{
  /* Init motor vars */
  pHandle->wPhaseOffset = 0u;
  pHandle->bInverted_pwm = INVERT_NONE;
  pHandle->bInverted_pwm_new = INVERT_NONE;
  pHandle->hFlags &= (~STBD3);
  pHandle->hFlags &= (~DSTEN);

  /* After reset value of DMA buffers */
  pHandle->hDmaBuff[0] = pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] = pHandle->Half_PWMPeriod >> 1;

  /* After reset value of dvDutyValues */
  pHandle->_Super.CntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhC = pHandle->Half_PWMPeriod >> 1;

  /* Default value of DutyValues */
  pHandle->hCntSmp1 = (pHandle->Half_PWMPeriod >> 1) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = (pHandle->Half_PWMPeriod >> 1) + pHandle->pParams_str->hTafter;

  /* Default value of sampling point */
  pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /*  Second point */
  pHandle->hCCDmaBuffCh4[1] = (pHandle->Half_PWMPeriod * 2u) - 1u; /* Update */
  pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */

  LL_TIM_DisableDMAReq_CC4(pHandle->pParams_str->TIMx);
}

/**
 * @brief Re-initializes motor variables of the component pointed by @p pHandle after each motor start
 */
__weak void R1VL1_1ShuntMotorVarsRestart( PWMC_R1_VL1_Handle_t * pHandle )
{
  /* Default value of DutyValues */
  pHandle->hCntSmp1 = (pHandle->Half_PWMPeriod >> 1) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = (pHandle->Half_PWMPeriod >> 1) + pHandle->pParams_str->hTafter;

  /* Default value of sampling point */
  pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /*  Second point */
  pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */

  /* After start value of DMA buffers */
  pHandle->hDmaBuff[0] = pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] = pHandle->Half_PWMPeriod >> 1;

  /* After start value of dvDutyValues */
  pHandle->_Super.CntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhC = pHandle->Half_PWMPeriod >> 1;

  /* Set the default previous value of Phase A,B,C current */
  pHandle->hCurrAOld = 0;
  pHandle->hCurrBOld = 0;
  pHandle->hCurrCOld = 0;

  LL_TIM_DisableDMAReq_CC4(pHandle->pParams_str->TIMx);
}

/**
  * @brief Computes and returns the most recently converted motor phase currents
  *
  * @param pHandle Handle on the PWMC component in charge of the target motor
  * @param pStatorCurrents pointer on the variable where the result is stored
  */
__weak void R1VL1_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t* pStatorCurrents )
{  
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0, hCurrC = 0;
  uint8_t bCurrASamp = 0u, bCurrBSamp = 0u, bCurrCSamp = 0u;
  PWMC_R1_VL1_Handle_t * pH = (PWMC_R1_VL1_Handle_t *) pHandle;
  
  /* Disabling the Injected conversion for ADCx after EOC*/
  pH->pParams_str->ADCx_Inj->CR2 &= CR2_JEXTTRIG_Reset;

  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pH->hFlags &= (~SOFOC);

  /* First sampling point */
  wAux = (int32_t)(pH->pParams_str->ADCx_Inj->JDR2);
  wAux *= 2;
  wAux -= (int32_t)( pH->wPhaseOffset );

  /* Check saturation */
  wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;
  
  switch ( pH->sampCur1 )
  {
  case SAMP_IA:
    hCurrA = (int16_t)( wAux );
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)( wAux );
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)( wAux );
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux;
    hCurrA = (int16_t)( wAux );
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux;
    hCurrB = (int16_t)( wAux );
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux;
    hCurrC = (int16_t)( wAux );
    bCurrCSamp = 1u;
    break;
  case SAMP_OLDA:
    hCurrA = pH->hCurrAOld;
    bCurrASamp = 1u;
    break;
  case SAMP_OLDB:
    hCurrB = pH->hCurrBOld;
    bCurrBSamp = 1u;
    break;
  default:
    break;
  }
  
  /* Second sampling point */
  wAux = (int32_t)(pH->pParams_str->ADCx_Inj->JDR1);
  wAux *= 2;
  wAux -= (int32_t)( pH->wPhaseOffset );
  
  /* Check saturation */
  wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;
  
  switch ( pH->sampCur2 )
  {
  case SAMP_IA:
    hCurrA = (int16_t)( wAux );
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)( wAux );
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)( wAux );
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux;
    hCurrA = (int16_t)( wAux );
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux;
    hCurrB = (int16_t)( wAux );
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux;
    hCurrC = (int16_t)( wAux );
    bCurrCSamp = 1u;
    break;
  default:
    break;
  }
    
  /* Computation of the third value */
  if ( bCurrASamp == 0u )
  {
    wAux = -((int32_t)( hCurrB )) - ((int32_t)( hCurrC ));

    /* Check saturation */
    wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;

    hCurrA = (int16_t) wAux;
  }
  if ( bCurrBSamp == 0u )
  {
    wAux = -((int32_t)( hCurrA )) - ((int32_t)( hCurrC ));

    /* Check saturation */
    wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;

    hCurrB = (int16_t) wAux;
  }
  if ( bCurrCSamp == 0u )
  {
    wAux = -((int32_t)( hCurrA )) - ((int32_t)( hCurrB ));

    /* Check saturation */
    wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;

    hCurrC = (int16_t) wAux;
  }
  
  /* hCurrA, hCurrB, hCurrC values are the sampled values */
    
  pH->hCurrAOld = hCurrA;
  pH->hCurrBOld = hCurrB;
  pH->hCurrCOld = hCurrC;
  
  pStatorCurrents->a = hCurrA;
  pStatorCurrents->b = hCurrB;

  pHandle->Ia = pStatorCurrents->a;
  pHandle->Ib = pStatorCurrents->b;
  pHandle->Ic = -pStatorCurrents->a - pStatorCurrents->b;

}

/**
 * @brief  Turns on Low Sides Switches of the power stage.
 *
 * This function is intended to be used for charging the boot capacitors of the driving
 * section. It has to be called each motor start-up when using high voltage drivers
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 */
__weak void R1VL1_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R1_VL1_Handle_t *pHandle = (PWMC_R1_VL1_Handle_t *) pHdl;

  pHandle->_Super.TurnOnLowSidesAction = true;
  
  pHandle->pParams_str->TIMx->CCR1 = 0u;
  pHandle->pParams_str->TIMx->CCR2 = 0u;
  pHandle->pParams_str->TIMx->CCR3 = 0u;

  LL_TIM_ClearFlag_UPDATE (pHandle->pParams_str->TIMx);
  while ( LL_TIM_IsActiveFlag_UPDATE(pHandle->pParams_str->TIMx) == RESET )
  {
  }

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs (pHandle->pParams_str->TIMx); 
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);	
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return;
}

/**
 * @brief Starts PWM generation for the target motor.
 *
 * This function enables the update event and the single shunt distortion for the
 * motor targeted by @p pHandle.
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 */
__weak void R1VL1_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R1_VL1_Handle_t *pHandle = (PWMC_R1_VL1_Handle_t *) pHdl;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Enabling distortion for single shunt */
  pHandle->hFlags |= DSTEN;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(pHandle->pParams_str->TIMx);
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
      LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);	
      LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }

  /* Enable UPDATE ISR */
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  LL_TIM_EnableIT_UPDATE(pHandle->pParams_str->TIMx);

  return;
}

/**
 * @brief  Stops PWM generation for the target motor.
 *
 * This function disables PWM generation on the proper Timer peripheral acting on
 * MOE bit, disables the single shunt distortion and reset the TIM status.
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 */
__weak void R1VL1_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  uint16_t hAux;
  PWMC_R1_VL1_Handle_t *pHandle = (PWMC_R1_VL1_Handle_t *) pHdl;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(pHandle->pParams_str->TIMx);
  
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);	
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE(pHandle->pParams_str->TIMx);

  /* Disabling distortion for single */
  pHandle->hFlags &= (~DSTEN);

  while ( LL_TIM_IsActiveFlag_UPDATE(pHandle->pParams_str->TIMx) == RESET )
  {
    if ( pHandle->pParams_str->TIMx->DIER & LL_TIM_DIER_UIE )
    {
      break;
    }
  }

  /* Disabling all DMA previous setting */
  LL_TIM_DisableDMAReq_CC4(pHandle->pParams_str->TIMx);

  /* Set all duty to 50% */
  hAux = pHandle->Half_PWMPeriod >> 1u;
  pHandle->pParams_str->TIMx->CCR1 = (uint32_t)(hAux);
  pHandle->pParams_str->TIMx->CCR2 = (uint32_t)(hAux);
  pHandle->pParams_str->TIMx->CCR3 = (uint32_t)(hAux);
}

/**
 * @brief Computes the duty cycle for the next PWM period of the target motor
 *
 * Implementation of the single shunt algorithm to setup the TIM1 register and
 * DMA buffers values for the next PWM period of the motor targeted by @p pHandle.
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 *
 * @retval # MC_FOC_DURATION if the TIMx update occurs before the end of FOC
 *         algorithm; # MC_NO_ERROR otherwise.
 */
__weak uint16_t R1VL1_CalcDutyCycles( PWMC_Handle_t * pHdl )
{
  TIM_TypeDef* TIMx;
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint16_t hAux;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  PWMC_R1_VL1_Handle_t *pHandle = (PWMC_R1_VL1_Handle_t *) pHdl;

  bSector = (uint8_t)pHandle->_Super.Sector;
  
  if ((pHandle->hFlags & DSTEN) != 0u)
  {
    switch ( bSector )
    {
    case SECTOR_1:
      hDutyV_2 = pHandle->_Super.CntPhA;
      hDutyV_1 = pHandle->_Super.CntPhB;
      hDutyV_0 = pHandle->_Super.CntPhC;
      break;
    case SECTOR_2:
      hDutyV_2 = pHandle->_Super.CntPhB;
      hDutyV_1 = pHandle->_Super.CntPhA;
      hDutyV_0 = pHandle->_Super.CntPhC;
      break;
    case SECTOR_3:
      hDutyV_2 = pHandle->_Super.CntPhB;
      hDutyV_1 = pHandle->_Super.CntPhC;
      hDutyV_0 = pHandle->_Super.CntPhA;
      break;
    case SECTOR_4:
      hDutyV_2 = pHandle->_Super.CntPhC;
      hDutyV_1 = pHandle->_Super.CntPhB;
      hDutyV_0 = pHandle->_Super.CntPhA;
      break;
    case SECTOR_5:
      hDutyV_2 = pHandle->_Super.CntPhC;
      hDutyV_1 = pHandle->_Super.CntPhA;
      hDutyV_0 = pHandle->_Super.CntPhB;
      break;
    case SECTOR_6:
      hDutyV_2 = pHandle->_Super.CntPhA;
      hDutyV_1 = pHandle->_Super.CntPhC;
      hDutyV_0 = pHandle->_Super.CntPhB;
      break;
    default:
      break;
    }
    
    /* Compute delta duty */
    hDeltaDuty_0 = (int16_t)( hDutyV_1 ) - (int16_t)( hDutyV_0 );
    hDeltaDuty_1 = (int16_t)( hDutyV_2 ) - (int16_t)( hDutyV_1 );

    /* Check region */
    if ((uint16_t)hDeltaDuty_0<=pHandle->pParams_str->hTMin)
    {
      if ((uint16_t)hDeltaDuty_1<=pHandle->pParams_str->hTMin)
      {
        bStatorFluxPos = BOUNDARY_3;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_2;
      }
    }
    else
    {
      if ((uint16_t)hDeltaDuty_1>pHandle->pParams_str->hTMin)
      {
        bStatorFluxPos = REGULAR;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_1;
      }
    }
        
    if ( bStatorFluxPos == REGULAR )
    {
      pHandle->bInverted_pwm_new = INVERT_NONE;
    }
    else if ( bStatorFluxPos == BOUNDARY_1 ) /* Adjust the lower */
    {
      switch ( bSector )
      {
      case SECTOR_5:
      case SECTOR_6:
        if (pHandle->_Super.CntPhA - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
        {
          pHandle->bInverted_pwm_new = INVERT_A;
          pHandle->_Super.CntPhA -=pHandle->pParams_str->hHTMin;
          if (pHandle->_Super.CntPhA < hDutyV_1)
          {
            hDutyV_1 = pHandle->_Super.CntPhA;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pHandle->hFlags & STBD3) == 0u)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.CntPhA -=pHandle->pParams_str->hHTMin;
            pHandle->hFlags |= STBD3;
          } 
          else
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.CntPhB -=pHandle->pParams_str->hHTMin;
            pHandle->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_2:
      case SECTOR_1:
        if (pHandle->_Super.CntPhB - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
        {
          pHandle->bInverted_pwm_new = INVERT_B;
          pHandle->_Super.CntPhB -=pHandle->pParams_str->hHTMin;
          if (pHandle->_Super.CntPhB < hDutyV_1)
          {
            hDutyV_1 = pHandle->_Super.CntPhB;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pHandle->hFlags & STBD3) == 0u)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.CntPhA -=pHandle->pParams_str->hHTMin;
            pHandle->hFlags |= STBD3;
          } 
          else
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.CntPhB -=pHandle->pParams_str->hHTMin;
            pHandle->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_4:
      case SECTOR_3:
        if (pHandle->_Super.CntPhC - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
        {
          pHandle->bInverted_pwm_new = INVERT_C;
          pHandle->_Super.CntPhC -=pHandle->pParams_str->hHTMin;
          if (pHandle->_Super.CntPhC < hDutyV_1)
          {
            hDutyV_1 = pHandle->_Super.CntPhC;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pHandle->hFlags & STBD3) == 0u)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.CntPhA -=pHandle->pParams_str->hHTMin;
            pHandle->hFlags |= STBD3;
          } 
          else
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.CntPhB -=pHandle->pParams_str->hHTMin;
            pHandle->hFlags &= (~STBD3);
          }
        }
        break;
      default:
        break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_2) /* Adjust the middler */
    {
      switch (bSector)
      {
      case SECTOR_4:
      case SECTOR_5: /* Invert B */
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.CntPhB -=pHandle->pParams_str->hHTMin;
        if (pHandle->_Super.CntPhB > 0xEFFFu)
        {
          pHandle->_Super.CntPhB = 0u;
        }
        break;
      case SECTOR_2:
      case SECTOR_3: /* Invert A */
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.CntPhA -=pHandle->pParams_str->hHTMin;
        if (pHandle->_Super.CntPhA > 0xEFFFu)
        {
          pHandle->_Super.CntPhA = 0u;
        }
        break;
      case SECTOR_6:
      case SECTOR_1: /* Invert C */
        pHandle->bInverted_pwm_new = INVERT_C;
        pHandle->_Super.CntPhC -=pHandle->pParams_str->hHTMin;
        if (pHandle->_Super.CntPhC > 0xEFFFu)
        {
          pHandle->_Super.CntPhC = 0u;
        }
        break;
      default:
        break;
      }
    }
    else
    {
      if ((pHandle->hFlags & STBD3) == 0u)
      {
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.CntPhA -=pHandle->pParams_str->hHTMin;
        pHandle->hFlags |= STBD3;
      } 
      else
      {
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.CntPhB -=pHandle->pParams_str->hHTMin;
        pHandle->hFlags &= (~STBD3);
      }
    }
        
    if (bStatorFluxPos == REGULAR) /* Regular zone */
    {
      /* First point */
      if ((hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime)> pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      if ((hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime)> pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp2 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp2 >>= 1;
      }
      else
      {
        pHandle->hCntSmp2 = hDutyV_2 - pHandle->pParams_str->hTbefore;
      }
    }
    
    if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
    {      
      /* First point */
      if ((hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime)> pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod + pHandle->pParams_str->hHTMin - pHandle->pParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
    {
      /* First point */
      if ((hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime)>= pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp1 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_2 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod + pHandle->pParams_str->hHTMin - pHandle->pParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_3)  
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_0-pHandle->pParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod + pHandle->pParams_str->hHTMin - pHandle->pParams_str->hTSample;
    }
  }
  else
  {
    pHandle->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }
    
  /* Update Timer Ch 1,2,3 (These value are required before update event) */
    
  pHandle->hFlags |= EOFOC;
  /* Check if DMA transition has been completed */
  if (pHandle->bDMACur == 0u)
  { 
    TIMx = pHandle->pParams_str->TIMx;
    
    /* Preload Enable */
    TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
    TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
    
    TIMx->CCR1 = pHandle->_Super.CntPhA;
    TIMx->CCR2 = pHandle->_Super.CntPhB;
    TIMx->CCR3 = pHandle->_Super.CntPhC;

    /* Update ADC Trigger DMA buffer */
    pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /* Second point */
    pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */
  }

  /* Limit for update event */

  /* Check the status of bSOFOC flags. if is set, the next
   * update event has occurred so an error will be reported*/
  if ((pHandle->hFlags & SOFOC) != 0u)
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  /* The following instruction can be executed after Update handler 
   before the get phase current (Second EOC) */

  /* Set the current sampled */
  if ( bStatorFluxPos == REGULAR ) /* Regular zone */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
  {
    pHandle->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_3)  
  {
    if (pHandle->bInverted_pwm_new == INVERT_A)
    {
      pHandle->sampCur1 = SAMP_OLDB;
      pHandle->sampCur2 = SAMP_IA;
    }
    if (pHandle->bInverted_pwm_new == INVERT_B)
    {
      pHandle->sampCur1 = SAMP_OLDA;
      pHandle->sampCur2 = SAMP_IB;
    }
  }
    
  /* Limit for the Get Phase current (Second EOC Handler) */
  
  return (hAux);
}

/**
 * @brief  It contains the TIM1 Update event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void * R1VL1_TIM1_UP_IRQHandler( PWMC_R1_VL1_Handle_t * pHandle )
{   
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  
  uint8_t bInverted_pwm_new;

  /* Critical point start */

  /* Enabling the Injected conversion for ADCx */
  pHandle->pParams_str->ADCx_Inj->CR2 |= LL_ADC_INJ_TRIG_EXT_RISING;

  /* Critical point stop */

  /* temp var to speedup execution */
  bInverted_pwm_new = pHandle->bInverted_pwm_new;

  if ( bInverted_pwm_new != pHandle->bInverted_pwm )
  {
    /* Set the DMA destination */
    switch ( bInverted_pwm_new )
    {
      case INVERT_A:
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, TIM1_CCR1_Address);
        LL_TIM_EnableDMAReq_CC4(TIMx);
        break;

      case INVERT_B:
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, TIM1_CCR2_Address);
        LL_TIM_EnableDMAReq_CC4(TIMx);
        break;

      case INVERT_C:
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, TIM1_CCR3_Address);
        LL_TIM_EnableDMAReq_CC4(TIMx);
        break;

      default:
        LL_TIM_DisableDMAReq_CC4(TIMx);
        break;
    }
  }

  /* Clear of End of FOC Flags */
  pHandle->hFlags &= (~EOFOC);

  /* Preload Disable */
  TIM1->CCMR1 &= CCMR1_PRELOAD_DISABLE_MASK;
  TIM1->CCMR2 &= CCMR2_PRELOAD_DISABLE_MASK;

  switch ( bInverted_pwm_new )
  {
    case INVERT_A:
      pHandle->hDmaBuff[1] = pHandle->_Super.CntPhA;
      pHandle->bDMACur = pHandle->bDMATot;
      break;

    case INVERT_B:
      pHandle->hDmaBuff[1] = pHandle->_Super.CntPhB;
      pHandle->bDMACur = pHandle->bDMATot;
      break;

    case INVERT_C:
      pHandle->hDmaBuff[1] = pHandle->_Super.CntPhC;
      pHandle->bDMACur = pHandle->bDMATot;
      break;

    default:
      pHandle->bDMACur = 0u;
      break;
  }

  pHandle->bInverted_pwm = bInverted_pwm_new;

  /* Set the bSOFOC flags to indicate the execution of Update IRQ*/
  pHandle->hFlags |= SOFOC;

  return MC_NULL ;
}

__weak void * R1VL1_DMA_TC_IRQHandler( PWMC_R1_VL1_Handle_t * pHandle )
{

  pHandle->bDMACur--;
  if ( pHandle->bDMACur == 0u )
  {
    if ( (pHandle->hFlags & EOFOC) != 0u )
    {
      /* Preload Enable */
      pHandle->pParams_str->TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
      pHandle->pParams_str->TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;

      /* Compare register update */
      pHandle->pParams_str->TIMx->CCR1 = pHandle->_Super.CntPhA;
      pHandle->pParams_str->TIMx->CCR2 = pHandle->_Super.CntPhB;
      pHandle->pParams_str->TIMx->CCR3 = pHandle->_Super.CntPhC;

      /* Update ADC Trigger DMA buffer */
      pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /* Second point */
      pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */
    }
  }

  return MC_NULL ;
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void *R1VL1_BRK_IRQHandler(PWMC_R1_VL1_Handle_t *pHandle)
{

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_v_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_u_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  pHandle->OverCurrentFlag = true;

  return &(pHandle->_Super.Motor);
}

/**
 * @brief  Checks if an over current condition occurred since last call.
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 *
 * @retval # MC_BREAK_IN if an over current condition has been detected since the
 *         function was last called; # MC_NO_FAULTS otherwise.
 */
__weak uint16_t R1VL1_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_R1_VL1_Handle_t * pHandle = (PWMC_R1_VL1_Handle_t *) pHdl;
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

/**
* @}
*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
