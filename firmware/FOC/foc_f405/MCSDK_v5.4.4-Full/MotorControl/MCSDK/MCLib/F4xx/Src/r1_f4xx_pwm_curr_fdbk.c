/**
  ******************************************************************************
  * @file    r1_f4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used. It is specifically designed for STM32F302x8
  *          microcontrollers and implements the successive sampling of two motor
  *          current using only one ADC.
  *           + MCU peripheral and handle initialization fucntion
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
  */

/* Includes ------------------------------------------------------------------*/
#include "r1_f4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup r1_f4XX_pwm_curr_fdbk R1 F4xx PWM & Current Feedback
 *
 * @brief STM32F4, 1-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F4 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Private defines -----------------------------------------------------------*/


#define EOFOC 0x0001u /*!< Flag to indicate end of FOC duty available */
#define STBD3 0x0002u /*!< Flag to indicate which phase has been distorted
                           in boudary 3 zone (A or B)*/
#define DSTEN 0x0004u /*!< Flag to indicate if the distortion must be performed
                           or not (in case of charge of bootstrap capacitor phase
                           is not required)*/
#define SOFOC 0x0008u /*!< This flag will be reset to zero at the begin of FOC
                           and will be set in the UP IRQ. If at the end of
                           FOC it is set the software error must be generated*/

/* Direct address of the registers used by DMA */
#define TIM1_CCR1_Address   0x40010034u
#define TIM1_CCR2_Address   0x40010038u
#define TIM1_CCR3_Address   0x4001003Cu
#define TIM4_CCR2_Address   0x40000838u

#define TIM8_CCR1_Address   0x40010434u
#define TIM8_CCR2_Address   0x40010438u
#define TIM8_CCR3_Address   0x4001043Cu
#define TIM5_CCR4_Address   0x40000C40u

#define REGULAR         ((uint8_t)0u)
#define BOUNDARY_1      ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2      ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3      ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE 0u
#define INVERT_A 1u
#define INVERT_B 2u
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

#define CH1NORMAL           0x0060u
#define CH2NORMAL           0x6000u
#define CH3NORMAL           0x0060u
#define CH4NORMAL           0x7000u

#define CCMR1_PRELOAD_DISABLE_MASK 0xF7F7u
#define CCMR2_PRELOAD_DISABLE_MASK 0xFFF7u

#define CCMR1_PRELOAD_ENABLE_MASK 0x0808u
#define CCMR2_PRELOAD_ENABLE_MASK 0x0008u

/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((uint32_t)0x00000001u)
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFEu)

#define CR2_JEXTSEL_Reset       ((uint32_t)0xFFFF8FFFu)
#define CR2_JEXTTRIG_Set        ((uint32_t)0x00100000u)
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFEFFFFFu)

#define TIM_DMA_ENABLED_CC1 0x0200u
#define TIM_DMA_ENABLED_CC2 0x0400u
#define TIM_DMA_ENABLED_CC3 0x0800u

#define CR2_ADON_Set                ((uint32_t)0x00000001u)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define CR2_EXTTRIG_SWSTART_Set     ((uint32_t)0x00500000)

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)
#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u)

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC, SAMP_NIC, SAMP_NIA, SAMP_NIA, SAMP_NIB, SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA, SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC, SAMP_IA, SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC};

/* Private function prototypes -----------------------------------------------*/
static void R1F4XX_TIMxInit( TIM_TypeDef * TIMx, TIM_TypeDef * TIMx_2, PWMC_Handle_t * pHdl );
static void R1F4XX_1ShuntMotorVarsInit( PWMC_Handle_t * pHdl );
static void R1F4XX_1ShuntMotorVarsRestart( PWMC_Handle_t * pHdl );
static void R1F4XX_HFCurrentsCalibration( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );

/**
  * @brief  It initializes TIM, ADC, GPIO, DMA and NVIC for single shunt current
  *         reading configuration using STM32 F4 family.
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R1F4XX_Init( PWMC_R1_F4_Handle_t * pHandle )
{
  uint16_t hAux;

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    R1F4XX_1ShuntMotorVarsInit( &pHandle->_Super );

    if ( pHandle->pParams_str->TIMx_2 == TIM5 ) /* Used to trigger ADC */
    {
      /* DMA Event related to TIM5 Channel 4 used for ADC trigger*/
      /* DMA1 Stream1 channel 6 configuration */
      LL_DMA_SetMemoryAddress( DMA1, LL_DMA_STREAM_1, ( uint32_t )pHandle->hCCDmaBuffCh4 );
      LL_DMA_SetPeriphAddress( DMA1, LL_DMA_STREAM_1, ( uint32_t ) & ( TIM5->CCR4 ) );
      LL_DMA_SetDataLength( DMA1, LL_DMA_STREAM_1, 3 );
      LL_DMA_EnableStream( DMA1, LL_DMA_STREAM_1 );
      pHandle->pTIMx_2_CCR = &( TIM5->CCR4 );
    }

    if ( pHandle->pParams_str->TIMx_2 == TIM4 ) /* Used to trigger ADC */
    {
      /* DMA Event related to TIM4 Channel 2 used for ADC trigger*/
      /* DMA1 Stream3 channel2 configuration */
      LL_DMA_SetMemoryAddress( DMA1, LL_DMA_STREAM_3, ( uint32_t )pHandle->hCCDmaBuffCh4 );
      LL_DMA_SetPeriphAddress( DMA1, LL_DMA_STREAM_3, ( uint32_t ) & ( TIM4->CCR2 ) );
      LL_DMA_SetDataLength( DMA1, LL_DMA_STREAM_3, 3 );
      LL_DMA_EnableStream( DMA1, LL_DMA_STREAM_3 );
      pHandle->pTIMx_2_CCR = &( TIM4->CCR2 );
    }


    if ( pHandle->pParams_str->TIMx == TIM1 )
    {
      LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
      /* DMA Event related to TIM1 Channel 4 */
      /* DMA2 Stream4 Channel6 configuration ----------------------------------------------*/
      LL_DMA_SetMemoryAddress( DMA2, LL_DMA_STREAM_4, ( uint32_t )pHandle->hDmaBuff );
      LL_DMA_SetPeriphAddress( DMA2, LL_DMA_STREAM_4, ( uint32_t ) & ( TIM1->CCR1 ) );
      LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_4, 2 );
      LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_4 );

      if ( pHandle->pParams_str->RepetitionCounter > 1u )
      {
        /* Only if REP RATE > 1 */
        /* Enable DMA2_Stream4 TC IRQ */
        LL_DMA_EnableIT_TC ( DMA2, LL_DMA_STREAM_4 );
        pHandle->bDMATot = ( pHandle->pParams_str->RepetitionCounter + 1u ) / 2u;
      }
      else
      {
        /* REP RATE = 1 */
        LL_DMA_DisableIT_TC ( DMA2, LL_DMA_STREAM_4 );
        pHandle->bDMATot = 0u;
      }
    }
#ifdef TIM8     
    else
    {
      LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
      /* DMA Event related to TIM8 Channel 4 */
      /* DMA2 Stream7 Channel7 configuration ----------------------------------------------*/
      LL_DMA_SetMemoryAddress( DMA2, LL_DMA_STREAM_7, ( uint32_t )pHandle->hDmaBuff );
      LL_DMA_SetPeriphAddress( DMA2, LL_DMA_STREAM_7, ( uint32_t ) & ( TIM8->CCR1 ) );
      LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_7, 2 );
      LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_7 );

      if ( pHandle->pParams_str->RepetitionCounter > 1u )
      {
        /* Only if REP RATE > 1 */
        /* Enable DMA2 Stream7 TC IRQ */
        LL_DMA_EnableIT_TC ( DMA2, LL_DMA_STREAM_7 );
        pHandle->bDMATot = ( pHandle->pParams_str->RepetitionCounter + 1u ) / 2u;
      }
      else
      {
        /* REP RATE = 1 */
        LL_DMA_DisableIT_TC ( DMA2, LL_DMA_STREAM_7 );
        pHandle->bDMATot = 0u;
      }

    }
#endif /* TIM8 condition*/    
    R1F4XX_TIMxInit( pHandle->pParams_str->TIMx, pHandle->pParams_str->TIMx_2, &pHandle->_Super );


    /* ADC registers configuration -----------------------------------*/
    /* Enable ADC */
    if ( LL_ADC_IsEnabled ( pHandle->pParams_str->ADCx_Inj ) == 0 )
    {
      LL_ADC_Enable ( pHandle->pParams_str->ADCx_Inj );
    }

    R1F4XX_1ShuntMotorVarsRestart( &pHandle->_Super );

    /*  Set TIMx_2 CCx start value */
    if ( pHandle->pParams_str->TIMx_2 == TIM4 )
    {
      hAux = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
      pHandle->pParams_str->TIMx_2->CCR2 = ( uint32_t )( hAux );
      LL_TIM_EnableDMAReq_CC2 ( pHandle->pParams_str->TIMx_2 );
    }
    if ( pHandle->pParams_str->TIMx_2 == TIM5 )
    {
      hAux = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
      pHandle->pParams_str->TIMx_2->CCR4 = ( uint32_t )( hAux );
      LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx_2 );
    }

    /* work-around cubeMX code generator bug */
    /* overwrite cubeMX ADC JSQR setting     */
    LL_ADC_INJ_SetSequencerRanks(pHandle->pParams_str->ADCx_Inj, LL_ADC_INJ_RANK_1, pHandle->pParams_str->hIChannel);
    LL_ADC_INJ_SetSequencerRanks(pHandle->pParams_str->ADCx_Inj, LL_ADC_INJ_RANK_2, pHandle->pParams_str->hIChannel);
    LL_ADC_INJ_SetSequencerLength(pHandle->pParams_str->ADCx_Inj, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
    /*****************************************/
    
    BB_REG_BIT_CLR ( &pHandle->pParams_str->ADCx_Inj->CR2, ADC_CR2_JEXTEN_Pos );
    LL_ADC_EnableIT_JEOS ( pHandle->pParams_str->ADCx_Inj );

    pHandle->OverCurrentFlag = false;
    pHandle->_Super.DTTest = 0u;
  }
}

/**
  * @brief  It initializes TIMx and TIMx_2 peripheral for PWM generation,
  *          active vector insertion and adc triggering.
  * @param  TIMx Timer to be initialized
  * @param  TIMx_2 Auxiliary timer to be initialized used for adc triggering
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F4XX_TIMxInit( TIM_TypeDef * TIMx, TIM_TypeDef * TIMx_2, PWMC_Handle_t * pHdl )
{

  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  /* disable main and auxiliary TIM counters to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter( TIMx );
  LL_TIM_DisableCounter( TIMx_2 );

  if ( ( pHandle->pParams_str->EmergencyStop ) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK( TIMx );
    LL_TIM_EnableIT_BRK( TIMx );
  }

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE( TIMx );
  LL_TIM_GenerateEvent_UPDATE( TIMx_2 );

  if ( pHandle->pParams_str->bFreqRatio == 2u )
  {
    if ( pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->RepetitionCounter == 3u )
      {
        /* Set TIM1 repetition counter to 1 */
        TIMx->RCR = 0x01u;
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        TIMx->RCR = 0x03u;
      }
    }
  }

  if ( ( pHandle->_Super.Motor == M1 ) || ( pHandle->pParams_str->bFreqRatio == 2u ) )
  {
    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
    LL_TIM_SetCounter( TIMx_2, ( uint32_t )( pHandle->Half_PWMPeriod / 2u ) );
  }

  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

  if ( TIMx_2 == TIM5 )
  {
    LL_TIM_OC_DisablePreload( TIMx_2, LL_TIM_CHANNEL_CH4 );
  }
  if ( TIMx_2 == TIM4 )
  {
    LL_TIM_OC_DisablePreload( TIMx_2, LL_TIM_CHANNEL_CH2 );
  }

  LL_TIM_CC_EnableChannel ( TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
  {
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N );
  }

  /* Enable PWM channel */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

}

/**
  * @brief  It stores into the component's handle the voltage present on the
  *         current feedback analog channel when no current is flowin into the
  *         motor
  * @param  pHandle handler of the current instance of the PWM component
  */
__weak void R1F4XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint16_t htempCCER, haux;

  pHandle->wPhaseOffset = 0u;

  pHandle->bIndex = 0u;

  /* Force inactive level on TIMx CHy and TIMx CHyN */
  htempCCER =  TIMx->CCER;
  haux = htempCCER & TIMxCCER_MASK;
  haux |= TIMx_CC4E_BIT;
  TIMx->CCER = haux;
  LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R1F4XX_HFCurrentsCalibration;

  R1F4XX_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd( TIMx,
  		                  &pHandle->_Super.SWerror,
  						  pHandle->pParams_str->RepetitionCounter,
  						  &pHandle->bIndex );

  R1F4XX_SwitchOffPWM( &pHandle->_Super );

  pHandle->wPhaseOffset = pHandle->wPhaseOffset / NB_CONVERSIONS;
  pHandle->wPhaseOffset <<= 1;

  /* Set back TIMx CCER register */
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R1F4XX_GetPhaseCurrents;
}

/**
  * @brief  First initialization of class members
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F4XX_1ShuntMotorVarsInit( PWMC_Handle_t * pHdl )
{
  uint16_t hAux;

  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  /* Init motor vars */
  pHandle->wPhaseOffset = 0u;
  pHandle->bInverted_pwm = INVERT_NONE;
  pHandle->bInverted_pwm_new = INVERT_NONE;
  pHandle->hFlags &= ( ~STBD3 );
  pHandle->hFlags &= ( ~DSTEN );

  /* After reset value of DMA buffers */
  pHandle->hDmaBuff[0] = pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] = pHandle->Half_PWMPeriod >> 1;

  /* After reset value of dvDutyValues */
  pHandle->_Super.CntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhC = pHandle->Half_PWMPeriod >> 1;

  /* Default value of DutyValues */
  pHandle->hCntSmp1 = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = ( pHandle->Half_PWMPeriod >> 1 ) + pHandle->pParams_str->hTafter;

  /* Default value of sampling point */
  pHandle->hCCDmaBuffCh4[0] = ( uint32_t )( pHandle->hCntSmp2 ) >> 1u; /*  Second point */
  hAux = pHandle->Half_PWMPeriod - 1u;
  pHandle->hCCDmaBuffCh4[1] = ( uint32_t )( hAux ); /* Update */
  pHandle->hCCDmaBuffCh4[2] = ( uint32_t )( pHandle->hCntSmp1 ) >> 1u; /* First point */

  LL_TIM_DisableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
}

/**
  * @brief  Initialization of class members after each motor start
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F4XX_1ShuntMotorVarsRestart( PWMC_Handle_t * pHdl )
{

  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  /* Default value of DutyValues */
  pHandle->hCntSmp1 = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = ( pHandle->Half_PWMPeriod >> 1 ) + pHandle->pParams_str->hTafter;

  /* Default value of sampling point */
  pHandle->hCCDmaBuffCh4[0] = ( uint32_t )( pHandle->hCntSmp2 ) >> 1u; /*  Second point */
  pHandle->hCCDmaBuffCh4[2] = ( uint32_t )( pHandle->hCntSmp1 ) >> 1u; /* First point */

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

  LL_TIM_DisableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param pHandle: handler of the current instance of the PWM component
  * @retval ab_t Ia and Ib current in ab_t format
  */
__weak void R1F4XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  int32_t wAux;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC = 0;
  uint8_t bCurrASamp = 0u;
  uint8_t bCurrBSamp = 0u;
  uint8_t bCurrCSamp = 0u;

  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;


  /* Disabling the Injectec conversion for ADCx after EOC*/
  BB_REG_BIT_CLR ( &pHandle->pParams_str->ADCx_Inj->CR2, ADC_CR2_JEXTEN_Pos );

  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pHandle->hFlags &= ( ~SOFOC );

  /* First sampling point */
  wAux = ( int32_t )( pHandle->pParams_str->ADCx_Inj->JDR1 );
  wAux *= 2;
  wAux -= ( int32_t )( pHandle->wPhaseOffset );

  /* Check saturation */
  wAux = ( wAux > -INT16_MAX ) ? ( ( wAux < INT16_MAX ) ? wAux : INT16_MAX ) : -INT16_MAX;

  switch ( pHandle->sampCur1 )
  {
    case SAMP_IA:
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_IB:
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_IC:
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    case SAMP_NIA:
      wAux = -wAux;
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_NIB:
      wAux = -wAux;
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_NIC:
      wAux = -wAux;
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    case SAMP_OLDA:
      hCurrA = pHandle->hCurrAOld;
      bCurrASamp = 1u;
      break;
    case SAMP_OLDB:
      hCurrB = pHandle->hCurrBOld;
      bCurrBSamp = 1u;
      break;
    default:
      break;
  }

  /* Second sampling point */
  wAux = ( int32_t )( pHandle->pParams_str->ADCx_Inj->JDR2 );
  wAux *= 2;
  wAux -= ( int32_t )( pHandle->wPhaseOffset );

  /* Check saturation */
  wAux = ( wAux > -INT16_MAX ) ? ( ( wAux < INT16_MAX ) ? wAux : INT16_MAX ) : -INT16_MAX;

  switch ( pHandle->sampCur2 )
  {
    case SAMP_IA:
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_IB:
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_IC:
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    case SAMP_NIA:
      wAux = -wAux;
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_NIB:
      wAux = -wAux;
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_NIC:
      wAux = -wAux;
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    default:
      break;
  }

  /* Computation of the third value */
  if ( bCurrASamp == 0u )
  {
    wAux = -( ( int32_t )( hCurrB ) ) - ( ( int32_t )( hCurrC ) );

    /* Check saturation */
    wAux = ( wAux > -INT16_MAX ) ? ( ( wAux < INT16_MAX ) ? wAux : INT16_MAX ) : -INT16_MAX;

    hCurrA = ( int16_t )wAux;
  }
  if ( bCurrBSamp == 0u )
  {
    wAux = -( ( int32_t )( hCurrA ) ) - ( ( int32_t )( hCurrC ) );

    /* Check saturation */
    wAux = ( wAux > -INT16_MAX ) ? ( ( wAux < INT16_MAX ) ? wAux : INT16_MAX ) : -INT16_MAX;

    hCurrB = ( int16_t )wAux;
  }
  if ( bCurrCSamp == 0u )
  {
    wAux = -( ( int32_t )( hCurrA ) ) - ( ( int32_t )( hCurrB ) );

    /* Check saturation */
    wAux = ( wAux > -INT16_MAX ) ? ( ( wAux < INT16_MAX ) ? wAux : INT16_MAX ) : -INT16_MAX;

    hCurrC = ( int16_t )wAux;
  }

  /* hCurrA, hCurrB, hCurrC values are the sampled values */

  pHandle->hCurrAOld = hCurrA;
  pHandle->hCurrBOld = hCurrB;
  pHandle->hCurrCOld = hCurrC;

  pStator_Currents->a = hCurrA;
  pStator_Currents->b = hCurrB;

  pHandle->_Super.Ia = pStator_Currents->a;
  pHandle->_Super.Ib = pStator_Currents->b;
  pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;
}

/**
  * @brief  It sum up injected conversion data into wPhaseOffset. It is called
  *         only during current calibration
  * @param pHandle: handler of the current instance of the PWM component
  * @retval ab_t It always returns {0,0} in ab_t format
  */
static void R1F4XX_HFCurrentsCalibration( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  /* Disabling the Injectec conversion for ADCx after EOC*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pHandle->ADCx,DISABLE);*/
  pHandle->pParams_str->ADCx_Inj->CR2 &= CR2_JEXTTRIG_Reset;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->hFlags &= ( ~SOFOC );

  if ( pHandle->bIndex < NB_CONVERSIONS )
  {
    pHandle->wPhaseOffset += pHandle->pParams_str->ADCx_Inj->JDR1;
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R1F4XX_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  pHandle->_Super.TurnOnLowSidesAction = true;

  pHandle->pParams_str->TIMx->CCR1 = 0u;
  pHandle->pParams_str->TIMx->CCR2 = 0u;
  pHandle->pParams_str->TIMx->CCR3 = 0u;

  LL_TIM_ClearFlag_UPDATE ( pHandle->pParams_str->TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs ( pHandle->pParams_str->TIMx );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin ( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin ( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin ( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  return;
}

/**
  * @brief  This function enables the update event and the single shunt distortion
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R1F4XX_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  uint16_t hAux;

  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  hAux = pHandle->Half_PWMPeriod >> 1u;
  pHandle->pParams_str->TIMx->CCR1 = ( uint32_t )( hAux );
  pHandle->pParams_str->TIMx->CCR2 = ( uint32_t )( hAux );
  pHandle->pParams_str->TIMx->CCR3 = ( uint32_t )( hAux );

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(pHandle->pParams_str->TIMx) == 0)
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);

  /* Enabling distortion for single shunt */
  pHandle->hFlags |= DSTEN;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( pHandle->pParams_str->TIMx );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( ( pHandle->pParams_str->TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin ( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin ( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin ( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin ( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin ( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin ( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }

  /* Enable UPDATE ISR */
  LL_TIM_EnableIT_UPDATE ( pHandle->pParams_str->TIMx );

  return;
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit, disables the single shunt distortion and reset the TIM status
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
__weak void R1F4XX_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  uint16_t hAux;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE ( pHandle->pParams_str->TIMx );

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs ( pHandle->pParams_str->TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin ( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin ( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin ( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  /* Disabling distortion for single */
  pHandle->hFlags &= ( ~DSTEN );

  /* Disabling all DMA previous setting */
  LL_TIM_DisableDMAReq_CC4 ( pHandle->pParams_str->TIMx );

  /* Set all duty to 50% */
  hAux = pHandle->Half_PWMPeriod >> 1u;
  pHandle->pParams_str->TIMx->CCR1 = ( uint32_t )( hAux );
  pHandle->pParams_str->TIMx->CCR2 = ( uint32_t )( hAux );
  pHandle->pParams_str->TIMx->CCR3 = ( uint32_t )( hAux );

  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);

  return;
}

/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs
  *         before the end of FOC algorithm else returns MC_NO_ERROR
  */
__weak uint16_t R1F4XX_CalcDutyCycles( PWMC_Handle_t * pHdl )
{
  TIM_TypeDef * TIMx;
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint16_t hAux;
  uint8_t bSector;
  uint8_t bStatorFluxPos;

  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  bSector = ( uint8_t )pHandle->_Super.Sector;

  if ( ( pHandle->hFlags & DSTEN ) != 0u )
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
    hDeltaDuty_0 = ( int16_t )( hDutyV_1 ) - ( int16_t )( hDutyV_0 );
    hDeltaDuty_1 = ( int16_t )( hDutyV_2 ) - ( int16_t )( hDutyV_1 );

    /* Check region */
    if ( ( uint16_t )hDeltaDuty_0 <= pHandle->pParams_str->hTMin )
    {
      if ( ( uint16_t )hDeltaDuty_1 <= pHandle->pParams_str->hTMin )
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
      if ( ( uint16_t )hDeltaDuty_1 > pHandle->pParams_str->hTMin )
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
          if ( pHandle->_Super.CntPhA - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin )
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.CntPhA -= pHandle->pParams_str->hHTMin;
            if ( pHandle->_Super.CntPhA < hDutyV_1 )
            {
              hDutyV_1 = pHandle->_Super.CntPhA;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ( ( pHandle->hFlags & STBD3 ) == 0u )
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.CntPhA -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.CntPhB -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags &= ( ~STBD3 );
            }
          }
          break;
        case SECTOR_2:
        case SECTOR_1:
          if ( pHandle->_Super.CntPhB - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin )
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.CntPhB -= pHandle->pParams_str->hHTMin;
            if ( pHandle->_Super.CntPhB < hDutyV_1 )
            {
              hDutyV_1 = pHandle->_Super.CntPhB;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ( ( pHandle->hFlags & STBD3 ) == 0u )
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.CntPhA -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.CntPhB -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags &= ( ~STBD3 );
            }
          }
          break;
        case SECTOR_4:
        case SECTOR_3:
          if ( pHandle->_Super.CntPhC - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin )
          {
            pHandle->bInverted_pwm_new = INVERT_C;
            pHandle->_Super.CntPhC -= pHandle->pParams_str->hHTMin;
            if ( pHandle->_Super.CntPhC < hDutyV_1 )
            {
              hDutyV_1 = pHandle->_Super.CntPhC;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ( ( pHandle->hFlags & STBD3 ) == 0u )
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.CntPhA -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.CntPhB -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags &= ( ~STBD3 );
            }
          }
          break;
        default:
          break;
      }
    }
    else if ( bStatorFluxPos == BOUNDARY_2 ) /* Adjust the middler */
    {
      switch ( bSector )
      {
        case SECTOR_4:
        case SECTOR_5: /* Invert B */
          pHandle->bInverted_pwm_new = INVERT_B;
          pHandle->_Super.CntPhB -= pHandle->pParams_str->hHTMin;
          if ( pHandle->_Super.CntPhB > 0xEFFFu )
          {
            pHandle->_Super.CntPhB = 0u;
          }
          break;
        case SECTOR_2:
        case SECTOR_3: /* Invert A */
          pHandle->bInverted_pwm_new = INVERT_A;
          pHandle->_Super.CntPhA -= pHandle->pParams_str->hHTMin;
          if ( pHandle->_Super.CntPhA > 0xEFFFu )
          {
            pHandle->_Super.CntPhA = 0u;
          }
          break;
        case SECTOR_6:
        case SECTOR_1: /* Invert C */
          pHandle->bInverted_pwm_new = INVERT_C;
          pHandle->_Super.CntPhC -= pHandle->pParams_str->hHTMin;
          if ( pHandle->_Super.CntPhC > 0xEFFFu )
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
      if ( ( pHandle->hFlags & STBD3 ) == 0u )
      {
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.CntPhA -= pHandle->pParams_str->hHTMin;
        pHandle->hFlags |= STBD3;
      }
      else
      {
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.CntPhB -= pHandle->pParams_str->hHTMin;
        pHandle->hFlags &= ( ~STBD3 );
      }
    }

    if ( bStatorFluxPos == REGULAR ) /* Regular zone */
    {
      /* First point */
      if ( ( hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime ) > pHandle->pParams_str->hMaxTrTs )
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      if ( ( hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime ) > pHandle->pParams_str->hMaxTrTs )
      {
        pHandle->hCntSmp2 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp2 >>= 1;
      }
      else
      {
        pHandle->hCntSmp2 = hDutyV_2 - pHandle->pParams_str->hTbefore;
      }
    }

    if ( bStatorFluxPos == BOUNDARY_1 ) /* Two small, one big */
    {
      /* First point */
      if ( ( hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime ) > pHandle->pParams_str->hMaxTrTs )
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

    if ( bStatorFluxPos == BOUNDARY_2 ) /* Two big, one small */
    {
      /* First point */
      if ( ( hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime ) >= pHandle->pParams_str->hMaxTrTs )
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

    if ( bStatorFluxPos == BOUNDARY_3 )
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_0 - pHandle->pParams_str->hTbefore; /* Dummy trigger */
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
  if ( pHandle->bDMACur == 0u )
  {
    TIMx = pHandle->pParams_str->TIMx;

    /* Preload Enable */
    TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
    TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;

    TIMx->CCR1 = pHandle->_Super.CntPhA;
    TIMx->CCR2 = pHandle->_Super.CntPhB;
    TIMx->CCR3 = pHandle->_Super.CntPhC;

    /* Update ADC Trigger DMA buffer */
    hAux = pHandle->hCntSmp2 >> 1; /* Second point */
    pHandle->hCCDmaBuffCh4[0] = ( uint32_t )( hAux ); /* Second point */
    hAux = pHandle->hCntSmp1 >> 1; /* First point */
    pHandle->hCCDmaBuffCh4[2] = ( uint32_t )( hAux ); /* First point */
  }

  /* Limit for update event */

  /* Check the status of bSOFOC flags if is set the next update event has been
  occurred so an error will be reported*/
  if ( ( pHandle->hFlags & SOFOC ) != 0u )
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
  if ( bStatorFluxPos == REGULAR ) /* Regual zone */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }

  if ( bStatorFluxPos == BOUNDARY_1 ) /* Two small, one big */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }

  if ( bStatorFluxPos == BOUNDARY_2 ) /* Two big, one small */
  {
    pHandle->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }

  if ( bStatorFluxPos == BOUNDARY_3 )
  {
    if ( pHandle->bInverted_pwm_new == INVERT_A )
    {
      pHandle->sampCur1 = SAMP_OLDB;
      pHandle->sampCur2 = SAMP_IA;
    }
    if ( pHandle->bInverted_pwm_new == INVERT_B )
    {
      pHandle->sampCur1 = SAMP_OLDA;
      pHandle->sampCur2 = SAMP_IB;
    }
  }

  /* Limit for the Get Phase current (Second EOC Handler) */

  return ( hAux );
}

/**
 * @brief  It contains the TIM1 Update event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void * R1F4XX_TIM1_UP_IRQHandler( PWMC_R1_F4_Handle_t * pHandle )
{
  uint8_t bInverted_pwm_new;

  /* Critical point start */

  /* Update timer for first sampling */
  *( pHandle->pTIMx_2_CCR ) = pHandle->hCCDmaBuffCh4[2];

  /* Enabling the Injectec conversion for ADCx*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pHandle->ADCx,ENABLE); */
  BB_REG_BIT_SET ( &pHandle->pParams_str->ADCx_Inj->CR2, ADC_CR2_JEXTEN_Pos );

  /* Critical point stop */

  /* TMP var to speedup the execution */
  bInverted_pwm_new = pHandle->bInverted_pwm_new;

  if ( bInverted_pwm_new != pHandle->bInverted_pwm )
  {

    /* Set the DMA destination */
    switch ( bInverted_pwm_new )
    {
      case INVERT_A:
        /* Disable DMA to access PAR */
        DMA2_Stream4->CR &= ~( uint32_t )DMA_SxCR_EN;
        /* Check that the DMA is indeed disabled */
        while ( ( DMA2_Stream4->CR & DMA_SxCR_EN ) == DMA_SxCR_EN );
        LL_DMA_ClearFlag_TC4 ( DMA2 ); /* Clear TC flag set due DMA disabling */
        DMA2_Stream4->PAR = TIM1_CCR1_Address;
        DMA2_Stream4->CR |= ( uint32_t )DMA_SxCR_EN; /* Enable DMA */

        /* NVIC->ICPR[1] = 0x10000000;  Clear NVIC pending FLAG */
        LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;

      case INVERT_B:
        DMA2_Stream4->CR &= ~( uint32_t )DMA_SxCR_EN; /* Disable DMA to access PAR */
        /* Check that the DMA is indeed disabled */
        while ( ( DMA2_Stream4->CR & DMA_SxCR_EN ) == DMA_SxCR_EN );
        LL_DMA_ClearFlag_TC4 ( DMA2 ); /* Clear TC flag set due DMA disabling */
        DMA2_Stream4->PAR = TIM1_CCR2_Address;
        DMA2_Stream4->CR |= ( uint32_t )DMA_SxCR_EN; /* Enable DMA */
        /* NVIC->ICPR[1] = 0x10000000;  Clear NVIC pending FLAG */
        LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;

      case INVERT_C:
        DMA2_Stream4->CR &= ~( uint32_t )DMA_SxCR_EN; /* Disable DMA to access PAR */
        /* Check that the DMA is indeed disabled */
        while ( ( DMA2_Stream4->CR & DMA_SxCR_EN ) == DMA_SxCR_EN );
        LL_DMA_ClearFlag_TC4 ( DMA2 ); /* Clear TC flag set due DMA disabling */
        DMA2_Stream4->PAR = TIM1_CCR3_Address;
        DMA2_Stream4->CR |= ( uint32_t )DMA_SxCR_EN; /* Enable DMA */
        /* NVIC->ICPR[1] = 0x10000000;  Clear NVIC pending FLAG */
        LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;

      default:
        LL_TIM_DisableDMAReq_CC4 ( pHandle->pParams_str->TIMx );

        break;
    }
  }

  /* Clear of End of FOC Flags */
  pHandle->hFlags &= ( ~EOFOC );

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


  return &( pHandle->_Super.Motor );
}
#ifdef TIM8
/**
 * @brief  It contains the TIM8 Update event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void * R1F4XX_TIM8_UP_IRQHandler( PWMC_R1_F4_Handle_t * pHandle )
{
  uint8_t bInverted_pwm_new;

  /* Critical point start */

  /* Update timer for first sampling */
  *( pHandle->pTIMx_2_CCR ) = pHandle->hCCDmaBuffCh4[2];

  /* Enabling the Injectec conversion for ADCx*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pHandle->ADCx,ENABLE); */
  pHandle->pParams_str->ADCx_Inj->CR2 |= CR2_JEXTTRIG_Set;

  /* Critical point stop */

  /* TMP var to speedup the execution */
  bInverted_pwm_new = pHandle->bInverted_pwm_new;

  if ( bInverted_pwm_new != pHandle->bInverted_pwm )
  {
    /* Set the DMA destination */
    switch ( bInverted_pwm_new )
    {
      case INVERT_A:
        /* Disable DMA to access PAR */
        DMA2_Stream7->CR &= ~( uint32_t )DMA_SxCR_EN;
        /* Check that the DMA is indeed disabled */
        while ( ( DMA2_Stream7->CR & ( uint32_t )DMA_SxCR_EN ) == ( uint32_t )DMA_SxCR_EN );
        LL_DMA_ClearFlag_TC7 ( DMA2 ); /* Clear TC flag set due DMA disabling */

        DMA2_Stream7->PAR = TIM8_CCR1_Address;

        DMA2_Stream7->CR |= ( uint32_t )DMA_SxCR_EN; /* Enable DMA */
        /* NVIC->ICPR[2] = 0x00000040;  Clear NVIC pending FLAG */
        LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;

      case INVERT_B:
        /* Disable DMA to access PAR */
        DMA2_Stream7->CR &= ~( uint32_t )DMA_SxCR_EN;
        /* Check that the DMA is indeed disabled */
        while ( ( DMA2_Stream7->CR & ( uint32_t )DMA_SxCR_EN ) == ( uint32_t )DMA_SxCR_EN );
        LL_DMA_ClearFlag_TC7 ( DMA2 ); /* Clear TC flag set due DMA disabling */

        DMA2_Stream7->PAR = TIM8_CCR2_Address;

        DMA2_Stream7->CR |= ( uint32_t )DMA_SxCR_EN; /* Enable DMA */
        /* NVIC->ICPR[2] = 0x00000040;  Clear NVIC pending FLAG */
        LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;

      case INVERT_C:
        /* Disable DMA to access PAR */
        DMA2_Stream7->CR &= ~( uint32_t )DMA_SxCR_EN;
        /* Check that the DMA is indeed disabled */
        while ( ( DMA2_Stream7->CR & ( uint32_t )DMA_SxCR_EN ) == ( uint32_t )DMA_SxCR_EN );
        LL_DMA_ClearFlag_TC7 ( DMA2 ); /* Clear TC flag set due DMA disabling */

        DMA2_Stream7->PAR = TIM8_CCR3_Address;

        DMA2_Stream7->CR |= ( uint32_t )DMA_SxCR_EN; /* Enable DMA */
        /* NVIC->ICPR[2] = 0x00000040;  Clear NVIC pending FLAG */
        LL_TIM_EnableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;

      default:
        LL_TIM_DisableDMAReq_CC4 ( pHandle->pParams_str->TIMx );
        break;
    }
  }

  /* Clear of End of FOC Flags */
  pHandle->hFlags &= ( ~EOFOC );

  /* Preload Disable */
  TIM8->CCMR1 &= CCMR1_PRELOAD_DISABLE_MASK;
  TIM8->CCMR2 &= CCMR2_PRELOAD_DISABLE_MASK;

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

  return &( pHandle->_Super.Motor );
}
#endif /* TIM8 condition*/
/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void * R1F4XX_BRK_IRQHandler( PWMC_R1_F4_Handle_t * pHandle )
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
 * @brief  It contains the DMA transfer complete event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void * R1F4XX_DMA_TC_IRQHandler( PWMC_R1_F4_Handle_t * pHandle )
{
  uint16_t hAux;

  pHandle->bDMACur--;
  if ( pHandle->bDMACur == 0u )
  {
    if ( ( pHandle->hFlags & EOFOC ) != 0u )
    {
      /* Preload Enable */
      pHandle->pParams_str->TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
      pHandle->pParams_str->TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;

      /* Compare register update */
      pHandle->pParams_str->TIMx->CCR1 = pHandle->_Super.CntPhA;
      pHandle->pParams_str->TIMx->CCR2 = pHandle->_Super.CntPhB;
      pHandle->pParams_str->TIMx->CCR3 = pHandle->_Super.CntPhC;

      /* Update ADC Trigger DMA buffer */
      hAux = pHandle->hCntSmp2 >> 1; /* Second point */
      pHandle->hCCDmaBuffCh4[0] = ( uint32_t )( hAux ); /* Second point */
      hAux = pHandle->hCntSmp1 >> 1; /* First point */
      pHandle->hCCDmaBuffCh4[2] = ( uint32_t )( hAux ); /* First point */
    }
  }

  return &( pHandle->_Super.Motor );
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
__weak uint16_t R1F4XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F4_Handle_t * pHandle = ( PWMC_R1_F4_Handle_t * )pHdl;

  uint16_t retVal = MC_NO_FAULTS;
  if ( pHandle->OverCurrentFlag == true )
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
