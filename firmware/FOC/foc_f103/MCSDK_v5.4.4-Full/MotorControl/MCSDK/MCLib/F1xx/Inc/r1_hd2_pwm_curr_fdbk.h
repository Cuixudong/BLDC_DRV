/**
  ******************************************************************************
  * @file    r1_hd2_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r1_hd2_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r1_hd2_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R1_HD2_PWM_CURR_FDBK_H
#define __R1_HD2_PWM_CURR_FDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "r1_dd_pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r1_hd2_pwm_curr_fdbk
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @{ */
#define EOFOC 0x0001u /**< Flag to indicate end of FOC duty available */
#define STBD3 0x0002u /**< Flag to indicate which phase has been distorted
                           in boundary 3 zone (A or B)*/
#define DSTEN 0x0004u /**< Flag to indicate if the distortion must be performed
                           or not (in case of charge of bootstrap capacitor phase
                           is not required)*/
#define SOFOC 0x0008u /**< This flag will be reset to zero at the begin of FOC
                           and will be set in the UP IRQ. If at the end of
                           FOC it is set the software error must be generated*/
/** @} */

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief The PWMC_R1_HD2_Handle_t structure defines the handle of PWM & Current Feedback
  *        component designed for STM32F103 High Density with 1 shunt current sensing topology.
  *
  * The design of the PWMC_R1_HD2 component is based on that of the PWMC generic component.
  */
typedef struct
{
	PWMC_Handle_t _Super;       /**< The handle on the base PWMC component. */
  uint32_t ADCTrigger;       /**< ADC trigger selection */
  uint32_t PhaseOffset;      /**< Offset of Phase current sensing network  */
  uint16_t Half_PWMPeriod;    /**< Half PWM Period in timer clock counts */
	uint16_t DmaBuff[2];       /**< Buffer used for PWM distortion points*/
	uint16_t CCDmaBuffCh4[4];  /**< Buffer used for dual ADC sampling points*/
	uint16_t CntSmp1;          /**< First sampling point express in timer counts*/
	uint16_t CntSmp2;          /**< Second sampling point express in timer counts*/
	uint8_t sampCur1;           /**< Current sampled in the first sampling point*/
	uint8_t sampCur2;           /**< Current sampled in the second sampling point*/
	int16_t CurrAOld;          /**< Previous measured value of phase A current*/
	int16_t CurrBOld;          /**< Previous measured value of phase B current*/
	int16_t CurrCOld;          /**< Previous measured value of phase C current*/
  uint16_t Flags;            /**< Internal Flags used for operation.
                                   EOFOC: Flag to indicate end of FOC duty available
                                   STBD3: Flag to indicate which phase has been distorted
                                          in boudary 3 zone (A or B)
                                   DSTEN: Flag to indicate if the distortion must be
                                          performed or not (charge of bootstrap
                                          capacitor phase)
                                   SOFOC: This flag will be reset to zero at the begin of FOC
                                          and will be set in the UP IRQ. If at the end of
                                          FOC it is set the software error must be generated*/
	uint8_t Inverted_pwm;      /**< This value indicates the type of the previous
                                     PWM period (Regular, Distort PHA, PHB or PHC) */
	uint8_t Inverted_pwm_new;  /**< This value indicates the type of the current PWM period.
                                      PWM period (Regular, Distort PHA, PHB or PHC) */
	uint8_t DMATot;            /**< Value to indicate the total number of expected
                                     DMA TC events*/
	uint8_t DMACur;            /**< Current number of DMA TC events occurred */
	uint8_t  PolarizationCounter;   /**< Number of conversions performed during the calibration phase*/

  bool OverCurrentFlag;       /**< This flag is set when an over current occurs.*/

	R1_DDParams_t const * pParams_str; /**< Pointer on the parameters structure for the PWMC R1 HD2 component */
} PWMC_R1_HD2_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Initializes a PWMC_R1_HD2 component */
void R1HD2_Init( PWMC_R1_HD2_Handle_t * pHandle );

/* Switches on PWM generation */
void R1HD2_SwitchOnPWM( PWMC_Handle_t * pHandle );

/* Switches off PWM generation */
void R1HD2_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* Turns the low side switches on */
void R1HD2_TurnOnLowSides( PWMC_Handle_t * pHandle );

/* Returns the last phase currents values measured*/
void R1HD2_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t* pStator_Currents );

/* Handles the Timer Interrupts of a PWMC_R1_HD2 component */
void * R1HD2_TIM1_UP_IRQHandler( PWMC_R1_HD2_Handle_t * pHandle );
void * R1HD2_TIM8_UP_IRQHandler( PWMC_R1_HD2_Handle_t * pHandle );
void * R1HD2_DMA_TC_IRQHandler( PWMC_R1_HD2_Handle_t * pHandle );
/* Calibrates current reading for a PWMC_R1_HD2 component */
void R1HD2_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/* Computes the duty cycle for the next PWM period */
uint16_t R1HD2_CalcDutyCycles( PWMC_Handle_t * pHandle );

/* @brief  It contains the Break event interrupt */
void *R1HD2_BRK_IRQHandler(PWMC_R1_HD2_Handle_t *pHandle);

/* Returns whether an over current condition has occurred */
uint16_t R1HD2_IsOverCurrentOccurred( PWMC_Handle_t * pHandle );

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

#endif /* __R1_HD2_PWM_CURR_FDBK_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
