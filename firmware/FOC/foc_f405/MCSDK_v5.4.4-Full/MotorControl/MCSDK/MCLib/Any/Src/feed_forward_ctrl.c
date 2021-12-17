/**
  ******************************************************************************
  * @file    feed_forward_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the Feed Forward
  *          Control component of the Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "feed_forward_ctrl.h"
#include <stddef.h>

#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"
#include "r_divider_bus_voltage_sensor.h"

extern RDivider_Handle_t * pBusSensorM1;

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup FeedForwardCtrl Feed Forward Control
  * @brief Feed Forward Control component of the Motor Control SDK
  *
  * @todo Document the Feed Forward Control "module".
  *
  * @{
  */

/* Private macros ------------------------------------------------------------*/
#define SEGMNUM (uint8_t)7 /* coeff no. -1 */
#define SATURATION_TO_S16(a)    if ((a) > 32767)                \
  {                               \
    (a) = 32767;                  \
  }                               \
  else if ((a) < -32767)          \
  {                               \
    (a) = -32767;                 \
  }                               \
  else                            \
  {}                              \




/**
  * @brief  Initializes all the component variables
  * @param  pHandle Feed forward init strutcture.
  * @param  pBusSensor VBus Sensor.
  * @param  pPIDId Id PID.
  * @param  pPIDIq Iq PID.
  * @retval none
  */
__weak void FF_Init( FF_Handle_t * pHandle, BusVoltageSensor_Handle_t * pBusSensor, PID_Handle_t * pPIDId,
              PID_Handle_t * pPIDIq )
{

  pHandle->wConstant_1D = pHandle->wDefConstant_1D;
  pHandle->wConstant_1Q = pHandle->wDefConstant_1Q;
  pHandle->wConstant_2  = pHandle->wDefConstant_2;

  pHandle->pBus_Sensor = pBusSensor;

  pHandle->pPID_d = pPIDId;

  pHandle->pPID_q = pPIDIq;
}

/**
  * @brief  It should be called before each motor restart and clears the Flux
  *         weakening internal variables.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
__weak void FF_Clear( FF_Handle_t * pHandle )
{
  pHandle->Vqdff.q = ( int16_t )0;
  pHandle->Vqdff.d = ( int16_t )0;

  return;
}

/**
  * @brief  It implements feed-forward controller by computing new Vqdff value.
  *         This will be then summed up to PI output in IMFF_VqdConditioning
  *         method.
  * @param  pHandle Feed forward  strutcture.
  * @param  Iqdref Idq reference componets used to calcupate the feed forward
  *         action.
  * @param  pSTC  Speed sensor.
  * @retval none
  */
__weak void FF_VqdffComputation( FF_Handle_t * pHandle, qd_t Iqdref, SpeednTorqCtrl_Handle_t * pSTC )
{
  int32_t wtemp1, wtemp2;
  int16_t hSpeed_dpp;
  uint16_t hAvBusVoltage_d;
  SpeednPosFdbk_Handle_t * SpeedSensor;

  SpeedSensor = STC_GetSpeedSensor( pSTC );
  hSpeed_dpp = SPD_GetElSpeedDpp( SpeedSensor );
  hAvBusVoltage_d = VBS_GetAvBusVoltage_d( &( pBusSensorM1->_Super ) ) / 2u;

  if (hAvBusVoltage_d != (uint16_t)0)
  {
    /*q-axes ff voltage calculation */
    wtemp1 = ( ( ( int32_t )( hSpeed_dpp ) * Iqdref.d ) / ( int32_t )32768 );
    wtemp2 = ( wtemp1 * pHandle->wConstant_1D ) / ( int32_t )( hAvBusVoltage_d );
    wtemp2 *= ( int32_t )2;

    wtemp1 = ( ( pHandle->wConstant_2 * hSpeed_dpp ) / ( int32_t )hAvBusVoltage_d )
                   * ( int32_t )16;

    wtemp2 = wtemp1 + wtemp2 + pHandle->VqdAvPIout.q;

    SATURATION_TO_S16( wtemp2 )

    pHandle->Vqdff.q = ( int16_t )( wtemp2 );

    /* d-axes ff voltage calculation */
    wtemp1 = ( ( ( int32_t )( hSpeed_dpp ) * Iqdref.q ) / ( int32_t )32768 );
    wtemp2 = ( wtemp1 * pHandle->wConstant_1Q ) / ( int32_t )( hAvBusVoltage_d );
    wtemp2 *= ( int32_t )2;

    wtemp2 = ( int32_t )pHandle->VqdAvPIout.d - wtemp2;

    SATURATION_TO_S16( wtemp2 )

    pHandle->Vqdff.d = ( int16_t )( wtemp2 );
  }
  else
  {
    pHandle->Vqdff.q = ( int16_t ) 0;
    pHandle->Vqdff.d = ( int16_t ) 0;
  }
}

#if defined (__ICCARM__)
  // false positive  
  #pragma cstat_disable = "MISRAC2012-Rule-2.2_b"  
#endif /* __ICCARM__ */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It return the Vqd componets fed in input plus the feed forward
  *         action and store the last Vqd values in the internal variable.
  * @param  pHandle Feed forward  strutcture.
  * @param  Vqd Initial value of Vqd to be manipulated by FF .
  * @retval qd_t Vqd conditioned values.
  */
__weak qd_t FF_VqdConditioning( FF_Handle_t * pHandle, qd_t Vqd )
{
  int32_t wtemp;
  qd_t lVqd;

  pHandle->VqdPIout = Vqd;

  wtemp = ( int32_t )( Vqd.q ) + pHandle->Vqdff.q;

  SATURATION_TO_S16( wtemp )

  lVqd.q= ( int16_t )wtemp;

  wtemp = ( int32_t )( Vqd.d ) + pHandle->Vqdff.d;

  SATURATION_TO_S16( wtemp )

  lVqd.d = ( int16_t )wtemp;

  return ( lVqd );
}
#if defined (__ICCARM__)
  // false positive  
  #pragma cstat_restore = "MISRAC2012-Rule-2.2_b"  
#endif /* __ICCARM__ */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It low-pass filters the Vqd voltage coming from the speed PI. Filter
  *         bandwidth depends on hVqdLowPassFilterBW parameter.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
__weak void FF_DataProcess( FF_Handle_t * pHandle )
{
  int32_t wAux;
  int32_t lowPassFilterBW = ( int32_t ) pHandle->hVqdLowPassFilterBW - ( int32_t )1;
  
#ifdef FULL_MISRA_C_COMPLIANCY
  /* Computation of average Vqd as output by PI(D) current controllers, used by
     feed-forward controller algorithm */
  wAux = ( int32_t )( pHandle->VqdAvPIout.q ) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.q;

  pHandle->VqdAvPIout.q = ( int16_t )( wAux /
                                      ( int32_t )( pHandle->hVqdLowPassFilterBW ) );

  wAux = ( int32_t )( pHandle->VqdAvPIout.d ) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.d;

  pHandle->VqdAvPIout.d = ( int16_t )( wAux /
                                      ( int32_t )( pHandle->hVqdLowPassFilterBW ) );
#else
  /* Computation of average Vqd as output by PI(D) current controllers, used by
     feed-forward controller algorithm */
  wAux = ( int32_t )( pHandle->VqdAvPIout.q ) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.q;
  pHandle->VqdAvPIout.q = ( int16_t ) ( wAux >>
                                        pHandle->hVqdLowPassFilterBWLOG );

  wAux = ( int32_t )( pHandle->VqdAvPIout.d ) * lowPassFilterBW;
  wAux += pHandle->VqdPIout.d;
  pHandle->VqdAvPIout.d = ( int16_t )  ( wAux >>
                                         pHandle->hVqdLowPassFilterBWLOG );

#endif
}

/**
  * @brief  Use this method to initialize FF vars in START_TO_RUN state.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
__weak void FF_InitFOCAdditionalMethods( FF_Handle_t * pHandle )
{
  pHandle->VqdAvPIout.q = 0;
  pHandle->VqdAvPIout.d = 0;
  PID_SetIntegralTerm( pHandle->pPID_q, 0 );
  PID_SetIntegralTerm( pHandle->pPID_d, 0 );
}

/**
  * @brief  Use this method to set new values for the constants utilized by
  *         feed-forward algorithm.
  * @param  pHandle Feed forward  strutcture.
  * @param  sNewConstants The FF_TuningStruct_t containing constants utilized by
  *         feed-forward algorithm.
  * @retval none
  */
__weak void FF_SetFFConstants( FF_Handle_t * pHandle, FF_TuningStruct_t sNewConstants )
{
  pHandle->wConstant_1D = sNewConstants.wConst_1D;
  pHandle->wConstant_1Q = sNewConstants.wConst_1Q;
  pHandle->wConstant_2  = sNewConstants.wConst_2;
}

#if defined (__ICCARM__)
  // false positive  
  #pragma cstat_disable = "MISRAC2012-Rule-2.2_b"  
#endif /* __ICCARM__ */

/**
  * @brief  Use this method to get present values for the constants utilized by
  *         feed-forward algorithm.
  * @param  pHandle Feed forward  strutcture.
  * @retval FF_TuningStruct_t Values of the constants utilized by
  *         feed-forward algorithm.
  */
__weak FF_TuningStruct_t FF_GetFFConstants( FF_Handle_t * pHandle )
{
  FF_TuningStruct_t LocalConstants;

  LocalConstants.wConst_1D = pHandle->wConstant_1D;
  LocalConstants.wConst_1Q = pHandle->wConstant_1Q;
  LocalConstants.wConst_2 =  pHandle->wConstant_2;

  return ( LocalConstants );
}
#if defined (__ICCARM__)
  // false positive  
  #pragma cstat_restore = "MISRAC2012-Rule-2.2_b"  
#endif /* __ICCARM__ */

/**
  * @brief  Use this method to get present values for the Vqd feed-forward
  *         components.
  * @param  pHandle Feed forward  strutcture.
  * @retval qd_t Vqd feed-forward components.
  */
__weak qd_t FF_GetVqdff( const FF_Handle_t * pHandle )
{
  return ( pHandle->Vqdff );
}

/**
  * @brief  Use this method to get values of the averaged output of qd axes
  *         currents PI regulators.
  * @param  pHandle Feed forward  strutcture.
  * @retval qd_t Averaged output of qd axes currents PI regulators.
  */
__weak qd_t FF_GetVqdAvPIout( const FF_Handle_t * pHandle )
{
  return ( pHandle->Vqdff );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
