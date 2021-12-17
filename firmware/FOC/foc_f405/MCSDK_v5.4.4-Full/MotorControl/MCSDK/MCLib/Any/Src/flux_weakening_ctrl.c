/**
  ******************************************************************************
  * @file    flux_weakening_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the Flux Weakening
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
#include "flux_weakening_ctrl.h"
#include "mc_math.h"

#include "mc_type.h"
#include "pid_regulator.h"


/** @addtogroup MCSDK
  * @{
  */

/** @defgroup FluxWeakeningCtrl Flux Weakening Control
  * @brief Flux Weakening Control Component component of the Motor Control SDK
  *
  * @todo Document the Flux Weakening Control "module".
  *
  * @{
  */

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param  pHandle Flux weakening init strutcture.
  * @param  pPIDSpeed Speed PID strutcture.
  * @param  PIDFluxWeakeningHandle FW PID strutcture.
  * @retval none.
  */
__weak void FW_Init( FW_Handle_t * pHandle, PID_Handle_t * pPIDSpeed, PID_Handle_t * pPIDFluxWeakeningHandle )
{
  pHandle->hFW_V_Ref = pHandle->hDefaultFW_V_Ref;

  pHandle->pFluxWeakeningPID = pPIDFluxWeakeningHandle;

  pHandle->pSpeedPID = pPIDSpeed;
}

/**
  * @brief  It should be called before each motor restart and clears the Flux
  *         weakening internal variables with the exception of the target
  *         voltage (hFW_V_Ref).
  * @param  pHandle Flux weakening init strutcture.
  * @retval none
  */
__weak void FW_Clear( FW_Handle_t * pHandle )
{
  qd_t V_null = {( int16_t )0, ( int16_t )0};

  PID_SetIntegralTerm( pHandle->pFluxWeakeningPID, ( int32_t )0 );
  pHandle->AvVolt_qd = V_null;
  pHandle->AvVoltAmpl = ( int16_t )0;
  pHandle->hIdRefOffset = ( int16_t )0;
}

/**
  * @brief  It computes Iqdref according the flux weakening algorithm.  Inputs
  *         are the starting Iqref components.
  *         As soon as the speed increases beyond the nominal one, fluxweakening
  *         algorithm take place and handles Idref value. Finally, accordingly
  *         with new Idref, a new Iqref saturation value is also computed and
  *         put into speed PI.
  * @param  pHandle Flux weakening init strutcture.
  * @param  Iqdref The starting current components that have to be
  *         manipulated by the flux weakening algorithm.
  * @retval qd_t Computed Iqdref.
  */
__weak qd_t FW_CalcCurrRef( FW_Handle_t * pHandle, qd_t Iqdref )
{
  int32_t wIdRef, wIqSatSq, wIqSat, wAux1, wAux2;
  uint32_t wVoltLimit_Ref;
  int16_t hId_fw;

  /* Computation of the Id contribution coming from flux weakening algorithm */
  wVoltLimit_Ref = ( ( uint32_t )( pHandle->hFW_V_Ref ) * pHandle->hMaxModule )
                   / 1000u;
  wAux1 = ( int32_t )( pHandle->AvVolt_qd.q ) *
          pHandle->AvVolt_qd.q;
  wAux2 = ( int32_t )( pHandle->AvVolt_qd.d ) *
          pHandle->AvVolt_qd.d;
  wAux1 += wAux2;

  wAux1 = MCM_Sqrt( wAux1 );
  pHandle->AvVoltAmpl = ( int16_t )wAux1;

  /* Just in case sqrt rounding exceeded INT16_MAX */
  if ( wAux1 > INT16_MAX )
  {
    wAux1 = ( int32_t )INT16_MAX;
  }

  hId_fw = PI_Controller( pHandle->pFluxWeakeningPID, ( int32_t )wVoltLimit_Ref - wAux1 );

  /* If the Id coming from flux weakening algorithm (Id_fw) is positive, keep
  unchanged Idref, otherwise sum it to last Idref available when Id_fw was
  zero */
  if ( hId_fw >= ( int16_t )0 )
  {
    pHandle->hIdRefOffset = Iqdref.d;
    wIdRef = ( int32_t )Iqdref.d;
  }
  else
  {
    wIdRef = ( int32_t )pHandle->hIdRefOffset + hId_fw;
  }

  /* Saturate new Idref to prevent the rotor from being demagnetized */
  if ( wIdRef < pHandle->hDemagCurrent )
  {
    wIdRef =  pHandle->hDemagCurrent;
  }

  Iqdref.d = ( int16_t )wIdRef;

  /* New saturation for Iqref */
  wIqSatSq =  pHandle->wNominalSqCurr - wIdRef * wIdRef;
  wIqSat = MCM_Sqrt( wIqSatSq );

  /* Iqref saturation value used for updating integral term limitations of
  speed PI */
  wAux1 = wIqSat * ( int32_t )PID_GetKIDivisor( pHandle->pSpeedPID );

  PID_SetLowerIntegralTermLimit( pHandle->pSpeedPID, -wAux1 );
  PID_SetUpperIntegralTermLimit( pHandle->pSpeedPID, wAux1 );

  /* Iqref saturation value used for updating integral term limitations of
  speed PI */
  if ( Iqdref.q > wIqSat )
  {
    Iqdref.q = ( int16_t )wIqSat;
  }
  else if ( Iqdref.q < -wIqSat )
  {
    Iqdref.q = -( int16_t )wIqSat;
  }
  else
  {
  }

  return ( Iqdref );
}

#if defined (__ICCARM__)
  /* shift value is written during init and computed
     by MC Workbench */
  #pragma cstat_disable = "ATH-shift-bounds"    
#endif /* __ICCARM__ */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It low-pass filters both the Vqd voltage components. Filter
  *         bandwidth depends on hVqdLowPassFilterBW parameter
  * @param  pHandle Flux weakening init strutcture.
  * @param  Vqd Voltage componets to be averaged.
  * @retval none
  */
__weak void FW_DataProcess( FW_Handle_t * pHandle, qd_t Vqd )
{
  int32_t wAux;
  int32_t lowPassFilterBW = ( int32_t )( pHandle->hVqdLowPassFilterBW ) - ( int32_t )1 ;
  
#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t )( pHandle->AvVolt_qd.q ) * lowPassFilterBW;
  wAux += Vqd.q;

  pHandle->AvVolt_qd.q = ( int16_t )( wAux /
                                     ( int32_t )( pHandle->hVqdLowPassFilterBW ) );

  wAux = ( int32_t )( pHandle->AvVolt_qd.d ) * lowPassFilterBW;
  wAux += Vqd.d;

  pHandle->AvVolt_qd.d = ( int16_t )( wAux /
                                     ( int32_t )pHandle->hVqdLowPassFilterBW );
#else
  wAux = ( int32_t )( pHandle->AvVolt_qd.q ) * lowPassFilterBW;
  wAux += Vqd.q;

  pHandle->AvVolt_qd.q = ( int16_t )( wAux >>
                                      pHandle->hVqdLowPassFilterBWLOG );
  
  wAux = ( int32_t )( pHandle->AvVolt_qd.d ) * lowPassFilterBW;
  wAux += Vqd.d;
  pHandle->AvVolt_qd.d = ( int16_t )( wAux >>
                                      pHandle->hVqdLowPassFilterBWLOG );
  
#endif
  return;
}

#if defined (__ICCARM__)
  /* shift value is written during init and computed
     by MC Workbench */
  #pragma cstat_enable = "ATH-shift-bounds"  
#endif /* __ICCARM__ */

/**
  * @brief  Use this method to set a new value for the voltage reference used by
  *         flux weakening algorithm.
  * @param  pHandle Flux weakening init strutcture.
  * @param  uint16_t New target voltage value, expressend in tenth of percentage
  *         points of available voltage.
  * @retval none
  */
__weak void FW_SetVref( FW_Handle_t * pHandle, uint16_t hNewVref )
{
  pHandle->hFW_V_Ref = hNewVref;
}

/**
  * @brief  It returns the present value of target voltage used by flux
  *         weakening algorihtm.
  * @param  pHandle Flux weakening init strutcture.
  * @retval int16_t Present target voltage value expressed in tenth of
  *         percentage points of available voltage.
  */
__weak uint16_t FW_GetVref( FW_Handle_t * pHandle )
{
  return ( pHandle->hFW_V_Ref );
}

/**
  * @brief  It returns the present value of voltage actually used by flux
  *         weakening algorihtm.
  * @param  pHandle Flux weakening init strutcture.
  * @retval int16_t Present averaged phase stator voltage value, expressed
  *         in s16V (0-to-peak), where
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
  */
__weak int16_t FW_GetAvVAmplitude( FW_Handle_t * pHandle )
{
  return ( pHandle->AvVoltAmpl );
}

/**
  * @brief  It returns the measure of present voltage actually used by flux
  *         weakening algorihtm as percentage of available voltage.
  * @param  pHandle Flux weakening init strutcture.
  * @retval uint16_t Present averaged phase stator voltage value, expressed in
  *         tenth of percentage points of available voltage.
  */
__weak uint16_t FW_GetAvVPercentage( FW_Handle_t * pHandle )
{
  return ( uint16_t )( ( uint32_t )( pHandle->AvVoltAmpl ) * 1000u /
                       ( uint32_t )( pHandle->hMaxModule ) );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
