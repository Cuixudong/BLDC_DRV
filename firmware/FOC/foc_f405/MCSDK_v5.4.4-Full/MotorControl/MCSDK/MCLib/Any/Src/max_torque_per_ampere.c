/**
  ******************************************************************************
  * @file    max_torque_per_ampere.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Maximum Torque Per Ampere (MTPA) Control component of the Motor Control SDK:
  *
  *           * Initialize the parameter for  MTPA
  *           * Calculate and output id and iq reference based on torque input
  *           * Calculate and output id based on iq input
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
#include "max_torque_per_ampere.h"
#include <stdint.h>
/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MTPA Maximum Torque Per Ampere Control
  * @brief Maximum Torque Per Ampere (MTPA) Control component of the Motor Control SDK
  *
  *  The torque of the PMSM can be expressed with the equation shown below:
  *
  *  @f[
  *  T_e = \frac{3}{2}\times p \times \phi \times i_q + \frac{3}{2}\times p\times(L_d-L_q)\times i_q\times i_d
  *  @f]
  *
  *    When the motor is surface permanent magnet synchronous motor SPMSM the @f$ L_d @f$ and @f$ L_q @f$
  *  is almost same, so only the @f$ i_q @f$ can influence the torque. For internal permanent
  *  magnet synchronous motor the @f$ L_d @f$  is not equel to @f$ L_q @f$. Both @f$ i_d @f$ and @f$ i_q @f$ will influence
  *  the torque.
  *    The aim of the MTPA (maximum-torque-per-ampere) control is to calculate the
  *  reference currents (@f$ i_qs@f$ , @f$ i_ds@f$) which maximize the ratio between produced
  *  electro-magnetic torque and copper losses.
  *    Call this function after you get the @f$ I_q @f$  refernece.the input of this component
  * is the @f$ i_q @f$  reference, the output of this component is @f$ i_d @f$ reference.
  *
  * @{
  */

/**
  * @brief  Calculate and output the Id based on input Iq
  * @param  Param1: MTPA_Handle_t   *pHandle
  *         Param2: qd_t Iqdref. 
  *
  * @retval Id value
  */
__weak void MTPA_CalcCurrRefFromIq( MTPA_Handle_t * pHandle, qd_t *Iqdref )
{
  int32_t id;
  int16_t aux;
  int16_t iq;
  uint8_t segment;

  iq = ( (Iqdref->q < 0) ? ( -Iqdref->q ) : ( Iqdref->q ) ); /* Teref absolute value */

  aux = iq / pHandle->SegDiv;
  segment = ( uint8_t )aux;

  if ( segment > SEGMENT_NUM )
  {
    segment = SEGMENT_NUM;
  }
  
#if defined (__ICCARM__)
  /* index for array access is verified above */
  #pragma cstat_disable = "MISRAC2012-Rule-18.1_d"
#endif /* __ICCARM__ */
  
  id = (( pHandle->AngCoeff[segment] * iq ) / 32768) + pHandle->Offset[segment];

#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-18.1_d"
#endif /* __ICCARM__ */
  
  Iqdref->d = (int16_t)id;
  
  return;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
