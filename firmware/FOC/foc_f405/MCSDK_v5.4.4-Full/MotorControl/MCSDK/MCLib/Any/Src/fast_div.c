/**
  ******************************************************************************
  * @file    fast_div.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the Fast Division
  *          component of the Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "fast_div.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup FD STM32F0 Fast Division
  * @brief FD Fast Division component of the Motor Control SDK. Used for STM32F0 MCU only.
  *
  * Detailed documentation for the component.
  * @{
  */



/* Static functions prototypes -----------------------------------------------*/

static int32_t FD_abs( int32_t x );
static FD_FastDivMagicNumber_t FD_magic( int32_t d );
static int32_t FD_mulhs( uint32_t u, uint32_t v );



/* Functions ---------------------------------------------------------*/
/**
  * @brief  Software initialization of a Fast Div component
  * @param  pHandle pointer on the handle of the component to initialize.
  */
__weak void FD_Init( FastDiv_Handle_t * pHandle )
{
  uint8_t i;
  pHandle->fd_div_element = 0;

  for ( i = 0u; i < FD_MAX_FDIV; i++ )
  {
    pHandle->fd_m[i].M = 0;
    pHandle->fd_m[i].s = 0;
    pHandle->fd_m[i].d = 0;
  }
}

/**
  * @brief  Executes the fast division.
  *
  * Note: The first execution of new divider will take more time
  * and need to be done in low frequency low priority task.
  * Further execution will be fast if same divider is used.
  * See #FD_MAX_FDIV definition for the programmed buffer
  * length of dividers.
  *
  * @param  pHandle Pointer on the handle of the component
  * @param  n Numerator
  * @param  d Denominator
  * @retval resault of the integer division of n by d
  */
__weak int32_t FD_FastDiv( FastDiv_Handle_t * pHandle, int32_t n, int32_t d )
{
  int32_t qf = 0;
  int8_t i = 0, findAtElement = -1;
  FD_FastDivMagicNumber_t x;
  int8_t fd_div_element = pHandle->fd_div_element;
  uint32_t absN = (n>0)?n:-n;

  if ( d == 0 )
  {
    return 0; /* Division by zero */
  }

  if ( d == 1 )
  {
    return n; /* Division by 1 */
  }
  
  for ( i = 0; i < FD_MAX_FDIV; i++ )
  {
    if ( d == pHandle->fd_m[i].d )
    {
      findAtElement = i;
      break;
    }
  }

  if ( findAtElement == -1 )
  {
    x = FD_magic( d );
    pHandle->fd_m[fd_div_element] = x;

    fd_div_element++;
    if ( fd_div_element >= FD_MAX_FDIV )
    {
      fd_div_element = 0;
    }
    pHandle->fd_div_element = fd_div_element;
  }
  else
  {
    x = pHandle->fd_m[findAtElement];
  }
  
  qf = FD_mulhs( absN, x.M );
  qf >>= x.s;
  return (n>0)?qf:-qf;
}

/**
  * @brief  Absolute value of int32_t
  * @param  int32_t Input integer.
  * @retval int32_t Absolute value of input integer.
  */
static inline int32_t FD_abs( int32_t x )
{
  int32_t r;
  if ( x < 0 )
    r = -x;
  else
    r = x;
  return r;
}

/**
  * @brief  Computation of magic number used for fast software division.
  * @param  int32_t Divider.
  * @retval int32_t It return the structure with the computed magic number.
  */
static FD_FastDivMagicNumber_t FD_magic( int32_t d )
{
  int32_t p;
  uint32_t ad, anc, delta, q1, r1, q2, r2, t;
  const uint32_t two31 = 0x80000000;

  FD_FastDivMagicNumber_t retVal;
  retVal.d = d;

  ad = FD_abs( d );
  t = two31 + ( ( uint32_t )d >> 31 );
  anc = t - 1 - t % ad;

  p = 31;
  q1 = two31 / anc;
  r1 = two31 - q1 * anc;
  q2 = two31 / ad;
  r2 = two31 - q2 * ad;

#if defined(STM32_PROTECTED)
  FSgma();
#endif

  do
  {
    p = p + 1;
    q1 = 2 * q1;
    r1 = 2 * r1;
    if ( r1 >= anc )
    {
      q1 = q1 + 1;
      r1 = r1 - anc;
    }
    q2 = 2 * q2;
    r2 = 2 * r2;

    if ( r2 >= ad )
    {
      q2 = q2 + 1;
      r2 = r2 - ad;
    }
    delta = ad - r2;
  }
  while ( q1 < delta || ( q1 == delta && r1 == 0 ) );
  retVal.M = q2 + 1;
  if ( d < 0 )
  {
    retVal.M = -retVal.M;
  }
  retVal.s = p - 32;

  return retVal;
}

/**
  * @brief  Multiplictaion between two 32 bit unsigned and
  *         keep the most significant 32bit.
  * @param  uint32_t First operand.
  * @param  uint32_t Second operand.
  * @retval int32_t It return the multiplictaion between the two inputs
  *         keeping the most significant 32bit.
  */
static int32_t FD_mulhs( uint32_t u, uint32_t v )
{
  uint32_t u0, v0, w0;
  uint32_t u1, v1, w1, w2, t;
  u0 = u & 0xFFFF;
  u1 = u >> 16;
  v0 = v & 0xFFFF;
  v1 = v >> 16;
  w0 = u0 * v0;
  t = u1 * v0 + ( w0 >> 16 );
  w1 = t & 0xFFFF;
  w2 = t >> 16;
  w1 = u0 * v1 + w1;
  return ( u1 * v1 + w2 + ( w1 >> 16 ) );
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
