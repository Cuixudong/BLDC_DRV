/**
  ******************************************************************************
  * @file    mc_math.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
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
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

/* Private macro -------------------------------------------------------------*/

#define SIN_COS_TABLE {\
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
    0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
    0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
    0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
    0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
    0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
    0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
    0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
    0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
    0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
    0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
    0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
    0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
    0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
    0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
    0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
    0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
    0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
    0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
    0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
    0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
    0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
    0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
    0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
    0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
    0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
    0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
    0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
    0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
    0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE}

#define ATAN1DIV1     (int16_t)8192
#define ATAN1DIV2     (int16_t)4836
#define ATAN1DIV4     (int16_t)2555
#define ATAN1DIV8     (int16_t)1297
#define ATAN1DIV16    (int16_t)651
#define ATAN1DIV32    (int16_t)326
#define ATAN1DIV64    (int16_t)163
#define ATAN1DIV128   (int16_t)81
#define ATAN1DIV256   (int16_t)41
#define ATAN1DIV512   (int16_t)20
#define ATAN1DIV1024  (int16_t)10
#define ATAN1DIV2048  (int16_t)5
#define ATAN1DIV4096  (int16_t)3
#define ATAN1DIV8192  (int16_t)1

#define SIN_MASK        0x0300u
#define U0_90           0x0200u
#define U90_180         0x0300u
#define U180_270        0x0000u
#define U270_360        0x0100u

/* Private variables ---------------------------------------------------------*/
const int16_t hSin_Cos_Table[256] = SIN_COS_TABLE;

#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315*/

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator values a and b (which are
  *         directed along axes each displaced by 120 degrees) into values
  *         alpha and beta in a stationary qd reference frame.
  *                               alpha = a
  *                       beta = -(2*b+a)/sqrt(3)
  * @param  Input: stator values a and b in ab_t format
  * @retval Stator values alpha and beta in alphabeta_t format
  */
__weak alphabeta_t MCM_Clarke( ab_t Input  )
{
  alphabeta_t Output;

  int32_t a_divSQRT3_tmp, b_divSQRT3_tmp ;
  int32_t wbeta_tmp;
  int16_t hbeta_tmp;

  /* qIalpha = qIas*/
  Output.alpha = Input.a;

  a_divSQRT3_tmp = divSQRT_3 * ( int32_t )Input.a;

  b_divSQRT3_tmp = divSQRT_3 * ( int32_t )Input.b;

  /*qIbeta = -(2*qIbs+qIas)/sqrt(3)*/
#ifdef FULL_MISRA_C_COMPLIANCY
  wbeta_tmp = ( -( a_divSQRT3_tmp ) - ( b_divSQRT3_tmp ) -
                 ( b_divSQRT3_tmp ) ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */

  wbeta_tmp = ( -( a_divSQRT3_tmp ) - ( b_divSQRT3_tmp ) -
                 ( b_divSQRT3_tmp ) ) >> 15;
#endif

  /* Check saturation of Ibeta */
  if ( wbeta_tmp > INT16_MAX )
  {
    hbeta_tmp = INT16_MAX;
  }
  else if ( wbeta_tmp < ( -32768 ) )
  {
    hbeta_tmp = ( -32768 );
  }
  else
  {
    hbeta_tmp = ( int16_t )( wbeta_tmp );
  }

  Output.beta = hbeta_tmp;

  if ( Output.beta == ( int16_t )( -32768 ) )
  {
    Output.beta = -32767;
  }

  return ( Output );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator values alpha and beta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as q and d.
  *                   d= alpha *sin(theta)+ beta *cos(Theta)
  *                   q= alpha *cos(Theta)- beta *sin(Theta)
  * @param  Input: stator values alpha and beta in alphabeta_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator values q and d in qd_t format
  */
__weak qd_t MCM_Park( alphabeta_t Input, int16_t Theta )
{
  qd_t Output;
  int32_t d_tmp_1, d_tmp_2, q_tmp_1, q_tmp_2;
  Trig_Components Local_Vector_Components;
  int32_t wqd_tmp;
  int16_t hqd_tmp;

  Local_Vector_Components = MCM_Trig_Functions( Theta );

  /*No overflow guaranteed*/
  q_tmp_1 = Input.alpha * ( int32_t )Local_Vector_Components.hCos;

  /*No overflow guaranteed*/
  q_tmp_2 = Input.beta * ( int32_t )Local_Vector_Components.hSin;

  /*Iq component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wqd_tmp = ( q_tmp_1 - q_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = ( q_tmp_1 - q_tmp_2 ) >> 15;
#endif

  /* Check saturation of Iq */
  if ( wqd_tmp > INT16_MAX )
  {
    hqd_tmp = INT16_MAX;
  }
  else if ( wqd_tmp < ( -32768 ) )
  {
    hqd_tmp = ( -32768 );
  }
  else
  {
    hqd_tmp = ( int16_t )( wqd_tmp );
  }

  Output.q = hqd_tmp;

  if ( Output.q == ( int16_t )( -32768 ) )
  {
    Output.q = -32767;
  }

  /*No overflow guaranteed*/
  d_tmp_1 = Input.alpha * ( int32_t )Local_Vector_Components.hSin;

  /*No overflow guaranteed*/
  d_tmp_2 = Input.beta * ( int32_t )Local_Vector_Components.hCos;

  /*Id component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wqd_tmp = ( d_tmp_1 + d_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = ( d_tmp_1 + d_tmp_2 ) >> 15;
#endif

  /* Check saturation of Id */
  if ( wqd_tmp > INT16_MAX )
  {
    hqd_tmp = INT16_MAX;
  }
  else if ( wqd_tmp < ( -32768 ) )
  {
    hqd_tmp = ( -32768 );
  }
  else
  {
    hqd_tmp = ( int16_t )( wqd_tmp );
  }

  Output.d = hqd_tmp;

  if ( Output.d == ( int16_t )( -32768 ) )
  {
    Output.d = -32767;
  }

  return ( Output );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Input: stator voltage Vq and Vd in qd_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator voltage Valpha and Vbeta in qd_t format
  */
__weak alphabeta_t MCM_Rev_Park( qd_t Input, int16_t Theta )
{
  int32_t alpha_tmp1, alpha_tmp2, beta_tmp1, beta_tmp2;
  Trig_Components Local_Vector_Components;
  alphabeta_t Output;

  Local_Vector_Components = MCM_Trig_Functions( Theta );

  /*No overflow guaranteed*/
  alpha_tmp1 = Input.q * ( int32_t )Local_Vector_Components.hCos;
  alpha_tmp2 = Input.d * ( int32_t )Local_Vector_Components.hSin;

#ifdef FULL_MISRA_C_COMPLIANCY
  Output.alpha = ( int16_t )( ( ( alpha_tmp1 ) + ( alpha_tmp2 ) ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  Output.alpha = ( int16_t )( ( ( alpha_tmp1 ) + ( alpha_tmp2 ) ) >> 15 );
#endif

  beta_tmp1 = Input.q * ( int32_t )Local_Vector_Components.hSin;
  beta_tmp2 = Input.d * ( int32_t )Local_Vector_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  Output.beta = ( int16_t )( ( beta_tmp2 - beta_tmp1 ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  Output.beta = ( int16_t )( ( beta_tmp2 - beta_tmp1 ) >> 15 );
#endif

  return ( Output );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Sin(angle) and Cos(angle) in Trig_Components format
  */

__weak Trig_Components MCM_Trig_Functions( int16_t hAngle )
{

  int32_t shindex;
  uint16_t uhindex;

  Trig_Components Local_Components;

  /* 10 bit index computation  */
  shindex = ( ( int32_t )32768 + ( int32_t )hAngle );
  uhindex = ( uint16_t )shindex;
  uhindex /= ( uint16_t )64;

  switch ( ( uint16_t )( uhindex ) & SIN_MASK )
  {
    case U0_90:
      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( uhindex )];
      Local_Components.hCos = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      break;

    case U90_180:
      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( uhindex )];
      break;

    case U180_270:
      Local_Components.hSin = -hSin_Cos_Table[( uint8_t )( uhindex )];
      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      break;

    case U270_360:
      Local_Components.hSin =  -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      Local_Components.hCos =  hSin_Cos_Table[( uint8_t )( uhindex )];
      break;
    default:
      break;
  }
  return ( Local_Components );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It calculates the square root of a non-negative int32_t. It returns 0
  *         for negative int32_t.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
__weak int32_t MCM_Sqrt( int32_t wInput )
{
  int32_t wtemprootnew;

  if ( wInput > 0 )
  {
  uint8_t biter = 0u;
  int32_t wtemproot;

    if ( wInput <= ( int32_t )2097152 )
    {
      wtemproot = ( int32_t )128;
    }
    else
    {
      wtemproot = ( int32_t )8192;
    }

    do
    {
      wtemprootnew = ( wtemproot + wInput / wtemproot ) / ( int32_t )2;
      if ( wtemprootnew == wtemproot )
      {
        biter = 6u;
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while ( biter < 6u );

  }
  else
  {
    wtemprootnew = ( int32_t )0;
  }

  return ( wtemprootnew );
}

/**
  * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  wBemf_alfa_est estimated Bemf alpha on the stator reference frame
  *         wBemf_beta_est estimated Bemf beta on the stator reference frame
  * @retval int16_t rotor electrical angle (s16degrees)
  */
inline int16_t MCM_PhaseComputation( int32_t wBemf_alfa_est, int32_t wBemf_beta_est )
{

  int16_t hAngle;
  int32_t wXi, wYi, wXold;

  /*Determining quadrant*/
  if ( wBemf_alfa_est < 0 )
  {
    if ( wBemf_beta_est < 0 )
    {
      /*Quadrant III, add 90 degrees so as to move to quadrant IV*/
      hAngle = 16384;
      wXi = - ( wBemf_beta_est / 2 );
      wYi = wBemf_alfa_est / 2;
    }
    else
    {
      /*Quadrant II, subtract 90 degrees so as to move to quadrant I*/
      hAngle = -16384;
      wXi = wBemf_beta_est / 2;
      wYi = - ( wBemf_alfa_est / 2 );
    }
  }
  else
  {
    /* Quadrant I or IV*/
    hAngle = 0;
    wXi = wBemf_alfa_est / 2;
    wYi = wBemf_beta_est / 2;
  }
  wXold = wXi;

  /*begin the successive approximation process*/
  /*iteration0*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV1;
    wXi = wXi - wYi;
    wYi = wXold + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV1;
    wXi = wXi + wYi;
    wYi = -wXold + wYi;
  }
  wXold = wXi;

  /*iteration1*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV2;
    wXi = wXi - wYi / 2;
    wYi = wXold / 2 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV2;
    wXi = wXi + wYi / 2;
    wYi = -wXold / 2 + wYi;
  }
  wXold = wXi;

  /*iteration2*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV4;
    wXi = wXi - wYi / 4;
    wYi = wXold / 4 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV4;
    wXi = wXi + wYi / 4;
    wYi = -wXold / 4 + wYi;
  }
  wXold = wXi;

  /*iteration3*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV8;
    wXi = wXi - wYi / 8;
    wYi = wXold / 8 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV8;
    wXi = wXi + wYi / 8;
    wYi = -wXold / 8 + wYi;
  }
  wXold = wXi;

  /*iteration4*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV16;
    wXi = wXi - wYi / 16;
    wYi = wXold / 16 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV16;
    wXi = wXi + wYi / 16;
    wYi = -wXold / 16 + wYi;
  }
  wXold = wXi;

  /*iteration5*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV32;
    wXi = wXi - wYi / 32;
    wYi = wXold / 32 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV32;
    wXi = wXi + wYi / 32;
    wYi = -wXold / 32 + wYi;
  }
  wXold = wXi;

  /*iteration6*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV64;
    wXi = wXi - wYi / 64;
    wYi = wXold / 64 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV64;
    wXi = wXi + wYi / 64;
    wYi = -wXold / 64 + wYi;
  }
  wXold = wXi;

  /*iteration7*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV128;
    wXi = wXi - wYi / 128;
    wYi = wXold / 128 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV128;
    wXi = wXi + wYi / 128;
    wYi = -wXold / 128 + wYi;
  }

  return ( -hAngle );

}

/**
  * @brief  This function codify a floating point number into the relative
  *         32bit integer.
  * @param  float Floating point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
__weak uint32_t MCM_floatToIntBit( float x )
{
  uint32_t * pInt;
  pInt = ( uint32_t * )( &x );
  return *pInt;
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
