/**
  ******************************************************************************
  * @file    revup_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Rev-Up Control component of the Motor Control SDK:
  *
  *          * Main Rev-Up procedure to execute programmed phases
  *          * On the Fly (OTF)
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
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
#include "revup_ctrl.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup RevUpCtrl Rev-Up Control
  * @brief Rev-Up Control component of the Motor Control SDK
  *
  * Used to start up the motor with a set of pre-programmed phases.
  *
  * The number of phases of the Rev up procdure can range from 0 to 5.
  * The Rev-Up controller must be called at speed loop frequency.
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/

/**
  * @brief Timeout used to reset integral term of PLL.
  *  It is expressed in ms.
  *
  */
#define RUC_OTF_PLL_RESET_TIMEOUT 100u


/**
  * @brief  Initialize and configure the RevUpCtrl Component
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  pSTC: Pointer on speed and torque controller structure.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  * @param  pSNSL: Pointer on sensorles state observer structure.
  * @param  pPWM: Pointer on the PWM structure.
  *  @retval none
  */
__weak void RUC_Init( RevUpCtrl_Handle_t * pHandle,
               SpeednTorqCtrl_Handle_t * pSTC,
               VirtualSpeedSensor_Handle_t * pVSS,
               STO_Handle_t * pSNSL,
               PWMC_Handle_t * pPWM )
{
  RevUpCtrl_PhaseParams_t * pRUCPhaseParams = &pHandle->ParamsData[0];
  uint8_t bPhase = 0u;

  pHandle->pSTC = pSTC;
  pHandle->pVSS = pVSS;
  pHandle->pSNSL = pSNSL;
  pHandle->pPWM = pPWM;
  pHandle->OTFSCLowside = false;
  pHandle->EnteredZone1 = false;

  while ( ( pRUCPhaseParams != MC_NULL ) && ( bPhase < RUC_MAX_PHASE_NUMBER ) )
  {
    pRUCPhaseParams = pRUCPhaseParams->pNext;
    bPhase++;
  }
  pHandle->ParamsData[bPhase - 1u].pNext = MC_NULL;

  pHandle->bPhaseNbr = bPhase;

  pHandle->bResetPLLTh = ( uint8_t )( ( RUC_OTF_PLL_RESET_TIMEOUT * pHandle->hRUCFrequencyHz ) / 1000u );

}

/**
  * @brief  Initialize internal RevUp controller state.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  hMotorDirection: rotor rotation direction.
  *         This parameter must be -1 or +1.
  *  @retval none
  */
__weak void RUC_Clear( RevUpCtrl_Handle_t * pHandle, int16_t hMotorDirection )
{
  VirtualSpeedSensor_Handle_t * pVSS = pHandle->pVSS;
  SpeednTorqCtrl_Handle_t * pSTC = pHandle->pSTC;
  RevUpCtrl_PhaseParams_t * pPhaseParams = pHandle->ParamsData;

  pHandle->hDirection = hMotorDirection;
  pHandle->EnteredZone1 = false;

  /*Initializes the rev up stages counter.*/
  pHandle->bStageCnt = 0u;
  pHandle->bOTFRelCounter = 0u;
  pHandle->OTFSCLowside = false;

  /* Calls the clear method of VSS.*/
  VSS_Clear( pVSS );

  /* Sets the STC in torque mode.*/
  STC_SetControlMode( pSTC, STC_TORQUE_MODE );

  /* Sets the mechanical starting angle of VSS.*/
  VSS_SetMecAngle( pVSS, pHandle->hStartingMecAngle * hMotorDirection );

  /* Sets to zero the starting torque of STC */
  STC_ExecRamp( pSTC, 0, 0u );

  /* Gives the first command to STC and VSS.*/
  STC_ExecRamp( pSTC, pPhaseParams->hFinalTorque * hMotorDirection,
                ( uint32_t )( pPhaseParams->hDurationms ) );

  VSS_SetMecAcceleration( pVSS, pPhaseParams->hFinalMecSpeedUnit * hMotorDirection,
                          pPhaseParams->hDurationms );

  /* Compute hPhaseRemainingTicks.*/
  pHandle->hPhaseRemainingTicks =
    ( uint16_t )( ( ( uint32_t )pPhaseParams->hDurationms *
                    ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );

  pHandle->hPhaseRemainingTicks++;

  /*Set the next phases parameter pointer.*/
  pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

  /*Timeout counter for PLL reset during OTF.*/
  pHandle->bResetPLLCnt = 0u;
}

/**
  * @brief  Main revup controller procedure executing overall programmed phases and
  *         on-the-fly startup handling.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to false when entire revup phases have been completed.
  */
__weak bool RUC_OTF_Exec( RevUpCtrl_Handle_t * pHandle )
{
  bool IsSpeedReliable;
  bool retVal = true;
  bool condition = false;

  if ( pHandle->hPhaseRemainingTicks > 0u )
  {
    /* Decrease the hPhaseRemainingTicks.*/
    pHandle->hPhaseRemainingTicks--;

    /* OTF start-up */
    if ( pHandle->bStageCnt == 0u )
    {
      if ( pHandle->EnteredZone1 == false )
      {
        if ( pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL )
        {
          pHandle->bResetPLLCnt++;
          if ( pHandle->bResetPLLCnt > pHandle->bResetPLLTh )
          {
            pHandle->pSNSL->pFctStoOtfResetPLL( pHandle->pSNSL );
            pHandle->bOTFRelCounter = 0u;
            pHandle->bResetPLLCnt = 0u;
          }
        }

        IsSpeedReliable = pHandle->pSNSL->pFctSTO_SpeedReliabilityCheck( pHandle->pSNSL );

        if ( IsSpeedReliable )
        {
          if ( pHandle->bOTFRelCounter < 127u )
          {
            pHandle->bOTFRelCounter++;
          }
        }
        else
        {
          pHandle->bOTFRelCounter = 0u;
        }

        if ( pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL )
        {
          if ( pHandle->bOTFRelCounter == ( pHandle->bResetPLLTh >> 1 ) )
          {
            condition = true;
          }
        }
        else
        {
          if ( pHandle->bOTFRelCounter == 127 )
          {
            condition = true;
          }
        }

        if ( condition == true )
        {
          bool bCollinearSpeed = false;
          int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit( pHandle->pSNSL->_Super );
          int16_t hObsSpeedUnitAbsValue =
                  ( hObsSpeedUnit < 0 ? ( -hObsSpeedUnit ) : ( hObsSpeedUnit ) ); /* hObsSpeedUnit absolute value */

          if ( pHandle->hDirection > 0 )
          {
            if ( hObsSpeedUnit > 0 )
            {
              bCollinearSpeed = true; /* actual and reference speed are collinear*/
            }
          }
          else
          {
            if ( hObsSpeedUnit < 0 )
            {
              bCollinearSpeed = true; /* actual and reference speed are collinear*/
            }
          }

          if ( bCollinearSpeed == false )
          {
            /*reverse speed management*/
            pHandle->bOTFRelCounter = 0u;
          }
          else /*speeds are collinear*/
          {
            if ( ( uint16_t )( hObsSpeedUnitAbsValue ) > pHandle->hMinStartUpValidSpeed )
            {
              /* startup end, go to run */
              pHandle->pSNSL->pFctForceConvergency1( pHandle->pSNSL );
              pHandle->EnteredZone1 = true;
            }
            else if ( ( uint16_t )( hObsSpeedUnitAbsValue ) > pHandle->hMinStartUpFlySpeed )
            {
              /* synch with startup*/
              /* nearest phase search*/
              int16_t hOldFinalMecSpeedUnit = 0;
              int16_t hOldFinalTorque = 0;
              int32_t wDeltaSpeedRevUp;
              int32_t wDeltaTorqueRevUp;
              bool bError = false;
              VSS_SetCopyObserver( pHandle->pVSS );
              pHandle->pSNSL->pFctForceConvergency2( pHandle->pSNSL );

              if (pHandle->pCurrentPhaseParams == MC_NULL)
              {
                bError = true;
                pHandle->hPhaseRemainingTicks = 0u;
              }
              else
              {
                while ( pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit < hObsSpeedUnitAbsValue )
                {
                  if ( pHandle->pCurrentPhaseParams->pNext == MC_NULL )
                  {
                    /* sets for Revup fail error*/
                    bError = true;
                    pHandle->hPhaseRemainingTicks = 0u;
                    break;
                  }
                  else
                  {
                    /* skips this phase*/
                    hOldFinalMecSpeedUnit = pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit;
                    hOldFinalTorque = pHandle->pCurrentPhaseParams->hFinalTorque;
                    pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;
                    pHandle->bStageCnt++;
                  }
                }
              }

              if ( bError == false )
              {
                /* calculation of the transition phase from OTF to standard revup */
                int16_t hTorqueReference;

                wDeltaSpeedRevUp = ( int32_t )( pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit ) - ( int32_t )( hOldFinalMecSpeedUnit );
                wDeltaTorqueRevUp = ( int32_t )( pHandle->pCurrentPhaseParams->hFinalTorque ) - ( int32_t )( hOldFinalTorque );

                hTorqueReference = ( int16_t )( ( ( ( int32_t )hObsSpeedUnit ) * wDeltaTorqueRevUp ) / wDeltaSpeedRevUp ) +
                        hOldFinalTorque;

                STC_ExecRamp( pHandle->pSTC, hTorqueReference, 0u );

                pHandle->hPhaseRemainingTicks = 1u;

                pHandle->pCurrentPhaseParams = &pHandle->OTFPhaseParams;

                pHandle->bStageCnt = 6u;
              } /* no MC_NULL error */
            } /* speed > MinStartupFly */
            else
            {
            }
          } /* speeds are collinear */
        } /* speed is reliable */
      }/*EnteredZone1 1 is false */
      else
      {
        pHandle->pSNSL->pFctForceConvergency1( pHandle->pSNSL );
      }
    } /*stage 0*/
  } /* hPhaseRemainingTicks > 0 */

  if ( pHandle->hPhaseRemainingTicks == 0u )
  {
    if ( pHandle->pCurrentPhaseParams != MC_NULL )
    {
      if ( pHandle->bStageCnt == 0u )
      {
        /*end of OTF*/
        PWMC_SwitchOffPWM( pHandle->pPWM );
        pHandle->OTFSCLowside = true;
        PWMC_TurnOnLowSides( pHandle->pPWM );
        pHandle->bOTFRelCounter = 0u;
      }
      else if ( ( pHandle->bStageCnt == 1u ) )
      {
        PWMC_SwitchOnPWM( pHandle->pPWM );
        pHandle->OTFSCLowside = false;
      }
      else
      {
      }

      /* If it becomes zero the current phase has been completed.*/
      /* Gives the next command to STC and VSS.*/
      STC_ExecRamp( pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                    ( uint32_t )( pHandle->pCurrentPhaseParams->hDurationms ) );

      VSS_SetMecAcceleration( pHandle->pVSS,
                              pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                              pHandle->pCurrentPhaseParams->hDurationms );

      /* Compute hPhaseRemainingTicks.*/
      pHandle->hPhaseRemainingTicks =
              ( uint16_t )( ( ( uint32_t )pHandle->pCurrentPhaseParams->hDurationms *
                      ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );
      pHandle->hPhaseRemainingTicks++;

      /*Set the next phases parameter pointer.*/
      pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;

      /*Increases the rev up stages counter.*/
      pHandle->bStageCnt++;
    }
    else
    {
      if ( pHandle->bStageCnt == pHandle->bPhaseNbr - 1 ) /* End of user programmed revup */
      {
        retVal = false;
      }
      else if ( pHandle->bStageCnt == 7u ) /* End of first OTF runs */
      {
        pHandle->bStageCnt = 0u; /* Breaking state */
        pHandle->hPhaseRemainingTicks = 0u;
      }
      else
      {
      }
    }
  }
  return retVal;
}

/**
  * @brief  Main revup controller procedure executing overall programmed phases.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to false when entire revup phases have been completed.
  */
__weak bool RUC_Exec( RevUpCtrl_Handle_t * pHandle )
{
  bool retVal = true;

  if ( pHandle->hPhaseRemainingTicks > 0u )
  {
    /* Decrease the hPhaseRemainingTicks.*/
    pHandle->hPhaseRemainingTicks--;

  } /* hPhaseRemainingTicks > 0 */

  if ( pHandle->hPhaseRemainingTicks == 0u )
  {
    if ( pHandle->pCurrentPhaseParams != MC_NULL )
    {

      /* If it becomes zero the current phase has been completed.*/
      /* Gives the next command to STC and VSS.*/
      STC_ExecRamp( pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                    ( uint32_t )( pHandle->pCurrentPhaseParams->hDurationms ) );

      VSS_SetMecAcceleration( pHandle->pVSS,
                              pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                              pHandle->pCurrentPhaseParams->hDurationms );

      /* Compute hPhaseRemainingTicks.*/
      pHandle->hPhaseRemainingTicks =
        ( uint16_t )( ( ( uint32_t )pHandle->pCurrentPhaseParams->hDurationms *
                        ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );
      pHandle->hPhaseRemainingTicks++;

      /*Set the next phases parameter pointer.*/
      pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;

      /*Increases the rev up stages counter.*/
      pHandle->bStageCnt++;
    }
    else
    {
      retVal = false;
    }
  }
  return retVal;
}

/**
  * @brief  Provide current state of revup controller procedure.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when entire revup phases have been completed.
  */
__weak bool RUC_Completed( RevUpCtrl_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->pCurrentPhaseParams == MC_NULL )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  Allow to exit from RevUp process at the current rotor speed.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval none
  */
__weak void RUC_Stop( RevUpCtrl_Handle_t * pHandle )
{
  VirtualSpeedSensor_Handle_t * pVSS = pHandle->pVSS;
  pHandle->pCurrentPhaseParams = MC_NULL;
  pHandle->hPhaseRemainingTicks = 0u;
  VSS_SetMecAcceleration( pVSS, SPD_GetAvrgMecSpeedUnit( & pVSS->_Super ), 0u );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Check that alignement and first acceleration stage are completed.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when first acceleration stage has been reached.
  */
__weak bool RUC_FirstAccelerationStageReached( RevUpCtrl_Handle_t * pHandle )
{
  bool retVal = false;

  if ( pHandle->bStageCnt >= pHandle->bFirstAccelerationStage )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  Allow to modify duration of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new duration shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hDurationms: new duration value required for associated phase.
  *         This parameter must be set in millisecond.
  *  @retval none
  */
__weak void RUC_SetPhaseDurationms( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase, uint16_t hDurationms )
{
  pHandle->ParamsData[bPhase].hDurationms = hDurationms;
}

/**
  * @brief  Allow to modify targetted mechanical speed of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new mechanical speed shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hFinalMecSpeedUnit: new targetted mechanical speed.
  *         This parameter must be expressed in 0.1Hz.
  *  @retval none
  */
__weak void RUC_SetPhaseFinalMecSpeedUnit( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase,
                                    int16_t hFinalMecSpeedUnit )
{
  pHandle->ParamsData[bPhase].hFinalMecSpeedUnit = hFinalMecSpeedUnit;
}

/**
  * @brief  Allow to modify targetted the motor torque of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new the motor torque shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hFinalTorque: new targetted motor torque.
  *  @retval none
  */
__weak void RUC_SetPhaseFinalTorque( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase, int16_t hFinalTorque )
{
  pHandle->ParamsData[bPhase].hFinalTorque = hFinalTorque;
}

/**
  * @brief  Allow to read duration set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where duration is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns duration used in selected phase.
  */
__weak uint16_t RUC_GetPhaseDurationms( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase )
{
  return ( ( uint16_t )pHandle->ParamsData[bPhase].hDurationms );
}

/**
  * @brief  Allow to read targetted rotor speed set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where targetted rotor speed is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns targetted rotor speed set in selected phase.
  */
__weak int16_t RUC_GetPhaseFinalMecSpeedUnit( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase )
{
  return ( ( int16_t )pHandle->ParamsData[bPhase].hFinalMecSpeedUnit );
}

/**
  * @brief  Allow to read targetted motor torque set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where targetted motor torque is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns targetted motor torque set in selected phase.
  */
__weak int16_t RUC_GetPhaseFinalTorque( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase )
{
  return ( ( int16_t )pHandle->ParamsData[bPhase].hFinalTorque );
}

/**
  * @brief  Allow to read total number of programmed phases.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Returns number of phases relative to the programmed revup.
  */
__weak uint8_t RUC_GetNumberOfPhases( RevUpCtrl_Handle_t * pHandle )
{
  return ( ( uint8_t )pHandle->bPhaseNbr );
}

/**
  * @brief  Allow to read status of On The Fly (OTF) feature.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true at the end of OTF precessing.
  */
__weak bool RUC_Get_SCLowsideOTF_Status( RevUpCtrl_Handle_t * pHandle )
{
  return ( pHandle->OTFSCLowside );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
