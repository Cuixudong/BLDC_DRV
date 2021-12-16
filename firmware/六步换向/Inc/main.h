/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_2
#define KEY0_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOE
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOF
#define SHUTDOWN_Pin GPIO_PIN_10
#define SHUTDOWN_GPIO_Port GPIOF
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */


#define H_PWM_L_ON

#define CCW                         (1)
#define CW                          (2)
#define HALL_ERROR                  (0xF0)
#define RUN                         (1)
#define STOP                        (0)

#define MOTOR_1                     1
#define MOTOR_2                     2

#define HALL1_TIM_CH1_PIN           GPIO_PIN_10     /* U */
#define HALL1_TIM_CH1_GPIO          GPIOH
#define HALL1_TIM_CH2_PIN           GPIO_PIN_11     /* V */
#define HALL1_TIM_CH2_GPIO          GPIOH
#define HALL1_TIM_CH3_PIN           GPIO_PIN_12     /* W */
#define HALL1_TIM_CH3_GPIO          GPIOH

#define M1_LOW_SIDE_U_PORT          GPIOB
#define M1_LOW_SIDE_U_PIN           GPIO_PIN_13
#define M1_LOW_SIDE_V_PORT          GPIOB
#define M1_LOW_SIDE_V_PIN           GPIO_PIN_14
#define M1_LOW_SIDE_W_PORT          GPIOB
#define M1_LOW_SIDE_W_PIN           GPIO_PIN_15

#define M1_ULL HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET)
#define M1_ULH HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_SET)
#define M1_VLL HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET)
#define M1_VLH HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET)
#define M1_WLL HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET)
#define M1_WLH HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_SET)

#define M2_ULL HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_RESET)
#define M2_ULH HAL_GPIO_WritePin(M2_LOW_SIDE_U_PORT,M2_LOW_SIDE_U_PIN,GPIO_PIN_SET)
#define M2_VLL HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_RESET)
#define M2_VLH HAL_GPIO_WritePin(M2_LOW_SIDE_V_PORT,M2_LOW_SIDE_V_PIN,GPIO_PIN_SET)
#define M2_WLL HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_RESET)
#define M2_WLH HAL_GPIO_WritePin(M2_LOW_SIDE_W_PORT,M2_LOW_SIDE_W_PIN,GPIO_PIN_SET)

#define M1_HALL_U   HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN)
#define M1_HALL_V   HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN)
#define M1_HALL_W   HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN)

typedef struct {
    __IO uint8_t    run_flag;       /* 运行标志 */
    __IO uint8_t    locked_rotor;   /* 堵转标记 */
    __IO uint8_t    step_sta;       /* 本次霍尔状态 */
    __IO uint8_t    hall_single_sta;/* 单个霍尔状态 */
    __IO uint8_t    hall_sta_edge;  /* 单个霍尔状态跳变 */
    __IO uint8_t    step_last;      /* 上次霍尔状态 */
    __IO uint8_t    dir;            /* 电机旋转方向 */
    __IO int32_t    pos;            /* 电机位置 */
    __IO int32_t    pos_old;        /* 电机历史位置 */
    __IO int32_t    speed;          /* 电机速度 */
    __IO int16_t    current;        /* 电机电流 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */
} _bldc_obj;

typedef void(*PF_MOS_CTR) (void);

#define TIM1_CCCC1E     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE)
#define TIM1_CCCC1NE    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE)
#define TIM1_CCCC2E     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE)
#define TIM1_CCCC2NE    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE)
#define TIM1_CCCC3E     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE)
#define TIM1_CCCC3NE    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE)

#define TIM1_CCCC1D     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE)
#define TIM1_CCCC1ND    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE)
#define TIM1_CCCC2D     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE)
#define TIM1_CCCC2ND    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_DISABLE)
#define TIM1_CCCC3D     TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE)
#define TIM1_CCCC3ND    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE)


extern _bldc_obj bldc_motor1;
extern PF_MOS_CTR pFuncList_M1[6];
extern const uint8_t Hall_Table_CW[6];
extern const uint8_t Hall_Table_CCW[6];
extern const uint8_t Hall_CW_Table[12];
extern const uint8_t Hall_CCW_Table[12];

void M1_MOS_UHVLPWM(void);
void M1_MOS_UHWLPWM(void);
void M1_MOS_VHWLPWM(void);
void M1_MOS_VHULPWM(void);
void M1_MOS_WHULPWM(void);
void M1_MOS_WHVLPWM(void);
uint8_t check_hall_dir(_bldc_obj * obj);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
