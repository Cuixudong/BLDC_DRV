/*
 * bldc_motor.h
 *
 *  Created on: 2021年12月29日
 *      Author: 27731
 */

#ifndef BLDC_MOTOR_H_
#define BLDC_MOTOR_H_

#include "main.h"

#define NUM_CLEAR(para,val)     {if(para >= val){para=0;}}
#define NUM_MAX_LIMIT(para,val) {if(para > val){para=val;}}
#define NUM_MIN_LIMIT(para,val) {if(para < val){para=val;}}

#define HALL_LOCK_MAX_TIME          4000 /* 220ms */

#define H_PWM_L_ON

#ifndef H_PWM_L_ON
#define H_PWM_L_PWM
#endif

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

#define SHUTDOWN_M1					HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port,SHUTDOWN_Pin,GPIO_PIN_RESET)
#define UN_SHUTDOWN_M1				HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port,SHUTDOWN_Pin,GPIO_PIN_SET)

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
    __IO float      speed_t;        /* 基于计时的速度 */
    __IO int16_t    current;        /* 电机电流 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */
    __IO uint8_t    lock_hd_mode;   /* 堵转策略 */
    __IO int32_t    encode;
    __IO int32_t    encode_old;
    __IO int32_t    speed_enc;
} _bldc_obj;

#define ADC2CURT    (float)(3.3f/4.096f/0.12f)
#define ADC2VBUS    (float)(3.3f*25.0f/4096.0f)
#define ADC2VOL     (float)(3.3f/4096.0f)

#define AVERAGE(p,le,ret) do{\
                                uint32_t temp = 0;\
                                uint16_t i;\
                                for(i=0;i<le;i++)\
                                {\
                                    temp += *((uint16_t *)p + i);\
                                }\
                                * ret = (float)(temp / le);\
                            }while(0)

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

#define M1_TIM_INST     TIM1

extern _bldc_obj bldc_motor1;
extern PF_MOS_CTR pFuncList_M1[6];
extern const uint8_t Hall_Table_CW[6];
extern const uint8_t Hall_Table_CCW[6];
extern const uint8_t Hall_CW_Table[12];
extern const uint8_t Hall_CCW_Table[12];

__STATIC_INLINE void M1_MOS_UHVLPWM(void);
__STATIC_INLINE void M1_MOS_UHWLPWM(void);
__STATIC_INLINE void M1_MOS_VHWLPWM(void);
__STATIC_INLINE void M1_MOS_VHULPWM(void);
__STATIC_INLINE void M1_MOS_WHULPWM(void);
__STATIC_INLINE void M1_MOS_WHVLPWM(void);
__STATIC_INLINE uint8_t check_hall_dir(_bldc_obj * obj);
__STATIC_INLINE void SoftReversing(uint8_t h_sta);
__STATIC_INLINE uint32_t HallSensor_GetPinState(uint8_t motor_id);

void Start_Motor1(void);
void Stop_Motor1(void);
void init(void);
void loop(void);

#endif /* BLDC_MOTOR_H_ */
