/*
 * bldc_motor.c
 *
 *  Created on: 2021年12月29日
 *      Author: 27731
 */

#include "bldc_motor.h"
#include "debug.h"

#include "tim.h"
#include "adc.h"
#include "dac.h"

_bldc_obj bldc_motor1 = {
    .run_flag = STOP,
    .locked_rotor = 0,
    .step_sta = 0,
    .hall_single_sta = CCW,
    .lock_hd_mode = 0,
    .encode = 0,
    .encode_old = 0,
};/*电机结构体*/

PF_MOS_CTR pFuncList_M1[6] =
{
    &M1_MOS_UHWLPWM, &M1_MOS_VHULPWM, &M1_MOS_VHWLPWM,
    &M1_MOS_WHVLPWM, &M1_MOS_UHVLPWM, &M1_MOS_WHULPWM
};

const uint8_t Hall_Table_CW[6] = {6,2,3,1,5,4};     /* 顺时针旋转表 */
const uint8_t Hall_Table_CCW[6] = {5,1,3,2,6,4};    /* 逆时针旋转表 */

const uint8_t Hall_CW_Table[12] = {0x62,0x23,0x31,0x15,0x54,0x46,0x63,0x21,0x35,0x14,0x56,0x42};
const uint8_t Hall_CCW_Table[12] = {0x45,0x51,0x13,0x32,0x26,0x64,0x41,0x53,0x12,0x36,0x24,0x65};

__STATIC_INLINE uint8_t check_hall_dir(_bldc_obj * obj)
{
    uint8_t temp,res = HALL_ERROR;
    if((obj->step_last <= 6)&&(obj->step_sta <= 6))
    {
        temp = ((obj->step_last & 0x0F) << 4)|(obj->step_sta & 0x0F);
        if((temp == Hall_CCW_Table[0])||(temp == Hall_CCW_Table[1])||\
                (temp == Hall_CCW_Table[2])||(temp == Hall_CCW_Table[3])||\
                (temp == Hall_CCW_Table[4])||(temp == Hall_CCW_Table[5]))
        {
            res  = CCW;
        }
        else if((temp == Hall_CW_Table[0])||(temp == Hall_CW_Table[1])||\
                (temp == Hall_CW_Table[2])||(temp == Hall_CW_Table[3])||\
                (temp == Hall_CW_Table[4])||(temp == Hall_CW_Table[5]))
        {
            res  = CW;
        }
    }
    return res;
}

__STATIC_INLINE void M1_MOS_UHVLPWM(void)
{
#ifdef H_PWM_L_PWM
    TIM1_CCCC1E;
    TIM1_CCCC1NE;

    TIM1_CCCC2E;
    TIM1_CCCC2NE;

    TIM1_CCCC3D;
    TIM1_CCCC3ND;

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,bldc_motor1.pwm_duty);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2, 0);
#endif
#ifdef H_PWM_L_ON
    M1_TIM_INST->CCR2 = 0;
    M1_TIM_INST->CCR3 = 0;

    M1_ULL;/*U相下桥臂关闭*/
    M1_WLL;/*W相下桥臂关闭*/

    M1_TIM_INST->CCR1 = bldc_motor1.pwm_duty;/*U相上桥臂PWM*/
    M1_VLH;/*V相下桥臂导通*/
#endif
}

__STATIC_INLINE void M1_MOS_UHWLPWM(void)
{
#ifdef H_PWM_L_PWM
    TIM1_CCCC1E;
    TIM1_CCCC1NE;

    TIM1_CCCC2D;
    TIM1_CCCC2ND;

    TIM1_CCCC3E;
    TIM1_CCCC3NE;
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,bldc_motor1.pwm_duty);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3, 0);
#endif
#ifdef H_PWM_L_ON
    M1_TIM_INST->CCR2 = 0;
    M1_TIM_INST->CCR3 = 0;

    M1_ULL;
    M1_VLL;

    M1_TIM_INST->CCR1 = bldc_motor1.pwm_duty;
    M1_WLH;
#endif
}

__STATIC_INLINE void M1_MOS_VHWLPWM(void)
{
#ifdef H_PWM_L_PWM
    TIM1_CCCC1D;
    TIM1_CCCC1ND;

    TIM1_CCCC2E;
    TIM1_CCCC2NE;

    TIM1_CCCC3E;
    TIM1_CCCC3NE;
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,bldc_motor1.pwm_duty);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3, 0);
#endif
#ifdef H_PWM_L_ON
    M1_TIM_INST->CCR1=0;
    M1_TIM_INST->CCR3=0;

    M1_ULL;
    M1_VLL;

    M1_TIM_INST->CCR2 = bldc_motor1.pwm_duty;
    M1_WLH;
#endif
}

__STATIC_INLINE void M1_MOS_VHULPWM(void)
{
#ifdef H_PWM_L_PWM
    TIM1_CCCC1E;
    TIM1_CCCC1NE;

    TIM1_CCCC2E;
    TIM1_CCCC2NE;

    TIM1_CCCC3D;
    TIM1_CCCC3ND;
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,bldc_motor1.pwm_duty);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, 0);
#endif
#ifdef H_PWM_L_ON
    M1_TIM_INST->CCR1 = 0;
    M1_TIM_INST->CCR3 = 0;

    M1_VLL;
    M1_WLL;

    M1_TIM_INST->CCR2 = bldc_motor1.pwm_duty;
    M1_ULH;
#endif
}


__STATIC_INLINE void M1_MOS_WHULPWM(void)
{
#ifdef H_PWM_L_PWM
    TIM1_CCCC1E;
    TIM1_CCCC1NE;

    TIM1_CCCC2D;
    TIM1_CCCC2ND;

    TIM1_CCCC3E;
    TIM1_CCCC3NE;
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,bldc_motor1.pwm_duty);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, 0);
#endif
#ifdef H_PWM_L_ON
    M1_TIM_INST->CCR2 = 0;
    M1_TIM_INST->CCR1 = 0;

    M1_VLL;
    M1_WLL;

    M1_TIM_INST->CCR3 = bldc_motor1.pwm_duty;
    M1_ULH;
#endif
}

__STATIC_INLINE void M1_MOS_WHVLPWM(void)
{
#ifdef H_PWM_L_PWM
    TIM1_CCCC1D;
    TIM1_CCCC1ND;

    TIM1_CCCC2E;
    TIM1_CCCC2NE;

    TIM1_CCCC3E;
    TIM1_CCCC3NE;
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,bldc_motor1.pwm_duty);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2, 0);
#endif
#ifdef H_PWM_L_ON
    M1_TIM_INST->CCR2 = 0;
    M1_TIM_INST->CCR1 = 0;

    M1_ULL;
    M1_WLL;

    M1_TIM_INST->CCR3 = bldc_motor1.pwm_duty;
    M1_VLH;
#endif
}

__STATIC_INLINE uint32_t HallSensor_GetPinState(uint8_t motor_id)
{
    __IO static uint32_t State ;
    State  = 0;
    if(motor_id == MOTOR_1)
    {
        if(M1_HALL_U != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            State |= 0x01U;
        }
        if(M1_HALL_V != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            State |= 0x02U;
        }
        if(M1_HALL_W != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            State |= 0x04U;
        }
    }
    return State;
}

void Start_Motor1(void)
{
    UN_SHUTDOWN_M1;
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

    M1_ULL;
    M1_VLL;
    M1_WLL;
}

void Stop_Motor1(void)
{
    SHUTDOWN_M1;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CNT = 0;

    TIM1_CCCC1D;
    TIM1_CCCC2D;
    TIM1_CCCC3D;
    TIM1_CCCC1ND;
    TIM1_CCCC2ND;
    TIM1_CCCC3ND;
    M1_ULL;
    M1_VLL;
    M1_WLL;

//    M1_ULH;M1_VLH;M1_WLH;
}

__STATIC_INLINE void SoftReversing(uint8_t h_sta)
{
    if(bldc_motor1.dir == CCW)
    {
        bldc_motor1.step_sta = h_sta;
    }
    else
    {
        bldc_motor1.step_sta = 7 - h_sta;
    }
    if((bldc_motor1.step_sta <= 6)&&(bldc_motor1.step_sta >= 1))
    {
        pFuncList_M1[bldc_motor1.step_sta-1]();
    }
    else    /* 编码器错误、接触不良、断开等情况 */
    {
        bldc_motor1.run_flag = STOP;
    }
}

#if USE_AUTO_PID

#include "PIDRun.h"
// 错误码 第一位代表 是否稳定； 第二位代表是否可以改变设定值：  第三位代表温度是否异常
// 0:表示稳定、可以改变、无异常 1：表示未稳定、不可改变、温度异常
int iErrorCodeBLDC = 0;

//系统时间 ms
unsigned int Current_time;
// 实例1
TEMP_PID_STRUCT sBLDC = { 0 };
PID_PARM_THREE sBLDC_PARM = { 0 };
double dBLDC_MaxOutPut = 0.0;

// 初始化实例1
void PID_Setup_BLDC(void)
{
    PID_INIT_PARM sInitParmBLDC = { 0 };
    // 此处可由大家自由配置
    {
        // 设定实例1对应的PWM输出上限
        dBLDC_MaxOutPut = 6000;
        // 设定PID计算周期
        sInitParmBLDC.iSampleTimeNor = 20;
        sInitParmBLDC.iDirection = DIRECT;
        // 自动调参阈值，如果系统敏感，则适宜将此设定值设大
        // 例如稍微增加一点输出，PV会迅速变化，那么需要将此值设大，此处可以设定为4倍的传感器精度
        sInitParmBLDC.aTuneNoise = 5;
        sInitParmBLDC.iType = 1;
        // 在开机后1秒钟后进入自动调参，如果系统很灵敏，那么可以适当减小此参数，如果此参数设置为0，那么系统就不会进入自动调参
        sInitParmBLDC.iSinTimes = 250;
        // 自动调参结束后连续7秒钟 PV在阈值范围内，则表示稳定，如果设置为0，那么用于认为系统稳定
        sInitParmBLDC.iOKTimes = 30000;
    }

    // 其中dCurVal 和 dSetVal这里没有意义，通过PID_Cal接口给出
    sBLDC_PARM.dCurVal = 0;
    sBLDC_PARM.dOutPut = 0;
    sBLDC_PARM.dSetVal = 0;
    sInitParmBLDC.dMax = dBLDC_MaxOutPut;
    sInitParmBLDC.dMin = -dBLDC_MaxOutPut;

    // 设定初始的KP/KI/KD
    sInitParmBLDC.KP = 0.05 * sInitParmBLDC.dMax;
    sInitParmBLDC.KI = 0.01;
    sInitParmBLDC.KD = 0;
    // 开启PID控制标志
    sInitParmBLDC.iMode = AUTOMATIC;
    // 这个参数尽量不要修改
    sInitParmBLDC.aTuneLookBack = 5;
    sInitParmBLDC.aTuneStartValue = 0;
    sInitParmBLDC.aTuneStep = 0;
    setupPID(&sBLDC, &(sBLDC_PARM.dCurVal), &(sBLDC_PARM.dSetVal), &(sBLDC_PARM.dOutPut), &Current_time);
    setupParm(&sBLDC, sInitParmBLDC);
}

int PID_Cal(TEMP_PID_STRUCT *sTemp, double dSetVal, double dCurVal)
{
    int iErrorCode = 0;
    *(sTemp->inputRun) = dCurVal;
    *(sTemp->setpointRun) = dSetVal;
    iErrorCode =  PID_Operation(sTemp);
    return iErrorCode;
}

#endif

const float Rp = 10000.0f;          /* 10K */
const float T2 = (273.15f + 25.0f); /* T2 */
const float Bx = 3380.0f;           /* B */
const float Ka = 273.15f;
float get_temp(uint16_t para)
{
    float Rt;
    float temp;
    Rt = 3.3f / (para * 3.3f / 4096.0f / 4700.0f) - 4700.0f;
    /* like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)) */
    temp = Rt / Rp;
    temp = log(temp);       /* ln(Rt/Rp) */
    temp /= Bx;             /* ln(Rt/Rp)/B */
    temp += (1.0f / T2);
    temp = 1.0f / (temp);
    temp -= Ka;
    return temp;
}

/* 编码器溢出计数 */
__IO int EncodeCount = 0;
/* ADC1原始数据 */
uint16_t adc1_normal_data_array[5];
/* ADC3原始数据 */
uint16_t adc3_normal_data_array[3];

/* ADC1电流滤波长度 */
#define ADC_FLITE_NUM 200
/* ADC1电流缓存位置、最大值、有效值存储 */
uint16_t adc_amp_p[9]= {0,};
/* ADC1滤波缓存 */
uint16_t adc_amp[3][ADC_FLITE_NUM];
/* ADC1零电流滤波长度 */
#define ADC_ZERO_FLITE_NUM 20
/* ADC1零电流缓存位置 */
uint16_t adc_amp_zero_p[3]= {0,};
/* ADC1零电流缓存 */
uint16_t adc_amp_zero[3][ADC_ZERO_FLITE_NUM];

/* 整周期三相电流缓存 */
uint16_t adc_amp_agv[3][6000];
/* 整周期三相电流采集标志 */
uint8_t adc_agv_flag = 0;
/* 整周期三相电流存储位置 */
uint16_t adc_amp_agv_p = 0;

/* 三相平均电流值 */
static float bldc_m1_amp[4] = {0.0f,0.0f,0.0f};
/* 三相平均电流值 */
static float bldc_m1_amp_zero[3] = {0.0f,0.0f,0.0f};

void M1_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_IT_UPDATE);
}

int32_t M1_Get_Encode(void)
{
    return ( int32_t )__HAL_TIM_GET_COUNTER(&htim3) + EncodeCount * 65536;
}

uint32_t calc_agv(uint16_t * p,uint16_t len)
{
    int i;
    uint32_t re = 0;
    double sum = 0;
    if(len > 0)
    {
        for(i=0; i<len; i++)
        {
            sum += (*(p + i)) * (*(p + i));
        }
        re = (uint32_t)sqrt(sum / len);
    }
    return re;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);

    uint8_t bldc_dir=0;
    static uint8_t hal_old_sta = 0xFF;
    static uint16_t l_cnt = 0;
    uint8_t hal_cur_sta = 0;
    /* 编码器溢出计数 */
    if (htim->Instance == TIM3)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
        {
            EncodeCount--;
        }
        else
        {
            EncodeCount++;
        }
    }
    /* 定时器1PWM中断 */
    else if(htim->Instance == M1_TIM_INST)//55us
    {
        /* PWM中断计数 */
        l_cnt ++;
        /* 编码器测速 */
        if(l_cnt % 200 == 1)
        {
            bldc_motor1.encode = M1_Get_Encode();

            bldc_motor1.speed_enc = (bldc_motor1.encode - bldc_motor1.encode_old);
            bldc_motor1.encode_old = bldc_motor1.encode;
        }
        /* 霍尔测速 */
        if(l_cnt == 400)//5.5ms
        {
            bldc_motor1.speed = (bldc_motor1.pos - bldc_motor1.pos_old);
            bldc_motor1.pos_old = bldc_motor1.pos;
            l_cnt = 0;
            if(0)printf("speed:%8d ",(int)bldc_motor1.speed);
            if(0)printf("speed_t:%.2f",bldc_motor1.speed_t);
        }
#ifdef H_PWM_L_ON
        /* 霍尔检测 */
        hal_cur_sta = HallSensor_GetPinState(MOTOR_1);
        if(hal_old_sta != hal_cur_sta)
        {
            /* 运行时 */
            if(bldc_motor1.run_flag == RUN)
            {
                /* 六步换向 */
                SoftReversing(hal_cur_sta);
            }
            hal_old_sta = hal_cur_sta;
        }
        /* 霍尔变化 */
        if(bldc_motor1.step_last != bldc_motor1.step_sta)
        {
            /* 霍尔保持时间法测速 */
            bldc_motor1.speed_t = bldc_motor1.speed_t * 0.95f + (1000 * 60 * 12 * 3 / (bldc_motor1.hall_keep_t * 55)) * 0.05f;
            bldc_dir = check_hall_dir(&bldc_motor1);
            /* 霍尔位置统计 */
            if(bldc_dir == CCW)
            {
                bldc_motor1.pos -=1;
            }
            else if(bldc_dir == CW)
            {
                bldc_motor1.pos +=1;
            }
            if(0)printf("adc_amp_p:%5d %5d %5d\r\n",adc_amp_p[0],adc_amp_p[1],adc_amp_p[2]);
            /* ADC完整周期电流采集 */
            if(bldc_motor1.step_sta == 0x01)
            {
                static uint8_t temp_f = 0;
                if(temp_f == 0)
                {
                    temp_f = 1;
                }
                else if(temp_f)
                {
                    if(adc_agv_flag == 0)
                    {
                        adc_agv_flag = 1;
                        adc_amp_agv_p = 0;
                    }
                    else if(adc_agv_flag == 1)
                    {
                        adc_agv_flag = 2;
                    }
                }
            }
            /* U相电流有效值计算 */
            if((bldc_motor1.step_sta == 0x00)||(bldc_motor1.step_sta == 0x02))
            {
                /* 平均值计算 */
                AVERAGE(adc_amp[0],adc_amp_p[0],&bldc_m1_amp[0]);
                /* 均方根计算 */
                adc_amp_p[6] = calc_agv(adc_amp[0],adc_amp_p[0]);
                /* 换向时采集位置清零 */
                adc_amp_p[0] = 0;
                adc_amp_p[3] = 0;
            }
            /* V相电流有效值计算 */
            else if((bldc_motor1.step_sta == 0x04)||(bldc_motor1.step_sta == 0x05))
            {
                AVERAGE(adc_amp[1],adc_amp_p[1],&bldc_m1_amp[1]);
                adc_amp_p[7] = calc_agv(adc_amp[1],adc_amp_p[1]);
                adc_amp_p[1]=0;
                adc_amp_p[4] = 0;
            }
            /* W相电流有效值计算 */
            else if((bldc_motor1.step_sta == 0x01)||(bldc_motor1.step_sta == 0x03))
            {
                AVERAGE(adc_amp[2],adc_amp_p[2],&bldc_m1_amp[2]);
                adc_amp_p[8] = calc_agv(adc_amp[2],adc_amp_p[2]);
                adc_amp_p[2]=0;
                adc_amp_p[5] = 0;
            }
            if(0)printf("pos:%d\r\n",(int)bldc_motor1.pos);
            bldc_motor1.step_last = bldc_motor1.step_sta;
            if(0)printf("hall keep:%d\r\n",(int)bldc_motor1.hall_keep_t);
            /* 霍尔位置调试 */
            HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,bldc_motor1.step_last * 4096 / 6);
            bldc_motor1.hall_keep_t=0;
        }
        else
        {
            /* 霍尔保持逻辑 */
            bldc_motor1.hall_keep_t++;//换向一次所需计数值（时间） 单位1/18k
            if(bldc_motor1.run_flag == RUN)    /*运行且霍尔保持时 */
            {
                if(bldc_motor1.hall_keep_t > HALL_LOCK_MAX_TIME)
                {
                    //堵转超时
                }
            }
        }
        /* 存储电流原始数据 */
        if(1)
        {
            /* U相静态电流存储 */
            if(bldc_motor1.step_sta == 0x04)
            {
                adc_amp_zero[0][adc_amp_zero_p[0]%ADC_ZERO_FLITE_NUM] = adc1_normal_data_array[0];
                /* U相电流存储位置 */
                adc_amp_zero_p[0]++;
                if(adc_amp_zero_p[0] == ADC_ZERO_FLITE_NUM)
                {
                    AVERAGE(adc_amp_zero[0],adc_amp_zero_p[0],&bldc_m1_amp_zero[0]);
                }
                NUM_CLEAR(adc_amp_zero_p[0],ADC_ZERO_FLITE_NUM);
            }
            if(bldc_motor1.step_sta == 0x01)
            {
                adc_amp_zero[1][adc_amp_zero_p[1]%ADC_ZERO_FLITE_NUM] = adc1_normal_data_array[1];
                /* V相电流存储位置 */
                adc_amp_zero_p[1]++;
                if(adc_amp_zero_p[1] == ADC_ZERO_FLITE_NUM)
                {
                    AVERAGE(adc_amp_zero[1],adc_amp_zero_p[1],&bldc_m1_amp_zero[1]);
                }
                NUM_CLEAR(adc_amp_zero_p[1],ADC_ZERO_FLITE_NUM);
            }
            if(bldc_motor1.step_sta == 0x06)
            {
                adc_amp_zero[2][adc_amp_zero_p[2]%ADC_ZERO_FLITE_NUM] = adc1_normal_data_array[2];
                /* W相电流存储位置 */
                adc_amp_zero_p[2]++;
                if(adc_amp_zero_p[2] == ADC_ZERO_FLITE_NUM)
                {
                    AVERAGE(adc_amp_zero[2],adc_amp_zero_p[2],&bldc_m1_amp_zero[2]);
                }
                NUM_CLEAR(adc_amp_zero_p[2],ADC_ZERO_FLITE_NUM);
            }
            if((bldc_motor1.step_sta == 0x01)||(bldc_motor1.step_sta == 0x03))
            {
                /* 存储U相电流原始数据 */
                adc_amp[0][adc_amp_p[0]%ADC_FLITE_NUM] = adc1_normal_data_array[0];
                /* U相电流存储位置 */
                adc_amp_p[0]++;
                NUM_CLEAR(adc_amp_p[0],ADC_FLITE_NUM);
                /* U相电流原始数据峰值计算 */
                if(adc1_normal_data_array[0] > adc_amp_p[3])
                {
                    adc_amp_p[3] = adc1_normal_data_array[0];
                }
            }
            else if((bldc_motor1.step_sta == 0x05)||(bldc_motor1.step_sta == 0x06))
            {
                adc_amp[1][adc_amp_p[1]%ADC_FLITE_NUM] = adc1_normal_data_array[1];
                adc_amp_p[1]++;
                NUM_CLEAR(adc_amp_p[1],ADC_FLITE_NUM);
                if(adc1_normal_data_array[1] > adc_amp_p[4])
                {
                    adc_amp_p[4] = adc1_normal_data_array[1];
                }
            }
            else if((bldc_motor1.step_sta == 0x02)||(bldc_motor1.step_sta == 0x04))
            {
                adc_amp[2][adc_amp_p[2]%ADC_FLITE_NUM] = adc1_normal_data_array[2];
                adc_amp_p[2]++;
                NUM_CLEAR(adc_amp_p[2],ADC_FLITE_NUM);
                if(adc1_normal_data_array[2] > adc_amp_p[5])
                {
                    adc_amp_p[5] = adc1_normal_data_array[2];
                }
            }
        }
#endif
    }
    HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
}

void BLDC_Ctrl(uint8_t motor_id,int32_t dir,uint16_t duty)
{
    if(motor_id == MOTOR_1)
    {
        /* 停机 */
        if(duty == 0)
        {
            Stop_Motor1();
            bldc_motor1.pwm_duty = 0;
            bldc_motor1.run_flag = STOP;
            bldc_motor1.dir = dir;
        }
        else
        {
            /* 之前是停机 */
            if(bldc_motor1.run_flag == STOP)
            {
                bldc_motor1.run_flag = RUN;
                bldc_motor1.dir = dir;            /* 方向 */
                bldc_motor1.pwm_duty = duty;      /* 占空比 */
                Start_Motor1();/*开启运行*/
                SoftReversing(HallSensor_GetPinState(MOTOR_1));
            }
            else
            {
                /* 旋转换向 */
                if(bldc_motor1.dir != dir)
                {
                    uint16_t spd = bldc_motor1.pwm_duty;
                    Stop_Motor1();
                    bldc_motor1.pwm_duty = 0;
                    bldc_motor1.run_flag = STOP;
                    /* 制动 */
                    M1_ULH;
                    M1_VLH;
                    M1_WLH;
                    if(spd >= 200)
                        HAL_Delay(spd>>6);
                    /* 取消制动 */
                    M1_ULL;
                    M1_VLL;
                    M1_WLL;
                    if(spd >= 200)
                        HAL_Delay(spd/38);
                }
                bldc_motor1.dir = dir;            /* 方向 */
                bldc_motor1.pwm_duty = duty;      /* 占空比 */
                bldc_motor1.run_flag = RUN;
                Start_Motor1();/*开启运行*/
                /* 六步换向 */
                SoftReversing(HallSensor_GetPinState(MOTOR_1));
            }
        }
    }
}

void test_fun1(void)
{
    BLDC_Ctrl(MOTOR_1, CCW, 1600);
    HAL_Delay(300);
    BLDC_Ctrl(MOTOR_1, CW, 1800);
    HAL_Delay(300);
    BLDC_Ctrl(MOTOR_1, CCW, 2600);
    HAL_Delay(300);
    BLDC_Ctrl(MOTOR_1, CW, 1800);
    HAL_Delay(300);
    BLDC_Ctrl(MOTOR_1, CCW, 0);
    while (1)
    {
        HAL_Delay(100);
        BLDC_Ctrl(MOTOR_1, CW, 2200);
        HAL_Delay(100);
        BLDC_Ctrl(MOTOR_1, CCW, 2800);
        HAL_Delay(100);
        BLDC_Ctrl(MOTOR_1, CW, 3200);
        HAL_Delay(100);
        BLDC_Ctrl(MOTOR_1, CCW, 2800);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,adc1_normal_data_array[0]);
        //HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc1_normal_data_array,5);
        if(adc_agv_flag == 1)
        {
            adc_amp_agv[0][(adc_amp_agv_p) % 6000] = adc1_normal_data_array[0];
            adc_amp_agv[1][(adc_amp_agv_p) % 6000] = adc1_normal_data_array[1];
            adc_amp_agv[2][(adc_amp_agv_p) % 6000] = adc1_normal_data_array[2];
            adc_amp_agv_p ++;
        }
    }
    if(hadc->Instance == ADC3)
    {
        //HAL_ADC_Start_DMA(&hadc3,(uint32_t *)adc3_normal_data_array,5);
    }
}


void init(void)
{
    printf("sys run...\r\n");

    HAL_TIM_Base_Start(&htim1);/*启动高级定时器1*/
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
    M1_Encoder_Init();

    //PID_Setup_BLDC();

    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc1_normal_data_array,5);
    HAL_ADC_Start_DMA(&hadc3,(uint32_t *)adc3_normal_data_array,3);

    debug_obj_init(&g_debug);

    HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
    HAL_Delay(2000);
    //BLDC_Ctrl(MOTOR_1, CW, 1520);
    //HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    //__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,20);
}

void loop(void)
{
#if EN_AUTO_PID
    Current_time = HAL_GetTick();
    dCurVal = (double)bldc_motor1.speed;
    // 计算输出
    iErrorCodeBLDC = PID_Cal(&sBLDC, dSetVal, dCurVal);
    dOutputBLDC = *(sBLDC.outputRun);
    switch(iErrorCodeBLDC)
    {
    case 0:
        printf("稳定期\r\n");
        break;
    case 1:
        printf("调参结束\r\n");
        break;
    case 2:
        printf("调参中\r\n");
        break;
    case 3:
        printf("震荡期\r\n");
        break;
    }
    if(1)
        printf("speed:%d out:%lf   %.1f  %.1f  %.1f\r\n",bldc_motor1.speed,dOutputBLDC,\
               sBLDC.sTempNor.dispKp,sBLDC.sTempNor.dispKi,sBLDC.sTempNor.dispKd);
    if(dOutputBLDC >= 0)
    {
        BLDC_Ctrl(MOTOR_1,CCW,(int)dOutputBLDC);
    }
    else
    {
        BLDC_Ctrl(MOTOR_1,CW,(int)-dOutputBLDC);
    }
#endif
    HAL_Delay(50);
    if(0)
    {
        printf("adc1:%5d,%5d,%5d,%5d,%5d ",adc1_normal_data_array[0],\
               adc1_normal_data_array[1],adc1_normal_data_array[2],\
               adc1_normal_data_array[3],adc1_normal_data_array[4]);

        printf("adc3:%5d,%5d,%5d\r\n",adc3_normal_data_array[0],\
               adc3_normal_data_array[1],adc3_normal_data_array[2]);
    }
#if 1
    if(1)
    {
        printf("vol:%.1f temp:%.1f",adc1_normal_data_array[3]*ADC2VBUS,get_temp(adc1_normal_data_array[4]));
    }
    if(0)
    {
        /* 有效扇区均值 */
        if((bldc_m1_amp[0] > 0)&&(bldc_m1_amp[1] > 0)&&(bldc_m1_amp[2] > 0))
        {
            printf("AMP U:%.2f  ",(bldc_m1_amp[0]*ADC2VOL - 1.25f) * ADC2CURT);
            printf("AMP V:%.2f  ",(bldc_m1_amp[1]*ADC2VOL - 1.25f) * ADC2CURT);
            printf("AMP W:%.2f  ",(bldc_m1_amp[2]*ADC2VOL - 1.25f) * ADC2CURT);
        }
    }
    else if(0)
    {
        /* 有效扇区最大值 */
        printf("AMP U:%.2f  ",(adc_amp_p[3]*ADC2VOL - 1.25f) * ADC2CURT);
        printf("AMP V:%.2f  ",(adc_amp_p[4]*ADC2VOL - 1.25f) * ADC2CURT);
        printf("AMP W:%.2f  ",(adc_amp_p[5]*ADC2VOL - 1.25f) * ADC2CURT);
    }
    else if(0)
    {
        /* 有效扇区均方根 */
        printf("AMP U:%.2f  ",(adc_amp_p[6]*ADC2VOL - 1.25f) * ADC2CURT);
        printf("AMP V:%.2f  ",(adc_amp_p[7]*ADC2VOL - 1.25f) * ADC2CURT);
        printf("AMP W:%.2f  ",(adc_amp_p[8]*ADC2VOL - 1.25f) * ADC2CURT);
    }
    else if(0)
    {
        /* 零电流 */
        printf("AMP U:%5d,%.2f  ",(int)bldc_m1_amp_zero[0],bldc_m1_amp_zero[0]*ADC2VOL);
        printf("AMP V:%5d,%.2f  ",(int)bldc_m1_amp_zero[1],bldc_m1_amp_zero[1]*ADC2VOL);
        printf("AMP W:%5d,%.2f  ",(int)bldc_m1_amp_zero[2],bldc_m1_amp_zero[2]*ADC2VOL);
    }
    else if(0)
    {
        /* 全周期均方根 */
        if(adc_agv_flag == 2)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
            printf("AMP U:%.2f  ",(calc_agv(adc_amp_agv[0],adc_amp_agv_p) - bldc_m1_amp_zero[0])*ADC2VOL);
            printf("AMP V:%.2f  ",(calc_agv(adc_amp_agv[1],adc_amp_agv_p) - bldc_m1_amp_zero[1])*ADC2VOL);
            printf("AMP W:%.2f  ",(calc_agv(adc_amp_agv[2],adc_amp_agv_p) - bldc_m1_amp_zero[2])*ADC2VOL);
            printf(" %d",adc_amp_agv_p);
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
            adc_agv_flag = 0;
        }
    }
#endif
    printf("\r\n");
    if(0)
    {
        debug_send_wave_data(1,adc1_normal_data_array[0]);
        debug_send_wave_data(2,adc1_normal_data_array[1]);
        debug_send_wave_data(3,adc1_normal_data_array[2]);
    }
}
