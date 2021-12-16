/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

_bldc_obj bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0,0,0,0,0};/*电机结构体*/

PF_MOS_CTR pFuncList_M1[6] =
{
    &M1_MOS_UHWLPWM, &M1_MOS_VHULPWM, &M1_MOS_VHWLPWM,
    &M1_MOS_WHVLPWM, &M1_MOS_UHVLPWM, &M1_MOS_WHULPWM
};

const uint8_t Hall_Table_CW[6] = {6,2,3,1,5,4};     /* 顺时针旋转表 */
const uint8_t Hall_Table_CCW[6] = {5,1,3,2,6,4};    /* 逆时针旋转表 */

const uint8_t Hall_CW_Table[12] = {0x62,0x23,0x31,0x15,0x54,0x46,0x63,0x21,0x35,0x14,0x56,0x42};
const uint8_t Hall_CCW_Table[12] = {0x45,0x51,0x13,0x32,0x26,0x64,0x41,0x53,0x12,0x36,0x24,0x65};

uint8_t check_hall_dir(_bldc_obj * obj)
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

void M1_MOS_UHVLPWM(void)
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
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    M1_ULL;/*U相下桥臂关闭*/
    M1_WLL;/*W相下桥臂关闭*/

    htim1.Instance->CCR1 = bldc_motor1.pwm_duty;/*U相上桥臂PWM*/
    M1_VLH;/*V相下桥臂导通*/
#endif
}

void M1_MOS_UHWLPWM(void)
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
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    M1_ULL;
    M1_VLL;

    htim1.Instance->CCR1 = bldc_motor1.pwm_duty;
    M1_WLH;
#endif
}

void M1_MOS_VHWLPWM(void)
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
    htim1.Instance->CCR1=0;
    htim1.Instance->CCR3=0;

    M1_ULL;
    M1_VLL;

    htim1.Instance->CCR2 = bldc_motor1.pwm_duty;
    M1_WLH;
#endif
}

void M1_MOS_VHULPWM(void)
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
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR3 = 0;

    M1_VLL;
    M1_WLL;

    htim1.Instance->CCR2 = bldc_motor1.pwm_duty;
    M1_ULH;
#endif
}


void M1_MOS_WHULPWM(void)
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
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR1 = 0;

    M1_VLL;
    M1_WLL;

    htim1.Instance->CCR3 = bldc_motor1.pwm_duty;
    M1_ULH;
#endif
}

void M1_MOS_WHVLPWM(void)
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
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR1 = 0;

    M1_ULL;
    M1_WLL;

    htim1.Instance->CCR3 = bldc_motor1.pwm_duty;
    M1_VLH;
#endif
}

uint32_t HallSensor_GetPinState(uint8_t motor_id)
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
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

    M1_ULL;
    M1_VLL;
    M1_WLL;
}

void Stop_Motor1(void)
{
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

void SoftReversing(uint8_t h_sta)
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t bldc_dir=0;
    static uint8_t hal_old_sta = 0xFF,l_cnt = 0;
    uint8_t hal_cur_sta = 0;
    if(htim->Instance == htim1.Instance)//55us
    {
        l_cnt ++;
        if(l_cnt == 100)
        {
            bldc_motor1.speed = (bldc_motor1.pos - bldc_motor1.pos_old);
            bldc_motor1.pos_old = bldc_motor1.pos;
            l_cnt = 0;
            printf("speed:%d\r\n",bldc_motor1.speed);
        }
#ifdef H_PWM_L_ON
        if(bldc_motor1.run_flag == RUN)
        {
            hal_cur_sta = HallSensor_GetPinState(MOTOR_1);
            if(hal_old_sta != hal_cur_sta)
            {
                SoftReversing(hal_cur_sta);
                hal_old_sta = hal_cur_sta;
                //HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
            }
            if(bldc_motor1.step_last != bldc_motor1.step_sta)
            {
                bldc_motor1.hall_keep_t=0;
                bldc_dir = check_hall_dir(&bldc_motor1);
                if(bldc_dir == CCW)
                {
                    bldc_motor1.pos -=1;
                }
                else if(bldc_dir == CW)
                {
                    bldc_motor1.pos +=1;
                }
                
                printf("pos:%d\r\n",bldc_motor1.pos);
                bldc_motor1.step_last = bldc_motor1.step_sta;
            }
            else if(bldc_motor1.run_flag == RUN)    /*运行且霍尔保持时 */
            {
                bldc_motor1.hall_keep_t++;//换向一次所需计数值（时间） 单位1/18k
            }
        }

#endif
    }
}

void BLDC_Ctrl(uint8_t motor_id,int32_t dir,uint16_t duty)
{
    if(motor_id == MOTOR_1)
    {
        if(duty == 0)
        {
            Stop_Motor1();
            bldc_motor1.pwm_duty = 0;
            bldc_motor1.run_flag = STOP;
            bldc_motor1.dir = dir;
        }
        else
        {
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
                if(bldc_motor1.dir != dir)
                {
                    uint16_t spd = bldc_motor1.pwm_duty;
                    Stop_Motor1();
                    bldc_motor1.pwm_duty = 0;
                    bldc_motor1.run_flag = STOP;

                    M1_ULH;
                    M1_VLH;
                    M1_WLH;
                    if(spd >= 200)
                    HAL_Delay(spd>>6);
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
                SoftReversing(HallSensor_GetPinState(MOTOR_1));
            }
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
    printf("sys run...\r\n");
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port,SHUTDOWN_Pin,GPIO_PIN_SET);
    HAL_TIM_Base_Start(&htim1);/*启动高级定时器1*/
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* TIM1_TRG_COM_TIM11_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
