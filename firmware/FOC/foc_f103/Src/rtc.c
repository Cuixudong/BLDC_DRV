/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

    /** Initialize RTC Only
    */
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

    if(rtcHandle->Instance==RTC)
    {
        /* USER CODE BEGIN RTC_MspInit 0 */

        /* USER CODE END RTC_MspInit 0 */
        HAL_PWR_EnableBkUpAccess();
        /* Enable BKP CLK enable for backup registers */
        __HAL_RCC_BKP_CLK_ENABLE();
        /* RTC clock enable */
        __HAL_RCC_RTC_ENABLE();
        /* USER CODE BEGIN RTC_MspInit 1 */

        /* USER CODE END RTC_MspInit 1 */
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

    if(rtcHandle->Instance==RTC)
    {
        /* USER CODE BEGIN RTC_MspDeInit 0 */

        /* USER CODE END RTC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_RTC_DISABLE();
        /* USER CODE BEGIN RTC_MspDeInit 1 */

        /* USER CODE END RTC_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
RTC_Time time; // 时间
RTC_Date date;	// 日期

/* 函数名称：void RTC_Init(void)
 * 功能描述：RTC初始化
 * 参数：无
 * 返回值：无
 */
void RTC_Init(void)
{
    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0xA5A2)	// 掉电了
    {
        // 重新写入时间
        RTCSet(2019,12,29,16,2,0);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xA5A2);
    }
}

/* 函数名称：uint8_t Is_Leap_Year(uint16_t year)
 * 功能描述：判断输入的年是否为闰年
 * 参数：year ---- 待判断的年
 * 返回值：1：是闰年   0：不是闰年
 * 备注：
 * 月份   1  2  3  4  5  6  7  8  9  10 11 12
 * 闰年		31 29 31 30 31 30 31 31 30 31 30 31
 * 非闰年	31 28 31 30 31 30 31 31 30 31 30 31
 */
uint8_t Is_Leap_Year(uint16_t year)
{
    if(year%4==0) // 必须能被4整除
    {
        if(year%100==0) // 能被100整除
        {
            if(year%400==0)return 1;// 也能被400整除
            else return 0;
        } else return 1;
    } else return 0;
}

// 月份数据表
const uint8_t table_week[12]= {0,3,3,6,1,4,6,2,5,0,3,5}; // 月修正表
// 平年的月份日期表
const uint8_t mon_table[12]= {31,28,31,30,31,30,31,31,30,31,30,31};

/* 函数名称：uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
 * 功能描述：判断输入的日期是星期几(1901-2099年)
 * 参数：year：年 month：月 day日
 * 返回值：星期1-6 星期天为0
 * 备注：
 */
uint8_t RTCGetWeek(uint16_t year,uint8_t month,uint8_t day)
{
    uint16_t temp2;
    uint8_t yearH,yearL;
    yearH=year/100;
    yearL=year%100;
    if (yearH>19)yearL+=100;				// 如果为21世纪，年份加100
    temp2=yearL+yearL/4;						// 所过闰年数只算1900年以后的
    temp2=temp2%7;
    temp2=temp2+day+table_week[month-1];
    if (yearL%4==0&&month<3)temp2--;
    return(temp2%7);
}

/* 函数名称：void RTC_Write_Cnt(uint32_t cnt)
 * 功能描述：把cnt写入到RTC的寄存器中
 * 参数：cnt ---- 代写入的值
 * 返回值：无
 * 备注：RTC有写保护 操作之前要去掉写保护才能写入
 */
void RTC_Write_Cnt(uint32_t cnt)
{
    // 等待写操作结束
    while((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET);
    // 进入配置模式
    hrtc.Instance->CRL |= 1<<4;
//	printf("Wcnt:[%d].\r\n", cnt);
    // 把这个值写入到RTC的计数器中去
    hrtc.Instance->CNTH = cnt>>16;			// 写入高16位
    hrtc.Instance->CNTL = (cnt&0xffff);	// 写入低16位
//	printf("cntH:[%d].\r\n", hrtc.Instance->CNTH);
//	printf("cntL:[%d].\r\n", hrtc.Instance->CNTL);
//	printf("cntL:[%d].\r\n", cnt&0xffff);
    // 退出配置模式
    hrtc.Instance->CRL &= ~(1<<4);
    // 等待写操作结束
    while((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET);
}

/* 函数名称：uint8_t RTC_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
 * 功能描述：设置RTC日期和时间
 * 参数：year：年 month：月 day日
 * 返回值：0：成功   其他：错误
 * 备注：把输入的时间归算成从1970年1月1日0时0分0秒开始的秒数存入RTC的计数器中
 */
uint8_t RTCSet(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
    uint16_t t;
    uint32_t seccount=0;
    // 从1970年开始计算 2099年计算出的秒溢出 所以合法年范围在1970-2099年
    if(syear<1970||syear>2099)return 1;
    // 累加从1970到设置年的所有秒
    for(t=1970; t<syear; t++)
    {
        if(Is_Leap_Year(t))seccount+=31622400;			// 如果是闰年就多一天 366天对应的秒31622400
        else seccount+=31536000;						// 如果是平年 就是365天对应31536000
    }
    // 累加月份对应的秒
    smon-=1;
    for(t=0; t<smon; t++)
    {
        seccount+=(uint32_t)mon_table[t]*86400;			// 平年每月天数*一天的秒数(86400)
        if(Is_Leap_Year(syear)&&t==1)seccount+=86400;	// 如果是闰年的2月 就29天 多加一天的秒数
    }
    // 累加天对应的秒
    seccount+=(uint32_t)(sday-1)*86400;
    // 累加小时对应的秒
    seccount+=(uint32_t)hour*3600;
    // 累加分钟对应的秒
    seccount+=(uint32_t)min*60;
    // 累加最后的秒
    seccount+=sec;
    // 把值写入寄存器
    RTC_Write_Cnt(seccount);
    return 0;
}


/* 函数名称：void RTC_Get(void)
 * 功能描述：获取当前时间
 * 参数：无
 * 返回值：无
 * 备注： 无
 */
void RTCGet(void)
{
    static uint32_t temp_day_bak = 0;
    uint32_t rtc_cnt = 0;
    uint32_t temp = 0;
    uint16_t temp_num = 0;
    // 获取RTC的计数
    rtc_cnt = hrtc.Instance->CNTH;
    rtc_cnt = rtc_cnt<<16 | hrtc.Instance->CNTL;
    temp = rtc_cnt/86400;												// 计算成天数
    if(temp_day_bak!=temp)												// 过了一天了 开始计算年月日周等
    {
        temp_day_bak = temp;
        temp_num = 1970;												// 使用LINUX时间戳 从1970年1月1日0时0分开始计算
        while(temp>=365)
        {
            if(Is_Leap_Year(temp_num))									// 是闰年
            {
                if(temp>=366)
                {
                    temp -= 366;										// 闰年一年366天
                }
                else
                {
                    temp_num ++;
                    break;
                }
            }
            else														// 不是闰年
            {
                temp -= 365;											// 平年一年365天
            }
            temp_num++;
        }
        date.Year = temp_num;											// 得到年份
        temp_num = 0;
        // temp_day被减得小于一年的天数了 开始计算月份
        while(temp>=28)
        {
            if(Is_Leap_Year(temp_num) && temp_num == 1)					// 是当年闰年的2月份
            {
                if(temp>=29)
                {
                    temp -= 29;	// 闰年的2月有29天
                }
                else
                    break;
            }
            else
            {
                if(temp>=mon_table[temp_num])temp-=mon_table[temp_num];	//平年
                else break;
            }
            temp_num ++;
        }
        date.Month = temp_num+1;
        date.Date = temp+1;
    }
    // 没有更新天 就更新时间就可以了
    temp = rtc_cnt%86400;												// 计算成秒
    time.Hours = temp/3600;												// 小时
    time.Minutes = (temp%3600)/60;										// 分钟
    time.Seconds = (temp%3600)%60;										// 秒
    date.WeekDay = RTCGetWeek(date.Year, date.Month, date.Date);
//	printf("现在时间是%d年-%d月-%d日 星期%d   %d:%d:%d.\r\n", date.Year, date.Month, date.Date, date.WeekDay,
//	time.Hours, time.Minutes, time.Seconds);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
