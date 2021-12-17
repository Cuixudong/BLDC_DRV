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
RTC_Time time; // ʱ��
RTC_Date date;	// ����

/* �������ƣ�void RTC_Init(void)
 * ����������RTC��ʼ��
 * ��������
 * ����ֵ����
 */
void RTC_Init(void)
{
    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0xA5A2)	// ������
    {
        // ����д��ʱ��
        RTCSet(2019,12,29,16,2,0);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xA5A2);
    }
}

/* �������ƣ�uint8_t Is_Leap_Year(uint16_t year)
 * �����������ж���������Ƿ�Ϊ����
 * ������year ---- ���жϵ���
 * ����ֵ��1��������   0����������
 * ��ע��
 * �·�   1  2  3  4  5  6  7  8  9  10 11 12
 * ����		31 29 31 30 31 30 31 31 30 31 30 31
 * ������	31 28 31 30 31 30 31 31 30 31 30 31
 */
uint8_t Is_Leap_Year(uint16_t year)
{
    if(year%4==0) // �����ܱ�4����
    {
        if(year%100==0) // �ܱ�100����
        {
            if(year%400==0)return 1;// Ҳ�ܱ�400����
            else return 0;
        } else return 1;
    } else return 0;
}

// �·����ݱ�
const uint8_t table_week[12]= {0,3,3,6,1,4,6,2,5,0,3,5}; // ��������
// ƽ����·����ڱ�
const uint8_t mon_table[12]= {31,28,31,30,31,30,31,31,30,31,30,31};

/* �������ƣ�uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
 * �����������ж���������������ڼ�(1901-2099��)
 * ������year���� month���� day��
 * ����ֵ������1-6 ������Ϊ0
 * ��ע��
 */
uint8_t RTCGetWeek(uint16_t year,uint8_t month,uint8_t day)
{
    uint16_t temp2;
    uint8_t yearH,yearL;
    yearH=year/100;
    yearL=year%100;
    if (yearH>19)yearL+=100;				// ���Ϊ21���ͣ���ݼ�100
    temp2=yearL+yearL/4;						// ����������ֻ��1900���Ժ��
    temp2=temp2%7;
    temp2=temp2+day+table_week[month-1];
    if (yearL%4==0&&month<3)temp2--;
    return(temp2%7);
}

/* �������ƣ�void RTC_Write_Cnt(uint32_t cnt)
 * ������������cntд�뵽RTC�ļĴ�����
 * ������cnt ---- ��д���ֵ
 * ����ֵ����
 * ��ע��RTC��д���� ����֮ǰҪȥ��д��������д��
 */
void RTC_Write_Cnt(uint32_t cnt)
{
    // �ȴ�д��������
    while((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET);
    // ��������ģʽ
    hrtc.Instance->CRL |= 1<<4;
//	printf("Wcnt:[%d].\r\n", cnt);
    // �����ֵд�뵽RTC�ļ�������ȥ
    hrtc.Instance->CNTH = cnt>>16;			// д���16λ
    hrtc.Instance->CNTL = (cnt&0xffff);	// д���16λ
//	printf("cntH:[%d].\r\n", hrtc.Instance->CNTH);
//	printf("cntL:[%d].\r\n", hrtc.Instance->CNTL);
//	printf("cntL:[%d].\r\n", cnt&0xffff);
    // �˳�����ģʽ
    hrtc.Instance->CRL &= ~(1<<4);
    // �ȴ�д��������
    while((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET);
}

/* �������ƣ�uint8_t RTC_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
 * ��������������RTC���ں�ʱ��
 * ������year���� month���� day��
 * ����ֵ��0���ɹ�   ����������
 * ��ע���������ʱ�����ɴ�1970��1��1��0ʱ0��0�뿪ʼ����������RTC�ļ�������
 */
uint8_t RTCSet(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
    uint16_t t;
    uint32_t seccount=0;
    // ��1970�꿪ʼ���� 2099������������� ���ԺϷ��귶Χ��1970-2099��
    if(syear<1970||syear>2099)return 1;
    // �ۼӴ�1970���������������
    for(t=1970; t<syear; t++)
    {
        if(Is_Leap_Year(t))seccount+=31622400;			// ���������Ͷ�һ�� 366���Ӧ����31622400
        else seccount+=31536000;						// �����ƽ�� ����365���Ӧ31536000
    }
    // �ۼ��·ݶ�Ӧ����
    smon-=1;
    for(t=0; t<smon; t++)
    {
        seccount+=(uint32_t)mon_table[t]*86400;			// ƽ��ÿ������*һ�������(86400)
        if(Is_Leap_Year(syear)&&t==1)seccount+=86400;	// ����������2�� ��29�� ���һ�������
    }
    // �ۼ����Ӧ����
    seccount+=(uint32_t)(sday-1)*86400;
    // �ۼ�Сʱ��Ӧ����
    seccount+=(uint32_t)hour*3600;
    // �ۼӷ��Ӷ�Ӧ����
    seccount+=(uint32_t)min*60;
    // �ۼ�������
    seccount+=sec;
    // ��ֵд��Ĵ���
    RTC_Write_Cnt(seccount);
    return 0;
}


/* �������ƣ�void RTC_Get(void)
 * ������������ȡ��ǰʱ��
 * ��������
 * ����ֵ����
 * ��ע�� ��
 */
void RTCGet(void)
{
    static uint32_t temp_day_bak = 0;
    uint32_t rtc_cnt = 0;
    uint32_t temp = 0;
    uint16_t temp_num = 0;
    // ��ȡRTC�ļ���
    rtc_cnt = hrtc.Instance->CNTH;
    rtc_cnt = rtc_cnt<<16 | hrtc.Instance->CNTL;
    temp = rtc_cnt/86400;												// ���������
    if(temp_day_bak!=temp)												// ����һ���� ��ʼ�����������ܵ�
    {
        temp_day_bak = temp;
        temp_num = 1970;												// ʹ��LINUXʱ��� ��1970��1��1��0ʱ0�ֿ�ʼ����
        while(temp>=365)
        {
            if(Is_Leap_Year(temp_num))									// ������
            {
                if(temp>=366)
                {
                    temp -= 366;										// ����һ��366��
                }
                else
                {
                    temp_num ++;
                    break;
                }
            }
            else														// ��������
            {
                temp -= 365;											// ƽ��һ��365��
            }
            temp_num++;
        }
        date.Year = temp_num;											// �õ����
        temp_num = 0;
        // temp_day������С��һ��������� ��ʼ�����·�
        while(temp>=28)
        {
            if(Is_Leap_Year(temp_num) && temp_num == 1)					// �ǵ��������2�·�
            {
                if(temp>=29)
                {
                    temp -= 29;	// �����2����29��
                }
                else
                    break;
            }
            else
            {
                if(temp>=mon_table[temp_num])temp-=mon_table[temp_num];	//ƽ��
                else break;
            }
            temp_num ++;
        }
        date.Month = temp_num+1;
        date.Date = temp+1;
    }
    // û�и����� �͸���ʱ��Ϳ�����
    temp = rtc_cnt%86400;												// �������
    time.Hours = temp/3600;												// Сʱ
    time.Minutes = (temp%3600)/60;										// ����
    time.Seconds = (temp%3600)%60;										// ��
    date.WeekDay = RTCGetWeek(date.Year, date.Month, date.Date);
//	printf("����ʱ����%d��-%d��-%d�� ����%d   %d:%d:%d.\r\n", date.Year, date.Month, date.Date, date.WeekDay,
//	time.Hours, time.Minutes, time.Seconds);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
