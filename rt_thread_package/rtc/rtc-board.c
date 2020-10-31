/*
LSE : 32.768Khz

*/

#include "stm32l1xx.h"
#include "rtthread.h"
#include "low_power.h"


RTC_HandleTypeDef rtc_hander;
RTC_AlarmTypeDef sAlarm = {0};

static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };


//void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
//{
//  if(rtcHandle->Instance==RTC)
//  {
//    __HAL_RCC_RTC_ENABLE();
//    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
//  }
//}

//void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
//{

//  if(rtcHandle->Instance==RTC)
//  {
//    __HAL_RCC_RTC_DISABLE();
//    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
//  }
//} 

static void (*rtc_callback)(void);


void HAL_RTC_AlarmAEventCallback( RTC_HandleTypeDef *hrtc )
{
	
	 if(rtc_callback != NULL)
	 {
			 rtc_callback();
	 }	 
}


void  rtc_register_callback(void (*cb)(void))
{
   rtc_callback = cb;
}


void RTC_Alarm_IRQHandler(void)
{
	  enter_stop_wk();
    HAL_RTC_AlarmIRQHandler(&rtc_hander);
}

void rtc_display()
{
		RTC_TimeTypeDef time;
		RTC_DateTypeDef data;
    HAL_RTC_GetTime(&rtc_hander,&time,  RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&rtc_hander,&data,  RTC_FORMAT_BIN);		
		rt_kprintf("year %d|| Month %d|| Date %d\n",data.Year,data.Month,data.Date);
		rt_kprintf("Hours %d|| Minutes %d|| Seconds:%d\n",time.Hours,time.Minutes,time.Seconds);
}
MSH_CMD_EXPORT(rtc_display,rtc_display)

void rtc_init()
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /**Initialize RTC Only 
  */
  rtc_hander.Instance = RTC;
  rtc_hander.Init.HourFormat = RTC_HOURFORMAT_24;
  rtc_hander.Init.AsynchPrediv = 127;
  rtc_hander.Init.SynchPrediv = 255;
  rtc_hander.Init.OutPut = RTC_OUTPUT_DISABLE;
  rtc_hander.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  rtc_hander.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&rtc_hander) != HAL_OK)
  {
      
  }
	//配置时分秒
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&rtc_hander, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
     //
  }
	//年月日
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x01;
  sDate.Year = 0x00;

  if (HAL_RTC_SetDate(&rtc_hander, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    //Error_Handler();
  }
  HAL_RTC_DeactivateAlarm( &rtc_hander, RTC_ALARM_A );
}

//关闭闹钟
void rtc_close_alarm()
{
    // Disable the Alarm A interrupt
    HAL_RTC_DeactivateAlarm( &rtc_hander, RTC_ALARM_A );

    // Clear RTC Alarm Flag
    __HAL_RTC_ALARM_CLEAR_FLAG( &rtc_hander, RTC_FLAG_ALRAF );

    // Clear the EXTI's line Flag for RTC Alarm
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );

}

extern void GenerateSystemReset(void);

//设定闹钟，timeout表示经过的s
void rtc_set_alarm(uint32_t timeout)
{
    uint16_t Seconds = 0;
    uint16_t Minutes = 0;
    uint16_t Hours = 0;
    uint16_t Days = 0;
	
	  RTC_TimeTypeDef now_time;
	  RTC_DateTypeDef now_data;
	  
	  rtc_close_alarm();
	
		if(timeout <=0) //传入为0则不再唤醒
		{
			  return;
		}
	
	  //获取当前时间
	  HAL_RTC_GetTime(&rtc_hander,&now_time,  RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&rtc_hander,&now_data,  RTC_FORMAT_BIN);	
	
		//每隔10天复位一次
		if(now_data.Date >=10)
		{
		     GenerateSystemReset();
		}
		
		
	  //计算闹钟时间
	  Days = now_data.Date;
	  while( timeout >= 86400 )
    {
        timeout -= 86400;
        Days++;
    }
		
	  Hours = now_time.Hours;
		while(timeout >= 3600)
		{
		   timeout -= 3600;
			 Hours ++;
		}
		
		Minutes = now_time.Minutes;
		while(timeout>= 60)
		{
		   timeout-= 60;
			 Minutes ++;
		}
		Seconds = now_time.Seconds + timeout;
	  
		//修正时间
    while( Seconds >= 60 )
    { 
        Seconds -= 60;
        Minutes++;
    }
    while( Minutes >= 60 )
    {
        Minutes -= 60;
        Hours++;
    }
    while( Hours >= 24 )
    {
        Hours -= 24;
        Days++;
    }		
		if(now_data.Year %4 == 0) //闰年
		{
		    if(Days > DaysInMonthLeapYear[now_data.Month -1])
				{
				    Days = Days & DaysInMonthLeapYear[now_data.Month -1]; //跨月了
				}
		}
		else
		{
				if(Days > DaysInMonth[now_data.Month -1])
				{
				    Days = Days & DaysInMonth[now_data.Month -1]; //跨月了
				}
			
		}
	
	rt_kprintf("\n-----------------------------\n");	
  rtc_display();		
	rt_kprintf("alarm: Day,Hour,Min,Sec :%d,%d,%d,%d\n",Days,Hours,Minutes,Seconds);	
	rt_kprintf("-----------------------------\n");		
		
  sAlarm.AlarmTime.Hours =   Hours;
  sAlarm.AlarmTime.Minutes = Minutes;
  sAlarm.AlarmTime.Seconds = Seconds;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = ( uint8_t )Days;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  //sAlarm.AlarmMask = RTC_ALARMMASK_NONE; //RTC_AlarmMask_DateWeekDay
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY; //只比对时分秒	
  sAlarm.Alarm = RTC_ALARM_A;
	
  HAL_RTC_SetAlarm_IT(&rtc_hander, &sAlarm, RTC_FORMAT_BIN);

}

/*测试用函数*/

//void rtc_test_add_one_data()
//{

//}

//void rtc_test_calculate_alarm()
//{

//}

