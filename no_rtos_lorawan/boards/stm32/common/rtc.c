#include "rtc.h"
#include "board.h"

/*!
 * Min timeout value
 */
#define MIN_TIMEOUT                             (2)

/*!
 * Number of days per month 
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Leap year number of days per month 
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Flag to indicate if the RTC is Initialized
 */
static bool RtcInitalized = false;

/*!
 * Last alarm time stored
 */
static TimerTime_t RtcAlarmContext = 0;

/*!
 * MCU wake up time 
 */
static uint32_t McuWakeUpTime = 2;

/*!
 * RTC low poer object
 */
static LowPowerList_t RtcLowPower;

/*!
 * \brief Configurate the base RTC
 */
static void RtcSetConfig( void )
{
    RTC_InitTypeDef RTC_InitStructure;
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;

    /* Enable the PWR clock */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE );

    /* Allow access to RTC */
    PWR_RTCAccessCmd( ENABLE );

    /* Reset RTC Domain */
    RCC_RTCResetCmd( ENABLE );
    RCC_RTCResetCmd( DISABLE );

    /* Enable the LSE OSC */
    RCC_LSEConfig( RCC_LSE_ON );

    /* Wait till LSE is ready */  
    while( RCC_GetFlagStatus( RCC_FLAG_LSERDY ) == RESET )
    {}

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig( RCC_RTCCLKSource_LSE );

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd( ENABLE );
    
    /* Configure the RTC data register and RTC prescaler */
    RTC_InitStructure.RTC_AsynchPrediv = 3;
    RTC_InitStructure.RTC_SynchPrediv  = 3;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init( &RTC_InitStructure );
   
    RTC_TimeStructInit( &RTC_TimeStruct );
    RTC_DateStructInit( &RTC_DateStruct );
    
    RTC_SetDate( RTC_Format_BCD, &RTC_DateStruct );
    RTC_SetTime( RTC_Format_BCD, &RTC_TimeStruct );
}

/*!
 * \brief Configurate the RTC alarm
 */
static void RtcSetAlarmConfig( void )
{
    EXTI_InitTypeDef EXTI_InitStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* EXTI configuration */
    EXTI_ClearITPendingBit( EXTI_Line17 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    /* Enable the RTC Alarm Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    /* Set the alarmA Masks */
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
    RTC_SetAlarm( RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure );

    /* Disable AlarmA interrupt */
    RTC_ITConfig( RTC_IT_ALRA, DISABLE );

    /* Disable the alarmA */
    RTC_AlarmCmd( RTC_Alarm_A, DISABLE );
}

void RtcInit( void )
{
    if( RtcInitalized == false )
    {
        RtcSetConfig( );
        RtcSetAlarmConfig( );
        
        LowPowerListInsert( &RtcLowPower, LOW_POWER_MODE_STOP );
        
        RtcInitalized = true;
    }
}

/*!
 * \brief Calculate time from calendar
 */
static uint32_t RtcCalculateCalendarTime( RTC_DateTypeDef *date, RTC_TimeTypeDef *time )
{
    uint32_t leapsToDate;
    uint32_t lastYear;
    uint32_t day;
    uint32_t MonthOffset[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };
    uint32_t result;
    
    if( date->RTC_Year > 0 )
    {
        lastYear = date->RTC_Year - 1;
        leapsToDate = lastYear / 4 + 1;
        if( ((date->RTC_Year % 4) == 0) && (date->RTC_Month > 2) )
        {
            day = 1;
        }
        else
        {
            day = 0;
        }
    }
    else
    {
        leapsToDate = 0;
        if( date->RTC_Month > 2 )
        {
            day = 1;
        }
        else
        {
            day = 0;
        }
    }
    day += date->RTC_Year * 365 + leapsToDate + MonthOffset[date->RTC_Month-1] + date->RTC_Date - 1;
    result = ((((day * 24) + time->RTC_Hours) * 60 + time->RTC_Minutes) * 60 + time->RTC_Seconds) / 2.048 + 0.5;
//    DEBUG_INFO( " --Time:%02d-%02d-%02d %02d:%02d:%02d %dms \r\n", date->RTC_Year, date->RTC_Month, date->RTC_Date,
//            time->RTC_Hours, time->RTC_Minutes, time->RTC_Seconds, result );
    
    return result;
}

uint32_t RtcGetTimerValue( void )
{
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
    
    RTC_WaitForSynchro();
    RTC_GetTime( RTC_Format_BIN, &RTC_TimeStruct );
    RTC_GetDate( RTC_Format_BIN, &RTC_DateStruct );
    
    return RtcCalculateCalendarTime( &RTC_DateStruct, &RTC_TimeStruct );
}

/*!
 * \brief Stop alarm
 */
static void RtcStopAlarm( void )
{
    /* Clear RTC Alarm Flag */
    RTC_ClearFlag(RTC_FLAG_ALRAF);

    /* Disable RTC Alarm A Interrupt */
    RTC_ITConfig( RTC_IT_ALRA, DISABLE );
  
    /* Disable the Alarm A */
    RTC_AlarmCmd( RTC_Alarm_A, DISABLE );
}

/*!
 * \brief Sets the alarm clock to wake up after a period time
 */
static void RtcStartAlarm( uint32_t timeoutValue )
{
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    uint8_t rtcAlarmSeconds;
    uint8_t rtcAlarmMinutes;
    uint8_t rtcAlarmHours;
    uint8_t rtcAlarmDays;

    RtcStopAlarm();
    RTC_WaitForSynchro();
    RTC_GetTime( RTC_Format_BIN, &RTC_TimeStruct );
    RTC_GetDate( RTC_Format_BIN, &RTC_DateStruct );
    RtcAlarmContext = RtcCalculateCalendarTime( &RTC_DateStruct, &RTC_TimeStruct );
//    DEBUG_INFO( " --RtcAlarmContext:%dms \r\n", RtcAlarmContext );
    timeoutValue = timeoutValue * 2.048;
    if( timeoutValue > 2160000 ) //25day
    {
        rtcAlarmSeconds = RTC_TimeStruct.RTC_Seconds;
        rtcAlarmMinutes = RTC_TimeStruct.RTC_Minutes;
        rtcAlarmHours = RTC_TimeStruct.RTC_Hours;
        rtcAlarmDays = RTC_DateStruct.RTC_Date + 25;
    }
    else
    {
        uint8_t rtcSeconds = 0;
        uint8_t rtcMinutes = 0;
        uint8_t rtcHours = 0;
        uint8_t rtcDays = 0;
        
        rtcSeconds = (timeoutValue % 60) + RTC_TimeStruct.RTC_Seconds;
        timeoutValue /= 60;
        rtcMinutes = (timeoutValue % 60) + RTC_TimeStruct.RTC_Minutes;
        timeoutValue /= 60;
        rtcHours = (timeoutValue % 24) + RTC_TimeStruct.RTC_Hours;
        rtcDays = (timeoutValue / 24) + RTC_DateStruct.RTC_Date;
        
        rtcAlarmSeconds = rtcSeconds % 60;
        rtcMinutes += rtcSeconds / 60;
        rtcAlarmMinutes = rtcMinutes % 60;
        rtcHours += rtcMinutes / 60;
        rtcAlarmHours = rtcHours % 24;
        rtcDays += rtcHours / 24;
        rtcAlarmDays = rtcDays;
    }
    
    if( (RTC_DateStruct.RTC_Year % 4) == 0 )
    {
        if( rtcAlarmDays > DaysInMonthLeapYear[RTC_DateStruct.RTC_Month-1] )
        {
            rtcAlarmDays = rtcAlarmDays % DaysInMonthLeapYear[ RTC_DateStruct.RTC_Month - 1];
        }
    }
    else
    {
        if( rtcAlarmDays > DaysInMonth[ RTC_DateStruct.RTC_Month - 1 ] )
        {   
            rtcAlarmDays = rtcAlarmDays % DaysInMonth[ RTC_DateStruct.RTC_Month - 1];
        }
    }

    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = rtcAlarmSeconds;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = rtcAlarmMinutes;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = rtcAlarmHours;
    RTC_AlarmStructure.RTC_AlarmDateWeekDay = rtcAlarmDays;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = 0;
    RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_None;
    RTC_SetAlarm( RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure );

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro( );
    
    /* Enable RTC Alarm A Interrupt */
    RTC_ITConfig( RTC_IT_ALRA, ENABLE );
  
    /* Enable the Alarm A */
    RTC_AlarmCmd( RTC_Alarm_A, ENABLE );
    
//    DEBUG_INFO( " --Alarm set:%02d %02d:%02d:%02d %dms \r\n", rtcAlarmDays, rtcAlarmHours, rtcAlarmMinutes, rtcAlarmSeconds, RtcTimeoutContext );
//    RTC_GetAlarm( RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure );
//    printf( " --Alarm get:%02d %02d:%02d:%02d \r\n", rtcAlarmDays, rtcAlarmHours, rtcAlarmMinutes, rtcAlarmSeconds );
}

void RtcSetTimeout( uint32_t timeout )
{
    if( timeout > 0 )
    {
        LowPowerMode_t context;
        
        if( timeout < 50 )
        {
            if( timeout < MIN_TIMEOUT )
            {
                timeout = MIN_TIMEOUT;
            }
            McuUpdateLowPowerMode( &RtcLowPower, LOW_POWER_MODE_SLEEP, false );
        }
        else
        {
            McuUpdateLowPowerMode( &RtcLowPower, LOW_POWER_MODE_STOP, false );
        }
        context = GetLowPowerModeContext();
        if( context == LOW_POWER_MODE_STOP )
        {
            timeout -= McuWakeUpTime;
        }
        RtcStartAlarm( timeout );
    }
    else
    {
        McuUpdateLowPowerMode( &RtcLowPower, LOW_POWER_MODE_STOP, false );
    }
}

uint32_t RtcGetElapsedAlarmTime( void )
{
    uint32_t time;
    
    time = RtcGetTimerValue();
    if( time >= RtcAlarmContext )
    {
        time = time - RtcAlarmContext;
    }
    else
    {
        time = time + 1540898437 - RtcAlarmContext;
    }
    
    return time;
}

uint32_t RtcComputeElapsedTime( uint32_t savedTime )
{
    uint32_t time;
    
    time = RtcGetTimerValue();
    if( time >= savedTime )
    {
        time = time - savedTime;
    }
    else
    {
        time = time + 1540898437 - savedTime;
    }
    
    return time;
}

void RtcDelayMs( uint32_t ms )
{
    uint32_t time;
    
    time = RtcGetTimerValue();
//    DEBUG_INFO( " --delay %dms \r\n", ms );
    while( RtcComputeElapsedTime( time ) < ms )
        ;
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */
void RTC_Alarm_IRQHandler( void )
{
    if( RTC_GetITStatus(RTC_IT_ALRA) != RESET )
    {
        RTC_ClearITPendingBit( RTC_IT_ALRA );
        EXTI_ClearITPendingBit( EXTI_Line17 );
        McuRecoverFromLowPowerMode();
        TimerIrqHandler();
    }
}
