/*
该文件实现了SX1276驱动的时钟

*/

#include "rtthread.h"
#include "sx1276_timer.h"

#define LOG_TAG              "76_tm"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <rtdbg.h>

//系统时钟

rt_timer_t SystimeRegister(const char *name,void *info ,void ( *callback )( void* parameter ) )
{
    rt_timer_t timeHander; 
    timeHander = rt_timer_create(name,      /* 定时器名字是 timer1 */
                                callback,   /* 超时时回调的处理函数 */
                                info,       /* 超时函数的入口参数 */
                                1000,       /* 定时长度，以OS Tick为单位，即10个OS Tick */
                                RT_TIMER_FLAG_ONE_SHOT); /* 单次定时器*/
	
    return timeHander;
}


void SystimeStart( rt_timer_t time )
{
    if (time != RT_NULL) 
            rt_timer_start(time);
}

void SystimeStop(  rt_timer_t time )
{
    if (time != RT_NULL) 
        rt_timer_stop(time);
}

void SystimeReset( rt_timer_t time)
{
    SystimeStop(time);
    SystimeStart(time);
}

void SystimeSetValue( rt_timer_t time, uint32_t value )
{
    value = value/(1000/RT_TICK_PER_SECOND);
    rt_timer_control(time, RT_TIMER_CTRL_SET_TIME, (void*)&value);

}

rt_tick_t SystimeGetCurrentTime( void )
{
    rt_tick_t temp;
    temp = rt_tick_get();//得到TTCK 
    temp*=(1000/RT_TICK_PER_SECOND);
    return temp; //返回ms
}

rt_tick_t SystimeGetElapsedTime( rt_tick_t past )
{
    rt_tick_t nowInTicks = rt_tick_get();
    rt_tick_t pastInTicks= rt_tick_from_millisecond(past);
    return ((nowInTicks - pastInTicks)*(1000/RT_TICK_PER_SECOND));
}

void SystimeDelayMs( uint32_t ms )
{
    rt_thread_mdelay(ms);
}

int SpecialTimerInit(struct SpecialTimerAPI_s * time)
{
    time->Register = SystimeRegister;
    time->Start = SystimeStart;
    time->Stop = SystimeStop;
    time->Reset = SystimeReset;
    time->SetValue = SystimeSetValue;
    time->GetCurrentTime = SystimeGetCurrentTime;
    time->GetElapsedTime = SystimeGetElapsedTime;
    time->DelayMs = SystimeDelayMs;
    return 0;
}
