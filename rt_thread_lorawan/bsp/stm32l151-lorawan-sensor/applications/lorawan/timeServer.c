

#include "timeServer.h"

void TimerInit( TimerEvent_t *obj, void ( *callback )( void* parameter ) )
{
	
	 obj->timeHander  =      rt_timer_create(obj->name,  /* 定时器名字是 timer1 */
                                           callback, /* 超时时回调的处理函数 */
                                           RT_NULL,  /* 超时函数的入口参数 */
                                           obj->ReloadValue,       /* 定时长度，以OS Tick为单位，即10个OS Tick */
                                           RT_TIMER_FLAG_ONE_SHOT); /* 单次定时器*/
	obj->Callback = callback;
	obj->IsRunning = false;
	
}

void TimerStart( TimerEvent_t *obj )
{
    if (obj->timeHander != RT_NULL) 
		{
			rt_timer_start(obj->timeHander);
			obj->IsRunning = true;
		}
}


//void TimerIrqHandler( void )
//{
//}

void TimerStop( TimerEvent_t *obj ) 
{
	 if (obj->timeHander != RT_NULL) 
	 {
			rt_timer_stop(obj->timeHander);
		  obj->IsRunning = false;
	 }
}  
  


void TimerReset( TimerEvent_t *obj )
{
   TimerStop(obj);
	 TimerStart(obj);
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
	//参数是ms，传入tick，相差10倍
	 value = value/(1000/RT_TICK_PER_SECOND);
   rt_timer_control(obj->timeHander, RT_TIMER_CTRL_SET_TIME, (void*)&value);
}

//获取一个当前时间，返回ms
TimerTime_t TimerGetCurrentTime( void )
{
	rt_tick_t temp;
	temp = rt_tick_get();//得到TTCK 
	temp*=(1000/RT_TICK_PER_SECOND);
  return temp; //返回ms
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
	rt_tick_t nowInTicks = rt_tick_get();
	rt_tick_t pastInTicks= rt_tick_from_millisecond(past);
	return ((nowInTicks - pastInTicks)*(1000/RT_TICK_PER_SECOND));
}

//static bool TimerExists( TimerEvent_t *obj )
//{
//   return 0;
//}
//static void TimerSetTimeout( TimerEvent_t *obj )
//{
//  
//}

//TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
//{
//    return 0;
//}


//static void TimerInsertTimer( TimerEvent_t *obj)
//{
//  
//}

//static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
//{
// 
//}

