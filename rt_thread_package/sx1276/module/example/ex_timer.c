//该文件用于验证SX1276驱动定时器功能的实现
//该文件仅仅与rtt系统和SX1276时钟驱动相关

/*
测试方式:
	实例化一个定时器控制器后开启一个进程，进程中注册一个定时器，周期超时输出当前时间。同时进程中使用延时方式输出。
现象:
while flag每4s打印一次，在输入ex_timer_start时，10s后输出fzj cb flag，如果在图中输入ex_timer_stop则不会输出，如果输出ex_timer_reset，则会重新计时
*/

#include "rtthread.h"
#include "sx1276_timer.h"

#define DBG_ENABLE
#define LOG_TAG              "ex_timer"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <rtdbg.h>
//#include <ulog.h>

static struct SpecialTimerAPI_s timer_mg;
static rt_timer_t timer;

void ex_timer_cb(void *p)
{
    if(p != RT_NULL)
    {
			  LOG_D("%s cb flag :%dms",p,timer_mg.GetCurrentTime() );
		}
    else
    {  
		    LOG_D("cb p ==NULL");
		}			
}

void ex_timer_thread_entry(void *p)
{
	  char *flag = "fzj";
    timer = timer_mg.Register("ex_timer",flag,ex_timer_cb);
	  timer_mg.SetValue(timer,10000);  
	  while(1)
		{
		   timer_mg.DelayMs(4000);
			 LOG_D("while flag");
		}
}

void ex_timer_init()
{
    SpecialTimerInit(&timer_mg);
	  rt_thread_t ex_timer_hander;
	  ex_timer_hander = rt_thread_create("ex_t",ex_timer_thread_entry,RT_NULL,1024,2,10);
    if(ex_timer_hander !=RT_NULL)
		{
		    rt_thread_startup(ex_timer_hander);
			  LOG_D("ex_timer start");
		}
		else
		{
		    LOG_D("ex_timer error");
		}
}

void ex_timer_start()
{
   timer_mg.Start(timer);
}
MSH_CMD_EXPORT(ex_timer_start,ex_timer_start);

void ex_timer_stop()
{
  timer_mg.Stop(timer);
}
MSH_CMD_EXPORT(ex_timer_stop,ex_timer_stop);

void ex_timer_reset()
{
  timer_mg.Reset(timer);
}
MSH_CMD_EXPORT(ex_timer_reset,ex_timer_reset);
