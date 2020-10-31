/*
sx1276驱动的定时器，在ports中进行具体实现。范例目前实现了系统定时器
该文件仅与rtt系统紧密相关
*/

#ifndef SX1276_TIMER_H_
#define SX1276_TIMER_H_

#include "rtthread.h"

struct SpecialTimerAPI_s
{
    //注册一个定时器，info参数是在定时器超时时传入的参数，主要目的是区别标识
    rt_timer_t (*Register)( const char *name, void *info ,void ( *callback )( void* parameter ));
    void (*Start)( rt_timer_t time );
    void (*Stop)(  rt_timer_t time ) ;
    void (*Reset)( rt_timer_t time);
    void (*SetValue)( rt_timer_t time, uint32_t value );
    //获取当前时间，单位ms
    rt_tick_t (*GetCurrentTime)( void );
    //获取差值，传入是ms，返回也是ms
    rt_tick_t (*GetElapsedTime)( rt_tick_t past );
    void (*DelayMs)( uint32_t ms );
};

//实例一个定时器控制器
extern int SpecialTimerInit(struct SpecialTimerAPI_s *time);


#endif
