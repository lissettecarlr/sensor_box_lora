#include "systick.h"
#include "board.h"

/*!
 * Systick timers list head pointer
 */
static SysTickTimer_t *SysTickTimerHead = NULL;

/*!
 * Flag to indicate if the systick timer is initialized
 */
static bool SysTickInitialized = false;

/*!
 * \brief Start the systik timer
 */
static void SysTickStart( void )
{
    if( SysTickInitialized == false )
    {
        if( SysTick_Config( SystemCoreClock / 1000 ) ) //初始化滴答定时器
        {
            while(1);
        }
        SysTickInitialized = true;
    }
}

/*!
 * \brief Stop the systik timer
 */
static void SysTickStop( void )
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //关闭滴答定时器
    SysTickInitialized = false;
}

void SysTickTimerInit( SysTickTimer_t *obj, void (* callback )( void ), char *name )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->Callback = callback;
    obj->name = name;
    obj->Next = NULL;
}

void SysTickTimerSetValue( SysTickTimer_t *obj, uint32_t value )
{
    obj->ReloadValue = value;
}


void SysTickTimerStart( SysTickTimer_t *obj )
{
    SysTickTimer_t **prev = &SysTickTimerHead;
    SysTickTimer_t *cur;
    uint32_t remain_time = obj->ReloadValue;
    
    BoardDisableIrq();
    
    if( obj == NULL )
    {
        BoardEnableIrq();
        return;
    }
    
    while( (cur = *prev) != NULL )
    {
        if( cur == obj )
        {
            BoardEnableIrq();
            return;
        }
        else
        {
            prev = &(cur->Next);
        }
    }
    
    prev = &SysTickTimerHead;
    while( (cur = *prev) != NULL )
    {
        if( remain_time <= cur->Timestamp )
        {
            cur->Timestamp -= remain_time;
            obj->Next = cur;
            obj->Timestamp = remain_time;
            *prev = obj;
            break;
        }
        else
        {
            remain_time -= cur->Timestamp;
            prev = &(cur->Next);
        }
    }
    
    if( *prev == NULL )
    {
        obj->Timestamp = remain_time;
        obj->Next = NULL;
        *prev = obj;
    }
    
    if( SysTickTimerHead != NULL )
    {
        SysTickStart();
    }
    
    BoardEnableIrq();
}

void SysTickTimerStop( SysTickTimer_t *obj )
{
    SysTickTimer_t **prev = &SysTickTimerHead;
    SysTickTimer_t *cur;
    
    BoardDisableIrq();
    
    if( obj == NULL )
    {
        BoardEnableIrq();
        return;
    }
    
    while( (cur = *prev) != NULL )
    {
        if( cur == obj )
        {
            if( cur->Next != NULL )
            {
                cur->Next->Timestamp += cur->Timestamp;
                *prev = cur->Next;
            }
            else
            {
                *prev = NULL;
            }
        }
        else
        {
            prev = &(cur->Next);
        }
    }
    
    if( SysTickTimerHead == NULL )
    {
        SysTickStop();
    }
    
    BoardEnableIrq();
}

void SysTickTimerReset( SysTickTimer_t *obj )
{
    SysTickTimerStop( obj );
    SysTickTimerStart( obj );
}

void SysTick_Handler( void )
{
    SysTickTimer_t **prev = &SysTickTimerHead;
    SysTickTimer_t *cur;
    
    BoardDisableIrq();
    cur = *prev;
    if( cur != NULL )
    {
        cur->Timestamp--;
        if( cur->Timestamp == 0 )
        {
            *prev = cur->Next;
            cur->Callback();
            
            while( (cur = *prev) != NULL )
            {
                if( cur->Timestamp == 0 )
                {
                    *prev = cur->Next;
                    cur->Callback();
                }
                else
                {
                    break;
                }
            }
        }
    }
    
    if( SysTickTimerHead == NULL )
    {
        SysTickStop();
    }
    
    BoardEnableIrq();
}
