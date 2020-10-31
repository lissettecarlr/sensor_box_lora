/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "timer.h"
#include "board.h"
#include "rtc.h"
#include "debug.h"

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj             Timer object to be become the new head
 * \param [IN]  remainingTime   Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj             Timer object to be added to the list
 * \param [IN]  remainingTime   Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

/*!
 * \brief Read the timer value of the currently running timer
 *
 * \retval value Current timer value
 */
static uint32_t TimerGetValue( void );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ), char *name )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
    obj->name = name;
    obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
    TimerTime_t elapsedTime = 0;
    TimerTime_t remainingTime = 0;

    BoardDisableIrq();

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        BoardEnableIrq();
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;
    
    if( obj->Timestamp == 0 )
    {
        if( obj->Callback != NULL )
        {
            obj->Callback( );
        }
        BoardEnableIrq();
        return;
    }

    if( TimerListHead == NULL )
    {
        TimerInsertNewHeadTimer( obj, obj->Timestamp );
    }
    else
    {
        if( TimerListHead->IsRunning == true )
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > TimerListHead->Timestamp )
            {
                elapsedTime = TimerListHead->Timestamp; // security but should never occur
            }
            remainingTime = TimerListHead->Timestamp - elapsedTime;
        }
        else
        {
            remainingTime = TimerListHead->Timestamp;
        }

        if( obj->Timestamp < remainingTime )
        {
            TimerInsertNewHeadTimer( obj, remainingTime );
        }
        else
        {
             TimerInsertTimer( obj, remainingTime );
        }
    }

    BoardEnableIrq();
}

static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    TimerTime_t aggregatedTimestamp = 0;      // hold the sum of timestamps
    TimerTime_t aggregatedTimestampNext = 0;  // hold the sum of timestamps up to the next event

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead->Next;

    if( cur == NULL )
    { // obj comes just after the head
        obj->Timestamp -= remainingTime;
        prev->Next = obj;
        obj->Next = NULL;
    }
    else
    {
        aggregatedTimestamp = remainingTime;
        aggregatedTimestampNext = remainingTime + cur->Timestamp;

        while( prev != NULL )
        {
            if( aggregatedTimestampNext > obj->Timestamp )
            {
                obj->Timestamp -= aggregatedTimestamp;
                if( cur != NULL )
                {
                    cur->Timestamp -= obj->Timestamp;
                }
                prev->Next = obj;
                obj->Next = cur;
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
                if( cur == NULL )
                { // obj comes at the end of the list
                    aggregatedTimestamp = aggregatedTimestampNext;
                    obj->Timestamp -= aggregatedTimestamp;
                    prev->Next = obj;
                    obj->Next = NULL;
                    break;
                }
                else
                {
                    aggregatedTimestamp = aggregatedTimestampNext;
                    aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp;
                }
            }
        }
    }
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    TimerEvent_t* cur = TimerListHead;

    if( cur != NULL )
    {
        cur->Timestamp = remainingTime - obj->Timestamp;
        cur->IsRunning = false;
    }

    obj->Next = cur;
    obj->IsRunning = true;
    TimerListHead = obj;
    TimerSetTimeout( TimerListHead );
}

void TimerIrqHandler( void )
{
    TimerEvent_t *elapsedTimer;
    uint32_t elapsedTime = 0;
    uint32_t compensation = 0;

    BoardDisableIrq();
    if( TimerListHead == NULL )
    {
        BoardEnableIrq();
        return;
    }
    
    elapsedTime = TimerGetValue();
    DEBUG_INFO( " <%s> elapsed time %dms, stamp time %dms \r\n", TimerListHead->name, elapsedTime, TimerListHead->Timestamp );

    if( elapsedTime > TimerListHead->Timestamp + 1 )
    {
        compensation = elapsedTime - TimerListHead->Timestamp;
        TimerListHead->Timestamp = 0;
        elapsedTimer = TimerListHead->Next;
        while( elapsedTimer != NULL )
        {
            if( compensation < elapsedTimer->Timestamp )
            {
                elapsedTimer->Timestamp -= compensation;
                break;
            }
            else
            {
                compensation -= elapsedTimer->Timestamp;
                elapsedTimer->Timestamp = 0;
                elapsedTimer = elapsedTimer->Next;
            }
        }
    }
    else if( elapsedTime < TimerListHead->Timestamp - 1 )
    {
        TimerListHead->Timestamp -= elapsedTime;
    }
    else
    {
        TimerListHead->Timestamp = 0;
    }

    TimerListHead->IsRunning = false;
    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp == 0 ) )
    {
        elapsedTimer = TimerListHead;
        TimerListHead = TimerListHead->Next;
        
        elapsedTimer->Next = NULL;
        if( elapsedTimer->Callback != NULL )
        {
            elapsedTimer->Callback( );
        }
    }

    // start the next TimerListHead if it exists
    if( TimerListHead != NULL )
    {
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );
        }
    }
    else
    {
        RtcSetTimeout( 0 );
    }
    BoardEnableIrq();
}

void TimerStop( TimerEvent_t *obj )
{
    TimerTime_t elapsedTime = 0;
    TimerTime_t remainingTime = 0;

    BoardDisableIrq();
    
    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead;
    
    // List is empty or the Obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        BoardEnableIrq();
        return;
    }

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsRunning == true ) // The head is already running
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > obj->Timestamp )
            {
                elapsedTime = obj->Timestamp;
            }

            remainingTime = obj->Timestamp - elapsedTime;

            if( TimerListHead->Next != NULL )
            {
                TimerListHead->IsRunning = false;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
                TimerListHead->IsRunning = true;
                TimerSetTimeout( TimerListHead );
            }
            else
            {
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
            {
                remainingTime = obj->Timestamp;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {
        remainingTime = obj->Timestamp;

        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                    cur->Timestamp += remainingTime;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
	
    BoardEnableIrq();
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    obj->Timestamp = value;
    obj->ReloadValue = value;
}

static uint32_t TimerGetValue( void )
{
    return RtcGetElapsedAlarmTime( );
}

TimerTime_t TimerGetCurrentTime( void )
{
    return RtcGetTimerValue( );
}

uint32_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    return RtcComputeElapsedTime( savedTime );
}

static void TimerSetTimeout( TimerEvent_t *obj )
{
    RtcSetTimeout( obj->Timestamp );
}

