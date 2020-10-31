/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: MCU RTC timer and low power modes management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __RTC_BOARD_H__
#define __RTC_BOARD_H__

#include "type.h"

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint32_t TimerTime_t;
#endif

/*!
 * \brief Initializes the RTC timer
 *
 * \remark The timer is based on the RTC
 */
void RtcInit( void );

/*!
 * \brief Start the RTC timer
 *
 * \remark The timer is based on the RTC Alarm running at 32.768KHz
 *
 * \param[IN] timeout       Duration of the Timer
 */
void RtcSetTimeout( uint32_t timeout );

/*!
 * \brief Get the RTC timer value
 *
 * \retval time RTC Timer value
 */
TimerTime_t RtcGetTimerValue( void );

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval time RTC Elapsed time since the last alarm
 */
uint32_t RtcGetElapsedAlarmTime( void );

/*!
 * \brief Get the RTC timer elapsed time since the saved time
 *
 * \param[IN] savedTime       Saved time
 *
 * \retval time RTC elapsed time since the saved time
 */
TimerTime_t RtcComputeElapsedTime( uint32_t savedTime );

/*!
 * \brief Perfoms a standard blocking delay in the code execution
 *
 * \param [IN] delay Delay value in ms
 */
void RtcDelayMs( uint32_t delay );

#endif // __RTC_BOARD_H__
