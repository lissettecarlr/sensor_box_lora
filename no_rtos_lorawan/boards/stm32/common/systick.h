#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "type.h"

/*!
 * \brief Systick timer object description
 */
typedef struct SysTickTimer_s
{
    uint32_t Timestamp;             //! Current timer value
    uint32_t ReloadValue;           //! Timer delay value
    void ( *Callback )( void );     //! Timer IRQ callback function
    char *name;                     //! Timer name
    struct SysTickTimer_s *Next;    //! Pointer to the next Timer object.
} SysTickTimer_t;

/*!
 * \brief Initializes the systick timer object
 *
 * \remark SysTickTimerInit function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Pointer to the systick timer object parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 * \param [IN] name         Pointer to the name of the systick timer object
 */
void SysTickTimerInit( SysTickTimer_t *obj, void (* callback )( void ), char *name );

/*!
 * \brief Set systick timer new timeout value
 *
 * \param [IN] obj      Structure containing the timer object parameters
 * \param [IN] value    New timer timeout value
 */
void SysTickTimerSetValue( SysTickTimer_t *obj, uint32_t value );

/*!
 * \brief Starts and adds the systick timer object to the list of timer events
 *
 * \param [IN] obj Structure containing the systick timer object parameters
 */
void SysTickTimerStart( SysTickTimer_t *obj );

/*!
 * \brief Stops and removes the systick timer object from the list of timer events
 *
 * \param [IN] obj Structure containing the systick timer object parameters
 */
void SysTickTimerStop( SysTickTimer_t *obj );

/*!
 * \brief Resets the systick timer object
 *
 * \param [IN] obj Structure containing the systick timer object parameters
 */
void SysTickTimerReset( SysTickTimer_t *obj );

#endif
