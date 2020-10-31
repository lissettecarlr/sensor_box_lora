#ifndef __POWER_H__
#define __POWER_H__

#include "type.h"

/*!
 * Low power supported modes
 */
typedef enum
{
    LOW_POWER_MODE_SLEEP = 1,
    LOW_POWER_MODE_STOP
} LowPowerMode_t;

/*!
 * \brief Low power object description
 */
typedef struct
{
    void *next;         //! Pointer to the next Low power object.
    LowPowerMode_t mode;  //! Current low power mode
} LowPowerList_t;


/*!
 * \brief Insert a low power object to the list
 *
 * \param [IN] p                Pointer to the low power object
 * \param [IN] mode             New low power mode
 */
void LowPowerListInsert( LowPowerList_t *p, LowPowerMode_t mode );

LowPowerMode_t GetLowPowerModeContext( void );

/*!
 * \brief Initializes the low power mode and other parameters
 *
 * \remark The function must be called when MCU initializing
 */
void McuLowPowerModeInit( void );

/*!
 * \brief Insert a low power object to the list
 *
 * \param [IN] p                Pointer to the low power object
 * \param [IN] mode             New low power mode
 * \param [IN] IsTimerUpdate    Wether or not to update timer
 */
void McuUpdateLowPowerMode( LowPowerList_t *p, LowPowerMode_t mode, bool IsTimerUpdate );

/*!
 * \brief Generate an interrupt to check if we have looped through the main enough times
 *        and go to stop mode
 */
void McuEnterLowPowerMode( void );

/*!
 * \brief Restore the MCU to its normal operation mode
 */
void McuRecoverFromLowPowerMode( void );

#endif
