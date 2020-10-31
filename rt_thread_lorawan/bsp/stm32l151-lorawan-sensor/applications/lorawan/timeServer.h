
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMESERVER_H__
#define __TIMESERVER_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "utilities.h" 
#include "rtdevice.h"

/* Exported types ------------------------------------------------------------*/

/*!
 * \brief Timer object description
 */
//typedef struct TimerEvent_s
//{
//    uint32_t Timestamp;         //! Expiring timer value in ticks from TimerContext
//    uint32_t ReloadValue;       //! Reload Value when Timer is restarted
//    bool IsRunning;             //! Is the timer currently running
//    void ( *Callback )( void ); //! Timer IRQ callback function
//    struct TimerEvent_s *Next;  //! Pointer to the next Timer object.
//} TimerEvent_t;
	 
typedef uint32_t TimerTime_t;

typedef struct TimerEvent_s
{
   char name[10];
	 rt_timer_t timeHander;
	 uint32_t  ReloadValue;
   void ( *Callback )( void* parameter ); 
	 bool IsRunning;             //! Is the timer currently running
	 
}TimerEvent_t;


/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/*!
 * \brief Initializes the timer object
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Structure containing the timer object parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit( TimerEvent_t *obj, void ( *callback )( void* parameter ) );

///*!
// * \brief Timer IRQ event handler
// *
// * \note Head Timer Object is automaitcally removed from the List
// *
// * \note e.g. it is snot needded to stop it
// */
//void TimerIrqHandler( void );

/*!
 * \brief Starts and adds the timer object to the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStart( TimerEvent_t *obj );

/*!
 * \brief Stops and removes the timer object from the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStop( TimerEvent_t *obj );

/*!
 * \brief Resets the timer object
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerReset( TimerEvent_t *obj );

/*!
 * \brief Set timer new timeout value
 *
 * \param [IN] obj   Structure containing the timer object parameters
 * \param [IN] value New timer timeout value
 */
void TimerSetValue( TimerEvent_t *obj, uint32_t value );

/*!
 * \brief Read the current time
 *
 * \retval returns current time in ms
 */
TimerTime_t TimerGetCurrentTime( void );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] savedTime    fix moment in Time
 * \retval time             returns elapsed time in ms
 */
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature );

#ifdef __cplusplus
}
#endif

#endif /* __TIMESERVER_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
