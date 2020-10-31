#ifndef __BOARD_H__
#define __BOARD_H__

#include "eeprom.h"
#include "gpio.h"
#include "power.h"
#include "rtc.h"
#include "spi.h"
#include "uart.h"
#include "systick.h"
#include "sx1276_board.h"
#include "debug.h"
#include "delay.h"
#include "fifo.h"
#include "timer.h"
#include "utilities.h"
#include "privmac.h"

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0  
#endif

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

void LedRadioTxDataToggle( void );
void LedRadioRxDataToggle( void );
void LedRadioBeforeTxJionRequestShow( void );
void LedRadioTxJoinRequestShow( void );
void LedRadioRxJoinAcceptShow( void );
void LedRadioTxDataShow( void );
void LedRadioRxDataShow( void );
void SystemReset( uint8_t time );

/*!
 * \brief Initializes the target board peripherals
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t BoardGetBatteryLevel( void );

#endif // __BOARD_H__
