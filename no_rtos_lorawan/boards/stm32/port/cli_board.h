#ifndef __CLI_BOARD_H__
#define __CLI_BOARD_H__

#include "type.h"
#include "fifo.h"

/*!
 * Support manner of command line print
 */
typedef enum
{
    CLI_USE_UART = 0,
    CLI_USE_WIRELESS,
} CliUse_t;

/*!
 * \brief Initializes the output of command line print
 */
void CliBoardInit( void );

/*!
 * \brief De-initializes the output of command line print
 */
void CliBoardDeInit( void );

void CliBoardConfig( void );

#endif
