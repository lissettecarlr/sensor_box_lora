#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "type.h"

/*!
 * Debug on or off status definitions
 */
#define DEBUG_LEVEL_ON                            1
#define DEBUG_LEVEL_OFF                           0

/*!
 * Debug level division definitions
 */
#define DEBUG_LEVEL_ERR                           1
#define DEBUG_LEVEL_NORMAL                        2
#define DEBUG_LEVEL_DEVELOP                       3
#define DEBUG_LEVEL_INFO                          4

#if 1
#define DEBUG printf
#define DEBUG_LEVEL( level, format, ... ) do { \
    if( DebugStatus && (level <= DebugLevel) ) { \
    if( level != DEBUG_LEVEL_ERR ) printf( format, ##__VA_ARGS__ ); \
    else printf( " location[%d,%s]"format, __LINE__, __func__, ##__VA_ARGS__ ); } \
    } while(0)
#define DEBUG_ERR( format, a... )                 DEBUG_LEVEL( DEBUG_LEVEL_ERR, format, ##a )
#define DEBUG_NORMAL( format, a... )              DEBUG_LEVEL( DEBUG_LEVEL_NORMAL, format, ##a )
#define DEBUG_DEVELOP( format, a... )             DEBUG_LEVEL( DEBUG_LEVEL_DEVELOP, format, ##a )
#define DEBUG_INFO( format, a... )                DEBUG_LEVEL( DEBUG_LEVEL_INFO, format, ##a )
#else
#define DEBUG( format, a... )
#define DEBUG_ERR( format, a... )
#define DEBUG_NORMAL( format, a... )
#define DEBUG_DEVELOP( format, a... )
#define DEBUG_INFO( format, a... )
#endif

/*!
 * Debug status definitions
 */
extern uint8_t DebugStatus;
    
/*!
 * Debug level definitions
 */
extern uint8_t DebugLevel;

/*!
 * Sets the debug status
 *
 * \param [IN] status         Debug status[1:on, 0:off]
 *
 * Result [0:success, -1:failure, parameter is error]
 */
int SetDebugStatus( uint8_t status );

/*!
 * Gets the debug status
 *
 * Result [1:on, 0:off]
 */
uint8_t GetDebugStatus( void );

/*!
 * Sets the debug level
 *
 * \param [IN] level         Debug level[1:error, 2:normal, 3:develop, 4:infomation]
 *
 * Result [0:success, -1:failure, parameter is error]
 */
int SetDebugLevel( uint8_t level );

/*!
 * Sets the debug level
 *
 * Result [1:error, 2:normal, 3:develop, 4:infomation]
 */
uint8_t GetDebugLevel( void );

#endif
