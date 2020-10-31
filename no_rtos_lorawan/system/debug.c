#include "debug.h"
#include "board.h"

uint8_t DebugStatus = DEBUG_LEVEL_ON;
uint8_t DebugLevel = DEBUG_LEVEL_NORMAL;
//uint8_t DebugLevel = DEBUG_LEVEL_DEVELOP;
//uint8_t DebugLevel = DEBUG_LEVEL_INFO;

int SetDebugStatus( uint8_t status )
{
    if( status > DEBUG_LEVEL_ON )
    {
        return -1;
    }
    else
    {
        BoardDisableIrq();
        DebugStatus = status;
        BoardEnableIrq();
        return 0;
    }
}

uint8_t GetDebugStatus( void )
{
    return DebugStatus;
}

int SetDebugLevel( uint8_t level )
{
    if( (level >= DEBUG_LEVEL_ERR) && (level <= DEBUG_LEVEL_INFO) )
    {
        BoardDisableIrq();
        DebugLevel = level;
        BoardEnableIrq();
        return 0;
    }
    
    return -1;
}

uint8_t GetDebugLevel( void )
{
    return DebugLevel;
}
