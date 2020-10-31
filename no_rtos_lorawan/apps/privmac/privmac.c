#include "board.h"
#include "LoRaMac.h"


#include "type.h"
#include "LoRaMac.h"
void TxEmptyPacket( void );


bool AddPrivMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t *buf, uint8_t *index, uint8_t bufLen )
{
    int valid = false;
    
    switch( cmd )
    {
        case MOTE_MAC_DEV_RESET_ANS:
            if( *index < bufLen )
            {
                buf[(*index)++] = cmd;
                valid = true;
            }
            break;
        
        default:
            break;
    }
    
    return valid;
}

bool ProcessPrivMacCommands( uint8_t cmd, uint8_t *payload, uint8_t *macIndex, uint8_t *buf, uint8_t *index, uint8_t bufLen )
{
    int valid = false;
    
    switch( cmd )
    {
        case SRV_MAC_DEV_RESET_REQ:
            AddMacCommand( MOTE_MAC_DEV_RESET_ANS, 0, 0 );
            TxEmptyPacket();
            DEBUG_NORMAL( " reset after 30s...\r\n\r\n\r\n" );
            DelayMs( 30 );
            SystemReset( 30 );
            valid = true;
            break;
        
        default:
            break;
    }
    
    return valid;
}
