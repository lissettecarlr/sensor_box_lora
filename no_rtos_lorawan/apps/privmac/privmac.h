#ifndef __PRIMAC_H__
#define __PRIMAC_H__

#include "type.h"

typedef enum ePrivateMacMoteCmd
{
    MOTE_MAC_DEV_RESET_ANS          = 0x80,
} PrivateMoteCmd_t;

typedef enum ePrivateMacSrvCmd
{
    SRV_MAC_DEV_RESET_REQ           = 0x80,
} PrivateMacSrvCmd_t;

bool AddPrivMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t *buf, uint8_t *index, uint8_t bufLen );
bool ProcessPrivMacCommands( uint8_t cmd, uint8_t *payload, uint8_t *macIndex, uint8_t *buf, uint8_t *index, uint8_t bufLen );

#endif
