#ifndef __API_H__
#define __API_H__

#include "type.h"
#include "LoRaMac.h"

void TxEmptyPacket( void );
int CallBack_TxPayloadFrame(uint8_t* Port, uint8_t *Buf, uint8_t* Len, bool *Confirmed);
void CallBack_RxPayloadFrame(McpsIndication_t *McpsIndication);

#endif
