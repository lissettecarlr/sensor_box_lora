#ifndef _SIX_BOX_PTO
#define _SIX_BOX_PTO

#include "stdint.h"
#include <string.h>

#define SB_MHAR 0XAA


typedef enum eSBCmd
{
    GET_DATA = 0x01,
}eSBCmd_t;

typedef enum eSBDataType
{
    SB_TYPE_DATA,
    SB_TYPE_GET,
}eSBDataType_t;

typedef union eSBCtrl
{
    uint8_t Value;
    struct sSBCtrlBits
    {
      uint8_t ACK        : 1;
      uint8_t NUL        : 7;  
    }Bits;
}eSBCtrl_t;

typedef struct sSBMhar
{
   //固定字节
   uint8_t mhar;
   uint8_t ver;
   eSBDataType_t type;
   eSBCtrl_t ctrl;
   uint16_t dataSize;
}sSBMhar_t;


void SB_init(void);
//type :数据类型  DataBuffer负载数据  SDBuffer封装完成的包
uint8_t SB_prepare_frame(eSBDataType_t type,uint8_t *DataBuffer,uint16_t DBsize,uint8_t *SDBuffer,uint16_t *SDsize);

//RcvBuffer 接收到的包   DataBuffer解包后的负载部分
uint8_t SB_disassemble(uint8_t *RcvBuffer,uint16_t RBsize,uint8_t *DataBuffer,uint16_t *DBsize);


#endif
