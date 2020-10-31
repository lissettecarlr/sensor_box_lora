
#include "stdint.h"

//byte_handle.c
uint8_t Util_hiUint16(uint16_t a);
uint8_t Util_loUint16(uint16_t a);
uint8_t *Util_bufferUint16(uint8_t *pBuf, uint16_t val);
uint16_t Util_buildUint16(uint8_t loByte, uint8_t hiByte);
uint16_t Util_parseUint16(uint8_t *pArray);
uint8_t Util_breakUint32(uint32_t var, int byteNum);
uint8_t *Util_bufferUint32(uint8_t *pBuf, uint32_t val);
uint32_t Util_buildUint32(uint8_t byte0, uint8_t byte1, uint8_t byte2,uint8_t byte3);
uint32_t Util_parseUint32(uint8_t *pArray);
//crc.c
uint16_t modbus_crc(uint8_t *modbus_buff,uint8_t size);