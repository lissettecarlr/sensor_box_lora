//用于byte的拆分或合并
//2018年7月19日10:15:35
//UTF-8

#include "stdint.h"
/*!
 Get the high byte of a uint16_t variable
 获取16位数据的高位
 */
uint8_t Util_hiUint16(uint16_t a)
{
    return((a >> 8) & 0xFF);
}

/*!
 Get the low byte of a uint16_t variable
 获取16位数据的地位
 */
uint8_t Util_loUint16(uint16_t a)
{
    return((a) & 0xFF);
}

/*
拼接两个byte组成一个16位的值
pBuf :数据指针
val  :拼接后返回值
return :如果传入的是一串数据，则返回使用后指针新的指向
*/

uint8_t *Util_bufferUint16(uint8_t *pBuf, uint16_t val)
{
    *pBuf++ = Util_loUint16(val);
    *pBuf++ = Util_hiUint16(val);
    return(pBuf);
}

//将两个8位组成一个十六位
//loByte :16位地位数据
//hiByte :16位高位数据
//return :拼接后数据
uint16_t Util_buildUint16(uint8_t loByte, uint8_t hiByte)
{
    return((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)));
}

//拼接该指针的首位和次位数据，首位位是16位的地位
uint16_t Util_parseUint16(uint8_t *pArray)
{
    return(Util_buildUint16(pArray[0], pArray[1]));
}

//拆分32位数据，获取某一个8位的数据
//var :原数据
//byteNum :拆分哪一个8位数据(0,1,2,3)
//return :返回所选择位的数据
uint8_t Util_breakUint32(uint32_t var, int byteNum)
{
    return(uint8_t)((uint32_t)(((var) >> ((byteNum) * 8)) & 0x00FF));
}

//将32位数据拆分保存在数组中
//pBuf :被保存数据的位置指针
//val :被拆分的32位数据
//return :数据依次保存后剩余空间的指针
uint8_t *Util_bufferUint32(uint8_t *pBuf, uint32_t val)
{
    *pBuf++ = Util_breakUint32(val, 0);
    *pBuf++ = Util_breakUint32(val, 1);
    *pBuf++ = Util_breakUint32(val, 2);
    *pBuf++ = Util_breakUint32(val, 3);
    return(pBuf);
}

//合并成32位数据
//byte :0~7
//byte :8~15
//byte :16~23
//byte :24~31
uint32_t Util_buildUint32(uint8_t byte0, uint8_t byte1, uint8_t byte2,
                            uint8_t byte3)
{
    return((uint32_t)((uint32_t)((byte0) & 0x00FF) +
                     ((uint32_t)((byte1) & 0x00FF) << 8) +
                     ((uint32_t)((byte2) & 0x00FF) << 16) +
                     ((uint32_t)((byte3) & 0x00FF) << 24)));
}

//直接将pArray组成一个32位数据，首地址位最低位
uint32_t Util_parseUint32(uint8_t *pArray)
{
    return(Util_buildUint32(pArray[0], pArray[1], pArray[2], pArray[3]));
}

//eg
// uint8_t *pBuf = lbtPacket.payload;
// *pBuf++ = (uint8_t)packet_flag_reported;
// pBuf = Util_bufferUint32(pBuf, node_info.Addr.shortAddr);//短地址
// pBuf = Util_bufferUint16(pBuf, sensor.power);//电压
// pBuf = Util_bufferUint32(pBuf, sensor.step);//步数
