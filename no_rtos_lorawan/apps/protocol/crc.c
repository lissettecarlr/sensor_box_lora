//crc校验
//2018年7月19日10:37:36
//UTF-8


#include "stdint.h"

//Modbus RTU CRC
//test:http://cht.nahua.com.tw/index.php?url=http://cht.nahua.com.tw/software/crc16/&key=Modbus,%20RTU,%20CRC16&title=%E8%A8%88%E7%AE%97%20Modbus%20RTU%20CRC16
uint16_t  modbus_crc(uint8_t *modbus_buff,uint8_t size)
{
    uint16_t crc_register;
    uint8_t index,pos;
    crc_register = 0xffff;
    for ( index = 0; index < size; index++ ){
        crc_register = crc_register ^ (modbus_buff[0] & 0x00ff);
        modbus_buff++;
        for ( pos = 0; pos < 8; pos++ ){
            if ( crc_register & 0x0001){
                crc_register = crc_register >> 1;//先将数据右移一位
                crc_register ^= 0xa001;//与0x001进行异或
            }else{
                crc_register = crc_register >> 1;
            }
        }
    }
    return crc_register;
}

