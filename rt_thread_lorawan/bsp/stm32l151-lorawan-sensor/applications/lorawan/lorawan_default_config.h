/*
该文件是lorawan默认配置，如需修改请另起文件
*/

#ifndef _LORAWAN_DEFAULT_CONFIG_H
#define _LORAWAN_DEFAULT_CONFIG_H


/*
#define JOIN_MODE_OTAA 0
#define JOIN_MODE_ABP  1
#define JOIN_MODE_MIXTURE_OTAA 2
#define JOIN_MODE_MIXTURE_ABP 3
*/
#define LORAWAN_DEFAULT_JOIN_MODE    0              
#define LORAWAN_DEFAULT_JOIN_ADDR    ( uint32_t )0X06302EDA 
#define LORAWAN_DEFAULT_JOIN_DEVEUI  { 0X68, 0XD9, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff }
#define LORAWAN_DEFAULT_JOIN_APPEUI  { 0x68, 0xd9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }
#define LORAWAN_DEFAULT_JOIN_APPKEY  { 0xf5, 0x61, 0x17, 0x37, 0x60, 0x53, 0x54, 0x2e, 0x89, 0x44, 0x80, 0xdd, 0x44, 0x6d, 0x9c, 0xb2 }

#define LORAWAN_DEFAULT_JOIN_NETSKEY {0X19,0XC2,0XF7,0X54,0X04,0X32,0X94,0XBE,0X6B,0X77,0X6D,0X7B,0X80,0X88,0X10,0XEA}
#define LORAWAN_DEFAULT_JOIN_APPSKEY {0X06,0XD3,0X3D,0XFD,0XD8,0X79,0X20,0XFB,0XC3,0X67,0X64,0X41,0X1C,0X4A,0X94,0XF9}

/*
#define DR_0                                        0  // SF12 - BW125 |
#define DR_1                                        1  // SF11 - BW125 |
#define DR_2                                        2  // SF10 - BW125 |
#define DR_3                                        3  // SF9  - BW125 |
#define DR_4                                        4  // SF8  - BW125 |
#define DR_5                                        5  // SF7  - BW125 |
#define TX_POWER_17_DBM                             0
#define TX_POWER_16_DBM                             1
#define TX_POWER_14_DBM                             2
#define TX_POWER_12_DBM                             3
#define TX_POWER_10_DBM                             4
#define TX_POWER_7_DBM                              5
#define TX_POWER_5_DBM                              6
#define TX_POWER_2_DBM                              7
*/

#define LORAWAN_DEFAULT_ADR       true
#define LORAWAN_DEFAULT_CLASS     CLASS_A
#define LORAWAN_DEFAULT_DATARATE  0
#define LORAWAN_DEFAULT_POWER     2
#define LROAWAN_DEFAULT_CHANNL  { 0x000C,0x0000,0x0000,0x0000,0x0000,0x0000}


		




#endif

