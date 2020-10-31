#ifndef _LORAWAN_APP_H
#define _LORAWAN_APP_H

#include "stdint.h"
#include "LoRaMac.h"

//该结构体涵盖各种入网参数
typedef struct sLoraWanJoinInfo
{
     uint8_t JoinMode;//入网模式分为四种
     uint32_t Addr;//设备短地址
     uint8_t DevEui[8];
     uint8_t AppEui[8];
     uint8_t AppKey[16];
     uint8_t NetSKey[16];
     uint8_t AppSKey[16];
}LoraWanJoinInfo_s;


//该结构体包含协议的一些配置信息
typedef struct sLoraWanCfg
{
    bool Adr; 
    int8_t DataRate;
    int8_t power;
    DeviceClass_t Class;
    uint16_t ChannelsMask[6];
}LoraWanCfg_s;


//该结构体包含协议状态
typedef struct sLoraWanStatue
{
	  uint32_t UpCounter;//保存上行计数
}LoraWanStatue_s;

typedef struct sLoraWanRcv	
{
   uint8_t Port;
	 int16_t Rssi;
	 uint8_t Snr;
	 uint8_t BufferSize;
	 uint8_t *Buffer;
}LoraWanRcv_s;

int lorawan_join_info_write(char *key,void *data);
int lorawan_join_info_read(char *key,void *data);
LoraWanJoinInfo_s* lorawan_get_join_info_p(void);
void lorawan_join_info_display(void);

int lorawan_cfg_write(char *key,void *data);
int lorawan_cfg_read(char *key,void *data);
LoraWanCfg_s* lorawan_get_cfg_p(void);
void lorawan_cfg_display(void);
	
int lorawan_statue_write(char *key,void *data);
int lorawan_statue_read(char *key,void *data);
LoraWanStatue_s* lorawan_get_statue_p(void);

int lorawan_init(void);
//statue 0表示入网  0xff表示混合入网的OTAA 
void lorawan_join(uint8_t times,void (*fun_cb)(uint8_t statue));
//mode=0表示无应答
int lorawan_send(uint8_t *buffer,uint8_t size,uint8_t mode);

void lorawan_set_config(char *key);
int lorawan_deinit(void);

void radio_sleep(void);

void lorawan_rcv_register_cb(void (*rcv_cb)(uint8_t flag) );
LoraWanRcv_s* lorawan_rcv_get(void);
void radio_rx_set(bool flag);
#endif

