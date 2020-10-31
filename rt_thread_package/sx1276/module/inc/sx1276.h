#ifndef __SX127X_H__
#define __SX127X_H__


#include "rtthread.h"

struct sx1276_dio_s
{
    rt_int32_t irq0;
    rt_int32_t irq1;
    rt_int32_t rst;
    rt_int32_t nss;
};


struct sx1276_bsp_s
{
	 char spi_name[20];
   struct sx1276_dio_s dio;
};

typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;


struct sx1276_cfg_rx_s
{
   RadioModems_t modem;
	 uint32_t bandwidth;
	 uint32_t datarate;
	 uint8_t coderate;
	 uint32_t bandwidthAfc;
	 uint16_t preambleLen;
	 uint16_t symbTimeout;
	 rt_bool_t fixLen;
	 uint8_t payloadLen;
	 rt_bool_t crcOn;
	 rt_bool_t freqHopOn;
	 uint8_t hopPeriod;
	 rt_bool_t iqInverted;
	 rt_bool_t rxContinuous; 
};


struct sx1276_cfg_tx_s
{
		RadioModems_t modem;
		int8_t power;
		uint32_t fdev;
		uint32_t bandwidth;
		uint32_t datarate;
		uint8_t coderate;
		uint16_t preambleLen;
		rt_bool_t fixLen;
		rt_bool_t crcOn;
		rt_bool_t freqHopOn;
		uint8_t hopPeriod;
		rt_bool_t iqInverted;
		uint32_t timeout;
};

struct sx1276_cfg_s
{
   //后续添加
   struct sx1276_cfg_tx_s cfg_tx;
	 struct sx1276_cfg_rx_s cfg_rx;
};

struct sx1276_timer_s
{
   rt_timer_t TxTimeoutTimer;
   rt_timer_t RxTimeoutTimer;
   rt_timer_t RxTimeoutSyncWord;
};

//单个设备的信息存储结构体
struct sx1276_t
{
   struct rt_device parent;
   rt_thread_t thread_hander;
	 void *pcfg;
	 void *event;
   struct sx1276_bsp_s bsp;
   struct sx1276_cfg_s cfg;
   struct sx1276_timer_s timer;
};


/*
dev_name : 当前注册的设备名称
spi_name : 使用的spi设备名称
dio : 使用的引脚结构体
cfg : 配置信息结构体
*/

int sx1276_device_register(const char *dev_name,const char *spi_name,struct sx1276_dio_s dio,struct sx1276_cfg_s *cfg);
int sx1276_device_init(void);
#endif /* __SX1276_H__ */
