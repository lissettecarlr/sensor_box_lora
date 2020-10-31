# 目录结构

* example :用于测试各个部分功能的代码
* inc :存放驱动相关头文件
* ports :驱动时钟的实现
* src :驱动主体代码


# SX1276组件

## 使用步骤
### 1引入头文件
首先将文件目录导入到KEIL的配置中去，使其能够搜寻到该目录下的头文件
然后在需要使用该组件的位置添加
```
#include "sx1276.h"
```
### 2初始化准备
该组件需要使用spi和4个引脚，spi需要在bsp中提前初始化完成，引脚则只需要实现下列结构体，填充对于引脚号即可
```
struct sx1276_dio_s
{
    rt_int32_t irq0;
    rt_int32_t irq1;
    rt_int32_t rst;
    rt_int32_t nss;
};
struct sx1276_dio_s dio={GET_PIN(B, 7),0,GET_PIN(A, 15),GET_PIN(B, 12)};
```
spi初始化示例，在spi2上注册一个名叫sx1276_spi的spi设备
```
	rt_err_t res;
	static struct rt_spi_device spi_dev_sx1276;
    res = rt_spi_bus_attach_device(&spi_dev_sx1276, "sx1276_spi", "spi2", NULL);
    if (res != RT_EOK)
    {}
    else
    {
    //配置
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 10000000;//20 * 1000 *1000; /* 20M,SPI max 42MHz,ssd1351 4-wire spi */
    rt_spi_configure(&spi_dev_sx1276, &cfg);
    rt_kprintf("spi device init :OK\n");
    }
```
### 3注册1276设备
将上列设备的参数填入，参数1是该1278设备名称，参数2是使用的SPI设备名称，参数3是使用引脚，参数4是配置
```
sx1276_device_register("test","sx1276_spi",dio,RT_NULL);
```

### 4配置
注册时如果未填写自定义配置，则使用默认配置。自定义配置结构体如下，实现各项参数即可
```
struct sx1276_cfg_s
{
   struct sx1276_cfg_tx_s cfg_tx;
   struct sx1276_cfg_rx_s cfg_rx;
};

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
```

### 5发送数据
在设备表中找到sx1276设备，获得句柄
```
    rt_device_t sx1276_hander;
    sx1276_hander = rt_device_find("test");
    if(sx1276_hander ==RT_NULL)
    {
        rt_kprintf("sx1276 hander error\n");
    }
```
打开设备，这里的open将填充默认频率和导入rx、tx的配置文件，第二个参数暂时无意义
```
rt_device_open(sx1276_hander,0);
```
发送数据1,2,3
```
uint8_t data[3] ={1,2,3};
rt_device_write(sx1276_hander,0,data,3);
```
绑定发送回调
```
rt_err_t tx_done(rt_device_t dev, void *buffer)
{
	rt_kprintf("tx name:%s,size:%d", dev->parent.name, * (rt_uint8_t *)buffer);
	return 0;
}

rt_device_set_tx_complete(sx1276_hander,tx_done);
```
### 6接收数据
### 7修改配置