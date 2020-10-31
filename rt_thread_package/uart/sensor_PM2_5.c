/*
PM2.5驱动，依赖sensor_uart
*/

#include "rtthread.h"
#include "sensor_PM2_5.h"
#include "sensor_uart_compatible.h"
#include "string.h"

#include "sensor_ctr.h"
static Sensor_t PM25_hander;

#define PM25_DEFAULT_STATUS SENSOR_STATUS_DEATH
#define PM25_NAME  "PM25"
#define PM25_DATA_LOSE   0XFFBB   //读取失败

/*读取数据命令*/
struct send_command get_cmd={7,{0x42,0x4d,0xe2,0x00,0x00,0x01,0x71}};
struct send_command cfg_cmd={7,{0x42,0x4d,0xe1,0x00,0x00,0x01,0x70}};

/*传感器数据暂存*/
static uint8_t PM25_data_lose[6]={0xff,0xbb,0xff,0xbb,0xff,0xbb};
static uint8_t PM25_data[6]={0xff,0xbb,0xff,0xbb,0xff,0xbb};


/*和校验*/
static uint8_t mCheck(uint8_t *Data,int Lenth)
{
	uint16_t CheckSum = (((uint16_t)Data[Lenth-2])<<8)+Data[Lenth-1];
	int temp=0;
	for(int i=0;i<Lenth-3;i++)
	{
		temp+=Data[i];
	}
	if(temp == CheckSum)
		return 1;
	else
		return 0;
}

/*获得的是串口的原始数据，进行解包*/
void rcv_data_unpack(struct rcv_buffer buffer)
{
	  rt_kprintf("pm25:rcv data %d\n",buffer.len);
	  if(buffer.len != 24)
		{
			 rt_memcpy(PM25_data,PM25_data_lose,6);
		   return;
		}
		
		if(mCheck( buffer.data , 24) )//和校验
		{
		   rt_memcpy(PM25_data,buffer.data+10,6);
		}
		else
		{
		   rt_memcpy(PM25_data,PM25_data_lose,6);
		}
}

uint8_t PM25_deinit()
{
	 return 0;
}

void PM25_pin_init()
{
	
}

uint8_t PM25_init()
{
	//int sensor_uart_conpatible_init(uint32_t i_cycle,struct send_command cmd,uint8_t rcv_key,void(*cb)(struct rcv_buffer buffer))
	sensor_uart_conpatible_init(10,get_cmd,24,rcv_data_unpack);
	sensor_uart_send(cfg_cmd.data,cfg_cmd.len);
	rt_thread_mdelay(1000);
	sensor_uart_start_update();
	return 0;
}

uint16_t PM25_start()
{
	sensor_uart_start_update();
	return 0;
}

uint16_t PM25_close()
{
	sensor_uart_close_update();
	return 0;
}


uint8_t PM25_read(uint8_t *s_data,uint8_t len)
{
	  if(len < 6)
		{
		  rt_kprintf("[PM25]{PM25_read len error}\n");
			return 0;
		}
		rt_memcpy(s_data,PM25_data,6);
    return 6;
}


void register_PM25(uint8_t status)
{
  	 //如果传入有误则使用默认
	  if(status != SENSOR_STATUS_ALIVE && status != SENSOR_STATUS_DEATH)
		{
		   PM25_hander.status = PM25_DEFAULT_STATUS;
		}
		else
		{
		 	PM25_hander.status = status;
		}
	  
	  strcpy(PM25_hander.name,PM25_NAME);
	  PM25_hander.init = PM25_init;
	  PM25_hander.read = PM25_read;
	  PM25_hander.deinit = PM25_deinit;
	  sensor_register(&PM25_hander);
}


void pm25_display()
{
	rt_kprintf("[PM25]{");
	for(int i=0;i<6;i++)
	{
	   rt_kprintf(" %d",PM25_data[i]);
	}
	rt_kprintf("}\n");
	
}
MSH_CMD_EXPORT(pm25_display,pm25_display);

