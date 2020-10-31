/*
光照的驱动，返回两个字节光照强度
*/

#include "stm32l1xx.h"
#include "rtthread.h"
#include "String.h"

#define BHAddWrite     0x46     
#define BHAddRead      0x47     
#define BHPowDown      0x00      
#define BHPowOn        0x01     
#define BHReset        0x07     
#define BHModeH1       0x10      
#define BHModeH2       0x11     
#define BHModeL        0x13     
#define BHSigModeH     0x20      
#define BHSigModeH2    0x21      
#define BHSigModeL     0x23     


#include "sensor_ctr.h"
static Sensor_t light_hander;
#define LIGHT_DEFAULT_STATUS SENSOR_STATUS_DEATH
#define LIGHT_NAME  "light"

I2C_HandleTypeDef hi2c1_l;

uint8_t light_deinit()
{
   HAL_I2C_DeInit(&hi2c1_l);
	 return 0;
}

void light_pin_init()
{
    HAL_I2C_MspInit(&hi2c1_l);
}

uint8_t light_init()
{
//IIC初始化
  hi2c1_l.Instance = I2C1;
  hi2c1_l.Init.ClockSpeed = 100000;
  hi2c1_l.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1_l.Init.OwnAddress1 = 0;
  hi2c1_l.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1_l.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1_l.Init.OwnAddress2 = 0;
  hi2c1_l.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1_l.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1_l) != HAL_OK)
  {
		  rt_kprintf("iic error\n");
      return 1;
  }
	return 0;
}

uint16_t light_start()
{
	uint8_t val = BHPowOn;
	if(HAL_I2C_Master_Transmit(&hi2c1_l,BHAddWrite,&val,1,100) != HAL_OK)
	{
		 rt_kprintf("light start error 1\n");
	   return 1;
	}
  val = BHReset;
	if(HAL_I2C_Master_Transmit(&hi2c1_l,BHAddWrite,&val,1,100) != HAL_OK)
	{
		 rt_kprintf("light start error 2\n");
	   return 2;
	}
	val = BHModeH2;
	if(HAL_I2C_Master_Transmit(&hi2c1_l,BHAddWrite,&val,1,100) != HAL_OK)
	{
		 rt_kprintf("light start error 3\n");
	   return 3;
	}
	rt_thread_mdelay(180);
	return 0;
}

uint16_t light_close()
{
	uint8_t val = BHPowDown;
	if(HAL_I2C_Master_Transmit(&hi2c1_l,BHAddWrite,&val,1,100) != HAL_OK)
	{
		 rt_kprintf("light end error 1\n");
	   return 1;
	}
	return 0;
}
MSH_CMD_EXPORT(light_close,light_close);

#define LIGHT_DATA_LOSE   0XFFBB   //读取失败

//保存值 = 实际值*10 例如12.1° = 121
uint16_t light_get()
{
    uint16_t temp=0;
	  float temp_f=0;
	  uint8_t buff[2];
	  light_start();
	
	  if(HAL_I2C_Master_Receive(&hi2c1_l,BHAddRead,buff,2,100) == HAL_OK)
		{
		    uint16_t ret = (buff[0] << 8) | (buff[1] & 0xfc);	
			  temp_f = ret/1.2;		
		    temp = (uint16_t)(temp_f);
		}
		else
		{
		   temp=LIGHT_DATA_LOSE;
		}
		light_close();
		rt_kprintf("light %d\n",temp);
	  return temp;
}

uint8_t light_read(uint8_t *data,uint8_t len)
{
	  uint16_t temp = light_get();
	  data[0] = (uint8_t)(temp>>8);
	  data[1] = (uint8_t)(temp); 
    return 2;
}


void register_light(uint8_t status)
{
  	 //如果传入有误则使用默认
	  if(status != SENSOR_STATUS_ALIVE && status != SENSOR_STATUS_DEATH)
		{
		   light_hander.status = LIGHT_DEFAULT_STATUS;
		}
		else
		{
		 	light_hander.status = status;
		}
	  
	  strcpy(light_hander.name,LIGHT_NAME);
	  light_hander.init = light_init;
	  light_hander.read = light_read;
	  light_hander.deinit = light_deinit;
	  sensor_register(&light_hander);
}


void light_display()
{
    uint16_t temp = light_get();
	  rt_kprintf("light :%d\n",temp);
}
MSH_CMD_EXPORT(light_display,light_display);

