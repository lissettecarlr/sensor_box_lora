/*
温湿度驱动，返回四个字节，前两个为温度的高低位，后两个为湿度的高低位。
*/

#include "stm32l1xx.h"
#include "sensor_ctr.h"
#include "String.h"
#include <rtthread.h>

I2C_HandleTypeDef hi2c1;
static Sensor_t htu21d_hander;


#define HTU21D_NAME "htu21d"
#define HTU21D_DEFAULT_STATUS  SENSOR_STATUS_DEATH
uint8_t htu21d_read(uint8_t *data,uint8_t len);

uint8_t htu21d_deinit()
{
   HAL_I2C_DeInit(&hi2c1);
	 return 0;
}

void htu21d_pin_init()
{
    HAL_I2C_MspInit(&hi2c1);
}

uint8_t htu21d_init()
{
  //IIC初始化
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
      return 1;
  }
	//sensor复位
	uint8_t val = 0xFE;
	if(HAL_I2C_Master_Transmit(&hi2c1,0X80,&val,1,100) == HAL_OK)
	{
		 
	   return 0;
	}
	else
	{
	   return 1;
	}		
}


#define HTU21D_DATA_ERROR  0XFFAA  //通过之前的数据进行对比，发现差异较大
#define HTU21D_DATA_LOSE   0XFFBB   //读取失败

//保存值 = 实际值*10 例如12.1° = 121
uint16_t htu21d_get_temp()
{
    uint16_t temp=0;
	  float temp_f=0;
	  uint8_t buff[2];
	  
	  if(HAL_I2C_Mem_Read(&hi2c1,0X81,0XE3,I2C_MEMADD_SIZE_8BIT,buff,2,100) == HAL_OK)
		{
		    uint16_t ret = (buff[0] << 8) | (buff[1] & 0xfc);	
			  temp_f = ret*175.72/65536-46.85;		
		    temp = (uint16_t)(temp_f*10.0);
		}
		else
		{
		   temp=HTU21D_DATA_LOSE;
		}
	  return temp;
}

uint16_t htu21d_get_humi()
{
    uint16_t humi=0;
	  uint8_t buff[2];
	  
	  if(HAL_I2C_Mem_Read(&hi2c1,0X81,0xE5,I2C_MEMADD_SIZE_8BIT,buff,2,100) == HAL_OK)
		{
		    uint16_t	ret = (buff[0] << 8) | (buff[1] & 0xfc);		
			  humi =(uint16_t)(ret*125/65536-6);	
		}
		else
		{
		    humi=HTU21D_DATA_LOSE;
		}
	  return humi;
}


uint8_t htu21d_read(uint8_t *data,uint8_t len)
{
    uint16_t temp,humi;
	  int i=3;
	  temp = htu21d_get_temp();
	  humi = htu21d_get_humi();
	
	  rt_kprintf("temp:%d\t",temp);
	  rt_kprintf("humi:%d\n",humi);
	
	   //确保尽量读取到数据
	   if( temp == HTU21D_DATA_LOSE)
		 {
		     while(i--)
				 {
				    temp = htu21d_get_temp();
					  if(temp !=HTU21D_DATA_LOSE)
						{
							  break;
						}
				 }
		 }
	
		 if( humi == HTU21D_DATA_LOSE)
		 {
			   i=3;
		     while(i--)
				 {
				    humi = htu21d_get_humi();
					  if(humi !=HTU21D_DATA_LOSE)
						{
							  break;
						}
				 }
		 }
		 if(len <4) //提供的buffer不足以填充数据
		 {
		    return 0;
		 }
		 data[0] = (uint8_t)(temp >>8);
		 data[1] = (uint8_t)temp; 
		 data[2] = (uint8_t)(humi>>8);
		 data[3] = (uint8_t)humi;
     return 4;		 
}



/*注册函数，绑定与传感器控制器*/
void register_htu21d(uint8_t status)
{
	  //如果传入有误则使用默认
	  if(status != SENSOR_STATUS_ALIVE && status != SENSOR_STATUS_DEATH)
		{
		   htu21d_hander.status = HTU21D_DEFAULT_STATUS;
		}
		else
		{
			htu21d_hander.status = status;
		}
	  
	  strcpy(htu21d_hander.name,HTU21D_NAME);
	  htu21d_hander.init = htu21d_init;
	  htu21d_hander.read = htu21d_read;
	  htu21d_hander.deinit = htu21d_deinit;
	  sensor_register(&htu21d_hander);
}



void display_th()
{
	rt_kprintf("temp:%d\t",htu21d_get_temp());
	rt_kprintf("humi:%d\t",htu21d_get_humi());
}
MSH_CMD_EXPORT(display_th,th);
