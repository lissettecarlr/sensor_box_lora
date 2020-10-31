#include "stm32l1xx.h"
#include "rtthread.h"
#include "light.h"

#define BHAddWrite     0x46      //????+??????
#define BHAddRead      0x47      //????+??????
#define BHPowDown      0x00      //????
#define BHPowOn        0x01      //??????????
#define BHReset        0x07      //?????????PowerOn?????
#define BHModeH1       0x10      //???? ??1lx ????120ms
#define BHModeH2       0x11      //??????2 ??0.5lx ????120ms
#define BHModeL        0x13      //???? ??4lx ????16ms
#define BHSigModeH     0x20      //?????? ?? ??????? PowerDown??
#define BHSigModeH2    0x21      //????
#define BHSigModeL     0x23      // ???



I2C_HandleTypeDef hi2c1_l;

void light_deinit()
{
   HAL_I2C_DeInit(&hi2c1_l);
}

void light_pin_init()
{
    HAL_I2C_MspInit(&hi2c1_l);
}

int light_init()
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
		   temp=0xffff;
		}
		light_close();
		rt_kprintf("light %d\n",temp);
	  return temp;
}

void light_display()
{
    uint16_t temp = light_get();
	  rt_kprintf("light :%d\n",temp);
}
MSH_CMD_EXPORT(light_display,light_display);

