/*电压驱动，返回两个字节电压*/

#include "stm32_HAL_adc.h"
#include "rtthread.h"
#include "String.h"


#include "sensor_ctr.h"
static Sensor_t power_hander;
#define POWER_DEFAULT_STATUS SENSOR_STATUS_DEATH
#define POWER_NAME  "voltage"


uint8_t power_deinit()
{
	 MAX_ADC_deInit();
	 return 0;
}

uint8_t power_init()
{
	MX_ADC_Init();
	return 0;
}


uint8_t power_read(uint8_t *data,uint8_t len)
{
	 if(len <2)
	 {
	    return 0;
	 }
	 uint16_t temp = ADC_READ_Valtage();
	 data[0] = (uint8_t)(temp >>8);
	 data[1] = (uint8_t)(temp);
	 return 2;
	
}



void register_power(uint8_t status)
{
  	//如果传入有误则使用默认
	  if(status != SENSOR_STATUS_ALIVE && status != SENSOR_STATUS_DEATH)
		{
		   power_hander.status = POWER_DEFAULT_STATUS;
		}
		else
		{
		 	 power_hander.status = status;
		}
	  
	  strcpy(power_hander.name,POWER_NAME);
	
	  power_hander.init = power_init;
	  power_hander.read = power_read;
	  power_hander.deinit = power_deinit;
	  sensor_register(&power_hander);
}


void power_display()
{
	rt_kprintf("power:%d\n",ADC_READ_Valtage());
}
MSH_CMD_EXPORT(power_display,power_display);

