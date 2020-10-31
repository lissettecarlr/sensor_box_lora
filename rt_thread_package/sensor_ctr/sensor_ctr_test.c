/*该文件用来测试传感器控制器的*/

#include "rtthread.h"
#include "sensor_ctr.h"
#include "sensor_htu21d.h"


void sensor_ctr_test_thread_entry(void *p)
{
	 uint8_t len;
	 uint8_t data[100];
	 while(1)
	 {
      rt_thread_mdelay(5000);
		  //更新传感器
			sensor_ctr_start_update(RT_NULL);
			len = sensor_read_data(data,100);
		  rt_kprintf("data:[");
		  for(int i=0;i<len;i++)
		  {
			    rt_kprintf("%02X ",data[i]);
			}
	    rt_kprintf("]\n");
	 }

}


void sensor_ctr_test(void)
{
    rt_thread_t sensor_ctr_test_hander;
	  sensor_ctr_init(); 
	

	  sensor_ctr_test_hander = rt_thread_create("sensor ctr test",sensor_ctr_test_thread_entry,RT_NULL,1024,6,10);
	  if(sensor_ctr_test_hander != RT_NULL)
		{
		   rt_thread_startup(sensor_ctr_test_hander);
		}
		else
		{
		   rt_kprintf("sensor ctr test thread error");
		}
	  
}
