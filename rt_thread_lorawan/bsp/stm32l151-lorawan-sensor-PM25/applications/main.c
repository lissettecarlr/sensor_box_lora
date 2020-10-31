/*
基于lorawan的传感器应用-空气小六-PM2.5
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "low_power.h"
#include "rtc-board.h"

#include "sensor_ctr.h"
#include "sensor_htu21d.h"
#include "sensor_light.h"
#include "sensor_power.h"
#include "sensor_PM2_5.h"

//#define LOG_TAG              "main"
//#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
//#include <rtdbg.h>



#define S_TEMPHUMI    1
#define S_LIGHT       1
#define S_POWER       1
#define S_PM2_5       1


#define S_VERSION     "V0_2"

extern void lorawan_rtt_init(void);
//void hook_sleep(void);


int main(void)
{
	 rtc_init();
	 low_power_init();
	 
/*上行数据格式  4bPM2.5数据+2b电压+2b光照+4b温湿度*/	
#if S_TEMPHUMI	
	 register_htu21d(SENSOR_STATUS_ALIVE);
#else	
	 register_htu21d(SENSOR_STATUS_DEATH);
#endif
	
#if S_LIGHT	
	 register_light(SENSOR_STATUS_ALIVE);
#else
	 register_light(SENSOR_STATUS_DEATH);
#endif
	
#if S_POWER
	 register_power(SENSOR_STATUS_ALIVE);
#else
	 register_power(SENSOR_STATUS_DEATH);
#endif
	
#if S_PM2_5
	 register_PM25(SENSOR_STATUS_ALIVE);
#else
	 register_PM25(SENSOR_STATUS_DEATH);
#endif
		
	

//#if (S_TEMPHUMI & S_LIGHT)	
//	 rt_kprintf("six box firmware:%s\n",S_VERSION);
//#else
//   rt_kprintf("sensor firmware:%s\n",S_VERSION);
//#endif

	 sensor_ctr_init(); 
	
	 lorawan_rtt_init();
	
   return RT_EOK;
}




#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/ 
__asm void GenerateSystemReset(void) 
{ 
 MOV R0, #1           //;  
 MSR FAULTMASK, R0    //; FAULTMASK 禁止一切中断产生 
 LDR R0, =0xE000ED0C  //; 
 LDR R1, =0x05FA0004  //;  
 STR R1, [R0]         //;    
  
deadloop 
    B deadloop        //;  
} 
#elif (defined (__ICCARM__)) /*------------------ ICC Compiler -------------------*/ 
//#pragma diag_suppress=Pe940 
void GenerateSystemReset(void) 
{ 
  __ASM("MOV R0, #1"); 
  __ASM("MSR FAULTMASK, R0"); 
  SCB->AIRCR = 0x05FA0004; 
  for(;;); 
}
#endif

void lw_rst()
{
  GenerateSystemReset();
}

MSH_CMD_EXPORT(lw_rst,rst);










////msh命令

////standy 0.2ma   stop 1.6ma   sleep 6ma
//void lw_mode(int argc,void **argv)
//{
//    if(argc!=2)
//    {
//			    rt_kprintf("sys_mode argc size:%d\n",argc);
//          return;
//    }
//    if( rt_strcmp( argv[1],"standy")==0 )
//		{
//		    rt_kprintf("go to standby\n");
//			  rt_thread_mdelay(1000);
//			  enter_standby_wakup();
//		}
//		else if(rt_strcmp(argv[1],"stop")==0)
//		{
//			  rt_kprintf("go to stop\n");			  
//			  rt_thread_mdelay(1000);
//			  enter_stop_exti();
//		}
//		else if(rt_strcmp(argv[1],"sleep")==0)
//		{
//				rt_kprintf("go to sleep\n");
//			  while(1)  
//				{
//					enter_sleep();
//				}			  
////			  rt_thread_mdelay(1000);
//		}
//    else
//		{
//		    rt_kprintf("sys_mode unknown\n");
//			  rt_thread_mdelay(1000);
//		}			
//    
//}
//MSH_CMD_EXPORT(lw_mode,low power);


