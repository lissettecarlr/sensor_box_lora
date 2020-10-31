/*
该应用层逻辑:
启动->尝试入网N次-->入网失败则进入休眠
                 -->入网成功则立即发送一个数据包->根据预设休眠时间开启RTC，进入STOP模式-->更新传感器，发送数据包,重复休眠
								                                                                       -->串口唤醒
																																											 -->外部触发唤醒-->停止RTC，重新入网
																																											                -->示意存活
*/

#include "lorawan_app.h"
#include "sx1276_board.h"
#include "rtc-board.h"
#include "string.h"
#include <ctype.h>  //toupper  isxdigit

#include "node_msg_store.h"
#include "low_power.h"
#include "fal.h"

#include "sensor_ctr.h"

#define LOG_TAG              "lora_t"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>

//other

//static void StrToHex(char *pbDest, char *pbSrc, int nLen)
//{
//    char h1,h2;
//    char s1,s2;
//    int i;

//    for (i=0; i<nLen; i++)
//    {
//        h1 = pbSrc[2*i];
//        h2 = pbSrc[2*i+1];

//        s1 = toupper(h1) - 0x30;
//        if (s1 > 9) 
//            s1 -= 7;

//        s2 = toupper(h2) - 0x30;
//        if (s2 > 9) 
//            s2 -= 7;

//        pbDest[i] = s1*16 + s2;
//    }
//}
/*********************************************************************************************************/
void lw_id(void);

//发送数据包结构体，这里定死了几个参数，其一是通道，其二是重传，后续完善
#define APP_DATA_MAX_SIZE 128
typedef struct sSendInfo
{
	 uint8_t mode;
	 uint8_t SendBufferSize;//待发送长度	
   uint8_t SendBuffer[APP_DATA_MAX_SIZE]; //待发送的缓冲区
}SendInfo_t;
SendInfo_t SendInfo={0,3,{1,2,3}};


rt_thread_t lorawan_hander=RT_NULL;
static struct rt_event lorawan_event;

#define LORAWAN_EVENT_JOIN          (uint32_t)1<<1
#define LORAWAN_EVENT_SEND          (uint32_t)1<<2
#define LORAWAN_EVENT_SLEEP   	    (uint32_t)1<<3
#define LORAWAN_EVENT_CFG           (uint32_t)1<<4

#define LORAWAN_STATUE_INIT       0
#define LORAWAN_STATUE_JOIN       1
#define LORAWAN_STATUE_SEND   	  2
#define LORAWAN_STATUE_WAIT_JOIN  3
#define LORAWAN_STATUE_CFG        4


static uint16_t cycle_join_time=60; //入网周期s
//static uint16_t sensor_update_time = 60;//传感器更新数据上行周期
static uint32_t sleep_time = 0; //s 进入休眠模式前唤醒闹钟设定值
static uint8_t attempte=2; //每个周期尝试入网次数

static uint8_t lwrt_mode=LORAWAN_STATUE_INIT;//表示当前状态，初始化时无法发包和入网，入网时无法发包。
//static uint8_t lwrt_last_mode = LORAWAN_STATUE_INIT; //用于在退出配置模式后恢复上一个状态

// 定时器回调，当此时大于阈值则不进行入网
#define DEFAULT_JOIN_TIMES  50
static uint8_t default_join_times = 0;

static void lorawan_timer_irq(void)
{	 
	 //判断被唤醒的状态
  //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); 
	 
	 //LOG_D("time irq");
	 switch (lwrt_mode)
	 {
		 case LORAWAN_STATUE_JOIN: //处于入网周期中
     {
			  if(default_join_times >=DEFAULT_JOIN_TIMES) //不在进行入网而直接休眠
				{
					  sleep_time = 0; //不再设定闹钟
					  rt_event_send(&lorawan_event,LORAWAN_EVENT_SLEEP); 
				    
				}
				else
				{
					  rt_event_send(&lorawan_event,LORAWAN_EVENT_JOIN);
				    default_join_times++;//入网次数累加 
				}	  
		 }break;
		 case LORAWAN_STATUE_SEND: //处于数据发送周期中
		 {
		      rt_event_send(&lorawan_event,LORAWAN_EVENT_SEND);
		 }break;
		  default:
		 {
		    //LOG_W("time irq unknow mode");
		 }
	 }
}


static void lorawan_join_irq(uint8_t statue)
{
    if(statue == 0) //入网成功
		{
		   LOG_D("join succeed\n");
			 if(lwrt_mode == LORAWAN_STATUE_JOIN)	//确保在配置模式下不进入周期上报模式
			 {				 
					lwrt_mode = LORAWAN_STATUE_SEND;
				  //radio_rx_set(false);
					rt_event_send(&lorawan_event,LORAWAN_EVENT_SEND);	 
			 }
		}
		else if(statue == 0xff) //混合入网
		{
		     save_node_msg();
			   lwrt_mode = LORAWAN_STATUE_SEND;
				 rt_event_send(&lorawan_event,LORAWAN_EVENT_SEND);	 
		}
		else
		{ 		
			 LOG_D("join fail\n");
			 if(lwrt_mode == LORAWAN_STATUE_JOIN)	
			 {
				  sleep_time = cycle_join_time;
				  rt_event_send(&lorawan_event,LORAWAN_EVENT_SLEEP); 			
			 }
		}
}

//该模式不接收下行数据
static void lorawan_data_irq(uint8_t flag)
{
	  LOG_D("send over\n");
    switch (flag)
		{ 
		  case 0: //有下行数据
			{
			
			}break;
			case 1: //接收超时
			{
			
			}break;				
		  default:
			{
			
			}
		}
		sleep_time = NodeMsg.CycleS;
		rt_event_send(&lorawan_event,LORAWAN_EVENT_SLEEP);
}



/*该段代码用于非低功耗下使用的定时器***************************************/
//static uint8_t lw_low_power_flag=1; //是否启用低功耗模式标志位，1启用RTC，不启用则使用系统时钟
static TimerEvent_t system_cycle_timer;
void system_cycle_timer_cb(void *p)
{
	 lorawan_timer_irq();
}
void system_cycle_timer_init()
{
	  if(NodeMsg.LowPower ==0)
		{
			  rt_kprintf("start system clock\n");
	      TimerInit(&system_cycle_timer,system_cycle_timer_cb);
		}
		else
		{
			  rt_kprintf("start RTC \n");
		    rtc_register_callback(lorawan_timer_irq);
		}
}
//单位 s
void system_cycle_timer_start(uint32_t timeout)
{
	if(NodeMsg.LowPower ==0)
	{
		 rt_kprintf("sleep :%d s\n",timeout);
	   TimerSetValue(&system_cycle_timer,timeout*1000);
		 TimerStart(&system_cycle_timer);
	}
  else
	{				
	  rtc_set_alarm(timeout);
		rt_thread_mdelay(100);
		//进入休眠
		enter_stop();
	}
}

//是否启用低功能模式，默认开启
void lw_lp(int argc,char **argv)
{
   if(argc != 2)
	 {
		  rt_kprintf("param error :more\n");
		  return;
	 }
   if( rt_strcmp( argv[1],"on")==0 )
	 {
	     //lw_low_power_flag=1;
		   NodeMsg.LowPower = 1;
		   rt_kprintf("[low power:on]\n");
	 }
   else if (rt_strcmp( argv[1],"off")==0)
	 {
	     //lw_low_power_flag=0;
		   NodeMsg.LowPower = 0;
		   rt_kprintf("[low power:off]\n");
	 }
   else
	 {
	    rt_kprintf("param error\n");
	 }
}
MSH_CMD_EXPORT(lw_lp,lw low power);
/**********************************************************************************/
static uint8_t store_flag=0;//标识是否读取到存储信息
void lorawan_rtt_thread_entry(void *p)
{
    int sta=0;
	  rt_uint32_t event;
	 	 
	  sta+=sx1276_board_init();
	  sta+=lorawan_init();
	  lorawan_rcv_register_cb(lorawan_data_irq);	 
	  if(sta !=0)
		{
			LOG_D("lorawan init error");			
			return;
		}
		rt_thread_mdelay(1000);
		lwrt_mode = LORAWAN_STATUE_WAIT_JOIN;
		
		if(store_flag == 0) //如果等待配置超时，且未读取到保存信息则直接进入休眠，该状态发送在工厂生产结束时。
	  {
		   LOG_I("go to sleep\n");
	     enter_stop();
		   return;
	  }
		//设置周期定时器
		//rtc_register_callback(lorawan_timer_irq);
		system_cycle_timer_init();
		
		//修改入网周围2分钟
		sleep_time = cycle_join_time;
		rt_event_send(&lorawan_event,LORAWAN_EVENT_JOIN);
    rt_thread_mdelay(2000); //延时等待lorawan 初始化完成			
   
    while(1)
		{
		    rt_event_recv(&lorawan_event,LORAWAN_EVENT_JOIN|LORAWAN_EVENT_SEND | LORAWAN_EVENT_SLEEP \
			                ,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &event);
			  //这里之后进行周期判断，太短则continue
			  if(event & LORAWAN_EVENT_JOIN)
				{
				   if(lwrt_mode ==LORAWAN_STATUE_INIT) //除了在初始化前，任何时候都能再次入网
						 continue;
					 
					 LOG_D("start join");
					 lwrt_mode = LORAWAN_STATUE_JOIN;
					 radio_rx_set(true);
					 lorawan_join(attempte,lorawan_join_irq);
				}
				if(event & LORAWAN_EVENT_SEND)
				{
					   radio_rx_set(false);
				    if(lwrt_mode != LORAWAN_STATUE_SEND)
						{
						     LOG_D("state error:%d",lwrt_mode);				 
						}
						else
						{
								LOG_D("start send");	
								rt_thread_mdelay(200);						

                //更新传感器
							  sensor_ctr_start_update(RT_NULL);
							  SendInfo.SendBufferSize = sensor_read_data(SendInfo.SendBuffer,APP_DATA_MAX_SIZE);
				
#if (1) //如果服务器通过上行计数筛选数据包的话，需要打开
							  static uint8_t save_cnt=0,now_cnt=1;									
							  if(NodeMsg.CycleS >= 60 * 60) //如果发送周期大于一个小时，则每个次上传都保存一次数据
								{
								    save_cnt=1;
								}
								else
								{
									  //计算出多少个包后进行一次保存
									  save_cnt = 3600 / NodeMsg.CycleS;
								}
								
								if(now_cnt >=save_cnt) 
								{
								   now_cnt=1;
									 save_node_msg();
								}
								else
								{
									 now_cnt++;
								}
#endif
							
								lorawan_send(SendInfo.SendBuffer,SendInfo.SendBufferSize,SendInfo.mode);
								//该处等待发送完成应答后进入休眠。由于存在没有应答的情况，所以之后添加一个超时定时器
								//超时后，重新初始化SX1278。
						}

				}
				if(event & LORAWAN_EVENT_SLEEP)
				{
					  if(lwrt_mode != LORAWAN_STATUE_CFG) //配置模式下不进入休眠
						{
								LOG_D("sleep");
								rt_thread_mdelay(200);
							  system_cycle_timer_start(sleep_time);
//								rtc_set_alarm(sleep_time);
//								rt_thread_mdelay(100);
//								//进入休眠
//								enter_stop();
						}
				}
		}
}


ALIGN(RT_ALIGN_SIZE)
static char thread_stack[1024];
static struct rt_thread lora_rtt_thread;


/*该段代码用于实现在运行正式程序前等待配置阶段******************************/
static uint8_t config_flag=1;//由于是否退出配置标识
static TimerEvent_t system_delay_timer;//该定时用来上电时等待配置
static uint8_t cnt=0; //用于倒数
void system_delay_timer_cb(void *p)
{
    config_flag=0;
}

void lw_set()
{
	 cnt=0;
   TimerStop(&system_delay_timer);
}
MSH_CMD_EXPORT(lw_set,lw_set);

void lw_start()
{
   config_flag = 0;
   store_flag = 1;	
}
MSH_CMD_EXPORT(lw_start,lw_start);

/************************************************************************/

/*使用UID作为DEVEUI******************************************************/

uint8_t Util_breakUint32(uint32_t var, int byteNum)
{
    return(uint8_t)((uint32_t)(((var) >> ((byteNum) * 8)) & 0x00FF));
}
//将32位数据拆分保存在数组中
//pBuf :被保存数据的位置指针
//val :被拆分的32位数据
//return :数据依次保存后剩余空间的指针
uint8_t *Util_bufferUint32(uint8_t *pBuf, uint32_t val)
{
    *pBuf++ = Util_breakUint32(val, 0);
    *pBuf++ = Util_breakUint32(val, 1);
    *pBuf++ = Util_breakUint32(val, 2);
    *pBuf++ = Util_breakUint32(val, 3);
    return(pBuf);
}

void use_uid_set_deveui()
{
	 uint32_t a = *(uint32_t *)(0x1FF80050 +0X04);
	 uint32_t b = *(uint32_t *)(0x1FF80050 +0X14);
//	 uint8_t temp[8];
	
//	 rt_kprintf("%04X,%04X\n",a,b);
	
	 Util_bufferUint32(LwrtInfo->DevEui,a);
	 Util_bufferUint32(LwrtInfo->DevEui+4,b);
	
//	 for(int i=0;i<8;i++)
//	 {
//       rt_kprintf(" %02X",temp[i]);
//	 }
}
//MSH_CMD_EXPORT(use_uid_set_deveui,use_uid_set_deveui);


/********************************************************************/

void lorawan_rtt_init()
{	 

	 TimerInit(&system_delay_timer,system_delay_timer_cb);
	
	 //挂载协议基本信息
	 lorawan_join_info_read("all",&LwrtInfo);
	 lorawan_cfg_read("all",&LwrtCfg);
	 lorawan_statue_read("all",&LwrtStatue);
	
	 if(LwrtInfo == RT_NULL || LwrtCfg ==RT_NULL || LwrtStatue ==RT_NULL)
	 {
		  LOG_W("Lwrt ptr error");
		  return;
	 }
	 //尝试读取flash信息
	 
	 //--下列方式为配置版本
 
	 if(read_node_msg() == 0)//如果读出成功
	 {
	     store_flag = 1;
		   TimerSetValue(&system_delay_timer,10*1000);
		   cnt = 10;
	 }
	 else
	 {
		   TimerSetValue(&system_delay_timer,30*1000);
	     store_flag = 0;
		   cnt =30;
	 }
	 use_uid_set_deveui();
	 TimerStart(&system_delay_timer);
 
	 //下列方式为自动入网版本，上电自动打印节点基本信息

//	 config_flag = 0;//不进行循环
//	 store_flag =1 ;//不对其进行判断休眠
//	 if(read_node_msg() == 0)//如果读出成功
//	 {
//	       
//	 }
//	 else
//	 {
//         use_uid_set_deveui();   
//	 }
//	 lw_id(); 
	 
 
   //设定超时定时器，超时则退出循环。通过lw_set来关闭定时器
	 while(config_flag)
	 {
		   if(cnt > 0)
			 {
		       LOG_I("Timeout is about to expire after %d seconds\n",cnt--);
			 }
	     rt_thread_mdelay(1000);
	 }
//如果在该处休眠 1278是没有休眠的	 
//	 if(store_flag == 0) //如果等待配置超时，且未读取到保存信息则直接进入休眠，该状态发送在工厂生产结束时。
//	 {
//		   LOG_I("go to sleep\n");
//	     enter_stop();
//		   return;
//	 }
	 
	 //这里将DEVEUI 修改为STM32芯片ID 
	 
	 rt_event_init(&lorawan_event,"lw_ev",RT_IPC_FLAG_FIFO);
	 //静态创建线程
	 rt_thread_init(&lora_rtt_thread,
                   "lora",
                   lorawan_rtt_thread_entry,
                   RT_NULL,
                   &thread_stack[0],
                   sizeof(thread_stack),
                   4, 10);
									 
   rt_thread_startup(&lora_rtt_thread);
}



/******************************************************************************************************/
void lw_id()
{
	  rt_kprintf("DEVEUI=[");
	  for(int i=0;i<8;i++)
	  {
       rt_kprintf(" %02X",LwrtInfo->DevEui[i]);
		}
		rt_kprintf("]\n");
		
	  rt_kprintf("APPKEY=[");
	  for(int i=0;i<16;i++)
	  {
       rt_kprintf(" %02X",LwrtInfo->AppKey[i]);
		}
		rt_kprintf("]\n");
}
MSH_CMD_EXPORT(lw_id,lw_id);

#include <stdlib.h>

void lw_cycle(int argc, char**argv)
{
   if(argc != 2)
	 {
	    rt_kprintf("param error");
		  return;
	 }
	 //uint8_t len = strlen(argv[1]);
	 uint32_t cycle = atoi(argv[1]);    
	 if(cycle > 30 && cycle <86400)  
	 {
	     NodeMsg.CycleS = cycle;
		   rt_kprintf("[cycle set :%d]\n",cycle);
	 }
	 else
	 {
	     rt_kprintf("[set cycle error]\n");
	 }
	 	
}
MSH_CMD_EXPORT(lw_cycle,cycle);



/*
命令列表
lw_join
lw_set_deveui 1122334455667788
display_node_msg
lw_save
lw_clear
lw_exit_cfg
*/

//配置命令

////--入网命令，使用默认方式发送入网包
//void lw_join()
//{
//   rt_event_send(&lorawan_event,LORAWAN_EVENT_JOIN);
//}
//MSH_CMD_EXPORT(lw_join,lw_join);

//--修改DEVEUI
//void lw_set_deveui(int argc, char**argv)
//{
//	 int i=0;
//   if(argc != 2)
//	 {
//	    LOG_I("param error");
//	 }
//	 else
//	 {
//	    	//判断非法字符
//				for(i=0;i<strlen(argv[1]);i++)
//				{
//					if( isxdigit(argv[1][i]) ==0)
//					{			 
//					   LOG_I("Enter illegal character\n");
//						 return;
//					}
//				}
//				
//				//判断字符长度
//				if(strlen(argv[1])!=16 )
//				{
//				   LOG_I("data len=%d\n",strlen(argv[1]));
//					 return;
//				}
//				StrToHex((char *)(LwrtInfo->DevEui),argv[1],8);
//	 }
//}
//MSH_CMD_EXPORT(lw_set_deveui,lw_set_deveui);


//-- 查看节点信息


//--发送一帧数据
//void lw_send_test()
//{		
//		rt_event_send(&lorawan_event,LORAWAN_EVENT_SEND);//事件方式
//}
//MSH_CMD_EXPORT(lw_send_test,lw_send_test);


//--修改通道
//void lw_set_ch(int argc, char**argv)
//{
//   if(argc != 3)
//	 {
//      LOG_I("param error"); 
//		  return 0;	 
//	 }
//	 uint8_t num = argv[1][0] - 0X30;
//	 uint16_t data;
//	 
//	 LwrtCfg->ChannelsMask[num] = data;
//}
//	    LwrtCfg->ChannelsMask[grp] = data;
//      lorawan_set_config("channel");


//--设置保存基本信息
//void lw_save()
//{
//		save_node_msg();
//}
//MSH_CMD_EXPORT(lw_save,lw_save);




//--退出配置模式
//void lw_exit_cfg()
//{
//		lwrt_mode = lwrt_last_mode;
//	  rt_event_send(&lorawan_event,LORAWAN_EVENT_SLEEP);
//}
//MSH_CMD_EXPORT(lw_exit_cfg,lw_exit_cfg);

//void lw_cfg_mod()
//{
//		rt_event_send(&lorawan_event,LORAWAN_EVENT_CFG);
//}
//MSH_CMD_EXPORT(lw_cfg_mod,lw_set_mod);
