/*
时间:2019年5月24日
版本:V1.0
说明:单独驱动SX1278，使用AT命令进行通信。在主函数中调用sx1278_rtt_init()函数即可。使用前请先启用ULOG，fal和AT组件，且fal组件存在eeprom2的分区
		 AT命令和使用方式可见同目录readme文档
*/

#include "sx1278_rtt.h"
#include "rtthread.h"
#include "sx1276_board.h"
#include "fal.h"
#include "at.h"
#include "string.h"
#include <ctype.h>  //toupper  isxdigit
#include "lorawan_parser.h"

#define LOG_TAG              "sx1276_rtt"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>

//other
static void StrToHex(char *pbDest, char *pbSrc, int nLen)
{
    char h1,h2;
    char s1,s2;
    int i;

    for (i=0; i<nLen; i++)
    {
        h1 = pbSrc[2*i];
        h2 = pbSrc[2*i+1];

        s1 = toupper(h1) - 0x30;
        if (s1 > 9) 
            s1 -= 7;

        s2 = toupper(h2) - 0x30;
        if (s2 > 9) 
            s2 -= 7;

        pbDest[i] = s1*16 + s2;
    }
}


#define DEFAULT_RF_FREQUENCY                                470300000 // Hz
#define DEFAULT_TX_POWER                                    14        // dBm

#define DEFAULT_BANDWIDTH                                   0         // [0: 125 kHz,
																																			//  1: 250 kHz,
																																			//  2: 500 kHz,
																																			//  3: Reserved]
#define DEFAULT_SPREADING_FACTOR                       			12        // [SF7..SF12]
#define DEFAULT_CODINGRATE                                	1         // [1: 4/5,
																																			//  2: 4/6,
																																			//  3: 4/7,
																																			//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         				// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0        					// Symbols

#define LORA_IQ_INVERSION_ON                        false


/*****************************************************************************************************/
//配置结构体
typedef struct sLoraCfg{
	uint8_t  tx_power;
	uint8_t  bandwidth;
	uint8_t  sf;
	uint8_t  rate;
	uint32_t frequency;
}LoraCfg_s;
LoraCfg_s LoraCfg={DEFAULT_TX_POWER,DEFAULT_BANDWIDTH,DEFAULT_SPREADING_FACTOR,DEFAULT_CODINGRATE,DEFAULT_RF_FREQUENCY,};


void display_lora_cfg()
{
    LOG_I("power:%d,bw:%d,sf:%d,rate:%d,fre:%ld",LoraCfg.tx_power,LoraCfg.bandwidth,LoraCfg.sf,LoraCfg.rate,LoraCfg.frequency);
}
MSH_CMD_EXPORT(display_lora_cfg,display_lora_cfg);

int read_lora_cfg()
{
   const struct fal_partition * bli = fal_partition_find("eeprom2") ;
	 if (bli ==RT_NULL)
	 {
		  LOG_E("fal error");
		  return 1;
	 }
	 LoraCfg_s LoraCfgTemp;
	 
	 fal_partition_read(bli,0,&(LoraCfgTemp.tx_power),sizeof(LoraCfg_s)) ;
	 
	 if((LoraCfgTemp.tx_power == 0 ) ||  (LoraCfgTemp.tx_power == 0xff))//判断是否保存过
	 {
			 LOG_I("not find node info");
		   return 3;
	 }
	 else
	 {
	    LOG_I("node info read succeed");
		  rt_memcpy( &LoraCfg, &LoraCfgTemp, sizeof(LoraCfg_s));
	    display_lora_cfg();
		  return 0;	    
	 }
} 
MSH_CMD_EXPORT(read_lora_cfg,read_lora_cfg);

//保存当前节点信息，返回0则表示保存成功
int save_lora_cfg()
{
	 const struct fal_partition * bli = fal_partition_find("eeprom2") ;
	 if (bli ==RT_NULL)
	 {
		  LOG_D("fal error");
		  return 1;
	 }
	 
		fal_partition_erase(bli,0,sizeof(LoraCfg_s));
	  rt_thread_mdelay(1000);
	 
	  display_lora_cfg();
	 
	  fal_partition_write(bli,0,&(LoraCfg.tx_power),sizeof(LoraCfg_s));
		return 0;
}
MSH_CMD_EXPORT(save_lora_cfg,save_lora_cfg);

//情况数据表
int clear_lora_cfg()
{
   const struct fal_partition * bli = fal_partition_find("eeprom2");
	 if (bli ==RT_NULL)
	 {
		  LOG_D("fal error");
		  return 1;
	 }
	 fal_partition_erase(bli,0,sizeof(LoraCfg_s));
	 return 0;
}
MSH_CMD_EXPORT(clear_lora_cfg,clear_lora_cfg);

/*****************************************************************************************************/
#define APP_DATA_MAX_SIZE 128
typedef struct sLoraSendData{
	  uint8_t len;
    uint8_t data[APP_DATA_MAX_SIZE];
}LoraSendData_s;
LoraSendData_s LoraSendData={0,{0}};

typedef struct sLoraRcvData{
		int8_t snr;
  	uint16_t len;
	  int16_t rssi;
	  uint8_t data[255];
}LoraRcvData_s;
LoraRcvData_s LoraRcvData;
static struct rt_event lora_event;
#define EVENT_LORA_ALL             0xFFFFFFFF
#define EVENT_LORA_RX_DO  		 (uint32_t)1<<1
#define EVENT_LORA_TX_DO			 (uint32_t)1<<2
#define EVENT_LORA_TX_TIMEOUT	 (uint32_t)1<<3
#define EVENT_LORA_RX_TIMEOUT  (uint32_t)1<<4
#define EVNET_LROA_RX_ERROR    (uint32_t)1<<5
#define EVENT_LORA_SEND        (uint32_t)1<<6
#define EVENT_SET_FRE 				 (uint32_t)1<<7
#define EVENT_SET_CFG 			   (uint32_t)1<<8

/*注意: 不同极性的数据包时收不到的*/
static bool lora_iq_inversion_switch = LORA_IQ_INVERSION_ON;
/*lorawan协议解析器开关*/
static bool lorawan_parser_switch = false;

//驱动回调函数
void lora_cb_tx_done( void )
{
	  LOG_D("OnTxDone");
	  rt_event_send(&lora_event,EVENT_LORA_TX_DO);
}

void lora_cb_rx_done( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	  LoraRcvData.len = size;
	  LoraRcvData.rssi = rssi;
	  LoraRcvData.snr = snr;
    rt_memcpy( LoraRcvData.data, payload, size );
	  LOG_D("rcv a data -RSSI:%d Snr=%d datasize=%d data:",rssi,snr,size);
	  rt_event_send(&lora_event,EVENT_LORA_RX_DO);
    ulog_hexdump("sx1276_rtt",16,LoraRcvData.data,size);
//	  for(i=0;i< size;i++)
//	  {
//       rt_kprintf("%02X  ",RcvBuffer[i]);    		   
//		}
}

//当TX超时时 将回发生射频重置
void lora_cb_tx_timeout( void )
{
		rt_event_send(&lora_event,EVENT_LORA_TX_TIMEOUT);
	  LOG_D("OnTxTimeout");
}

void lora_cb_rx_timeout( void )
{
		rt_event_send(&lora_event,EVENT_LORA_RX_TIMEOUT);
		LOG_D("OnRxTimeout");	
}

void lora_cb_tx_error( void )
{
		rt_event_send(&lora_event,EVNET_LROA_RX_ERROR);
		LOG_D("RxError");
}	

rt_thread_t lora_hander;
static RadioEvents_t RadioEvents;

void lora_rtt_thread_entry(void *p)
{
	 RadioEvents.TxDone = lora_cb_tx_done;
   RadioEvents.RxDone = lora_cb_rx_done;
   RadioEvents.TxTimeout = lora_cb_tx_timeout;
   RadioEvents.RxTimeout = lora_cb_rx_timeout;
   RadioEvents.RxError = lora_cb_tx_error;
   Radio.Init(&RadioEvents);
	
	 //读取配置
	 read_lora_cfg();
	 Radio.SetPublicNetwork( true );
	 //加载配置
	 //Radio.SetChannel(LoraCfg.frequency);
	 Radio.SetChannel( 0x1DD81360 );
	 Radio.SetTxConfig( MODEM_LORA, LoraCfg.tx_power, 0, LoraCfg.bandwidth,LoraCfg.sf, LoraCfg.rate,
                                                 LORA_PREAMBLE_LENGTH, false,
                                                 true, 0, 0, lora_iq_inversion_switch, 3000 );
																								 
																							 
	 //Radio.SetRxConfig( MODEM_LORA, 0, 0X0C, 1, 0, 8, 0X0C, false, 0, false, 0, 0, true, true );				
																								 
	 Radio.SetRxConfig( MODEM_LORA, LoraCfg.bandwidth, LoraCfg.sf,LoraCfg.rate, 0, LORA_PREAMBLE_LENGTH,
																								LORA_SYMBOL_TIMEOUT, false,
																								0, true, 0, 0, lora_iq_inversion_switch, true );
																								 
	  rt_thread_mdelay(2000);
	  at_server_printfln("+INIT:OK");
	  Radio.Rx(0);
    uint32_t event;
    while(1)
		{
					rt_event_recv(&lora_event, (EVENT_LORA_ALL),RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &event);
					if(event & EVENT_LORA_RX_DO)
					{
							at_server_printf("+RCV:%d,%d,%d,",LoraRcvData.rssi,LoraRcvData.snr,LoraRcvData.len);
						  LOG_D("+RCV:%d,%d,%d,",LoraRcvData.rssi,LoraRcvData.snr,LoraRcvData.len);
							for(int i=0;i<LoraRcvData.len;i++)
							{
									at_server_printf("%02X",LoraRcvData.data[i]);
								  LOG_D("%02X",LoraRcvData.data[i]);
							}
							at_server_printf("\r\n");
							
							//如果打开的lorawan协议解析器
							if(lorawan_parser_switch)
							{
									lw_parser(LoraRcvData.data,LoraRcvData.len);
							}
							Radio.Rx(0); 
							
					}
					if(event & EVENT_LORA_RX_TIMEOUT)
					{
					
					}
					if(event & EVENT_LORA_TX_DO)
					{
						 at_server_printfln("+SEND:OK");
						  LOG_D("+SEND:OK");
						 Radio.Rx(0); //发送完成后切换回接收状态					
					}
					if(event & EVENT_LORA_TX_TIMEOUT)
					{
						 
					   at_server_printfln("+SEND:TIMEOUT");
					}
					if(event & EVNET_LROA_RX_ERROR)
					{
					
					}
					if(event & EVENT_LORA_SEND)
					{
					    if(LoraSendData.len !=0)
							{
							    Radio.Send( LoraSendData.data, LoraSendData.len ); 
							}
							
					}
					if(event & EVENT_SET_FRE)
					{
						  LOG_I("change fre :%u",LoraCfg.frequency);
					    Radio.SetChannel(LoraCfg.frequency);
						  rt_thread_mdelay(100);
						  //频率切换后不发个包或者接收一个包，是切换不了的
						  uint8_t fre_byte[4];
						  fre_byte[0] = LoraCfg.frequency>>24;
						  fre_byte[0] = (uint8_t)(LoraCfg.frequency>>16);
						  fre_byte[0] = (uint8_t)(LoraCfg.frequency>>8);
						  fre_byte[0] = (uint8_t)LoraCfg.frequency;
						  Radio.Send( fre_byte, 4 ); 
						 
					}
					
					if(event & EVENT_SET_CFG)
					{					  
		 			
							Radio.SetTxConfig( MODEM_LORA, LoraCfg.tx_power, 0, LoraCfg.bandwidth,LoraCfg.sf, LoraCfg.rate,
                                                 LORA_PREAMBLE_LENGTH, false,
                                                 true, 0, 0, lora_iq_inversion_switch, 3000 );
	
							Radio.SetRxConfig( MODEM_LORA, LoraCfg.bandwidth, LoraCfg.sf,LoraCfg.rate, 0, LORA_PREAMBLE_LENGTH,
																							LORA_SYMBOL_TIMEOUT, false,
																							0, false, 0, 0, lora_iq_inversion_switch, true );																															
						  Radio.Rx(0);
					}
		}
}


void sx1278_rtt_init(void)
{
	 if(sx1276_board_init() != 0)
	 {
	    LOG_E("sx1276_board_init error");
		  at_server_printfln("+INIT:FAIL");
		  return;
	 }
	 rt_event_init(&lora_event,"lw_ev",RT_IPC_FLAG_FIFO);
	 
	 lora_hander =  rt_thread_create("lora_rtt"
									,lora_rtt_thread_entry
									,RT_NULL
									,2046
									,3
									,10
									);

	 if (lora_hander != RT_NULL)
	 {
       rt_thread_startup(lora_hander);
	 }
	 else
	 {
				LOG_E("lora thread create error");
		    at_server_printfln("+INIT:FAIL");
	 }
}

// AT

//发送数据
//--HEX发送，字符串转换为16进制数据，进行发送，例如AT+SENDHEX=112233，实际发送的是0X11,0X22,0X33
static at_result_t at_lora_send_hex(const char *args)
{
	  int argc,len,i;
	  char data[APP_DATA_MAX_SIZE];
	  char str[APP_DATA_MAX_SIZE*2]; //2个字符等于一个十六进制数据

	  //防越界
	  if(strlen(args) >(APP_DATA_MAX_SIZE*2))
		{
			 LOG_I("AT-SEND-HEX-ERROR: args:%d\n",strlen(args));
			 return AT_RESULT_FAILE; 
		}
		argc = at_req_parse_args(args, "=%s",str);
	  len = strlen(str)/2;
		if(len >APP_DATA_MAX_SIZE)//防越界
	  {
			   LOG_I("AT-SEND-HEX-ERROR: len:%d\n",len);
			   return AT_RESULT_FAILE;
		}
		//判断非法字符
		for(i=0;i<strlen(str);i++)
		{
		   if( isxdigit(str[i]) ==0)
			 {
					LOG_I("AT-SEND-HEX-ERROR: Enter illegal character\n");
					return AT_RESULT_FAILE;
			 }
		}
	  if ( argc == 1)
		{			  
			 StrToHex((char *)data,str,len);
			 LOG_I("AT-SEND-HEX-OK:");
       ulog_hexdump("lora_t",16,(uint8_t *)data,rt_strlen(str)/2 );				
			 memcpy(LoraSendData.data,data,len);
			 LoraSendData.len = len;
			 rt_event_send(&lora_event,EVENT_LORA_SEND);		  
		}
		else
		{
			  at_server_printfln("AT-SEND-HEX-ERROR:argc %d\n",argc);
			  return AT_RESULT_FAILE;
        //at_server_print_result(AT_RESULT_PARSE_FAILE);
		}
		return AT_RESULT_OK; 
}
AT_CMD_EXPORT("AT+SENDHEX","=<val1>",NULL,NULL,at_lora_send_hex,NULL);


//--设置保存基本信息
static at_result_t at_lora_save_cfg()
{
   save_lora_cfg();
	 return AT_RESULT_OK;
}

AT_CMD_EXPORT("AT+SAVE",RT_NULL,RT_NULL,RT_NULL,RT_NULL,at_lora_save_cfg);

//--清空保存的基本信息
static at_result_t at_lora_clear_cfg()
{
   clear_lora_cfg();
	 return AT_RESULT_OK;
}

AT_CMD_EXPORT("AT+CLEAR",RT_NULL,RT_NULL,RT_NULL,RT_NULL,at_lora_clear_cfg);


//--恢复默认配置
static at_result_t at_lora_cfg_default()
{
		LoraCfg.bandwidth = DEFAULT_BANDWIDTH;
		LoraCfg.frequency = DEFAULT_RF_FREQUENCY;
		LoraCfg.rate = DEFAULT_CODINGRATE;
		LoraCfg.sf = DEFAULT_SPREADING_FACTOR;
		LoraCfg.tx_power = DEFAULT_TX_POWER;
	
	  rt_event_send(&lora_event,EVENT_SET_FRE);
	  rt_event_send(&lora_event,EVENT_SET_CFG);
	  clear_lora_cfg();
	  return AT_RESULT_OK;
}
AT_CMD_EXPORT("AT+DEFAULT",RT_NULL,RT_NULL,RT_NULL,RT_NULL,at_lora_cfg_default);

static at_result_t at_lora_show_fre()
{
    at_server_printfln("+FRE:%d",LoraCfg.frequency);
	  return AT_RESULT_OK;
}
//--修改频率
static at_result_t at_lora_change_fre(const char *args)
{
   uint32_t fre;
	 int argc;
	 argc = at_req_parse_args(args,"=%u",&fre);
	 if(argc == 1)
	 {
			   //判断频率范围
		     if( fre >=137000000  && fre<=525000000)
				 {
						LoraCfg.frequency = fre;
						rt_event_send(&lora_event,EVENT_SET_FRE);
				 }
				 else
				 {
				     return AT_RESULT_FAILE; 
				 }
	 }
	 else
	 {
		  //at_server_print_result(AT_RESULT_PARSE_FAILE);
		  return AT_RESULT_FAILE; 
	 }
	 return AT_RESULT_OK;		 
}
AT_CMD_EXPORT("AT+FRE","=<val1>",at_lora_show_fre,RT_NULL,at_lora_change_fre,NULL);

//--修改配置,启用修改,查看配置(rate,sf,bd,power)
//---at+cfg?
static at_result_t at_lora_cfg_show()
{
	 at_server_printfln("+CFG:%d,%d,%d,%d,%u",LoraCfg.tx_power,LoraCfg.bandwidth,LoraCfg.sf,LoraCfg.rate,LoraCfg.frequency);
	 return AT_RESULT_OK;
}
//---at+cfg=1,2  //不可修改频率，频率单独命令
static at_result_t at_lora_cfg_set(const char *args)
{
   uint8_t param1,param2,argc;
	 argc = at_req_parse_args(args, "=%d,%d",&param1,&param2);
	 if(argc ==2)
	 {
	     LOG_D("AT-CFG:%d,%d\n",param1,param2);
		   switch(param1)
			 {
				  case 1: //tx_power
					{
						 LoraCfg.tx_power = param2;
					}break;
					case 2://bandwidth
					{
					   LoraCfg.bandwidth = param2;
					}break;
					case 3://sf
					{
					   LoraCfg.sf = param2;
					}break;
					case 4://rate
					{
					   LoraCfg.rate = param2;
					}break;
			 }
	 }
	 else
	 {
	    LOG_I("AT-CFG-ERROR:argc=%d",argc);
		  return AT_RESULT_FAILE;  
	 }
	 return AT_RESULT_OK;
}
//---at+cfg=?  //提示参数含义
static at_result_t at_lora_cfg_param()	
{
	 at_server_printfln("1:power = (<14)");	
	 at_server_printfln("2:bandwidth = (0,1,2,3)");
	 at_server_printfln("3:sf = (7~12)");
	 at_server_printfln("4:rate = (2,3,4)");
	 return AT_RESULT_OK;
}
	
//---at+cfg  //生效修改
static at_result_t at_lora_cfg_take_effect()
{
   rt_event_send(&lora_event,EVENT_SET_CFG);  
	 return AT_RESULT_OK;
}
AT_CMD_EXPORT("AT+CFG","=<val1>,<val2>", at_lora_cfg_param, at_lora_cfg_show, at_lora_cfg_set, at_lora_cfg_take_effect);


//--at+iq=0
static at_result_t at_lora_iq_set(const char *args)
{
     uint8_t param1,argc;
	   argc = at_req_parse_args(args, "=%d",&param1);
		 if(argc ==1)
	   {
	     LOG_D("AT-IQ=%d\n",param1);
       lora_iq_inversion_switch = param1;
			 rt_event_send(&lora_event,EVENT_SET_CFG); 
	   }
	  else
	  {
	    LOG_I("AT-IQ-ERROR:argc=%d",argc);
		  return AT_RESULT_FAILE;  
	  }
	  return AT_RESULT_OK;
}
//--at+iq=?
static at_result_t at_lora_iq_show()
{
   at_server_printfln("+IQ=%d",lora_iq_inversion_switch);
	 return AT_RESULT_OK;
}

AT_CMD_EXPORT("AT+IQ","=<val1>", RT_NULL, at_lora_iq_show, at_lora_iq_set, RT_NULL);

//--at+lorawaw_parser
static at_result_t at_lorawan_parser(const char *args)
{
	 int argc;
	 char str[10];
	 argc = at_req_parse_args(args, "=%s",str);
	 if(argc !=1)
	 {
	     return AT_RESULT_FAILE;
	 }
	 if(strcmp(str,"OFF") == 0)
	 {
				lorawan_parser_switch = false;
	 }
	 else if(strcmp(str,"ON") == 0)
	 {
				lorawan_parser_switch = true;
	 }
	 else
	 {
	    return AT_RESULT_FAILE;
	 }
	 return AT_RESULT_OK;
}
AT_CMD_EXPORT("AT+parser","=<val1>", RT_NULL, RT_NULL, at_lorawan_parser, RT_NULL);
