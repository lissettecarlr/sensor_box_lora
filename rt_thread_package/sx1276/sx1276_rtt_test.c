#include <rtthread.h>
#include <string.h>
#include "rtdevice.h"
//#include "sx1276.h"
#include "sx1276_board.h"

/**************************************************************************************************/
#ifdef RT_USING_ULOG

#define LOG_TAG              "sx1276_t"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>

#else

#define LOG_E(format, ...)                    do{rt_kprintf(format, ##__VA_ARGS__);rt_kprintf("\n");}while(0)
#define LOG_W(format, ...)                    do{rt_kprintf(format, ##__VA_ARGS__);rt_kprintf("\n");}while(0)
#define LOG_I(format, ...)                    do{rt_kprintf(format, ##__VA_ARGS__);rt_kprintf("\n");}while(0)
#define LOG_D(format, ...)                    do{rt_kprintf(format, ##__VA_ARGS__);rt_kprintf("\n");}while(0)
#define LOG_RAW(format, ...)                  do{rt_kprintf(format, ##__VA_ARGS__);rt_kprintf("\n");}while(0)
//#define LOG_HEX(format, ...)    						ulog_hex(format, ##__VA_ARGS__)

extern void ulog_hexdump(const char *tag, rt_size_t width, rt_uint8_t *buf, rt_size_t size);

#endif
/**************************************************************************************************/

#define SEND_MODE  0    //1:fsk  0:lora
#define TEST_SEND  1    //1 send 0:rcv
//lora                                              470700000Hz 
#define RF_FREQUENCY                                470300000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm


#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000

uint8_t SendBuffer[4]={'1','1','1',0};
uint8_t RcvLen=0;
uint8_t RcvBuffer[100];
int8_t RssiValue = 0;
int8_t SnrValue = 0;
uint8_t state=0; //是否允许发送和接收，0:否

static RadioEvents_t RadioEvents;

void OnTxDone( void )
{
		 Radio.Sleep( );
	   LOG_D("OnTxDone");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
//	  int i=0;
	  RcvLen = size;
    memcpy( RcvBuffer, payload, RcvLen );
		Radio.Sleep( );
	  LOG_I("rcv a data -RSSI:%d Snr=%d datasize=%d data:",rssi,snr,size);
	  ulog_hexdump("sx1276_t",16,RcvBuffer,size);
//	  for(i=0;i< size;i++)
//	  {
//       rt_kprintf("%02X  ",RcvBuffer[i]);    		   
//		}
}

//当TX超时时 将回发生射频重置
void OnTxTimeout( void )
{
		Radio.Sleep( );
	  //Radio.Init(&RadioEvents);
	  Radio.SetChannel(RF_FREQUENCY);
	  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
	  LOG_D("OnTxTimeout");
}

void OnRxTimeout( void )
{
		Radio.Sleep( );	
		LOG_D("OnRxTimeout");	
}

void OnRxError( void )
{
		Radio.Sleep( );
		LOG_D("OnRxTimeout");
}	



void sx1276_send(void *p)
{
  while(1)
	{
		 if(state)
		 {
			   LOG_I("send thread");
	       rt_thread_mdelay(5000);
			   //SX1276Send(SendBuffer,4);
         Radio.Send( SendBuffer, 4 );  
		 }
	} 
}
void sx1276_rcv(void *p)
{
	while(1)
	{
		 //rt_kprintf("open rcv\n");
     //Radio.Rx( RX_TIMEOUT_VALUE*5 );	 //10s 
		 Radio.Rx( 0);	 //10s 
		 rt_thread_mdelay(100);
	}
}

Sx1276TimerEvent_t test={"test",RT_NULL,100,RT_NULL};

void test_fun(void* parameter)
{
	 
   rt_kprintf("1111111111111111");
	 Sx1276TimerStart(&test);
}

void sx1267_lora_test_thread_entry(void *parameter)
{
	LOG_D("SX1276_thread\n");

  if(sx1276_board_init() != 0)
	{
	   
	}
//	 return ;
	
	uint8_t temp=SX1276Read(0x42);
	LOG_D("temp = %d\n",temp);
	
//	while(1)
//	{
//		uint8_t temp=SX1276Read(0x42);
//		rt_kprintf("temp = %d\n",temp);
//		rt_thread_mdelay(3000);
//	}
	
//  Sx1276TimerInit( &test, test_fun );
//	Sx1276TimerSetValue( &test, 1000 );
//	Sx1276TimerStart(&test);
//	while(1)
//	{
//	
//	}
	
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  Radio.Init(&RadioEvents);

	Radio.SetChannel(RF_FREQUENCY);
	//SX1276SetChannel(RF_FREQUENCY);
	Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
//   SX1276SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
//                                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
//                                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
//                                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
	
  state=1;
	
}

void sx1276_thread_init(void)
{
   rt_thread_t sx1276_hander;
	 sx1276_hander = rt_thread_create("sx1276_test"
									,sx1267_lora_test_thread_entry
									,RT_NULL
									,4096
									,2
									,10
									);

	 if (sx1276_hander != RT_NULL)
	 {
        rt_thread_startup(sx1276_hander);
	 }
	  
	 //再开启接收或者发送进程
#if TEST_SEND	 
	 rt_thread_t sx1276_send_hander;
	 sx1276_send_hander = rt_thread_create("sx1276_test_send"
									,sx1276_send
									,RT_NULL
									,1024
									,2
									,10
									);

	 if (sx1276_send_hander != RT_NULL)
	 {
        rt_thread_startup(sx1276_send_hander);
	 }
#else
   rt_thread_t sx1276_send_hander;
	 sx1276_send_hander = rt_thread_create("sx1276_test_rcv"
									,sx1276_rcv
									,RT_NULL
									,1024
									,4
									,10
									);

	 if (sx1276_send_hander != RT_NULL)
	 {
        rt_thread_startup(sx1276_send_hander);
	 } 
#endif
}	
