/*用于和无线模组通信*/

#include "fifo.h"
#include "uart.h"
#include "weichuan_m100d.h"
#include "systick.h"
#include "stdio.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "debug.h"

/***********************************************************************************************/

#define COM3_MAX_SER_RCV_DATA_FIFO_SIZE  200
#define COM3_JOIN_CYCLE_TIME  2
#define RCV_SEND_ACK_STATE_TIMEOUT 4
#define RCV_DOWNLOAD_TIMEOUT 4
//static Fifo_t Serial3ReciveFifo;
//static char Serial3ReciveDataBuffer[COM3_MAX_SER_RCV_DATA_FIFO_SIZE];

//static SendAckInfo RcvSendState;

static lora_send_pck SendModPck;
static lora_send_pck RcvModPck;
//static lora_send_pck RcvModOrder;

//static bool StateAckFlag=0;    //是否有状态回复包
//static bool StateDownload=0;   //是否有下行数据
//static bool SendAckFlag=0;   //发送数据包返回标志
//static uint8_t SendAckKey;
//static uint8_t StateAckKey;


enum{
  SYSTEM_CMD_DEEL = 0XFA,
  SYSTEM_CMD_SET = 0XFB,
};
//0XFA
enum
{
   SYSTEM_CMD_DEEL_RST =0XC3,
   SYSTEM_CMD_DEEL_RST_WIRELESS =0XC4,
};

//0XFB
enum
{
   SYSTEM_CMD_SET_UPLOAD_TIME =0XD1,
};

//状态应答
enum{
ACK_NODE_CONFIG=1,
ACK_NODE_OPERATION=2,
ACk_NODE_LIST=3,
};

static uint8_t network = 0;      //0 未入网，1第一次入网，2复位后入网 3 设备忙
static uint8_t wireless_send=0;  //0,不允许发送 1：允许发送

void (*cmd_set_upload_time)(int) = NULL;

uint8_t get_network_state()
{return network;}
void set_network_state(uint8_t net)
{network = net;}
uint8_t get_wireless_send_state()
{return wireless_send;}


//模组唤醒
void m100d_wake()
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	SysDelayMs(5);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	wireless_send=1;
}


void m100d_rst()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	SysDelayMs(110);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
    network = 0;
	wireless_send = 0;
}

void deel_mod_rcv_joinpck(uint8_t data)
{
  	  switch(data)
		  {
			 case LORA_JOIN_OK:						      
				 network=1;
				 wireless_send=1;
           //DEBUG_NORMAL("jion OK\n");
			     //SysDelayMs(100);
				 break;
			 case LORA_JOIN_FAIL:
				  network=0;
			      //DEBUG_NORMAL("jion fail\n");
			      //SysDelayMs(100);
				 break;
			 case LORA_JOIN_ING:
              //    DEBUG_NORMAL("jion ing\n");
			      //SysDelayMs(100);
				 break;   
		 }
}

//处理发送数据后返回的状态信息
void deel_mod_rcv_send_state(uint8_t data)
{
   	switch(data)
	{
	   case SEND_OK:
			   //DEBUG_NORMAL("send ok\n");
               //SysDelayMs(100);	   
			   break;
	   case SEND_FINISH: //发送完成
			   //DEBUG_NORMAL("send over\n");
	           //SysDelayMs(100);
			   wireless_send=1;
			   network=1; 
			   break;
		 case MSG_ERR_NOT_JOIN: //未入网
			   //DEBUG_NORMAL("send no network\n");
		       //SysDelayMs(100);
			   wireless_send=0;
			   network=0;
			 break;
		 case ERR_BUSY: //设备忙
			  // DEBUG_NORMAL("send busy\n");
		       //SysDelayMs(100);
			   network=3;
			   //m100d_rst();//解决唯传设备忙的BUG，直接复位无线模组
			 break;
		 default:
			   {
				wireless_send=1;
				//DEBUG_NORMAL("send data error\n");
				 //SysDelayMs(100);  
				network=0;
			   }
    }
}

//处理模组返回的状态
void deel_mod_rcv_state(uint8_t data)
{
	switch(data)
	{
	  case 2: //已入网
	  {
	   //DEBUG_NORMAL("state network ok\n");
	   //SysDelayMs(100);
	   network=1; 
	  } break;
	  case 3: //入网失败
	  {
	  // DEBUG_NORMAL("state network fail\n");
	   //SysDelayMs(100);
	   network=0; 
	  }break;
	  case 4://发送中
	  {
	   //DEBUG_NORMAL("state send while \n"); //建议发生问题将唯传模组复位	 
		//SysDelayMs(100);
	  }break;
	  case 5://发送成功
	  {
	   //DEBUG_NORMAL("state send ok \n");
	   //SysDelayMs(100);
	   network=1; 
	  }break;
	default:
	  {
	   //DEBUG_NORMAL("state send other \n");
	   //SysDelayMs(100);
	  }
	}
}

//下行命令处理
void switch_download_data(uint8_t *data)
{
   switch (data[0])
	 {
		 case SYSTEM_CMD_DEEL: //执行命令
			if(data[1] == SYSTEM_CMD_DEEL_RST_WIRELESS)//重启lora模组
			{
				m100d_rst();
			}	 
		    if(data[1] == SYSTEM_CMD_DEEL_RST)//重启锚点
			{
			  *((uint32_t *)0xE000ED0C) = 0x05fa0004;
			}
 			 break;
		 case SYSTEM_CMD_SET: //配置命令
			if(data[1] == SYSTEM_CMD_SET_UPLOAD_TIME)//修改上传周期
			{
			  int time = data[2] *256 +data[3];
			  cmd_set_upload_time(time);
			} 
			 break;
	 }
}

//串口数据发送函数
static void SendData(unsigned char *data,int lenth)
{
   UartPutChars(2,data,lenth);
}

//串口中断回调
static void Serial3ReceiveHandle( uint8_t notify, uint16_t value )
{
 if( notify == UART_NOTIFY_RX )
 {		
			//包头判断
		 static unsigned char flag=0,last=0;
		 static uint8_t data_num= 0 ; //用于保存接收数据存放位置
		 static uint8_t data[255];
	   
		 uint8_t c =(uint8_t)value;
		 if(c == 0xFE ) {
			 last=0xFE;
		 }else if(flag == 0 && (c == 0x01 || c ==0x02 || c ==0x03)) {
			if(last ==0xFE) {
				flag =1;
				data[data_num++] = 0xFE;
				data[data_num++] = c;
				return;
			} else {
				last = 0;
			}
		}else {
			last = 0;
		}
		
		if(flag) {//包头正确
			 data[data_num++] = c;
		}
		if(data_num >= 254)
		{
			//未找到尾包,数据抛弃
			data_num=0;
			flag = 0;
		}
		if(c == 0XEF)
		{
			
			flag=0;
			RcvModPck = M100FrameReciveData(data,data_num);//解包
			SendModPck = M100dFrameACK(SEND_OK);//应答包
			data_num = 0;
			
			if(RcvModPck.type == LORA_MOD_DATA) //如果接受到平台下发数据
			{
				switch_download_data(RcvModPck.data+1);//第一个包是端口号
				SendData(SendModPck.data,SendModPck.Lenth);
			}
			else if(RcvModPck.type == LORA_MOD_STATE)//启动入网时的返回包
			{
				deel_mod_rcv_joinpck(RcvModPck.data[0]);
				SendData(SendModPck.data,SendModPck.Lenth);
			}
			else if(RcvModPck.type == LORA_MOD_RST_OK)//复位后回包，不做处理
			{
				SendData(SendModPck.data,SendModPck.Lenth);
			}
			else if(RcvModPck.type == LORA_MOD_RESPONSE)//发送数据后的状态返回
			{
				deel_mod_rcv_send_state(RcvModPck.data[0]);
			}		
			else if(RcvModPck.type == LORA_MOD_RESPONSE_STATE) //请求命令得到的返回
			{
				deel_mod_rcv_state(RcvModPck.data[0]);
			}		
		
		}
		
  }

}


void qurey_wireless_state_pck()
{
   SendModPck = lora_send_request_state();
   SendData(SendModPck.data,SendModPck.Lenth);
}

void  wireless_send_data(uint8_t* data , int lenth)
{
   	 SendModPck = M100dFrameSend(data,lenth,1,128);
	 SendData(SendModPck.data,SendModPck.Lenth);
}

void M100d_Init(void(* callback)(int))
{
	//模组复位引脚初始化
    GPIO_InitTypeDef  GPIO_InitStructure;
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_1;	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  GPIO_SetBits(GPIOA,GPIO_Pin_1);
		GPIO_SetBits(GPIOA,GPIO_Pin_5);  
	
  	cmd_set_upload_time =  callback;//挂载修改上传周期的函数
	
 	  m100d_rst();
	  //FifoInit( &Serial3ReciveFifo, Serial3ReciveDataBuffer, COM3_MAX_SER_RCV_DATA_FIFO_SIZE );
    UartInit( 2, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    UartSetInterrupt( 2, UART_IT_FLAG_RX, 0, 0, Serial3ReceiveHandle);
	
}

/***********************************************************************************************/

