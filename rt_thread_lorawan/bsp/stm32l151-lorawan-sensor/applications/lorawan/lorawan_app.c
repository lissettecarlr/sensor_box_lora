/*
该文件是协议栈对外的简化交互接口，尽量不与其他组件纠葛
包含入网参数的结构体、包含lorawan配置信息的结构体、lorawan状态信息的结构体
混合入网流程:
先使用OTAA入网，然后存储入网参数，修改模式为ABP，在低电量和关机前必须进行一次状态存储
重启后读取保存的入网参数和包计数，使用ABP入网方式。


//混合入网: 使用lorawan_join注册绑定入网回调，该函数返回0XFF则表示混合入网，需要对信息进行flash保存，可以通过
lorawan_join_info_read函数获取到参数。

*/

#include "stdint.h"
#include "LoRaMac.h"
//config宏定义是保存设备的一些默认参数
#include "lorawan_config.h"
#include "lorawan_app.h"
#include "LoRaMacTest.h"


#define DEBUG  rt_kprintf

//入网模式
#define JOIN_MODE_OTAA 0
#define JOIN_MODE_ABP  1
#define JOIN_MODE_MIXTURE_OTAA 2
#define JOIN_MODE_MIXTURE_ABP 3

/**************************************************************************************************************************/


static LoraWanJoinInfo_s info={LORAWAN_DEFAULT_JOIN_MODE,LORAWAN_DEFAULT_JOIN_ADDR,\
                               LORAWAN_DEFAULT_JOIN_DEVEUI,LORAWAN_DEFAULT_JOIN_APPEUI,LORAWAN_DEFAULT_JOIN_APPKEY,\
                               LORAWAN_DEFAULT_JOIN_NETSKEY,LORAWAN_DEFAULT_JOIN_APPSKEY};


//入网参数的写,key表示需要修改的变量名，data为指向数据的指针
int lorawan_join_info_write(char *key,void *data)
{
    if(strcmp(key,"JoinMode") ==0)
    {
       info.JoinMode =  *((uint8_t *)data);
    }
    else if(strcmp(key,"Addr") ==0)
    {
       info.Addr =  *((uint32_t *)data);
    }
    else if(strcmp(key,"DevEui") ==0)
    {
       memcpy(info.DevEui,data,8);
    }
    else if(strcmp(key,"AppEui") ==0)
    {
        memcpy(info.AppEui,data,8);
    }
    else if(strcmp(key,"AppKey") ==0)
    {
        memcpy(info.AppKey,data,16);
    }
    else if(strcmp(key,"NetSKey") ==0)
    {
        memcpy(info.NetSKey,data,16);
    }
    else if(strcmp(key,"AppSKey") ==0)
    {
        memcpy(info.AppSKey,data,16);
    }
    else
    {
        return 1;
    }
    return 0;
}
//入网参数的读
int lorawan_join_info_read(char *key,void *data)
{
    if(strcmp(key,"JoinMode") ==0)
    {
       *((uint8_t *)data) = info.JoinMode;
    }
    else if(strcmp(key,"Addr") ==0)
    {

      *((uint32_t *)data)=info.Addr;
    }
    else if(strcmp(key,"DevEui") ==0)
    {
       memcpy(data,info.DevEui,8);
    }
    else if(strcmp(key,"AppEui") ==0)
    {
        memcpy(data,info.AppEui,8);
    }
    else if(strcmp(key,"AppKey") ==0)
    {
        memcpy(data,info.AppKey,16);
    }
    else if(strcmp(key,"NetSKey") ==0)
    {
        memcpy(data,info.NetSKey,16);
    }
    else if(strcmp(key,"AppSKey") ==0)
    {
        memcpy(data,info.AppSKey,16);
    }
		else if(strcmp(key,"all") ==0)
		{
		    //rt_memcpy(data,&info,sizeof(info));
			*((uint32_t*)data)=(uint32_t)&info;  //这里直接将info的指针给了出去，使其外部操作data实际上便是操作info
		}
    else
    {
        return 1;
    }
		return 0;
}
LoraWanJoinInfo_s* lorawan_get_join_info_p()
{
   return (&info);
}

//测试用打印入网信息表
void lorawan_join_info_display()
{
   DEBUG("JoinMode:%d\n",info.JoinMode);
   DEBUG("Addr:%08X\n",info.Addr);

   DEBUG("DevEui:");
   for(int i=0;i<8;i++)
    DEBUG(" %02X",info.DevEui[i]);
   DEBUG("\n");

    DEBUG("AppEui:");
   for(int i=0;i<8;i++)
    DEBUG(" %02X",info.AppEui[i]);
   DEBUG("\n");

    DEBUG("AppKey:");
   for(int i=0;i<16;i++)
    DEBUG(" %02X",info.AppKey[i]);
   DEBUG("\n");

    DEBUG("NetSKey:");
   for(int i=0;i<16;i++)
    DEBUG(" %02X",info.NetSKey[i]);
   DEBUG("\n");

    DEBUG("AppSKey:");
   for(int i=0;i<16;i++)
    DEBUG(" %02X",info.AppSKey[i]);
   DEBUG("\n");
}
MSH_CMD_EXPORT(lorawan_join_info_display,lorawan_join_info_display);


static LoraWanCfg_s cfg={LORAWAN_DEFAULT_ADR,LORAWAN_DEFAULT_DATARATE,LORAWAN_DEFAULT_POWER,LORAWAN_DEFAULT_CLASS,LROAWAN_DEFAULT_CHANNL};

//配置信息的写
int lorawan_cfg_write(char *key,void *data)
{
   if(strcmp(key,"Adr") ==0)
	 {
	     cfg.Adr =*((bool *)data);   
	 }
	 else if(strcmp(key,"DataRate") ==0)
	{
		cfg.DataRate =*((int8_t *)data);  
	}
	 else if(strcmp(key,"power") ==0)
	{
		cfg.power =*((int8_t *)data);  
	}
	 else if(strcmp(key,"Class") ==0)
	{
		cfg.Class =*((DeviceClass_t *)data);  
	}
	 else if(strcmp(key,"ChannelsMask") ==0)
	{
		  //rt_memcpy(cfg.ChannelsMask,data,12);
		  *((uint32_t*)data)=(uint32_t)&cfg;
	}
	else
	{
	   return 1;
	}
	return 0;
}
//配置信息的读
int lorawan_cfg_read(char *key,void *data)
{
   if(strcmp(key,"Adr") ==0)
	 {
	     *((bool *)data) = cfg.Adr ;   
	 }
	 else if(strcmp(key,"DataRate") ==0)
	{
		 *((int8_t *)data) =cfg.DataRate;  
	}
	 else if(strcmp(key,"power") ==0)
	{
		 *((int8_t *)data) = cfg.power;  
	}
	 else if(strcmp(key,"Class") ==0)
	{
		 *((DeviceClass_t *)data) = cfg.Class ;  
	}
	 else if(strcmp(key,"ChannelsMask") ==0)
	{
		  rt_memcpy(cfg.ChannelsMask,data,12);
	}
	else if(strcmp(key,"all") ==0)
	{
		  //rt_memcpy(data,&cfg,sizeof(cfg));
		 *((uint32_t*)data)=(uint32_t)&cfg;
	}
	else{return 1;}
	return 0;
}
LoraWanCfg_s* lorawan_get_cfg_p()
{
   return (&cfg);
}

void lorawan_cfg_display()
{
	rt_kprintf("adr:%d\tdatarate:%d\tpower:%d\tCLASS:%d\n",cfg.Adr,cfg.DataRate,cfg.power,cfg.Class);
	rt_kprintf("channl: ");
	for(int i=0;i<6;i++)
	{
	  rt_kprintf("%04X\t",cfg.ChannelsMask[i]);
	}
	rt_kprintf("\n");
}
MSH_CMD_EXPORT(lorawan_cfg_display,lorawan_cfg_display);

static LoraWanStatue_s statue={0};
//状态的读
int lorawan_statue_read(char *key,void *data)
{
    if(strcmp(key,"UpCounter") ==0)
    {
			 statue.UpCounter = lorawan_counter_read();
       *((uint32_t *)data) = statue.UpCounter;
    }
		else if(strcmp(key,"all") ==0)
		{
		    //rt_memcpy(data,&statue,sizeof(statue));
			 //*data = (&statue);
			 //data = &statue;
			 //*((uint32_t*)data)=(uint32_t)&statue;
			 	*((uint32_t*)data)=(uint32_t)&statue;
		}
		else
		{
				return 1;
		}
		return 0;
}
//状态的写
int lorawan_statue_write(char *key,void *data)
{
   if(strcmp(key,"UpCounter") ==0)
	 {
	      statue.UpCounter = *((uint32_t *)data) ;
	 }
	 else
	 {
	   return 1;
	 }
	 return 0;
}
//返回结构体指针
LoraWanStatue_s* lorawan_get_statue_p()
{
   return (&statue);
}
void lorwan_statue_display()
{
	 rt_kprintf("UpCounter:%d\n",statue.UpCounter);
}
MSH_CMD_EXPORT(lorwan_statue_display,lorwan_statue_display);
/**************************************************************************************************************************/

//lorawan回调函数
//--接收回调
static LoraWanRcv_s LoraWanRcv;	
static void (*lorawan_rcv_cb)(uint8_t flag);  //外部回调函数,后续考虑将其修改为仅仅作为一个提示，数据使用其他函数来读取
static void mcps_confirm( McpsConfirm_t *McpsConfirm )
{
    //数据接收窗口超时将会回调
	  lorawan_rcv_cb(1);
}
static void mcps_indication( McpsIndication_t *McpsIndication )
{
	  uint8_t flag = 0;
	
    if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }
    switch( McpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            //LOG_D(" MCPS_UNCONFIRMED");
					  flag = 2;
            break;
        }
        case MCPS_CONFIRMED:
        {
            //LOG_D(" MCPS_CONFIRMED");
					  flag = 3;
            break;
        }
        case MCPS_PROPRIETARY:
        {
            //LOG_D(" MCPS_PROPRIETARY");
					  flag = 4;
            break;
        }
        case MCPS_MULTICAST:
        {
            //LOG_D(" MCPS_MULTICAST");
					  flag = 5;
            break;
        }
        default:
            break;
    }

    if( McpsIndication->RxData == true )
    {
			 LoraWanRcv.BufferSize = McpsIndication->BufferSize;
			 LoraWanRcv.Port = McpsIndication->Port;
			 LoraWanRcv.Rssi = McpsIndication->Rssi;
			 LoraWanRcv.Snr = McpsIndication->Snr;
			 LoraWanRcv.Buffer = McpsIndication->Buffer;
			 flag = 0;
		//接收成功，可以进行数据处理
//		LOG_I("RecvData<<-[ Rssi:%d, Snr:%d, Port:%d, Size:%d]",McpsIndication->Rssi, McpsIndication->Snr, McpsIndication->Port, McpsIndication->BufferSize);
//		ulog_hexdump("lora_t",16,McpsIndication->Buffer,McpsIndication->BufferSize);
		//rt_kprintf( "\r\nRecvData<<-[ Rssi:%d, Snr:%d, Port:%d, Size:%d], Data:[",McpsIndication->Rssi, McpsIndication->Snr, McpsIndication->Port, McpsIndication->BufferSize);
//		for( uint8_t i = 0; i < McpsIndication->BufferSize; i++ )
//		{
//			rt_kprintf( " %02X", McpsIndication->Buffer[i] );
//		}
//		rt_kprintf( " ]\r\n" );
    }
		if(lorawan_rcv_cb != NULL)
		{ 
				lorawan_rcv_cb(flag);
		}
		
}

LoraWanRcv_s *lorawan_rcv_get(void)
{
   return (&LoraWanRcv);
}

//入网回调
static void (*lorawan_join_cb)(uint8_t statue); //返回入网状态
//static void (*lorawan_join_save_cb(void)); //信息存储回调
static void mlme_confirm( MlmeConfirm_t *MlmeConfirm )
{
    switch( MlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                //LOG_D( "Join Success!" );
								if(info.JoinMode == JOIN_MODE_MIXTURE_OTAA)//如果使用混合入网方式
								{
								    //取出NETSKEY  APPSKEY ADDR
									  //读取ABP的信息
								    MibRequestConfirm_t mibReq;

//								    mibReq.Type = MIB_NET_ID;
//							      LoRaMacMibGetRequestConfirm(&mibReq);
//							      JoinInfo.LorawanNetwordId = mibReq.Param.NetID;

										mibReq.Type = MIB_DEV_ADDR;
										LoRaMacMibGetRequestConfirm( &mibReq );
										info.Addr=mibReq.Param.DevAddr;
   
							
										mibReq.Type = MIB_NWK_SKEY;
										LoRaMacMibGetRequestConfirm( &mibReq );
										for(int i=0;i<16;i++)
										{
											info.NetSKey[i] = mibReq.Param.NwkSKey[i];
											//rt_kprintf(" %02X",JoinInfo.NwkSKey[i]);
										}
						
										mibReq.Type = MIB_APP_SKEY;
								    LoRaMacMibGetRequestConfirm( &mibReq );
							      for(int i=0;i<16;i++)
							      {
								     info.AppSKey[i] = mibReq.Param.AppSKey[i];
									   //rt_kprintf(" %02X",JoinInfo.AppSKey[i]);
								    }
										info.JoinMode = JOIN_MODE_MIXTURE_ABP;
										lorawan_join_cb(0xff);
								}
								else
								{
										if(lorawan_join_cb !=NULL)
										{
											lorawan_join_cb(0);
										}									
								}				
            }
            else
            {
                //LOG_D( "Join failed, go to sleep" );
                if(lorawan_join_cb !=NULL)
                {
                   lorawan_join_cb(1);
                }
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            break;
        }
        default:
            break;
    }
}

//后续可以外部传入函数指针来返回电压
uint8_t lorawan_get_power( void )
{
    return 255;
}

//外部修改了频率后需要
void lorawan_set_config(char *key)
{
    MibRequestConfirm_t mibReq;

	  if(strcmp(key,"channel")==0)
		{			
			//该版本实际使用的是默认通道
			rt_kprintf("test--lorawan config channel\n");
			mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
			mibReq.Param.ChannelsDefaultMask = cfg.ChannelsMask;
			LoRaMacMibSetRequestConfirm( &mibReq );

			mibReq.Type = MIB_CHANNELS_MASK;
			mibReq.Param.ChannelsMask = cfg.ChannelsMask;
			LoRaMacMibSetRequestConfirm( &mibReq );
		}
		else if(strcmp(key,"class")==0)
		{
			  rt_kprintf("test--lorawan config class\n");
		    mibReq.Type = MIB_DEVICE_CLASS;
				mibReq.Param.Class = cfg.Class;
				LoRaMacMibSetRequestConfirm( &mibReq );
		}
		else
		{}
}

//配置默认参数
void lorawan_config()
{
   	MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = cfg.ChannelsMask;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = cfg.ChannelsMask;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = cfg.Adr;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = true;
    LoRaMacMibSetRequestConfirm( &mibReq );
 
    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class = cfg.Class;
    LoRaMacMibSetRequestConfirm( &mibReq );

    //设置通道上的重复次数
    mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = 1;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = cfg.DataRate;  //DR_0
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = cfg.DataRate;
    LoRaMacMibSetRequestConfirm( &mibReq );


    mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
    mibReq.Param.ChannelsDefaultTxPower = cfg.power;//TX_POWER_14_DBM
    LoRaMacMibSetRequestConfirm( &mibReq );
    
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = cfg.power;
    LoRaMacMibSetRequestConfirm( &mibReq );

}

//仅仅初始化协议栈，未进行硬件初始化
static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
int lorawan_init()
{
    int sta=0;
    LoRaMacPrimitives.MacMcpsConfirm = mcps_confirm;
    LoRaMacPrimitives.MacMcpsIndication = mcps_indication;
    LoRaMacPrimitives.MacMlmeConfirm = mlme_confirm;
    LoRaMacCallbacks.GetBatteryLevel = lorawan_get_power;
    sta=LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );
	  //绑定接收超时回调
	  //lorawan_register_send_ok_cb();
    lorawan_config();
    return sta;
}

int lorawan_deinit()
{
   lorawan_core_thread_detele();
	 return 0;
}

void lorawan_join(uint8_t times,void (*fun_cb)(uint8_t statue))
{
    lorawan_join_cb = fun_cb;
    switch (info.JoinMode)
    {
        case JOIN_MODE_OTAA:
        {
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = info.DevEui;
            mlmeReq.Req.Join.AppEui = info.AppEui;
            mlmeReq.Req.Join.AppKey = info.AppKey;
            mlmeReq.Req.Join.NbTrials = times;
            LoRaMacMlmeRequest( &mlmeReq );
        }break;
        case JOIN_MODE_ABP:
        {
            MibRequestConfirm_t mibReq;

						mibReq.Type = MIB_NET_ID;
						mibReq.Param.NetID = 0X010203;
						LoRaMacMibSetRequestConfirm( &mibReq );  
					
            mibReq.Type = MIB_DEV_ADDR;
            mibReq.Param.DevAddr = info.Addr;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_NWK_SKEY;
            mibReq.Param.NwkSKey = info.NetSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_APP_SKEY;
            mibReq.Param.AppSKey = info.AppSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_NETWORK_JOINED;
            mibReq.Param.IsNetworkJoined = true;
            LoRaMacMibSetRequestConfirm( &mibReq );

            if(lorawan_join_cb !=NULL)
            {
                lorawan_join_cb(0);
            }
            
        }break;
        case JOIN_MODE_MIXTURE_OTAA: //MIXTURE模式入网必须外部将info保存到flash，否则等效于OTAA
        {
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = info.DevEui;
            mlmeReq.Req.Join.AppEui = info.AppEui;
            mlmeReq.Req.Join.AppKey = info.AppKey;
            mlmeReq.Req.Join.NbTrials = times;
            LoRaMacMlmeRequest( &mlmeReq );
					  
					  
        }break;
        case JOIN_MODE_MIXTURE_ABP:
        {
            MibRequestConfirm_t mibReq;

						mibReq.Type = MIB_NET_ID;
						mibReq.Param.NetID = 0X010203;
						LoRaMacMibSetRequestConfirm( &mibReq );  
					
            mibReq.Type = MIB_DEV_ADDR;
            mibReq.Param.DevAddr = info.Addr;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_NWK_SKEY;
            mibReq.Param.NwkSKey = info.NetSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_APP_SKEY;
            mibReq.Param.AppSKey = info.AppSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_NETWORK_JOINED;
            mibReq.Param.IsNetworkJoined = true;
            LoRaMacMibSetRequestConfirm( &mibReq );
	
						lorawan_statue_write("UpCounter",&(statue.UpCounter));//由于otaa入网后包号必须累加						
						lorawan_counter_wrtie(statue.UpCounter);
						
            if(lorawan_join_cb !=NULL)
            {
                lorawan_join_cb(0);
            }
        }break;

        default:
            break;
    }
}

//数据发送，参数包含模式但不包含端口
//mode: 0: MCPS_UNCONFIRMED   1: MCPS_CONFIRMED
int lorawan_send(uint8_t *buffer,uint8_t size,uint8_t mode)
{
        McpsReq_t mcpsReq;
        MibRequestConfirm_t mibReq;
        LoRaMacTxInfo_t txInfo;
        mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
        LoRaMacMibGetRequestConfirm( &mibReq );
	
	  if( LoRaMacQueryTxPossible( size, &txInfo ) != LORAMAC_STATUS_OK )
    {
        if( txInfo.MaxPossiblePayload < txInfo.CurrentPayloadSize )
        {
            //LOG_I( "Flush MAC commands!" );
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fBuffer = NULL;
            mcpsReq.Req.Confirmed.fBufferSize = 0;
            mcpsReq.Req.Confirmed.NbTrials = 1;
            mcpsReq.Req.Confirmed.Datarate = mibReq.Param.ChannelsDefaultDatarate;
            
            if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
            {
							 return 3;
            }
						else
						{
						  return 4;
						}
        }
        else
        {
            return 1;
        }
       }
	   else
	   {
            if( mode == 0 )
            {
                mcpsReq.Type = MCPS_UNCONFIRMED;
                mcpsReq.Req.Unconfirmed.fPort = 9;
                mcpsReq.Req.Unconfirmed.fBuffer = buffer;
                mcpsReq.Req.Unconfirmed.fBufferSize = size;
                mcpsReq.Req.Unconfirmed.Datarate = mibReq.Param.ChannelsDefaultDatarate;
            }
            else if(mode == 1)
            {
                mcpsReq.Type = MCPS_CONFIRMED;
                mcpsReq.Req.Confirmed.fPort = 9;
                mcpsReq.Req.Confirmed.fBuffer = buffer;
                mcpsReq.Req.Confirmed.fBufferSize = size;
                mcpsReq.Req.Confirmed.NbTrials = 3;// NbTrials;
                mcpsReq.Req.Confirmed.Datarate = mibReq.Param.ChannelsDefaultDatarate;
            }
            else{}
				
            if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
            {
                return 0;
            }
            else
            {
                return 2;
            }
            
	    }
}

void radio_sleep(void)
{
    lorawan_sleep();
}
MSH_CMD_EXPORT(radio_sleep,sleep);

//数据接收，注册回调函数
void lorawan_rcv_register_cb(void (*rcv_cb)(uint8_t flag) )
{
     lorawan_rcv_cb = rcv_cb;
}
//打开关闭接收窗口
void radio_rx_set(bool flag)
{
		LoRaMacTestRxWindowsOn(flag);
}
