

#include "board.h"
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "netarg.h"
#include "api.h"
#include "serial_board.h"
#include "six_box_pto.h"

#define APP_DATA_MAX_SIZE                           256
#define APP_TX_DUTYCYCLE_RND                        1000 //基础延时上的随机误差
#define DEFAULT_UPDATA_CYCLE                        60000*2
#define DEFAULT_JOIN_CYCLE                          15000

enum eDevicState
{
	  DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

static uint8_t AppPort=6;
static uint8_t AppDataSize;
static uint8_t AppData[APP_DATA_MAX_SIZE];
//static uint8_t PayloadLength;
static uint8_t NbTrials;

static uint8_t DevEui[8]   = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[8]   = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[16]  = LORAWAN_APPLICATION_KEY;
static uint8_t NwkSKey[16] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[16] = LORAWAN_APPSKEY;

static bool IsTxConfirmed = false;
static bool IsTxEmptyPacket = false;
static bool IsRxWindowsEnabled = true;
static bool TxPacketDone = false;
static bool TxNextPacket = false;

//static uint32_t Tx_Total = 0;

static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static TimerEvent_t TxNextPacketTimer;
static TimerEvent_t JoinTimer;
static uint8_t JoinFlag=1;

static int PrepareTxFrame()
{	
//  	SerialBoardSendData("123",strlen("123"));
	 CallBack_TxPayloadFrame(&AppPort, AppData, &AppDataSize, &IsTxConfirmed);
	  //填充数据 AppData AppDataSize
	 return 0;
}

static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    MibRequestConfirm_t mibReq;
    LoRaMacTxInfo_t txInfo;
    
    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    LoRaMacMibGetRequestConfirm( &mibReq );
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        if( txInfo.MaxPossiblePayload < txInfo.CurrentPayloadSize )
        {
            DEBUG_NORMAL( "\r\n Flush MAC commands! \r\n" );
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fBuffer = NULL;
            mcpsReq.Req.Confirmed.fBufferSize = 0;
            mcpsReq.Req.Confirmed.NbTrials = 1;
            mcpsReq.Req.Confirmed.Datarate = mibReq.Param.ChannelsDefaultDatarate;
            
            if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
            {
                BoardDisableIrq();
                if( IsTxEmptyPacket == true )
                {
                    IsTxEmptyPacket = false;
                }
                TxNextPacket = true;
                BoardEnableIrq();
            }
        }
        else
        {
            DEBUG_NORMAL( "\r\n Payload is too long! \r\n" );
            BoardDisableIrq();
            TxPacketDone = true;
            BoardEnableIrq();
        }
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = mibReq.Param.ChannelsDefaultDatarate;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = NbTrials;
            mcpsReq.Req.Confirmed.Datarate = mibReq.Param.ChannelsDefaultDatarate;
        }
        
        if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
        {
            BoardDisableIrq();
            if( IsTxEmptyPacket == true )
            {
                IsTxEmptyPacket = false;
            }
            BoardEnableIrq();
            return true;
        }
    }
    
    return false;
}

static void TxNextPacketCheck( void )
{
    if( (TxPacketDone == true) && (TxNextPacket == true) )
    {
        TxNextPacket = false;
        TxPacketDone = false;
        DeviceState = DEVICE_STATE_SEND;
    }
}

static void OnTxNextPacketTimerEvent( void )
{
    TxNextPacket = true;
   TxNextPacketCheck();
}

static void OnJoinTimerEvent( void )
{
	DeviceState = DEVICE_STATE_JOIN;
   JoinFlag=1;
}

//txdone callback 
void TxEmptyPacket( void )
{
    TimerStop( &TxNextPacketTimer );
    
    TxNextPacket = true;
    TxNextPacketCheck();
    IsTxEmptyPacket = true;
}
//txdone callback
static void McpsConfirm( McpsConfirm_t *McpsConfirm )
{
    if( McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( McpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbRetries
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    
    TxPacketDone = true;
    TxNextPacketCheck();
}
//RxDone callback
static void McpsIndication( McpsIndication_t *McpsIndication )
{
    if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( McpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            DEBUG_INFO(" MCPS_UNCONFIRMED \r\n");
            break;
        }
        case MCPS_CONFIRMED:
        {
            DEBUG_INFO(" MCPS_CONFIRMED \r\n");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            DEBUG_INFO(" MCPS_PROPRIETARY \r\n");
            break;
        }
        case MCPS_MULTICAST:
        {
            DEBUG_INFO(" MCPS_MULTICAST \r\n");
            break;
        }
        default:
            break;
    }
    
    if( (McpsIndication->FramePending == true) && (DeviceState != DEVICE_STATE_SEND) )
    {
        TimerStop( &TxNextPacketTimer );
        IsTxEmptyPacket = true;
        DeviceState = DEVICE_STATE_SEND;
    }

    if( McpsIndication->RxData == true )
    {
        CallBack_RxPayloadFrame(McpsIndication);
    }
}

static void MlmeConfirm( MlmeConfirm_t *MlmeConfirm )
{
    switch( MlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                if( IsRxWindowsEnabled == false )
                {
                    LoRaMacTestRxWindowsOn( false );
                }
                DeviceState = DEVICE_STATE_SEND;
                DEBUG_NORMAL( " Join Success! \r\n" );
            }
            else
            {
								TimerStart( &JoinTimer );
                DEBUG_NORMAL( " Join failed, go to sleep \r\n" );
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            // Check DemodMargin
            // Check NbGateways
            break;
        }
        default:
            break;
    }
}


void LoRaConfig(void)
{
	  MibRequestConfirm_t mibReq;
    uint16_t ChannelsDefaultMask[6] = {
			0x03FC,
			0x0000,
			0x0000,
			0x0000,
			0x0000,
			0x0000,
		};

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = ChannelsDefaultMask;
    LoRaMacMibSetRequestConfirm( &mibReq );
    
    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = ChannelsDefaultMask;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
    LoRaMacMibSetRequestConfirm( &mibReq );
    
    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class = LORAWAN_DEVICE_CLASS;
    LoRaMacMibSetRequestConfirm( &mibReq );
		
		mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = 1;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = LORAMAC_DEFAULT_DATARATE;
    LoRaMacMibSetRequestConfirm( &mibReq );
    
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = LORAMAC_DEFAULT_DATARATE;
    LoRaMacMibSetRequestConfirm( &mibReq );
    
    mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
    mibReq.Param.ChannelsDefaultTxPower = LORAMAC_DEFAULT_TX_POWER;
    LoRaMacMibSetRequestConfirm( &mibReq );
    
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER;
    LoRaMacMibSetRequestConfirm( &mibReq );
}

void LoRaJoin(bool otaa, uint8_t JoinTrials)
{
	if( otaa == true )
	{
		MlmeReq_t mlmeReq;

		DEBUG_NORMAL( "\r\n OTAA joining \r\n Time:%.1fs \r\n", TimerGetCurrentTime() / 1000.0 );
		LedRadioBeforeTxJionRequestShow();

		mlmeReq.Type = MLME_JOIN;
		mlmeReq.Req.Join.DevEui = DevEui;
		mlmeReq.Req.Join.AppEui = AppEui;
		mlmeReq.Req.Join.AppKey = AppKey;
		mlmeReq.Req.Join.NbTrials = JoinTrials;
		LoRaMacMlmeRequest( &mlmeReq );
	}
	else
	{
		MibRequestConfirm_t mibReq;

		mibReq.Type = MIB_NET_ID;
		mibReq.Param.NetID = LORAWAN_NETWORK_ID;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_DEV_ADDR;
		mibReq.Param.DevAddr = LORAWAN_DEVICE_ADDRESS;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_NWK_SKEY;
		mibReq.Param.NwkSKey = NwkSKey;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_APP_SKEY;
		mibReq.Param.AppSKey = AppSKey;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_NETWORK_JOINED;
		mibReq.Param.IsNetworkJoined = true;
		LoRaMacMibSetRequestConfirm( &mibReq );

		DEBUG_NORMAL( "\r\n ABP join \r\n" );
	}
}

int main( void )
{
    uint8_t  JoinTrials;
    uint32_t TxDutyCycleTime;
    uint32_t AppTxInterval;

    JoinTrials = 48;
    AppTxInterval = DEFAULT_UPDATA_CYCLE;
    
    IsTxConfirmed = false;
    NbTrials = 8;
	
    BoardInitMcu();
    BoardInitPeriph();
	  SB_init();
    DeviceState = DEVICE_STATE_JOIN;

	  //lorawan初始化
		LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
		LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
		LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
		LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
		LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );
		LoRaConfig();
		
		TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent, "tx next packet" );
		TimerInit( &JoinTimer, OnJoinTimerEvent, "Join timer" );
		
		TimerSetValue( &JoinTimer, DEFAULT_JOIN_CYCLE ); //10s DEFAULT_JOIN_CYCLE

    while( 1 )
    {
        switch( DeviceState )
        {					
            case DEVICE_STATE_JOIN:
							if(JoinFlag)
							{
							   LoRaJoin(OVER_THE_AIR_ACTIVATION,JoinTrials);
								 JoinFlag=0;
							}
							
							if (OVER_THE_AIR_ACTIVATION==true)
							{
								DeviceState = DEVICE_STATE_SLEEP;
							} else {
								DeviceState = DEVICE_STATE_SEND;
							}
                
                break;
                
            case DEVICE_STATE_SEND:
                if( PrepareTxFrame() == 0 )
                {
                    DEBUG_NORMAL( "\r\n Time:%.1fs \r\n", TimerGetCurrentTime() / 1000.0 );
                    SendFrame();
                    TxDutyCycleTime = AppTxInterval + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                    DeviceState = DEVICE_STATE_CYCLE;
                }
                break;
                
            case DEVICE_STATE_CYCLE:
                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                DeviceState = DEVICE_STATE_SLEEP;
                break;
            
            case DEVICE_STATE_SLEEP:
                break;
            
            default:
                break;
        }
        
        //McuEnterLowPowerMode();
    }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

//尝试获取一次传感器数据，成功返回0
int once_get_data(uint8_t *data)
{
	uint8_t frame[255];
	uint8_t RcvData[255];
	uint8_t cmd=1;
	uint16_t frameSize=0;
  //获取传感器数据
	SB_prepare_frame(SB_TYPE_GET,&cmd,1,frame,&frameSize);
	SerialBoardSendData(frame,frameSize);
	//延时2秒等待数据获取
	DelayMs( 2000 );
  if(isRcvData()) //如果接收到数据
	{
		     int size = GetSerialBoardReciveDataSize();
			   DEBUG_NORMAL("size %d\n",size);
			   GetSerialBoardReciveData(RcvData,6+9+4+2);
			   ClearSerialBoardBuffer();
		
		     uint8_t temp = SB_disassemble(RcvData,6+9+4+2,data,&frameSize) ;
		     if(!temp)
					 return 0;
				 else
					 return 1;     
	}
	else
		return 1;
	
}

int CallBack_TxPayloadFrame(uint8_t* Port, uint8_t *Buf, uint8_t* Len, bool *Confirmed)
{
	 int i=5;
	 *Port = 3;
	 *Len = 9+4;
	 *Confirmed = false;
	 
	 uint8_t Data[9+4] = {1,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	 uint8_t getReturn=0xff;
//	 getReturn = once_get_data(Data);
//	 DEBUG_NORMAL("********* get sensor data:%d",getReturn);
//	 DelayMs( 1000 );
//	 if(getReturn != 0 ) //读取失败
//	 {
//	     getReturn = once_get_data(Data);
//		   DEBUG_NORMAL("********* again  get sensor data:%d",getReturn);
//	 }
	 while(i--)
	 {
	     getReturn = once_get_data(Data);
		   if(getReturn != 0 ) //读取失败
			 {
				   DEBUG_NORMAL("********* read fail:%d again try%d\n:",getReturn,i);
				   DelayMs( 2000 );
			 }
			 else
			 {
				  DEBUG_NORMAL("********* read succeed !!");
			    break;
			 }
	 }
   
	 memcpy(Buf,Data,9+4);
	
	//封装数据
	return 0;
}

void CallBack_RxPayloadFrame(McpsIndication_t *McpsIndication) 
{
	DEBUG_NORMAL( "\r\nRecvData<<-[ Rssi:%d, Snr:%d, Port:%d, Size:%d], Data:[",McpsIndication->Rssi, McpsIndication->Snr, McpsIndication->Port, McpsIndication->BufferSize);
   for( uint8_t i = 0; i < McpsIndication->BufferSize; i++ )
   {
      DEBUG_NORMAL( " %02X", McpsIndication->Buffer[i] );
   }
   DEBUG_NORMAL( " ]\r\n" );
}

//CL3T78A35158DB44