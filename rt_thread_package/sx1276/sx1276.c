#include <rtthread.h>
#include "rtdevice.h"
#include "rthw.h"  //DelayMs

#include "radio.h"
#include "sx1276.h"

#include "math.h"
#include "string.h"

#include "timeServer.h"

//外部实现函数，并且在外部初始化完成，也就是使用此驱动的条件
//SPI初始化 | GPIO初始化 | 中断函数配置 | 定时器 | 填充基本信息
/*---------------------------------------------------------------------------------*/
//示例:

// 1 GPIO初始化，主要完成CS/RST/DIO等的初始化
//#define SX1276_DIO_RST    11
//#define SX1276_DIO_NSS    10
//#define SX1276_DIO_0      13
//#define SX1276_DIO_1      0

//void SX1276IoInit(void)
//{
//    rt_pin_mode(SX1276_DIO_0, PIN_MODE_INPUT_PULLDOWN);
//    rt_pin_mode(SX1276_DIO_1, PIN_MODE_INPUT_PULLDOWN);
//    rt_pin_mode(SX1276_DIO_NSS, PIN_MODE_OUTPUT);
//}
//void SX1276IoDeInit( void )
//{}
// 2 中断函数配置，函数给定，这里主要匹配对于引脚
//void SX1276IoIrqInit(void)
//{
//    rt_pin_attach_irq(SX1276_DIO_0, PIN_IRQ_MODE_RISING, SX1276OnDio0Irq_shell, RT_NULL);
//  	rt_pin_irq_enable(SX1276_DIO_0, PIN_IRQ_ENABLE);
//    rt_pin_attach_irq(SX1276_DIO_1, PIN_IRQ_MODE_RISING, SX1276OnDio1Irq_shell, RT_NULL);
//  	rt_pin_irq_enable(SX1276_DIO_1, PIN_IRQ_ENABLE);
//}
// 3 spi初始化，用外部选择使用哪个spi以及CS引脚
//static struct rt_spi_device spi_dev_sx1276;
//uint8_t cs;
//void SX1276SpiInit(void)
//{
//    rt_err_t res;
//    cs = SX1276_DIO_NSS;
//    rt_pin_mode(cs, PIN_MODE_OUTPUT);
//    res = rt_spi_bus_attach_device(&spi_dev_sx1276, "sx1276_spi", "spi1", (void*)&cs);
//    if (res != RT_EOK)
//    {
//        rt_kprintf("spi device init :error\r\n");
//    }
//    else
//    {
//    //配置
//    struct rt_spi_configuration cfg;
//    cfg.data_width = 8;
//    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
//    cfg.max_hz = 20000000;//20 * 1000 *1000; /* 20M,SPI max 42MHz,ssd1351 4-wire spi */
//    rt_spi_configure(&spi_dev_sx1276, &cfg);
//    rt_kprintf("spi device init :OK\n");
//    }
//}
//4 定时器，主要用来对超时做处理，外部实现对应函数
//void Sx1276TimerInit( TimerEvent_t *obj, void ( *callback )( void* parameter ) )
//{
//	
//	 obj->timeHander  =      rt_timer_create(obj->name,  /* 定时器名字是 timer1 */
//                                           callback, /* 超时时回调的处理函数 */
//                                           RT_NULL,  /* 超时函数的入口参数 */
//                                           obj->ReloadValue,       /* 定时长度，以OS Tick为单位，即10个OS Tick */
//                                           RT_TIMER_FLAG_ONE_SHOT); /* 单次定时器*/
//	 obj->Callback = callback;
//	
//}
//void Sx1276TimerStart( TimerEvent_t *obj )
//{
//    if (obj->timeHander != RT_NULL) 
//			rt_timer_start(obj->timeHander);
//}
//void Sx1276TimerStop( TimerEvent_t *obj ) 
//{
//	 if (obj->timeHander != RT_NULL) 
//			rt_timer_stop(obj->timeHander);
//} 
//void Sx1276TimerReset( TimerEvent_t *obj )
//{
//    Sx1276TimerStop(obj);
//	 Sx1276TimerStart(obj);
//}
//void Sx1276TimerSetValue( TimerEvent_t *obj, uint32_t value )
//{
//	//参数是ms，传入tick，相差10倍
//	 value = value/10;
//   rt_timer_control(obj->timeHander, RT_TIMER_CTRL_SET_TIME, (void*)&value);
//}
//TimerTime_t Sx1276TimerGetCurrentTime( void )
//{
//	rt_tick_t temp;
//	temp = rt_tick_get();//得到TTCK 
//	temp*=10;
//    return temp; //返回ms
//}
//TimerTime_t Sx1276TimerGetElapsedTime( TimerTime_t past )
//{
//	rt_tick_t nowInTicks = rt_tick_get();
//	rt_tick_t pastInTicks= rt_tick_from_millisecond(past);
//	return ((nowInTicks - pastInTicks)*10);
//}
//void Sx1276DelayMs( uint32_t ms )
//{
//	uint32_t i=0;
//   for(i=1;i<=ms;i++)
//		  rt_hw_us_delay(1000);
//}
/*----------------------------------------------------------------------------------------------*/


/**************************************************************************************************/
#ifdef RT_USING_ULOG

#define LOG_TAG              "sx1276"
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



void SX1276WriteFifo( uint8_t *buffer, uint8_t size );
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );
void SX1276SetTx( uint32_t timeout );
void SX1276SetRfTxPower( int8_t power );
uint8_t SX1276GetPaSelect( uint32_t channel );
static void RxChainCalibration( void );
void SX1276OnTimeoutIrq_shell(void* parameter);
void SX1276OnTimeoutIrq(void  );
void SX1267_thread_entry(void *parameter);
void SX1276Reset( void );
void SX1276SetOpMode( uint8_t opMode );
void SX1276OnDio0Irq( void );
void SX1276OnDio1Irq( void );



SX1276_t SX1276;
void SX1276SetHardwareInterface(uint32_t cs,uint32_t rst,struct rt_spi_device *spi)
{
  SX1276.pin_cs = cs;
  SX1276.pin_rst = rst;
  SX1276.spi_hander = spi;
}

//sx127x驱动部分
RadioRegisters_t RadioRegsInit[] = {                                                 \
                                        { MODEM_FSK , REG_LNA                , 0x23 },\
                                        { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
                                        { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
                                        { MODEM_FSK , REG_AFCFEI             , 0x01 },\
                                        { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
                                        { MODEM_FSK , REG_OSC                , 0x07 },\
                                        { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
                                        { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
                                        { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
                                        { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
                                        { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
                                        { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
                                        { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
                                        { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
                                        { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
                                        { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
                                    } ;



																		
#define SX1276_EVENT_ALL         0xFFFFFFFF
#define SX1276_EVENT_D0         (uint32_t)1<<0  
#define SX1276_EVENT_D1         (uint32_t)1<<1  
#define SX1276_EVENT_TIMEOUT    (uint32_t)1<<2
#define SX1276_EVENT_RX_ERROR   (uint32_t)1<<3
#define SX1276_EVENT_RX_DONE    (uint32_t)1<<4
#define SX1276_EVENT_TX_DONE    (uint32_t)1<<5
#define SX1276_EVENT_TX_TIMEOUT	(uint32_t)1<<6
#define SX1276_EVENT_RX_TIMEOUT	(uint32_t)1<<7																		

static struct rt_event sx1276_event;

static RadioEvents_t *RadioEvents;//外部事件回调函数指针

Sx1276TimerEvent_t TxTimeoutTimer={"txtimer",RT_NULL,100,RT_NULL};
Sx1276TimerEvent_t RxTimeoutTimer={"rxtimer",RT_NULL,100,RT_NULL};
Sx1276TimerEvent_t RxTimeoutSyncWord={"rxSwdtimer",RT_NULL,100,RT_NULL};

#define RX_BUFFER_SIZE                              256
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];
static uint8_t EventDataBuffer[RX_BUFFER_SIZE];
static uint8_t EventDataSize;
static int16_t EventDataRssi;
static int8_t EventDataSnr;

ALIGN(RT_ALIGN_SIZE)
static struct rt_thread sx1276Event_hander;
static char sx1276_thread_stack[1024];

uint32_t SX1276Init( RadioEvents_t *events  )
{
 
   rt_event_init(&sx1276_event, "sx1276_event", RT_IPC_FLAG_FIFO);	 
   RadioEvents = events;


   //外部完成上列准备工作，则无需调用下列函数
//   SX1276IoInit();
//   SX1276SpiInit();
//   SX1276IoIrqInit();
   
   Sx1276TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq_shell );
   Sx1276TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq_shell );
   Sx1276TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq_shell );
//SX1276SetHardwareInterface(SX1276_DIO_NSS,SX1276_DIO_RST,spi_dev_sx1276);//该函数在外部调用
  
    SX1276Reset( );
    Sx1276DelayMs( 10 );

    RxChainCalibration( );
    SX1276SetOpMode( RF_OPMODE_SLEEP );

    uint8_t i;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
 
    SX1276SetModem( MODEM_FSK );
    SX1276.Settings.State = RF_IDLE;
				
//    rt_thread_t sx1276Event_hander;
//    sx1276Event_hander = rt_thread_create("sx1276"
//                                        ,SX1267_thread_entry
//                                        ,RT_NULL
//                                        ,1024
//                                        ,2
//                                        ,10
//                                        );

//    if (sx1276Event_hander != RT_NULL)
//    {
//        rt_thread_startup(sx1276Event_hander);
//        LOG_D("sx1276_event_hander\n");
//    }

	  //静态
	  rt_thread_init(&sx1276Event_hander,
                   "sx1276",
                   SX1267_thread_entry,
                   RT_NULL,
                   &sx1276_thread_stack[0],
                   sizeof(sx1276_thread_stack),
                   2, 10);							
	  rt_thread_startup(&sx1276Event_hander);
		rt_kprintf("##-sx1278 start\n");		
    return 0;
}

void SX1267_thread_entry(void *parameter)
{
	rt_uint32_t event;
	rt_event_init(&sx1276_event, "sx1276_event", RT_IPC_FLAG_FIFO);	
	while(1)
	{
	      rt_event_recv(&sx1276_event,SX1276_EVENT_ALL ,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &event);
		    //下列事件处理函数在外部实现
		     if(event & SX1276_EVENT_D0)
          {
              SX1276OnDio0Irq();
          }
          if(event & SX1276_EVENT_D1)
          {
						  SX1276OnDio1Irq();
					}
          if(event & SX1276_EVENT_TIMEOUT) 
          {
					     SX1276OnTimeoutIrq();
					}
          if (event & SX1276_EVENT_RX_ERROR)
          {}
          if(event & SX1276_EVENT_RX_DONE)
          {
					    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
              {
                RadioEvents->RxDone( EventDataBuffer,EventDataSize,EventDataRssi, EventDataSnr );
              }
		      }
					if (event & SX1276_EVENT_TX_DONE) 
					{
					   	if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
             {
                RadioEvents->TxDone( );
             }    
					}
					if( event & SX1276_EVENT_TX_TIMEOUT) 
					{
					   if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
             {
              RadioEvents->TxTimeout( );					
             }
					}
					if( event & SX1276_EVENT_RX_TIMEOUT) 
          {
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
            {
               RadioEvents->RxTimeout( );
            }
				
			  	}    
    }
		  
}


//发送相关函数

void SX1276ReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    rt_pin_write(SX1276.pin_cs, 0);
    Sx1276DelayMs( 1 );
    addr=addr & 0x7f;
    rt_spi_send_then_recv(SX1276.spi_hander,&addr,1,buffer,size);
    rt_pin_write(SX1276.pin_cs, 1);
    Sx1276DelayMs( 1 );
}

void SX1276WriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    rt_pin_write(SX1276.pin_cs, 0);
    Sx1276DelayMs( 1 );
    addr= addr|0x80;
    rt_spi_send_then_send(SX1276.spi_hander,&addr, 1,buffer,size);
    rt_pin_write(SX1276.pin_cs, 1);
    Sx1276DelayMs( 1 );
}

uint8_t SX1276Read( uint16_t addr )
{
    uint8_t data;
    SX1276ReadBuffer( addr, &data, 1 );
    return data;
}

void SX1276Write( uint16_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}


void SX1276Reset( void )
{
    rt_pin_mode(SX1276.pin_rst, PIN_MODE_OUTPUT);
	  rt_pin_write(SX1276.pin_rst, 0);
   
    // Wait 1 ms
    Sx1276DelayMs( 1 );
    // Configure RESET as input
	  rt_pin_mode(SX1276.pin_rst, PIN_MODE_INPUT_PULLUP);
    // Wait 6 ms
    Sx1276DelayMs( 6 );
}

static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276Read( REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )SX1276Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276Write( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );
}

void SX1276SetChannel( uint32_t freq )
{
    SX1276.Settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}



void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX1276SetModem( modem );

    SX1276SetRfTxPower( power );

    switch( modem )
    {
      case MODEM_LORA:
        {
            SX1276.Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            SX1276Write( REG_LR_MODEMCONFIG1,
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276Write( REG_LR_MODEMCONFIG2,
                         ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            SX1276Write( REG_LR_MODEMCONFIG3,
                         ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}


void SX1276Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( SX1276.Settings.Modem )
    {

    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            SX1276.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1276Write( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX1276Write( REG_LR_FIFOTXBASEADDR, 0 );
            SX1276Write( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX1276Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1276SetStby( );
                Sx1276DelayMs( 1 );
            }
            // Write payload buffer
            SX1276WriteFifo( buffer, size );
            txTimeout = SX1276.Settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1276SetTx( txTimeout );
}

void SX1276SetStby( void )
{
    Sx1276TimerStop( &RxTimeoutTimer );
    Sx1276TimerStop( &TxTimeoutTimer );
    SX1276SetOpMode( RF_OPMODE_STANDBY );
    SX1276.Settings.State = RF_IDLE;
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

void SX1276SetTx( uint32_t timeout )
{
    Sx1276TimerSetValue( &TxTimeoutTimer, timeout );

    switch( SX1276.Settings.Modem )
    {

    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }
    SX1276.Settings.State = RF_TX_RUNNING;
    Sx1276TimerStart( &TxTimeoutTimer );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}


void SX1276OnDio0Irq( void )
{
	 volatile uint8_t irqFlags = 0;
    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING: //接收状态
        {
            switch( SX1276.Settings.Modem )
            {
                case MODEM_LORA:
                {
                    int8_t snr = 0;
                   // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );
                    irqFlags = SX1276Read( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
                        if( SX1276.Settings.LoRa.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                        }
                        Sx1276TimerStop( &RxTimeoutTimer );
                        //发送错误事件
                        rt_event_send(&sx1276_event, SX1276_EVENT_RX_ERROR);
                        break;
                    }
                    SX1276.Settings.LoRaPacketHandler.SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
                    if( SX1276.Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~SX1276.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( SX1276.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +snr;
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +snr;
                        }
                    }
                    else
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }
                    SX1276.Settings.LoRaPacketHandler.Size = SX1276Read( REG_LR_RXNBBYTES );
                    SX1276Write( REG_LR_FIFOADDRPTR, SX1276Read( REG_LR_FIFORXCURRENTADDR ) );
                    SX1276ReadFifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );

                    if( SX1276.Settings.LoRa.RxContinuous == false )
                    {
                        SX1276.Settings.State = RF_IDLE;
                    }
                    Sx1276TimerStop( &RxTimeoutTimer );
                    EventDataSize = SX1276.Settings.LoRaPacketHandler.Size;
                    EventDataRssi=  SX1276.Settings.LoRaPacketHandler.RssiValue; 
                    EventDataSnr =  SX1276.Settings.LoRaPacketHandler.SnrValue;
                    memcpy(EventDataBuffer,RxTxBuffer,EventDataSize);
                    rt_event_send(&sx1276_event, SX1276_EVENT_RX_DONE);	
                }break;
            }

        }break;

        case RF_TX_RUNNING://发送状态
        {
					  Sx1276TimerStop( &TxTimeoutTimer );
            switch( SX1276.Settings.Modem )
            {
               case MODEM_LORA:
                // Clear Irq
                 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
              default:
                SX1276.Settings.State = RF_IDLE;
							  rt_event_send(&sx1276_event, SX1276_EVENT_TX_DONE);
                              
            }
            
        }break;
    }
}






void SX1276OnDio0Irq_shell( void* parameter )
{
    rt_event_send(&sx1276_event, SX1276_EVENT_D0);
}
void SX1276OnDio1Irq_shell( void* parameter )
{
    rt_event_send(&sx1276_event, SX1276_EVENT_D1);
}
void SX1276OnTimeoutIrq_shell(void* parameter)
{
    rt_event_send(&sx1276_event, SX1276_EVENT_TIMEOUT);
}



void SX1276SetOpMode( uint8_t opMode )
{
    if( opMode == RF_OPMODE_SLEEP )
    {
      SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode ); 
      //将射频开关I/Os引脚设置为低功率模式 SX1276BoardSetAntSwLowPower( true );
    }
    else
    {
     //SX1276BoardSetAntSwLowPower( false );
     //SX1276BoardSetAntSw( opMode );
      SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
    }
}



void SX1276SetModem( RadioModems_t modem )
{

    if( ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        SX1276.Settings.Modem = MODEM_LORA;
    }
    else
    {
        SX1276.Settings.Modem = MODEM_FSK;
    }

    if( SX1276.Settings.Modem == modem )
    {
        return;
    }

    SX1276.Settings.Modem = modem;
    switch( SX1276.Settings.Modem )
    {
    default:
    case MODEM_FSK:
        SX1276SetSleep( );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetSleep( );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );
        
//        temp = SX1276Read( REG_OPMODE ) ;
//	      rt_kprintf("mode2=%02X\n",temp);    	
        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}


void SX1276SetRfTxPower( int8_t power )
{
  uint8_t paConfig = 0;
  uint8_t paDac = 0;
  
  paConfig = SX1276Read( REG_PACONFIG );
  paDac = SX1276Read( REG_PADAC );
  
  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
  paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
  
  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
  {
    if( power > 17 )
    {
      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
    }
    else
    {
      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
    }
    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
    {
      if( power < 5 )
      {
        power = 5;
      }
      if( power > 20 )
      {
        power = 20;
      }
      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
    }
    else
    {
      if( power < 2 )
      {
        power = 2;
      }
      if( power > 17 )
      {
        power = 17;
      }
      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
    }
  }
  else
  {
    if( power < -1 )
    {
      power = -1;
    }
    if( power > 14 )
    {
      power = 14;
    }
    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
  }
  SX1276Write( REG_PACONFIG, paConfig );
  SX1276Write( REG_PADAC, paDac );
}

void SX1276SetSleep( void )
{
    SX1276SetOpMode( RF_OPMODE_SLEEP );
    SX1276.Settings.State = RF_IDLE;
}
//MSH_CMD_EXPORT(SX1276SetSleep,SX1276 sleep);


#define RF_MID_BAND_THRESH                          525000000
uint8_t SX1276GetPaSelect( uint32_t channel )
{
  if( channel < RF_MID_BAND_THRESH )
  {
    return RF_PACONFIG_PASELECT_PABOOST;
  }
  else
  {
    return RF_PACONFIG_PASELECT_RFO;
  }
}

RadioState_t SX1276GetStatus( void )
{
    return SX1276.Settings.State;
}

void SX1276OnTimeoutIrq(void  )
{
    
    switch( SX1276.Settings.State )
    {
    case RF_RX_RUNNING:
				rt_event_send(&sx1276_event, SX1276_EVENT_RX_TIMEOUT);
        break;
    case RF_TX_RUNNING:
			  LOG_D("send fail\n");
			  //该状态不应该存在，如果发送则重新初始化
        SX1276Reset( );
		    Sx1276DelayMs( 10 );
        RxChainCalibration( );
        SX1276SetOpMode( RF_OPMODE_SLEEP );
        for( uint8_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            SX1276SetModem( RadioRegsInit[i].Modem );
            SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }
        SX1276SetModem( MODEM_FSK );
//        // Restore previous network type setting.
//        SX1276SetPublicNetwork( SX1276.Settings.LoRa.PublicNetwork );
        // END WORKAROUND
        SX1276.Settings.State = RF_IDLE;
			  rt_event_send(&sx1276_event, SX1276_EVENT_TX_TIMEOUT);
        break;
    default:
        break;
    }
}


void SX1276SetPublicNetwork( bool enable )
{
    SX1276SetModem( MODEM_LORA );
    SX1276.Settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
         SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

void SX1276SetRx( uint32_t timeout )
{
    bool rxContinuous = false;

	  //rt_kprintf("##########T1:%d",TimerGetCurrentTime());
	
    switch( SX1276.Settings.Modem )
    {	  
        case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( SX1276.Settings.LoRa.Bandwidth < 9 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX1276Write( REG_LR_TEST30, 0x00 );
                switch( SX1276.Settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX1276Write( REG_LR_TEST2F, 0x48 );
                    SX1276SetChannel(SX1276.Settings.Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = SX1276.Settings.LoRa.RxContinuous;

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1276Write( REG_LR_FIFORXBASEADDR, 0 );
            SX1276Write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    SX1276.Settings.State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        Sx1276TimerSetValue( &RxTimeoutTimer, timeout );
        Sx1276TimerStart( &RxTimeoutTimer );
    }

    if( SX1276.Settings.Modem == MODEM_FSK )
    {
        SX1276SetOpMode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            Sx1276TimerSetValue( &RxTimeoutSyncWord, SX1276.Settings.Fsk.RxSingleTimeout );
            Sx1276TimerStart( &RxTimeoutSyncWord );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}


void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1276SetModem( modem );

    switch( modem )
    {
     case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.PayloadLen = payloadLen;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1276Write( REG_LR_MODEMCONFIG1,
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276Write( REG_LR_MODEMCONFIG2,
                         ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            SX1276Write( REG_LR_MODEMCONFIG3,
                         ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX1276Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1276Write( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( SX1276.Settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x02 );
                SX1276Write( REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x02 );
                SX1276Write( REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x03 );
            }

            if( datarate == 6 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}


bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    SX1276SetModem( modem );

    SX1276SetChannel( freq );

    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    Sx1276DelayMs( 1 );

    carrierSenseTime = Sx1276TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( Sx1276TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = SX1276ReadRssi( modem );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    SX1276SetSleep( );
    return status;
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( SX1276Read( REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
        {
            rssi = RSSI_OFFSET_HF + SX1276Read( REG_LR_RSSIVALUE );
        }
        else
        {
					 rssi = RSSI_OFFSET_LF + SX1276Read( REG_LR_RSSIVALUE );  
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

uint32_t SX1276Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        Sx1276DelayMs( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX1276Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1276SetSleep( );

    return rnd;
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported	  
    return true;
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        {
            airTime = (uint32_t) round( ( 8 * ( SX1276.Settings.Fsk.PreambleLen +
                                     ( ( SX1276Read( REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( SX1276.Settings.Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( SX1276Read( REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( SX1276.Settings.Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     SX1276.Settings.Fsk.Datarate ) * 1000 );
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( SX1276.Settings.LoRa.Bandwidth )
            {
            //case 0: // 7.8 kHz
            //    bw = 7800;
            //    break;
            //case 1: // 10.4 kHz
            //    bw = 10400;
            //    break;
            //case 2: // 15.6 kHz
            //    bw = 15600;
            //    break;
            //case 3: // 20.8 kHz
            //    bw = 20800;
            //    break;
            //case 4: // 31.2 kHz
            //    bw = 31200;
            //    break;
            //case 5: // 41.4 kHz
            //    bw = 41400;
            //    break;
            //case 6: // 62.5 kHz
            //    bw = 62500;
            //    break;
            case 7: // 125 kHz
                bw = 125000;
                break;
            case 8: // 250 kHz
                bw = 250000;
                break;
            case 9: // 500 kHz
                bw = 500000;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << SX1276.Settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( SX1276.Settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * SX1276.Settings.LoRa.Datarate +
                                 28 + 16 * SX1276.Settings.LoRa.CrcOn -
                                 ( SX1276.Settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( SX1276.Settings.LoRa.Datarate -
                                 ( ( SX1276.Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( SX1276.Settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = (uint32_t) floor( tOnAir * 1000 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1276StartCad( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            SX1276.Settings.State = RF_CAD;
            SX1276SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

void SX1276SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )( time * 1000 );

    SX1276SetChannel( freq );

    SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    SX1276Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    SX1276Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    Sx1276TimerSetValue( &TxTimeoutTimer, timeout );

    SX1276.Settings.State = RF_TX_RUNNING;
    Sx1276TimerStart( &TxTimeoutTimer );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}



void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( SX1276.Settings.Fsk.FixLen == false )
        {
            SX1276Write( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX1276Write( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

uint32_t SX1276GetWakeupTime( void )
{
    return 0;
}


void SX1276OnDio1Irq( void )
{
    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_LORA:
                // Sync time out
                Sx1276TimerStop( &RxTimeoutTimer );
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                SX1276.Settings.State = RF_IDLE;
                rt_event_send(&sx1276_event, SX1276_EVENT_RX_TIMEOUT);
						    //rt_kprintf("###########DIO2:%d\n",TimerGetCurrentTime());
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}


/*复位射频，并进行基本初始化*/
void Sx1276Restart(void)
{
    SX1276Reset( );
    Sx1276DelayMs( 10 );

    RxChainCalibration( );
    SX1276SetOpMode( RF_OPMODE_SLEEP );

    uint8_t i;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
 
    SX1276SetModem( MODEM_FSK );
    SX1276.Settings.State = RF_IDLE;
}
