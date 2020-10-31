/*
SX1276驱动主程序，剔除FSK，仅实现lora功能

*/

#include <rtthread.h>
#include "rtdevice.h"
#include "rthw.h"  //DelayMs

#include "sx1276.h"
#include "sx1276_timer.h"

#include "math.h"
#include "string.h"

#include "sx1276Regs-LoRa.h"
#include "sx1276_default.h"
//#include "sx1276Regs-Fsk.h"

/**************************************************************************************************/
#define DBG_ENABLE
#define LOG_TAG              "sx1276"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <rtdbg.h>

//extern void ulog_hexdump(const char *tag, rt_size_t width, rt_uint8_t *buf, rt_size_t size);
/**************************************************************************************************/


/*define*/
#define FREQ_STEP                                   61.03515625
#define RF_MID_BAND_THRESH                          525000000
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157
#define TIMEOUT_FLAG                                0XFF
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

#define SX1276_EVENT_ALL         0xFFFFFFFF
#define SX1276_EVENT_D0         (uint32_t)1<<0  
#define SX1276_EVENT_D1         (uint32_t)1<<1  
#define SX1276_EVENT_TIMEOUT    (uint32_t)1<<2
#define SX1276_EVENT_RX_ERROR   (uint32_t)1<<3
#define SX1276_EVENT_RX_DONE    (uint32_t)1<<4
#define SX1276_EVENT_TX_DONE    (uint32_t)1<<5
#define SX1276_EVENT_TX_TIMEOUT	(uint32_t)1<<6
#define SX1276_EVENT_RX_TIMEOUT	(uint32_t)1<<7	
#define SX1276_EVENT_SYNC_WORD_TIMEOUT	(uint32_t)1<<8
/********/

/*struct*/

typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

typedef enum
{
    RF_IDLE = 0,   //!< The radio is idle
    RF_RX_RUNNING, //!< The radio is in reception state
    RF_TX_RUNNING, //!< The radio is in transmission state
    RF_CAD,        //!< The radio is doing channel activity detection
}RadioState_t;

typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    rt_bool_t     FixLen;
    uint8_t  PayloadLen;
    rt_bool_t     CrcOn;
    rt_bool_t     IqInverted;
    rt_bool_t     RxContinuous;
    uint32_t TxTimeout;
    uint32_t RxSingleTimeout;
}RadioFskSettings_t;

typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
}RadioFskPacketHandler_t;

typedef struct
{
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    rt_bool_t     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    rt_bool_t     FixLen;
    uint8_t  PayloadLen;
    rt_bool_t     CrcOn;
    rt_bool_t     FreqHopOn;
    uint8_t  HopPeriod;
    rt_bool_t     IqInverted;
    rt_bool_t     RxContinuous;
    uint32_t TxTimeout;
    rt_bool_t     PublicNetwork;
}RadioLoRaSettings_t;

typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_t;

typedef struct
{
    RadioState_t             State;
    RadioModems_t            Modem;
    uint32_t                 Channel;
    RadioFskSettings_t       Fsk;
    RadioFskPacketHandler_t  FskPacketHandler;
    RadioLoRaSettings_t      LoRa;
    RadioLoRaPacketHandler_t LoRaPacketHandler;
}RadioSettings_t;


typedef struct
{
    uint8_t Data[254];
	  uint8_t Size;
	  uint8_t Snr;
	  int16_t Rssi;
}RxBuffer_t;
typedef struct
{
     uint8_t Size;
	 uint8_t Data[254];
}TxBuffer_t;

struct sx1276_pcfg_s
{
      RadioSettings_t Settings;
	  RxBuffer_t RxBuffer;
	  TxBuffer_t TxBuffer;
};

/********/

static struct SpecialTimerAPI_s SpecialTimer;
//static struct rt_event sx1276_event;

RadioRegisters_t RadioRegsInit[] = {                                                 
                                        { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
                                    } ;

																
struct sx1276_cfg_s cfg_default={ 
                                   {MODEM_LORA,LORA_DEFAULT_TX_OUTPUT_POWER,0, LORA_DEFAULT_BANDWIDTH,LORA_DEFAULT_SPREADING_FACTOR, LORA_DEFAULT_CODINGRATE,
																   LORA_DEFAULT_PREAMBLE_LENGTH, LORA_DEFAULT_FIX_LENGTH_PAYLOAD_ON,RT_TRUE, 0, 0, LORA_DEFAULT_IQ_INVERSION_ON, 3000},
																	 
																   {MODEM_LORA, LORA_DEFAULT_BANDWIDTH, LORA_DEFAULT_SPREADING_FACTOR,LORA_DEFAULT_CODINGRATE, 0, LORA_DEFAULT_PREAMBLE_LENGTH,
																   LORA_DEFUALT_SYMBOL_TIMEOUT, LORA_DEFAULT_FIX_LENGTH_PAYLOAD_ON, 0, RT_TRUE, 0, 0, LORA_DEFAULT_IQ_INVERSION_ON, RT_TRUE},
                                 };
							
																
/********************回调shell*******************************************************/																		
void sx1276_dio0_cb( void* parameter )
{
	struct sx1276_t *dev = (struct sx1276_t *)parameter;
    if(parameter !=RT_NULL)
		{
			LOG_D("dio0 cb\n"); 
			rt_event_send(dev->event, SX1276_EVENT_D0);
		}
		else
			LOG_D("dio0 cb(pa =NULL)\n");
}

void sx1276_dio1_cb( void* parameter )
{
	   struct sx1276_t *dev = (struct sx1276_t *)parameter;
    if(parameter !=RT_NULL)
		{
			LOG_D("sx1276_dio1_cb\n");
		  rt_event_send(dev->event,SX1276_EVENT_D1);
		}
		else
			LOG_D("sx1276_dio1_cb(parameter ==NULL)\n");
}
void sx1276_rx_timeout(void* parameter)
{
		struct sx1276_t *dev = (struct sx1276_t *)parameter;
	  if(parameter !=RT_NULL)
		{
			LOG_D("RX_timeout");
		  rt_event_send(dev->event,SX1276_EVENT_RX_TIMEOUT);
		}
		else
			LOG_D("RX_timeout(parameter ==NULL)");
	
//    rt_kprintf("rx tiomeout!! \n");
//	  struct sx1276_t *dev = (struct sx1276_t *)parameter;
//	  SpecialTimer.Start(dev->timer.RxTimeoutTimer);
}

void sx1276_tx_timeout(void* parameter)
{
	  struct sx1276_t *dev = (struct sx1276_t *)parameter;
 	  if(parameter !=RT_NULL)
		{
			LOG_D("TX_timeout");
		  rt_event_send(dev->event,SX1276_EVENT_TX_TIMEOUT);
		}
		else
			LOG_D("TX_timeout(parameter==NULL)");
}
void sx1276_sync_word_timeout(void * parameter)
{
	  struct sx1276_t *dev = (struct sx1276_t *)parameter;
 	  if(parameter !=RT_NULL)
		{
			LOG_D("sync_word");
		  rt_event_send(dev->event,SX1276_EVENT_SYNC_WORD_TIMEOUT);
		}
		else
			LOG_D("sync_word(parameter==NULL)");
}


/*****************************************************************************/


static void sx1276_read_buffer( struct sx1276_bsp_s *bsp,uint16_t addr, uint8_t *buffer, uint8_t size )
{
    rt_pin_write(bsp->dio.nss, 0);
    SpecialTimer.DelayMs( 1 );
    addr=addr & 0x7f;
	
	  struct rt_spi_device *spi_hander;
    spi_hander = (struct rt_spi_device *)rt_device_find(bsp->spi_name);
    rt_spi_send_then_recv(spi_hander,&addr,1,buffer,size);
	
    rt_pin_write(bsp->dio.nss, 1);
    SpecialTimer.DelayMs( 1 );
}

static void sx1276_write_buffer( struct sx1276_bsp_s *bsp,uint16_t addr, uint8_t *buffer, uint8_t size )
{
    rt_pin_write(bsp->dio.nss, 0);
    SpecialTimer.DelayMs( 1 );
    addr= addr|0x80;
		struct rt_spi_device *spi_hander;
    spi_hander = (struct rt_spi_device *)rt_device_find(bsp->spi_name);
    rt_spi_send_then_send(spi_hander,&addr, 1,buffer,size);
	
    rt_pin_write(bsp->dio.nss, 1);
    SpecialTimer.DelayMs( 1 );
}

uint8_t sx1276_read( struct sx1276_bsp_s *bsp ,uint16_t addr )
{
    uint8_t data;
    sx1276_read_buffer( bsp,addr, &data, 1 );
    return data;
}

void sx1276_write( struct sx1276_bsp_s *bsp,uint16_t addr, uint8_t data )
{
     sx1276_write_buffer(  bsp, addr, &data, 1 );
}

void sx1276_write_fifo( struct sx1276_bsp_s *bsp, uint8_t *buffer, uint8_t size )
{
    sx1276_write_buffer( bsp ,0, buffer, size );
}

void sx1276_read_fifo(struct sx1276_bsp_s *bsp ,uint8_t *buffer, uint8_t size )
{
    sx1276_read_buffer( bsp,0, buffer, size );
}
/********************************************************************************************************************/

void sx1276_set_channel( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg,uint32_t freq )
{
	  pcfg->Settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    sx1276_write( bsp,REG_LR_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    sx1276_write( bsp,REG_LR_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    sx1276_write( bsp,REG_LR_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

static void sx1267_rx_chain_calibration( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = sx1276_read( bsp,REG_LR_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )sx1276_read( bsp,REG_LR_FRFMSB ) << 16 ) |
                              ( ( uint32_t )sx1276_read( bsp,REG_LR_FRFMID ) << 8 ) |
                              ( ( uint32_t )sx1276_read( bsp,REG_LR_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    sx1276_write( bsp,REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    sx1276_write( bsp,REG_IMAGECAL, ( sx1276_read(bsp, REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( sx1276_read( bsp,REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    sx1276_set_channel( bsp,pcfg,868000000 );

    // Launch Rx chain calibration for HF band
    sx1276_write( bsp,REG_IMAGECAL, ( sx1276_read( bsp,REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( sx1276_read( bsp,REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }
    // Restore context
    sx1276_write( bsp,REG_PACONFIG, regPaConfigInitVal );
    sx1276_set_channel( bsp,pcfg,initialFreq );
}

void sx1276_set_opmode( struct sx1276_bsp_s *bsp,uint8_t opMode )
{
    if( opMode == RFLR_OPMODE_SLEEP )
    {
      sx1276_write( bsp,REG_LR_OPMODE, ( sx1276_read(bsp, REG_LR_OPMODE ) & RFLR_OPMODE_MASK ) | opMode ); 
      //将射频开关I/Os引脚设置为低功率模式 SX1276BoardSetAntSwLowPower( true );
    }
    else
    {
     //SX1276BoardSetAntSwLowPower( false );
     //SX1276BoardSetAntSw( opMode );
      sx1276_write( bsp,REG_LR_OPMODE, ( sx1276_read( bsp,REG_LR_OPMODE ) & RFLR_OPMODE_MASK ) | opMode );
    }
}

void sx1276_reset( rt_int32_t rst )
{
    rt_pin_mode(rst, PIN_MODE_OUTPUT);
	  rt_pin_write(rst, 0);

    // Wait 1 ms
	  SpecialTimer.DelayMs(1);
    // Configure RESET as input
	  rt_pin_mode(rst, PIN_MODE_INPUT);
    // Wait 6 ms
  	SpecialTimer.DelayMs(6);

}

uint8_t sx1276_get_pa_select( uint32_t channel )
{
  if( channel < RF_MID_BAND_THRESH )
  {
    return RFLR_PACONFIG_PASELECT_PABOOST;
  }
  else
  {
    return RFLR_PACONFIG_PASELECT_RFO;
  }
}

void sx1276_set_rf_tx_power( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg,int8_t power )
{
  uint8_t paConfig = 0;
  uint8_t paDac = 0;
  
  paConfig = sx1276_read(bsp,REG_LR_PACONFIG );
  paDac = sx1276_read(bsp,REG_LR_PADAC );
  
  paConfig = ( paConfig & RFLR_PACONFIG_PASELECT_MASK ) | sx1276_get_pa_select( pcfg->Settings.Channel );
  paConfig = ( paConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
  
  if( ( paConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
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
  sx1276_write(bsp, REG_PACONFIG, paConfig );
  sx1276_write( bsp,REG_PADAC, paDac );
}


void sx1276_set_sleep( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg )
{
    sx1276_set_opmode( bsp,RF_OPMODE_SLEEP );
    pcfg->Settings.State = RF_IDLE;
}

void sx1276_set_modem( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg,RadioModems_t modem )
{

    if( ( sx1276_read(bsp, REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        pcfg->Settings.Modem = MODEM_LORA;
    }
    else
    {
        pcfg->Settings.Modem = MODEM_FSK;
    }

    if( pcfg->Settings.Modem == modem )
    {
        return;
    }

    pcfg->Settings.Modem = modem;
    switch( pcfg->Settings.Modem )
    {
    default:
    case MODEM_FSK:
        sx1276_set_sleep(bsp,pcfg);
        sx1276_write( bsp,REG_OPMODE, ( sx1276_read(bsp,REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
        sx1276_write( bsp,REG_DIOMAPPING1, 0x00 );
        sx1276_write( bsp,REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        sx1276_set_sleep(bsp,pcfg);
        sx1276_write(bsp, REG_OPMODE, ( sx1276_read( bsp,REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );
        
//        temp = SX1276Read( REG_OPMODE ) ;
//	      rt_kprintf("mode2=%02X\n",temp);    	
        sx1276_write( bsp,REG_DIOMAPPING1, 0x00 );
        sx1276_write( bsp,REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void sx1276_set_stby( struct sx1276_timer_s *timer ,struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg)
{
	  SpecialTimer.Stop(timer->RxTimeoutTimer);
    SpecialTimer.Stop(timer->TxTimeoutTimer);
    
	
    sx1276_set_opmode( bsp,RF_OPMODE_STANDBY );
    pcfg->Settings.State = RF_IDLE;
}

/***********************************************************************************************************TX*/

void sx1276_set_tx_config( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg,
	                         RadioModems_t modem, int8_t power, uint32_t fdev,
                           uint32_t bandwidth, uint32_t datarate,
                           uint8_t coderate, uint16_t preambleLen,
                           rt_bool_t fixLen, rt_bool_t crcOn, rt_bool_t freqHopOn,
                           uint8_t hopPeriod, rt_bool_t iqInverted, uint32_t timeout )
{
    sx1276_set_modem( bsp,pcfg,modem );

    sx1276_set_rf_tx_power(bsp,pcfg, power );

    switch( modem )
    {
      case MODEM_LORA:
      {
              pcfg->Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
               // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
							 LOG_E("bandwidth error");
               return;
             }
              bandwidth += 7;
							pcfg->Settings.LoRa.Bandwidth = bandwidth;
							pcfg->Settings.LoRa.Datarate = datarate;
							pcfg->Settings.LoRa.Coderate = coderate;
							pcfg->Settings.LoRa.PreambleLen = preambleLen;
							pcfg->Settings.LoRa.FixLen = fixLen;
							pcfg->Settings.LoRa.FreqHopOn = freqHopOn;
							pcfg->Settings.LoRa.HopPeriod = hopPeriod;
							pcfg->Settings.LoRa.CrcOn = crcOn;
							pcfg->Settings.LoRa.IqInverted = iqInverted;
							pcfg->Settings.LoRa.TxTimeout = timeout;

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
									pcfg->Settings.LoRa.LowDatarateOptimize = 0x01;
							}
							else
							{
									pcfg->Settings.LoRa.LowDatarateOptimize = 0x00;
							}

							if( pcfg->Settings.LoRa.FreqHopOn == RT_TRUE )
							{
									sx1276_write(bsp, REG_LR_PLLHOP, ( sx1276_read(bsp, REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
									sx1276_write( bsp,REG_LR_HOPPERIOD, pcfg->Settings.LoRa.HopPeriod );
							}

							sx1276_write( bsp,REG_LR_MODEMCONFIG1,
													 ( sx1276_read( bsp,REG_LR_MODEMCONFIG1 ) &
														 RFLR_MODEMCONFIG1_BW_MASK &
														 RFLR_MODEMCONFIG1_CODINGRATE_MASK &
														 RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
														 ( bandwidth << 4 ) | ( coderate << 1 ) |
														 fixLen );

							sx1276_write( bsp,REG_LR_MODEMCONFIG2,
													 ( sx1276_read( bsp,REG_LR_MODEMCONFIG2 ) &
														 RFLR_MODEMCONFIG2_SF_MASK &
														 RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
														 ( datarate << 4 ) | ( crcOn << 2 ) );

							sx1276_write( bsp,REG_LR_MODEMCONFIG3,
													 ( sx1276_read(bsp, REG_LR_MODEMCONFIG3 ) &
														 RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
														 ( pcfg->Settings.LoRa.LowDatarateOptimize << 3 ) );

							sx1276_write( bsp,REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
							sx1276_write( bsp,REG_LR_PREAMBLELSB, preambleLen & 0xFF );

							if( datarate == 6 )
							{
									sx1276_write( bsp,REG_LR_DETECTOPTIMIZE,
															 ( sx1276_read(bsp, REG_LR_DETECTOPTIMIZE ) &
																 RFLR_DETECTIONOPTIMIZE_MASK ) |
																 RFLR_DETECTIONOPTIMIZE_SF6 );
									sx1276_write( bsp,REG_LR_DETECTIONTHRESHOLD,
															 RFLR_DETECTIONTHRESH_SF6 );
							}
							else
							{
									sx1276_write( bsp,REG_LR_DETECTOPTIMIZE,
															 ( sx1276_read( bsp,REG_LR_DETECTOPTIMIZE ) &
															 RFLR_DETECTIONOPTIMIZE_MASK ) |
															 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
									sx1276_write( bsp,REG_LR_DETECTIONTHRESHOLD,
															 RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
							}
      } break;
    }
}


void sx1276_set_tx( struct sx1276_timer_s *timer,struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg,uint32_t timeout )
{
    SpecialTimer.SetValue(timer->TxTimeoutTimer,timeout);
    
    switch( pcfg->Settings.Modem )
    {
    case MODEM_LORA:
        {
            if( pcfg->Settings.LoRa.FreqHopOn == RT_TRUE )
            {
                sx1276_write(bsp, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                sx1276_write(bsp, REG_DIOMAPPING1, ( sx1276_read(bsp, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                sx1276_write(bsp, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                sx1276_write(bsp, REG_DIOMAPPING1, ( sx1276_read(bsp, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }
    pcfg->Settings.State = RF_TX_RUNNING;
    SpecialTimer.Start(timer->TxTimeoutTimer);
    sx1276_set_opmode( bsp,RF_OPMODE_TRANSMITTER );
}

void sx1276_send(struct sx1276_t *dev, uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;
	
    struct sx1276_pcfg_s *pcfg=dev->pcfg;
	  struct sx1276_bsp_s *bsp = &(dev->bsp);
	
    switch( pcfg->Settings.Modem )
    {

    case MODEM_LORA:
        {
            if( pcfg->Settings.LoRa.IqInverted == RT_TRUE )
            {
                sx1276_write(bsp, REG_LR_INVERTIQ, ( ( sx1276_read(bsp, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                sx1276_write(bsp, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                sx1276_write(bsp, REG_LR_INVERTIQ, ( ( sx1276_read(bsp, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                sx1276_write(bsp, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            pcfg->Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            sx1276_write(bsp, REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            sx1276_write(bsp, REG_LR_FIFOTXBASEADDR, 0 );
            sx1276_write(bsp, REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( sx1276_read(bsp, REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                sx1276_set_stby(&(dev->timer),bsp,pcfg);
								SpecialTimer.DelayMs(1);
							 
            }
            // Write payload buffer
//						pcfg->TxBuffer.Size = size;
//						rt_memcmp(pcfg->TxBuffer.Data, buffer,size);
						
            sx1276_write_fifo(bsp,buffer,size);
            txTimeout = pcfg->Settings.LoRa.TxTimeout;
        }
        break;
    }
		sx1276_set_tx(&(dev->timer),bsp,pcfg,txTimeout);
}



/***********************************************************************************************************RX*/

void sx1276_set_rx_config( struct sx1276_bsp_s *bsp,struct sx1276_pcfg_s *pcfg,
                         RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, rt_bool_t fixLen,
                         uint8_t payloadLen,
                         rt_bool_t crcOn, rt_bool_t freqHopOn, uint8_t hopPeriod,
                         rt_bool_t iqInverted, rt_bool_t rxContinuous )
{
    sx1276_set_modem( bsp,pcfg,modem );

    switch( modem )
    {
     case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
               // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
				LOG_E("bandwidth error");
               return;
            }
            bandwidth += 7;
            pcfg->Settings.LoRa.Bandwidth = bandwidth;
            pcfg->Settings.LoRa.Datarate = datarate;
            pcfg->Settings.LoRa.Coderate = coderate;
            pcfg->Settings.LoRa.PreambleLen = preambleLen;
            pcfg->Settings.LoRa.FixLen = fixLen;
            pcfg->Settings.LoRa.PayloadLen = payloadLen;
            pcfg->Settings.LoRa.CrcOn = crcOn;
            pcfg->Settings.LoRa.FreqHopOn = freqHopOn;
            pcfg->Settings.LoRa.HopPeriod = hopPeriod;
            pcfg->Settings.LoRa.IqInverted = iqInverted;
            pcfg->Settings.LoRa.RxContinuous = rxContinuous;

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
                pcfg->Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                pcfg->Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            sx1276_write(bsp, REG_LR_MODEMCONFIG1,
                         ( sx1276_read(bsp, REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            sx1276_write(bsp, REG_LR_MODEMCONFIG2,
                         ( sx1276_read(bsp, REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            sx1276_write(bsp, REG_LR_MODEMCONFIG3,
                         ( sx1276_read(bsp, REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( pcfg->Settings.LoRa.LowDatarateOptimize << 3 ) );

            sx1276_write(bsp, REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            sx1276_write(bsp, REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            sx1276_write(bsp, REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                sx1276_write(bsp, REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( pcfg->Settings.LoRa.FreqHopOn == RT_TRUE )
            {
                sx1276_write(bsp, REG_LR_PLLHOP, ( sx1276_read(bsp, REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                sx1276_write(bsp, REG_LR_HOPPERIOD, pcfg->Settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( pcfg->Settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                sx1276_write(bsp, REG_LR_TEST36, 0x02 );
                sx1276_write(bsp, REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                sx1276_write(bsp, REG_LR_TEST36, 0x02 );
                sx1276_write(bsp, REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                sx1276_write(bsp, REG_LR_TEST36, 0x03 );
            }

            if( datarate == 6 )
            {
                sx1276_write(bsp, REG_LR_DETECTOPTIMIZE,
                             ( sx1276_read(bsp, REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                sx1276_write(bsp, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                sx1276_write(bsp, REG_LR_DETECTOPTIMIZE,
                             ( sx1276_read(bsp, REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                sx1276_write(bsp, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void sx1276_set_rx(struct sx1276_t *dev,uint32_t timeout)
{
	 struct sx1276_pcfg_s *pcfg=dev->pcfg;
	 struct sx1276_bsp_s *bsp = &(dev->bsp);
   rt_bool_t rx_continuous = RT_FALSE;
	 switch(pcfg ->Settings.Modem)
	 {
	   case MODEM_LORA:
		 {
				if( pcfg->Settings.LoRa.IqInverted == RT_TRUE )
				{
						sx1276_write(bsp, REG_LR_INVERTIQ, ( ( sx1276_read(bsp, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
						sx1276_write(bsp, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
				}	
				else
				{
						sx1276_write(bsp, REG_LR_INVERTIQ, ( ( sx1276_read(bsp, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
						sx1276_write(bsp, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
				}  
			 if( pcfg->Settings.LoRa.Bandwidth < 9 )
			 {
					sx1276_write(bsp, REG_LR_DETECTOPTIMIZE, sx1276_read(bsp, REG_LR_DETECTOPTIMIZE ) & 0x7F );
					sx1276_write(bsp, REG_LR_TEST30, 0x00 );
					switch( pcfg->Settings.LoRa.Bandwidth )
					{
					case 0: // 7.8 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x48 );
							sx1276_set_channel( bsp,pcfg,pcfg->Settings.Channel + 7810 );
							break;
					case 1: // 10.4 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x44 );
							sx1276_set_channel(bsp,pcfg,pcfg->Settings.Channel + 10420 );
							break;
					case 2: // 15.6 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x44 );
							sx1276_set_channel(bsp,pcfg,pcfg->Settings.Channel + 15620 );
							break;
					case 3: // 20.8 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x44 );
							sx1276_set_channel(bsp,pcfg,pcfg->Settings.Channel + 20830 );
							break;
					case 4: // 31.2 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x44 );
							sx1276_set_channel(bsp,pcfg,pcfg->Settings.Channel + 31250 );
							break;
					case 5: // 41.4 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x44 );
							sx1276_set_channel(bsp,pcfg,pcfg->Settings.Channel + 41670 );
							break;
					case 6: // 62.5 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x40 );
							break;
					case 7: // 125 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x40 );
							break;
					case 8: // 250 kHz
							sx1276_write(bsp, REG_LR_TEST2F, 0x40 );
							break;
					}
			}
			else
			{
					sx1276_write(bsp, REG_LR_DETECTOPTIMIZE, sx1276_read(bsp, REG_LR_DETECTOPTIMIZE ) | 0x80 );
			} 

			rx_continuous = pcfg->Settings.LoRa.RxContinuous;

			if( pcfg->Settings.LoRa.FreqHopOn == RT_TRUE )
			{
					sx1276_write(bsp, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
																								//RFLR_IRQFLAGS_RXDONE |
																								//RFLR_IRQFLAGS_PAYLOADCRCERROR |
																								RFLR_IRQFLAGS_VALIDHEADER |
																								RFLR_IRQFLAGS_TXDONE |
																								RFLR_IRQFLAGS_CADDONE |
																								//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																								RFLR_IRQFLAGS_CADDETECTED );

					// DIO0=RxDone, DIO2=FhssChangeChannel
					sx1276_write(bsp, REG_DIOMAPPING1, ( sx1276_read(bsp, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
			}
			else
			{
					sx1276_write(bsp, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
																								//RFLR_IRQFLAGS_RXDONE |
																								//RFLR_IRQFLAGS_PAYLOADCRCERROR |
																								RFLR_IRQFLAGS_VALIDHEADER |
																								RFLR_IRQFLAGS_TXDONE |
																								RFLR_IRQFLAGS_CADDONE |
																								RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																								RFLR_IRQFLAGS_CADDETECTED );

					// DIO0=RxDone
					sx1276_write(bsp, REG_DIOMAPPING1, ( sx1276_read(bsp, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
			}	
			sx1276_write(bsp, REG_LR_FIFORXBASEADDR, 0 );
			sx1276_write(bsp, REG_LR_FIFOADDRPTR, 0 );
								
		 }break;
	 }
		rt_memset(pcfg->RxBuffer.Data,0,sizeof(pcfg->RxBuffer.Data));		
		pcfg->Settings.State = RF_RX_RUNNING;      
		if( timeout != 0 )
		{
			SpecialTimer.SetValue(dev->timer.RxTimeoutTimer,timeout);
			SpecialTimer.Start(dev->timer.RxTimeoutTimer);
		}
		if(pcfg->Settings.Modem ==MODEM_LORA)
		{
		   if( rx_continuous == RT_TRUE )//是否开启连续接收
			 {
			     sx1276_set_opmode(bsp, RFLR_OPMODE_RECEIVER );
			 }
			 else
			 {
			     sx1276_set_opmode(bsp, RFLR_OPMODE_RECEIVER_SINGLE );
			 }
		}
}
/****************************************Othre*******************************************************/
void sx1276_reg_init(struct sx1276_t *dev)
{
	  struct sx1276_pcfg_s *pcfg=dev->pcfg;
		sx1276_reset(dev->bsp.dio.rst);
		SpecialTimer.DelayMs(10);

		//DEBUG 读取设备ID，检查是否能正常通讯
		uint8_t temp_id=sx1276_read( &(dev->bsp),0x42);
		LOG_D("CHIP ID = %d",temp_id);

		sx1267_rx_chain_calibration(& (dev->bsp) ,pcfg);
		sx1276_set_opmode( & (dev->bsp),RF_OPMODE_SLEEP );

		uint8_t i;
		for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
		{
			sx1276_set_opmode( &(dev->bsp),RadioRegsInit[i].Modem );
			sx1276_write(&(dev->bsp) ,RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
		}
		sx1276_set_opmode( &(dev->bsp), MODEM_LORA ); 
		pcfg->Settings.State = RF_IDLE;
}

RadioState_t sx1276_get_status( struct sx1276_pcfg_s *pcfg  )
{
    return pcfg->Settings.State;
}

void sx1276_set_public_network(struct sx1276_t *dev,rt_bool_t enable )
{
	  struct sx1276_bsp_s *bsp = &(dev->bsp);
		struct sx1276_pcfg_s *pcfg = dev->pcfg;
	
    sx1276_set_modem( bsp,pcfg,MODEM_LORA );
    pcfg->Settings.LoRa.PublicNetwork = enable;
    if( enable == RT_TRUE )
    {
        // Change LoRa modem SyncWord
        sx1276_write(bsp, REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        sx1276_write(bsp, REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

uint32_t sx1276_random( struct sx1276_t *dev )
{
	 struct sx1276_pcfg_s * pcfg = dev->pcfg;
	 struct sx1276_bsp_s * bsp = &(dev->bsp);
	 uint8_t i;
	 uint32_t rnd=0;
   sx1276_set_modem(bsp,pcfg,MODEM_LORA);
	 // Disable LoRa modem interrupts
   sx1276_write(bsp, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );
    // Set radio in continuous reception
    sx1276_set_opmode(bsp, RF_OPMODE_RECEIVER );
	  for( i = 0; i < 32; i++ )
    {
			  SpecialTimer.DelayMs(1);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )sx1276_read(bsp, REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }
		sx1276_set_sleep(bsp,pcfg);
		return rnd;
}

uint32_t sx1267_get_time_on_air( struct sx1276_t *dev,RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;
	  struct sx1276_pcfg_s *pcfg = dev->pcfg;
	  switch (modem)
		{
			case MODEM_LORA:
			{
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( pcfg->Settings.LoRa.Bandwidth )
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
            double rs = bw / ( 1 << pcfg->Settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( pcfg->Settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * pcfg->Settings.LoRa.Datarate +
                                 28 + 16 * pcfg->Settings.LoRa.CrcOn -
                                 ( pcfg->Settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( pcfg->Settings.LoRa.Datarate -
                                 ( ( pcfg->Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( pcfg->Settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = (uint32_t) floor( tOnAir * 1000 + 0.999 );			
			
			}break;
		}
		return airTime;
}

int16_t sx1276_read_rssi( struct sx1276_t *dev,RadioModems_t modem )
{
		struct sx1276_pcfg_s *pcfp = dev->pcfg;
	  struct sx1276_bsp_s *bsp = &(dev->bsp);
    int16_t rssi = 0;
    switch( modem )
    {
				case MODEM_LORA:
				{
					if( pcfp->Settings.Channel > RF_MID_BAND_THRESH )
					{
            rssi = RSSI_OFFSET_HF + sx1276_read(bsp, REG_LR_RSSIVALUE );
					}
					else
					{
            rssi = RSSI_OFFSET_LF + sx1276_read(bsp, REG_LR_RSSIVALUE );
					}
        }break;
				default:
					rssi = -1;
        break;
    }
    return rssi;    
}

rt_bool_t sx1276_is_channel_free( struct sx1276_t *dev,RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
	  struct sx1276_bsp_s *bsp = &(dev->bsp);
	  struct sx1276_pcfg_s *pcfg = dev->pcfg;
    rt_bool_t status = RT_TRUE;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;   
	  sx1276_set_modem(bsp,pcfg,modem);
	  sx1276_set_channel(bsp,pcfg,freq);
	  sx1276_set_opmode(bsp,RF_OPMODE_RECEIVER);
	  SpecialTimer.DelayMs(1);
	  carrierSenseTime =  SpecialTimer.GetCurrentTime();
	  while(SpecialTimer.GetElapsedTime(carrierSenseTime) < maxCarrierSenseTime  )
		{
			 rssi = sx1276_read_rssi(dev,modem);
			 if(rssi > rssiThresh)
			 {
			    status = RT_FALSE;
				  break;
			 }
		}
		sx1276_set_sleep(bsp,pcfg);
		return status;
}

void sx1276_start_cad( struct sx1276_t *dev )
{
	 	struct sx1276_bsp_s *bsp = &(dev->bsp);
	  struct sx1276_pcfg_s *pcfg = dev->pcfg;
    switch( pcfg->Settings.Modem )
    {
			case MODEM_LORA:
      {
            sx1276_write(bsp, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            sx1276_write(bsp, REG_DIOMAPPING1, ( sx1276_read(bsp, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );
            pcfg->Settings.State = RF_CAD;
            sx1276_set_opmode(bsp, RFLR_OPMODE_CAD );
      }break;
     default:
        break;
    }
}

void sx1276_set_continuous_wave( struct sx1276_t *dev,uint32_t freq, int8_t power, uint16_t time )
{
	  struct sx1276_bsp_s *bsp = &(dev->bsp);
	  struct sx1276_pcfg_s *pcfg = dev->pcfg;
    uint32_t timeout = ( uint32_t )( time * 1000 );

    sx1276_set_channel( bsp,pcfg,freq );

    sx1276_set_tx_config( bsp,pcfg,MODEM_FSK, power, 0, 0, 4800, 0, 5, RT_FALSE, RT_FALSE, 0, 0, 0, timeout );

    sx1276_write(bsp, REG_PACKETCONFIG2, ( sx1276_read(bsp, REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    sx1276_write(bsp, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    sx1276_write(bsp, REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

	  SpecialTimer.SetValue(dev->timer.TxTimeoutTimer,timeout);
    
    pcfg->Settings.State = RF_TX_RUNNING;
    
	  SpecialTimer.Start(dev->timer.TxTimeoutTimer);  
	  sx1276_set_opmode(bsp,RF_OPMODE_TRANSMITTER);
}

void sx1276_set_max_payload_length( struct sx1276_t *dev,RadioModems_t modem, uint8_t max )
{
	  struct sx1276_bsp_s *bsp = &(dev->bsp);
		struct sx1276_pcfg_s *pcfg = dev->pcfg;
    sx1276_set_modem( bsp,pcfg,modem );

    switch( modem )
    {
     case MODEM_LORA:
        sx1276_write(bsp, REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}


rt_bool_t sx1276_check_rt_frequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return RT_TRUE;
}



uint32_t sx1276_get_wakeup_time( void )
{
    return 0;
}

/***********************************device api****************************************************/
static rt_err_t sx1276_device_api_init(rt_device_t dev)
{
	  //struct sx1276_t *dev = (struct sx1276_t *)dev;
    return RT_EOK;
}

static rt_err_t sx1276_device_api_open(rt_device_t parent,rt_uint16_t oflag)
{  
	   struct sx1276_t *dev = (struct sx1276_t *)parent; 
	   struct sx1276_bsp_s *bsp = &(dev->bsp);
	   struct sx1276_pcfg_s *pcfg = dev->pcfg;
	   //使用默认频率
	   sx1276_set_channel(bsp,pcfg,LORA_DEFAULT_RF_FREQUENCY);
     //配置在注册的时候已经填充完毕了
	   sx1276_set_tx_config(bsp,pcfg,dev->cfg.cfg_tx.modem,dev->cfg.cfg_tx.power,dev->cfg.cfg_tx.fdev,
													dev->cfg.cfg_tx.bandwidth,dev->cfg.cfg_tx.datarate,
													dev->cfg.cfg_tx.coderate,dev->cfg.cfg_tx.preambleLen,
													dev->cfg.cfg_tx.fixLen,dev->cfg.cfg_tx.crcOn,dev->cfg.cfg_tx.freqHopOn,
													dev->cfg.cfg_tx.hopPeriod,dev->cfg.cfg_tx.iqInverted,dev->cfg.cfg_tx.timeout);
	
	   sx1276_set_rx_config(bsp,pcfg,dev->cfg.cfg_rx.modem,dev->cfg.cfg_rx.bandwidth,dev->cfg.cfg_rx.datarate,
													dev->cfg.cfg_rx.coderate,dev->cfg.cfg_rx.bandwidthAfc,dev->cfg.cfg_rx.preambleLen,
													dev->cfg.cfg_rx.symbTimeout,dev->cfg.cfg_rx.fixLen,dev->cfg.cfg_rx.payloadLen,
	                        dev->cfg.cfg_rx.crcOn,dev->cfg.cfg_rx.freqHopOn,dev->cfg.cfg_rx.hopPeriod,
													dev->cfg.cfg_rx.iqInverted,dev->cfg.cfg_rx.rxContinuous);
	
     return RT_EOK;
}
static rt_err_t sx1276_device_api_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t sx1276_device_api_control(rt_device_t dev,int cmd,void *args)
{
    return RT_EOK;
}

static rt_size_t sx1276_device_api_write(rt_device_t parent,rt_off_t pos,const void * buffer,rt_size_t size)
{
	  struct sx1276_t *dev = (struct sx1276_t *)parent; 
	  struct sx1276_pcfg_s *pcfg = dev->pcfg;
	  rt_memcpy(pcfg->TxBuffer.Data,buffer,size);
	  pcfg->TxBuffer.Size = size;
	  
	  sx1276_send(dev,pcfg->TxBuffer.Data,size);
    return 0;
}

//这里的size 为接收窗口打开的时间，单位ms
static rt_size_t sx1276_device_api_read(rt_device_t parent,rt_off_t pos, void *buffer,rt_size_t size)
{  
	    struct sx1276_t *dev = (struct sx1276_t *)parent;
	    sx1276_set_rx(dev,size);
			return 0;
}

#ifdef RT_USING_DEVICE_OPS

const static struct rt_deveice sx1276_ops=
{
   sx1276_device_api_init,
   sx1276_device_api_open,
   sx1276_device_api_close,
   sx1276_device_api_read,
   sx1276_device_api_write,
   sx1276_device_api_control,
};
#endif


/****************************EVENT FUNTION**********************************************************/
void sx1276_irq_0_source( struct sx1276_t *dev )
{

	 volatile uint8_t irqFlags = 0;
	 struct sx1276_bsp_s *bsp= &(dev->bsp);
   struct sx1276_timer_s *timer = &(dev->timer);
   struct sx1276_pcfg_s *pcfg = dev->pcfg;

   switch( pcfg->Settings.State )
    {
        case RF_RX_RUNNING: //接收状态
        {
            switch( pcfg->Settings.Modem )
            {
                case MODEM_LORA:
                {
                    int8_t snr = 0;
                // Clear Irq
									
                    sx1276_write(bsp, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );
                    irqFlags = sx1276_read( bsp, REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        sx1276_write(bsp, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
                        if( pcfg->Settings.LoRa.RxContinuous == RT_FALSE )
                        {
                            pcfg->Settings.State = RF_IDLE;
                        }
												SpecialTimer.Stop(timer->RxTimeoutTimer);
                        //发送错误事件
                        rt_event_send(dev->event, SX1276_EVENT_RX_ERROR);
                        break;
                    }
                    pcfg->Settings.LoRaPacketHandler.SnrValue = sx1276_read(bsp, REG_LR_PKTSNRVALUE );
                    if( pcfg->Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~pcfg->Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( pcfg->Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = sx1276_read(bsp, REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        if( pcfg->Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            pcfg->Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +snr;
                        }
                        else
                        {
                            pcfg->Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +snr;
                        }
                    }
                    else
                    {
                        if( pcfg->Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            pcfg->Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            pcfg->Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }
                    pcfg->Settings.LoRaPacketHandler.Size = sx1276_read(bsp, REG_LR_RXNBBYTES );
                    sx1276_write(bsp, REG_LR_FIFOADDRPTR, sx1276_read(bsp, REG_LR_FIFORXCURRENTADDR ) );
                    sx1276_read_fifo( bsp,pcfg->RxBuffer.Data, pcfg->Settings.LoRaPacketHandler.Size );

                    if( pcfg->Settings.LoRa.RxContinuous == RT_FALSE )
                    {
                        pcfg->Settings.State = RF_IDLE;
                    }
										SpecialTimer.Stop(timer->RxTimeoutTimer);
                    pcfg->RxBuffer.Size = pcfg->Settings.LoRaPacketHandler.Size;
                    pcfg->RxBuffer.Rssi=  pcfg->Settings.LoRaPacketHandler.RssiValue; 
                    pcfg->RxBuffer.Snr =  pcfg->Settings.LoRaPacketHandler.SnrValue;
										
                    //memcpy(EventDataBuffer,RxTxBuffer,EventDataSize);
                    rt_event_send(dev->event, SX1276_EVENT_RX_DONE);	
                }break;
            }

        }break;

        case RF_TX_RUNNING://发送状态
        {
					  SpecialTimer.Stop(timer->TxTimeoutTimer);
            switch( pcfg->Settings.Modem )
            {
               case MODEM_LORA:
                // Clear Irq
                 sx1276_write(bsp, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
              default:
                pcfg->Settings.State = RF_IDLE;
			        	rt_event_send(dev->event, SX1276_EVENT_TX_DONE);
                              
            }
            
        }break;
    }
}

void sx1276_tx_timeout_source(struct sx1276_t *dev)
{
	 rt_device_t parent_p = &(dev->parent);
	 rt_uint8_t re = TIMEOUT_FLAG; //返回0XFF表示发送超时
	//复位1276
	 sx1276_reg_init(dev);
	 parent_p->tx_complete(parent_p,&re);
}
void sx1276_rx_timeout_source(struct sx1276_t *dev)
{
    //如果注册了外部接口函数 则调用
	  rt_device_t parent_p = &(dev->parent);
	  parent_p->rx_indicate(parent_p,TIMEOUT_FLAG);
}

//确认发送成功时产生的回调，调用发送中断，传入发送的长度
void sx1276_tx_done_source(struct sx1276_t *dev)
{
	 struct sx1276_pcfg_s * pfcg = (struct sx1276_pcfg_s *)(dev->pcfg);
	 rt_uint8_t re = pfcg->TxBuffer.Size;
	 if(dev->parent.tx_complete != RT_NULL)
     dev->parent.tx_complete(&(dev->parent),&re);
	 else
		 rt_kprintf("rx fun not have\n");
}

//确认接收，数据指针保存在设备的user_data中。
void sx1276_rx_done_source(struct sx1276_t *dev)
{
    struct sx1276_pcfg_s * pfcg = (struct sx1276_pcfg_s *)(dev->pcfg);
	  rt_device_t parent_p = &(dev->parent);
    parent_p->user_data = pfcg->RxBuffer.Data;
	  if(dev->parent.rx_indicate != RT_NULL)
	    dev->parent.rx_indicate(parent_p,pfcg->RxBuffer.Size); //error
		else
			rt_kprintf("rx fun not have\n");
}
/***************************THREAD*****************************************************************/

void sx1276_thread_entry(void *parameter)
{
	  rt_uint32_t event;
	  struct sx1276_t *dev = (struct sx1276_t *)parameter;
	  rt_event_init(dev->event, "sx1276_event", RT_IPC_FLAG_FIFO);	
	  SpecialTimer.DelayMs(1);
	  //启动中断
	  rt_pin_irq_enable(dev->bsp.dio.irq0, PIN_IRQ_ENABLE);
  	//rt_pin_irq_enable(dev->bsp.dio.irq1, PIN_IRQ_ENABLE);
		while(1) 
	  {
	      rt_event_recv(dev->event,SX1276_EVENT_ALL ,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &event);
		    if(event & SX1276_EVENT_D0)
        {
					  //处理接收发和发送事件
					  sx1276_irq_0_source(dev);
				}
        if(event & SX1276_EVENT_D1)
        {}
        if(event & SX1276_EVENT_TIMEOUT) 
        {}
        if (event & SX1276_EVENT_RX_ERROR)
        {}
        if(event & SX1276_EVENT_RX_DONE)
        {
			sx1276_rx_done_source(dev);
		}
		if (event & SX1276_EVENT_TX_DONE) 
		{
			sx1276_tx_done_source(dev);
		}
				if( event & SX1276_EVENT_TX_TIMEOUT) 
				{
				    sx1276_tx_timeout_source(dev);
				}
				if( event & SX1276_EVENT_RX_TIMEOUT) 
        {
				    sx1276_rx_timeout_source(dev);
				}
        if(event & SX1276_EVENT_SYNC_WORD_TIMEOUT)
				{}					
    }
}


/*****************************************API*******************************************************/

int sx1276_device_init(void)
{
	  //定时器初始化
	  SpecialTimerInit(&SpecialTimer);
		return 0;
}


int sx1276_device_register(const char *dev_name,const char *spi_name,struct sx1276_dio_s dio,struct sx1276_cfg_s* cfg)
{
    struct sx1276_t *dev= NULL;
	  struct sx1276_pcfg_s *pcfg=NULL;
	  struct rt_event * sx1276_event=NULL;
	
    dev = (struct sx1276_t *)rt_malloc(sizeof(struct sx1276_t));
	  pcfg = (struct sx1276_pcfg_s *)rt_malloc(sizeof(struct sx1276_pcfg_s));
	  sx1276_event = (struct rt_event *)rt_malloc(sizeof(struct rt_event));
	  dev->pcfg = pcfg;
	  dev->event = sx1276_event;
	
	  strcpy(dev->bsp.spi_name,spi_name);
      rt_memcpy( &(dev->bsp.dio),&dio,sizeof(struct sx1276_dio_s));
	  if(cfg !=RT_NULL)
      rt_memcpy( &(dev->cfg),cfg,sizeof(struct sx1276_cfg_s));
	  else
	  rt_memcpy( &(dev->cfg),&cfg_default,sizeof(struct sx1276_cfg_s));
	  
    //初始化引脚状态
		rt_pin_mode(dev->bsp.dio.irq0, PIN_MODE_INPUT);
		rt_pin_mode(dev->bsp.dio.irq1, PIN_MODE_INPUT);
		rt_pin_mode(dev->bsp.dio.nss, PIN_MODE_OUTPUT); 
	
    //外部中断回调注册
	  rt_pin_attach_irq(dev->bsp.dio.irq0, PIN_IRQ_MODE_RISING, sx1276_dio0_cb, dev);
      rt_pin_attach_irq(dev->bsp.dio.irq1, PIN_IRQ_MODE_RISING, sx1276_dio1_cb, dev);
  	
		//初始化时钟
		sx1276_device_init();
		//注册三个定时器
		char name1[20] = "rxTime_";
		strcat(name1,dev_name);
		dev->timer.RxTimeoutTimer =  SpecialTimer.Register(name1,dev,sx1276_rx_timeout);
		
		char name2[20] = "txTime_";
		strcat(name2,dev_name);
		dev->timer.TxTimeoutTimer =  SpecialTimer.Register(name2,dev,sx1276_tx_timeout);
		
		char name3[20] = "SyncWord_";
		strcat(name3,dev_name);
		dev->timer.RxTimeoutSyncWord =  SpecialTimer.Register(name3,dev,sx1276_sync_word_timeout);
		
		//复位设备
		sx1276_reg_init(dev);
//		sx1276_reset(dev->bsp.dio.rst);
//		SpecialTimer.DelayMs(10);
//		
//		//DEBUG 读取设备ID，检查是否能正常通讯
//		uint8_t temp_id=sx1276_read( &(dev->bsp),0x42);
//		LOG_D("CHIP ID = %d",temp_id);
//		
//		sx1267_rx_chain_calibration(& (dev->bsp) ,pcfg);
//    sx1276_set_opmode( & (dev->bsp),RF_OPMODE_SLEEP );
//		
//		uint8_t i;
//    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
//    {
//        sx1276_set_opmode( &(dev->bsp),RadioRegsInit[i].Modem );
//        sx1276_write(&(dev->bsp) ,RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
//    }
//		sx1276_set_opmode( &(dev->bsp), MODEM_LORA ); 
//		pcfg->Settings.State = RF_IDLE;
		
		
		//注册设备
		dev->parent.type = RT_Device_Class_SPIDevice;
		dev->parent.rx_indicate = RT_NULL;
		dev->parent.tx_complete = RT_NULL;
#ifdef RT_USING_DEVICE_OPS
    dev->parent.ops       = &sx1276_ops;
#else
    dev->parent.init      = sx1276_device_api_init;
    dev->parent.open      = sx1276_device_api_open;
    dev->parent.close     = sx1276_device_api_close;
    dev->parent.read      = sx1276_device_api_read;
    dev->parent.write     = sx1276_device_api_write;
    dev->parent.control   = sx1276_device_api_control;
#endif
		rt_device_register( &(dev->parent),dev_name,RT_DEVICE_FLAG_RDWR);
		
		//开启进程
		char name4[20] = "thread_";
		strcat(name4,dev_name);
		rt_thread_t sx1276Event_hander;
    sx1276Event_hander = rt_thread_create(name4,sx1276_thread_entry,dev,1024,2,10);
    if (sx1276Event_hander != RT_NULL)
    {
        rt_thread_startup(sx1276Event_hander);
        LOG_D("%s create ok\n",name4);
    }
		else
		{
		    LOG_D("%s create error\n",name4);
		}
		dev->thread_hander = sx1276Event_hander;
		
	  return 0;
}

