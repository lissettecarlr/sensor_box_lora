#ifndef __SX1276_H__
#define __SX1276_H__

#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"
#include "radio.h"
#include <rtthread.h>
#include "rtdevice.h"

//外部需要实现的定时器接口
typedef uint32_t Sx1276TimerTime_t;
typedef struct Sx1276TimerEvent_s
{
   char name[10];
	 rt_timer_t timeHander;
	 uint32_t  ReloadValue;
   void ( *Callback )( void* parameter ); 
}Sx1276TimerEvent_t;


extern void Sx1276TimerInit( Sx1276TimerEvent_t *obj, void ( *callback )( void* parameter ) );
extern void Sx1276TimerStart( Sx1276TimerEvent_t *obj );
extern void Sx1276TimerStop( Sx1276TimerEvent_t *obj ) ;
extern void Sx1276TimerReset( Sx1276TimerEvent_t *obj );
extern void Sx1276TimerSetValue( Sx1276TimerEvent_t *obj, uint32_t value );
extern Sx1276TimerTime_t Sx1276TimerGetCurrentTime( void );
extern Sx1276TimerTime_t Sx1276TimerGetElapsedTime( Sx1276TimerTime_t past );
extern void Sx1276DelayMs( uint32_t ms );
//待匹配引脚中断函数
void SX1276OnDio0Irq_shell( void* parameter );
void SX1276OnDio1Irq_shell( void* parameter );

void SX1276SetHardwareInterface(uint32_t cs,uint32_t rst,struct rt_spi_device *spi);

void SX1276Reset( void );
void Sx1276Restart(void);
//radio 函数
uint32_t SX1276Init( RadioEvents_t *events  );
RadioState_t SX1276GetStatus( void );
void SX1276SetModem( RadioModems_t modem );
void SX1276SetChannel( uint32_t freq );
bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime );
uint32_t SX1276Random( void );
void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous );
void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout );
bool SX1276CheckRfFrequency( uint32_t frequency );
uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen );                 
void SX1276Send( uint8_t *buffer, uint8_t size );
void SX1276SetSleep( void );
void SX1276SetStby( void );
void SX1276SetRx( uint32_t timeout );
void SX1276StartCad( void );
void SX1276SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );
int16_t SX1276ReadRssi( RadioModems_t modem );
void SX1276Write( uint16_t addr, uint8_t data );
uint8_t SX1276Read( uint16_t addr );
void SX1276ReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size );
void SX1276WriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size );
void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max );
void SX1276SetPublicNetwork( bool enable );
uint32_t SX1276GetWakeupTime( void );




#define FREQ_STEP                                   61.03515625
#define RF_MID_BAND_THRESH                          525000000
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12



typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
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
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    bool     PublicNetwork;
}RadioLoRaSettings_t;

typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_t;
/*!
 * Radio Settings
 */
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

typedef struct SX1276_s
{
    struct rt_spi_device *spi_hander;
    uint32_t pin_cs;
    uint32_t pin_rst;
    RadioSettings_t Settings;
}SX1276_t;



#endif /* __SX1276_H__ */
