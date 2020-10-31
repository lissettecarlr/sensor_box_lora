#include "LoRaMac.h"
#include "LoRaMacCrypto.h"
#include "LoRaMacTest.h"
#include "radio.h"
#include "math.h"

#define LORAWAN_CORE_EVENT_ALL             0xFFFFFFFF
#define LORAWAN_CORE_EVENT_STATE_CHECK     (uint32_t)1<<0  
#define LORAWAN_CORE_EVENT_WIN1_TIMEOUT    (uint32_t)1<<1
#define LORAWAN_CORE_EVENT_WIN2_TIMEOUT    (uint32_t)1<<2
#define LORAWAN_CORE_EVENT_ACK_TIMEOUT     (uint32_t)1<<3
#define LORAWAN_CORE_EVENT_TXDELAY_TIMEOUT (uint32_t)1<<4 


/**************************************************************************************************/
#ifdef RT_USING_ULOG

#define LOG_TAG              "lorawan"
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

//新添加标注位，表示是否入网,0表示为入网 1表示已入网，入网后就不再打开接收窗口
uint8_t lorawan_join_flag=0;

//lorawan协议内部事件
static struct rt_event lorawan_core_event;
void lorawan_core_thread_entry(void *p);
/*!
 * Maximum PHY layer payload size
 */
#define LORAMAC_PHY_MAXPAYLOAD                      255

/*!
 * Maximum MAC commands buffer size
 */
#define LORA_MAC_COMMAND_MAX_LENGTH                 15

/*!
 * FRMPayload overhead to be used when setting the Radio.SetMaxPayloadLength
 * in RxWindowSetup function.
 * Maximum PHYPayload = MaxPayloadOfDatarate/MaxPayloadOfDatarateRepeater + LORA_MAC_FRMPAYLOAD_OVERHEAD
 */
#define LORA_MAC_FRMPAYLOAD_OVERHEAD                13 // MHDR(1) + FHDR(7) + Port(1) + MIC(4)

/*!
 * LoRaMac duty cycle for the back-off procedure during the first hour.
 */
#define BACKOFF_DC_1_HOUR                           20

/*!
 * LoRaMac duty cycle for the back-off procedure during the next 10 hours.
 */
#define BACKOFF_DC_10_HOURS                         200

/*!
 * LoRaMac duty cycle for the back-off procedure during the next 24 hours.
 */
#define BACKOFF_DC_24_HOURS                         2000

/*!
 * Device IEEE EUI
 */
static uint8_t *LoRaMacDevEui;

/*!
 * Application IEEE EUI
 */
static uint8_t *LoRaMacAppEui;

/*!
 * AES encryption/decryption cipher application key
 */
static uint8_t *LoRaMacAppKey;

/*!
 * AES encryption/decryption cipher network session key
 */
static uint8_t LoRaMacNwkSKey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*!
 * AES encryption/decryption cipher application session key
 */
static uint8_t LoRaMacAppSKey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*!
 * Device nonce is a random value extracted by issuing a sequence of RSSI
 * measurements
 */
static uint16_t LoRaMacDevNonce;

/*!
 * Network ID ( 3 bytes )
 */
static uint32_t LoRaMacNetID;

/*!
 * Mote Address
 */
static uint32_t LoRaMacDevAddr;

/*!
 * Multicast channels linked list
 */
static MulticastParams_t *MulticastChannels = NULL;

/*!
 * Actual device class
 */
static DeviceClass_t LoRaMacDeviceClass;

/*!
 * Indicates if the node is connected to a private or public network
 */
static bool PublicNetwork;

/*!
 * Indicates if the node supports repeaters
 */
static bool RepeaterSupport;

/*!
 * Buffer containing the data to be sent or received.
 */
static uint8_t LoRaMacBuffer[LORAMAC_PHY_MAXPAYLOAD];

/*!
 * Length of packet in LoRaMacBuffer
 */
static uint16_t LoRaMacBufferPktLen = 0;

/*!
 * Length of the payload in LoRaMacBuffer
 */
static uint8_t LoRaMacTxPayloadLen = 0;

/*!
 * Buffer containing the upper layer data.
 */
static uint8_t LoRaMacRxPayload[LORAMAC_PHY_MAXPAYLOAD];

/*!
 * LoRaMAC frame counter. Each time a packet is sent the counter is incremented.
 * Only the 16 LSB bits are sent上行计数，服务器累计OTAA入网后的包号，不可低于其值，否则会被抛弃
 */
static uint32_t UpLinkCounter = 0;

/*!
 * LoRaMAC frame counter. Each time a packet is received the counter is incremented.
 * Only the 16 LSB bits are received
 */
static uint32_t DownLinkCounter = 0;

/*!
 * IsPacketCounterFixed enables the MIC field tests by fixing the
 * UpLinkCounter value
 */
static bool IsUpLinkCounterFixed = false;

/*!
 * Used for test purposes. Disables the opening of the reception windows.
 */
static bool IsRxWindowsEnabled = true;

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static bool IsLoRaMacNetworkJoined = false;

/*!
 * LoRaMac ADR control status
 */
static bool AdrCtrlOn = false;

/*!
 * Counts the number of missed ADR acknowledgements
 */
static uint32_t AdrAckCounter = 0;

/*!
 * If the node has sent a FRAME_TYPE_DATA_CONFIRMED_UP this variable indicates
 * if the nodes needs to manage the server acknowledgement.
 */
static bool NodeAckRequested = false;

/*!
 * If the server has sent a FRAME_TYPE_DATA_CONFIRMED_DOWN this variable indicates
 * if the ACK bit must be set for the next transmission
 */
static bool SrvAckRequested = false;

/*!
 * Indicates if the MAC layer wants to send MAC commands
 */
static bool MacCommandsInNextTx = false;

/*!
 * Contains the current MacCommandsBuffer index
 */
static uint8_t MacCommandsBufferIndex = 0;

/*!
 * Contains the current MacCommandsBuffer index for MAC commands to repeat
 */
static uint8_t MacCommandsBufferToRepeatIndex = 0;

/*!
 * Buffer containing the MAC layer commands
 */
static uint8_t MacCommandsBuffer[LORA_MAC_COMMAND_MAX_LENGTH];

/*!
 * Buffer containing the MAC layer commands which must be repeated
 */
static uint8_t MacCommandsBufferToRepeat[LORA_MAC_COMMAND_MAX_LENGTH];




/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7 };

/*!
 * Bandwidths table definition in Hz
 */
const uint32_t Bandwidths[] = { 125e3, 125e3, 125e3, 125e3, 125e3, 125e3 };

/*!
 * Maximum payload with respect to the datarate index. Cannot operate with repeater.
 */
const uint8_t MaxPayloadOfDatarate[] = { 51, 51, 51, 115, 222, 222 };

/*!
 * Maximum payload with respect to the datarate index. Can operate with repeater.
 */
const uint8_t MaxPayloadOfDatarateRepeater[] = { 51, 51, 51, 115, 222, 222 };

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 17, 16, 14, 12, 10, 7, 5, 2 };

/*!
 * LoRaMac bands
 */
static Band_t Bands[LORA_MAX_NB_BANDS] =
{
    BAND0,
};

/*!
 * LoRaMAC channels
 */
static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS];

/*!
 * Defines the first channel for RX window 1 for CN470 band
 */
#define LORAMAC_FIRST_RX1_CHANNEL           ( (uint32_t) 500.3e6 )

/*!
 * Defines the last channel for RX window 1 for CN470 band
 */
#define LORAMAC_LAST_RX1_CHANNEL            ( (uint32_t) 509.7e6 )

/*!
 * Defines the step width of the channels for RX window 1
 */
#define LORAMAC_STEPWIDTH_RX1_CHANNEL       ( (uint32_t) 200e3 )


/*!
 * LoRaMac parameters
 */
LoRaMacParams_t LoRaMacParams;

/*!
 * LoRaMac default parameters
 */
LoRaMacParams_t LoRaMacParamsDefaults;

/*!
 * Uplink messages repetitions counter
 */
static uint8_t ChannelsNbRepCounter = 0;

/*!
 * Maximum duty cycle
 * \remark Possibility to shutdown the device.
 */
static uint8_t MaxDCycle = 0;

/*!
 * Aggregated duty cycle management
 */
static uint16_t AggregatedDCycle;
static TimerTime_t AggregatedLastTxDoneTime;
static TimerTime_t AggregatedTimeOff;

/*!
 * Enables/Disables duty cycle management (Test only)
 */
static bool DutyCycleOn;

/*!
 * Current channel index
 */
static uint8_t Channel;

/*!
 * Stores the time at LoRaMac initialization.
 *
 * \remark Used for the BACKOFF_DC computation.
 */
static TimerTime_t LoRaMacInitializationTime = 0;

/*!
 * LoRaMac internal states
 */
enum eLoRaMacState
{
    LORAMAC_IDLE          = 0x00000000,
    LORAMAC_TX_RUNNING    = 0x00000001,
    LORAMAC_RX            = 0x00000002,
    LORAMAC_ACK_REQ       = 0x00000004,
    LORAMAC_ACK_RETRY     = 0x00000008,
    LORAMAC_TX_DELAYED    = 0x00000010,
    LORAMAC_TX_CONFIG     = 0x00000020,
    LORAMAC_RX_ABORT      = 0x00000040,
};

/*!
 * LoRaMac internal state
 */
uint32_t LoRaMacState = LORAMAC_IDLE;

/*!
 * LoRaMac timer used to check the LoRaMacState (runs every second)
 */
static TimerEvent_t MacStateCheckTimer;

/*!
 * LoRaMac upper layer event functions
 */
static LoRaMacPrimitives_t *LoRaMacPrimitives;

/*!
 * LoRaMac upper layer callback functions
 */
static LoRaMacCallback_t *LoRaMacCallbacks;

/*!
 * LoRaMac duty cycle delayed Tx timer
 */
static TimerEvent_t TxDelayedTimer;

/*!
 * LoRaMac reception windows timers
 */
static TimerEvent_t RxWindowTimer1;
static TimerEvent_t RxWindowTimer2;

/*!
 * LoRaMac reception windows delay
 * \remark normal frame: RxWindowXDelay = ReceiveDelayX - RADIO_WAKEUP_TIME
 *         join frame  : RxWindowXDelay = JoinAcceptDelayX - RADIO_WAKEUP_TIME
 */
static uint32_t RxWindow1Delay;
static uint32_t RxWindow2Delay;

/*!
 * Rx window parameters
 */
typedef struct
{
    int8_t Datarate;
    uint8_t Bandwidth;
    uint32_t RxWindowTimeout;
    int32_t RxOffset;
}RxConfigParams_t;

/*!
 * Rx windows params
 */
static RxConfigParams_t RxWindowsParams[2];

/*!
 * Acknowledge timeout timer. Used for packet retransmissions.
 */
static TimerEvent_t AckTimeoutTimer;

/*!
 * Number of trials to get a frame acknowledged
 */
static uint8_t AckTimeoutRetries = 1;

/*!
 * Number of trials to get a frame acknowledged
 */
static uint8_t AckTimeoutRetriesCounter = 1;

/*!
 * Indicates if the AckTimeout timer has expired or not
 */
static bool AckTimeoutRetry = false;

/*!
 * Last transmission time on air
 */
TimerTime_t TxTimeOnAir = 0;

/*!
 * Number of trials for the Join Request
 */
static uint8_t JoinRequestTrials;

/*!
 * Maximum number of trials for the Join Request
 */
static uint8_t MaxJoinRequestTrials;

/*!
 * Structure to hold an MCPS indication data.
 */
static McpsIndication_t McpsIndication;

/*!
 * Structure to hold MCPS confirm data.
 */
static McpsConfirm_t McpsConfirm;

/*!
 * Structure to hold MLME confirm data.
 */
static MlmeConfirm_t MlmeConfirm;

/*!
 * Holds the current rx window slot
 */
static uint8_t RxSlot = 0;

/*!
 * LoRaMac tx/rx operation state
 */
LoRaMacFlags_t LoRaMacFlags;

static bool LoraMacTestFlag = false;
static uint32_t DownLinkTotal = 0;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
//static void OnRadioTxDone( void );

/*!
 * \brief This function prepares the MAC to abort the execution of function
 *        OnRadioRxDone in case of a reception error.
 */
//static void PrepareRxDoneAbort( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
//static void OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
//static void OnRadioTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx error event
 */
//static void OnRadioRxError( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
//static void OnRadioRxTimeout( void );

/*!
 * \brief Function executed on Resend Frame timer event.
 */ 
static void OnMacStateCheckTimerEvent(void);
static void OnMacStateCheckTimerEvent_shell(void *p)
{
    rt_event_send(&lorawan_core_event,LORAWAN_CORE_EVENT_STATE_CHECK);
}

/*!
 * \brief Function executed on duty cycle delayed Tx  timer event
 */
static void OnTxDelayedTimerEvent(void);
static void OnTxDelayedTimerEvent_shell(void *p)
{
   rt_event_send(&lorawan_core_event,LORAWAN_CORE_EVENT_TXDELAY_TIMEOUT); 
}

/*!
 * \brief Function executed on first Rx window timer event
 */
static void OnRxWindow1TimerEvent(void);
static void OnRxWindow1TimerEvent_shell(void *p)
{
    rt_event_send(&lorawan_core_event,LORAWAN_CORE_EVENT_WIN1_TIMEOUT); 
}

/*!
 * \brief Function executed on second Rx window timer event
 */
static void OnRxWindow2TimerEvent(void);
static void OnRxWindow2TimerEvent_shell(void *p)
{
     rt_event_send(&lorawan_core_event,LORAWAN_CORE_EVENT_WIN2_TIMEOUT); 
}

/*!
 * \brief Function executed on AckTimeout timer event
 */
static void OnAckTimeoutTimerEvent(void);
static void OnAckTimeoutTimerEvent_shell(void *p)
{
   rt_event_send(&lorawan_core_event,LORAWAN_CORE_EVENT_ACK_TIMEOUT); 
}

/*!
 * \brief Searches and set the next random available channel
 *
 * \param [OUT] Time to wait for the next transmission according to the duty
 *              cycle.
 *
 * \retval status  Function status [1: OK, 0: Unable to find a channel on the
 *                                  current datarate]
 */
static bool SetNextChannel( TimerTime_t* time );

/*!
 * \brief Initializes and opens the reception window
 *
 * \param [IN] freq window channel frequency
 * \param [IN] datarate window channel datarate
 * \param [IN] bandwidth window channel bandwidth
 * \param [IN] timeout window channel timeout
 *
 * \retval status Operation status [true: Success, false: Fail]
 */
static bool RxWindowSetup( uint32_t freq, int8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous );

/*!
 * \brief Verifies if the RX window 2 frequency is in range
 *
 * \param [IN] freq window channel frequency
 *
 * \retval status  Function status [1: OK, 0: Frequency not applicable]
 */
static bool Rx2FreqInRange( uint32_t freq );

/*!
 * \brief Parses the MAC commands which must be repeated.
 *
 * \Remark MAC layer internal function
 *
 * \param [IN] cmdBufIn  Buffer which stores the MAC commands to send
 * \param [IN] length  Length of the input buffer to parse
 * \param [OUT] cmdBufOut  Buffer which stores the MAC commands which must be
 *                         repeated.
 *
 * \retval Size of the MAC commands to repeat.
 */
static uint8_t ParseMacCommandsToRepeat( uint8_t* cmdBufIn, uint8_t length, uint8_t* cmdBufOut );

/*!
 * \brief Validates if the payload fits into the frame, taking the datarate
 *        into account.
 *
 * \details Refer to chapter 4.3.2 of the LoRaWAN specification, v1.0
 *
 * \param lenN Length of the application payload. The length depends on the
 *             datarate and is region specific
 *
 * \param datarate Current datarate
 *
 * \param fOptsLen Length of the fOpts field
 *
 * \retval [false: payload does not fit into the frame, true: payload fits into
 *          the frame]
 */
static bool ValidatePayloadLength( uint8_t lenN, int8_t datarate, uint8_t fOptsLen );

/*!
 * \brief Counts the number of bits in a mask.
 *
 * \param [IN] mask A mask from which the function counts the active bits.
 * \param [IN] nbBits The number of bits to check.
 *
 * \retval Number of enabled bits in the mask.
 */
static uint8_t CountBits( uint16_t mask, uint8_t nbBits );

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
/*!
 * \brief Counts the number of enabled 125 kHz channels in the channel mask.
 *        This function can only be applied to US915 band.
 *
 * \param [IN] channelsMask Pointer to the first element of the channel mask
 *
 * \retval Number of enabled channels in the channel mask
 */
static uint8_t CountNbEnabled125kHzChannels( uint16_t *channelsMask );

#if defined( USE_BAND_915_HYBRID )
/*!
 * \brief Validates the correctness of the channel mask for US915, hybrid mode.
 *
 * \param [IN] mask Block definition to set.
 * \param [OUT] channelsMask Pointer to the first element of the channel mask
 */
static void ReenableChannels( uint16_t mask, uint16_t* channelsMask );

/*!
 * \brief Validates the correctness of the channel mask for US915, hybrid mode.
 *
 * \param [IN] channelsMask Pointer to the first element of the channel mask
 *
 * \retval [true: channel mask correct, false: channel mask not correct]
 */
static bool ValidateChannelMask( uint16_t* channelsMask );
#endif

#endif

/*!
 * \brief Validates the correctness of the datarate against the enable channels.
 *
 * \param [IN] datarate Datarate to be check
 * \param [IN] channelsMask Pointer to the first element of the channel mask
 *
 * \retval [true: datarate can be used, false: datarate can not be used]
 */
static bool ValidateDatarate( int8_t datarate, uint16_t* channelsMask );

/*!
 * \brief Limits the Tx power according to the number of enabled channels
 *
 * \param [IN] txPower txPower to limit
 * \param [IN] maxBandTxPower Maximum band allowed TxPower
 *
 * \retval Returns the maximum valid tx power
 */
static int8_t LimitTxPower( int8_t txPower, int8_t maxBandTxPower );

/*!
 * \brief Verifies, if a value is in a given range.
 *
 * \param value Value to verify, if it is in range
 *
 * \param min Minimum possible value
 *
 * \param max Maximum possible value
 *
 * \retval Returns the maximum valid tx power
 */
static bool ValueInRange( int8_t value, int8_t min, int8_t max );

/*!
 * \brief Calculates the next datarate to set, when ADR is on or off
 *
 * \param [IN] adrEnabled Specify whether ADR is on or off
 *
 * \param [IN] updateChannelMask Set to true, if the channel masks shall be updated
 *
 * \param [OUT] datarateOut Reports the datarate which will be used next
 *
 * \retval Returns the state of ADR ack request
 */
static bool AdrNextDr( bool adrEnabled, bool updateChannelMask, int8_t* datarateOut );

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 ) || defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
/*!
 * \brief Disables channel in a specified channel mask
 *
 * \param [IN] id - Id of the channel
 *
 * \param [IN] mask - Pointer to the channel mask to edit
 *
 * \retval [true, if disable was successful, false if not]
 */
static bool DisableChannelInMask( uint8_t id, uint16_t* mask );
#endif

/*!
 * \brief Decodes MAC commands in the fOpts field and in the payload
 */
static void ProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, uint8_t snr );

/*!
 * \brief LoRaMAC layer generic send frame
 *
 * \param [IN] macHdr      MAC header field
 * \param [IN] fPort       MAC payload port
 * \param [IN] fBuffer     MAC data buffer to be sent
 * \param [IN] fBufferSize MAC data buffer size
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );

/*!
 * \brief LoRaMAC layer frame buffer initialization
 *
 * \param [IN] macHdr      MAC header field
 * \param [IN] fCtrl       MAC frame control field
 * \param [IN] fOpts       MAC commands buffer
 * \param [IN] fPort       MAC payload port
 * \param [IN] fBuffer     MAC data buffer to be sent
 * \param [IN] fBufferSize MAC data buffer size
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );

/*
 * \brief Schedules the frame according to the duty cycle
 *
 * \retval Status of the operation
 */
static LoRaMacStatus_t ScheduleTx( void );

/*
 * \brief Sets the duty cycle for the join procedure.
 *
 * \retval Duty cycle
 */
static uint16_t JoinDutyCycle( void );

/*
 * \brief Calculates the back-off time for the band of a channel.
 *
 * \param [IN] channel     The last Tx channel index
 */
static void CalculateBackOff( uint8_t channel );

/*
 * \brief Alternates the datarate of the channel for the join request.
 *
 * \param [IN] nbTrials    Number of performed join requests.
 * \retval Datarate to apply
 */
static int8_t AlternateDatarate( uint16_t nbTrials );

/*!
 * \brief LoRaMAC layer prepared frame buffer transmission with channel specification
 *
 * \remark PrepareFrame must be called at least once before calling this
 *         function.
 *
 * \param [IN] channel     Channel parameters
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t SendFrameOnChannel( ChannelParams_t channel );

/*!
 * \brief Sets the radio in continuous transmission mode
 *
 * \remark Uses the radio parameters set on the previous transmission.
 *
 * \param [IN] timeout     Time in seconds while the radio is kept in continuous wave mode
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t SetTxContinuousWave( uint16_t timeout );

/*!
 * \brief Sets the radio in continuous transmission mode
 *
 * \remark Uses the radio parameters set on the previous transmission.
 *
 * \param [IN] timeout     Time in seconds while the radio is kept in continuous wave mode
 * \param [IN] frequency   RF frequency to be set.
 * \param [IN] power       RF ouptput power to be set.
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t SetTxContinuousWave1( uint16_t timeout, uint32_t frequency, uint8_t power );

/*!
 * \brief Resets MAC specific parameters to default
 */
static void ResetMacParameters( void );

/*
 * Rx window precise timing
 *
 * For more details please consult the following document, chapter 3.1.2.
 * http://www.semtech.com/images/datasheet/SX1272_settings_for_LoRaWAN_v2.0.pdf
 * or
 * http://www.semtech.com/images/datasheet/SX1276_settings_for_LoRaWAN_v2.0.pdf
 *
 *                 Downlink start: T = Tx + 1s (+/- 20 us)
 *                            |
 *             TRxEarly       |        TRxLate
 *                |           |           |
 *                |           |           +---+---+---+---+---+---+---+---+
 *                |           |           |       Latest Rx window        |
 *                |           |           +---+---+---+---+---+---+---+---+
 *                |           |           |
 *                +---+---+---+---+---+---+---+---+
 *                |       Earliest Rx window      |
 *                +---+---+---+---+---+---+---+---+
 *                            |
 *                            +---+---+---+---+---+---+---+---+
 *Downlink preamble 8 symbols |   |   |   |   |   |   |   |   |
 *                            +---+---+---+---+---+---+---+---+
 *
 *                     Worst case Rx window timings
 *
 * TRxLate  = DEFAULT_MIN_RX_SYMBOLS * tSymbol - RADIO_WAKEUP_TIME
 * TRxEarly = 8 - DEFAULT_MIN_RX_SYMBOLS * tSymbol - RxWindowTimeout - RADIO_WAKEUP_TIME
 *
 * TRxLate - TRxEarly = 2 * DEFAULT_SYSTEM_MAX_RX_ERROR
 *
 * RxOffset = ( TRxLate + TRxEarly ) / 2
 *
 * RxWindowTimeout = ( 2 * DEFAULT_MIN_RX_SYMBOLS - 8 ) * tSymbol + 2 * DEFAULT_SYSTEM_MAX_RX_ERROR
 * RxOffset = 4 * tSymbol - RxWindowTimeout / 2 - RADIO_WAKE_UP_TIME
 *
 * Minimal value of RxWindowTimeout must be 5 symbols which implies that the system always tolerates at least an error of 1.5 * tSymbol
 */
/*!
 * Computes the Rx window parameters.
 *
 * \param [IN] datarate     Rx window datarate to be used
 * \param [IN] rxError      Maximum timing error of the receiver. in milliseconds
 *                          The receiver will turn on in a [-rxError : +rxError] ms
 *                          interval around RxOffset
 *
 * \retval rxConfigParams   Returns a RxConfigParams_t structure.
 */
static RxConfigParams_t ComputeRxWindowParameters( int8_t datarate, uint32_t rxError );

//static void OnRadioTxDone( void )
void OnRadioTxDone( void )
{
    TimerTime_t curTime = TimerGetCurrentTime( );

    LOG_D(" Send Frame Done ");

    if( LoRaMacDeviceClass != CLASS_C )
    {
        Radio.Sleep( );
    }
    else
    {
        OnRxWindow2TimerEvent( );
    }

    // Setup timers
    if( IsRxWindowsEnabled == true )
    {
        TimerSetValue( &RxWindowTimer1, RxWindow1Delay );
        TimerStart( &RxWindowTimer1 );
        if( LoRaMacDeviceClass != CLASS_C )
        {
            TimerSetValue( &RxWindowTimer2, RxWindow2Delay );
            TimerStart( &RxWindowTimer2 );
        }
        if( ( LoRaMacDeviceClass == CLASS_C ) || ( NodeAckRequested == true ) )
        {
            TimerSetValue( &AckTimeoutTimer, RxWindow2Delay + ACK_TIMEOUT +
                                             randr( -ACK_TIMEOUT_RND, ACK_TIMEOUT_RND ) );
            TimerStart( &AckTimeoutTimer );
        }
    }
    else
    {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;

        if( LoRaMacFlags.Value == 0 )
        {
            LoRaMacFlags.Bits.McpsReq = 1;
        }
        LoRaMacFlags.Bits.MacDone = 1;
    }

    // Update last tx done time for the current channel
    Bands[Channels[Channel].Band].LastTxDoneTime = curTime;
    // Update Aggregated last tx done time
    AggregatedLastTxDoneTime = curTime;
    // Update Backoff
    CalculateBackOff( Channel );

    if( NodeAckRequested == false )
    {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
        ChannelsNbRepCounter++;
    }
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//LoRaMacState &= ~LORAMAC_TX_RUNNING;
}

static void PrepareRxDoneAbort( void )
{
    LoRaMacState |= LORAMAC_RX_ABORT;

    if( NodeAckRequested )
    {
        OnAckTimeoutTimerEvent( );
    }

    LoRaMacFlags.Bits.McpsInd = 1;
    LoRaMacFlags.Bits.MacDone = 1;

    // Trig OnMacCheckTimerEvent call as soon as possible
    TimerSetValue( &MacStateCheckTimer, 1 );
    TimerStart( &MacStateCheckTimer );
}

//static void OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
void OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    bool skipIndication = false;

    uint8_t pktHeaderLen = 0;
    uint32_t address = 0;
    uint8_t appPayloadStartIndex = 0;
    uint8_t port = 0xFF;
    uint8_t frameLen = 0;
    uint32_t mic = 0;
    uint32_t micRx = 0;

    uint16_t sequenceCounter = 0;
    uint16_t sequenceCounterPrev = 0;
    uint16_t sequenceCounterDiff = 0;
    uint32_t downLinkCounter = 0;

    MulticastParams_t *curMulticastParams = NULL;
    uint8_t *nwkSKey = LoRaMacNwkSKey;
    uint8_t *appSKey = LoRaMacAppSKey;

    uint8_t multicast = 0;
    
//    uint8_t i;

    bool isMicOk = false;

    McpsConfirm.AckReceived = false;
    McpsIndication.Rssi = rssi;
    McpsIndication.Snr = snr;
    McpsIndication.RxSlot = RxSlot;
    McpsIndication.Port = 0;
    McpsIndication.Multicast = 0;
    McpsIndication.FramePending = 0;
    McpsIndication.Buffer = NULL;
    McpsIndication.BufferSize = 0;
    McpsIndication.RxData = false;
    McpsIndication.AckReceived = false;
    McpsIndication.DownLinkCounter = 0;
    McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

    Radio.Sleep( );
    TimerStop( &RxWindowTimer2 );
    
//    rt_kprintf(" \r\nRecvData<<-[ RSSI:%d SNR:%d SIZE:%d]", rssi, snr, size);
//    for( i = 0; i < size; i++ )
//    {
//        rt_kprintf(" %02X", payload[i]);
//    }
//    rt_kprintf(" ]\r\n");

    macHdr.Value = payload[pktHeaderLen++];

    switch( macHdr.Bits.MType )
    {
        case FRAME_TYPE_JOIN_ACCEPT:
            if( IsLoRaMacNetworkJoined == true )
            {
                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ERROR;
                PrepareRxDoneAbort( );
                return;
            }
            LoRaMacJoinDecrypt( payload + 1, size - 1, LoRaMacAppKey, LoRaMacRxPayload + 1 );

            LoRaMacRxPayload[0] = macHdr.Value;

            LoRaMacJoinComputeMic( LoRaMacRxPayload, size - LORAMAC_MFR_LEN, LoRaMacAppKey, &mic );

            micRx |= ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN];
            micRx |= ( ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 1] << 8 );
            micRx |= ( ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 2] << 16 );
            micRx |= ( ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 3] << 24 );

            if( micRx == mic )
            {
                uint32_t LoRaMacAppNonce;
                
                LoRaMacJoinComputeSKeys( LoRaMacAppKey, LoRaMacRxPayload + 1, LoRaMacDevNonce, LoRaMacNwkSKey, LoRaMacAppSKey );

                LoRaMacNetID = ( uint32_t )LoRaMacRxPayload[4];
                LoRaMacNetID |= ( ( uint32_t )LoRaMacRxPayload[5] << 8 );
                LoRaMacNetID |= ( ( uint32_t )LoRaMacRxPayload[6] << 16 );

                LoRaMacDevAddr = ( uint32_t )LoRaMacRxPayload[7];
                LoRaMacDevAddr |= ( ( uint32_t )LoRaMacRxPayload[8] << 8 );
                LoRaMacDevAddr |= ( ( uint32_t )LoRaMacRxPayload[9] << 16 );
                LoRaMacDevAddr |= ( ( uint32_t )LoRaMacRxPayload[10] << 24 );
                
                LoRaMacAppNonce = ( uint32_t )LoRaMacRxPayload[1];
                LoRaMacAppNonce |= ( ( uint32_t )LoRaMacRxPayload[2] << 8 );
                LoRaMacAppNonce |= ( ( uint32_t )LoRaMacRxPayload[3] << 16 );
#if 1
                rt_kprintf(" AppNonce:%06X \r\n", LoRaMacAppNonce);
                rt_kprintf(" DevNonce:%04X \r\n", LoRaMacDevNonce);
                rt_kprintf(" NetID:%06X \r\n", LoRaMacNetID);
                rt_kprintf(" DevAddr:%08X \r\n", LoRaMacDevAddr);
                rt_kprintf( " NwkSKey:" );
                for(int i = 0; i < 16; i++ )
                {
                    rt_kprintf( "%02X", LoRaMacNwkSKey[i] );
                }
                rt_kprintf( "\r\n" );
                rt_kprintf( " AppSKey:" );
                for(int i = 0; i < 16; i++ )
                {
                    rt_kprintf( "%02X", LoRaMacAppSKey[i] );
                }
                rt_kprintf( "\r\n" );
#endif
								//这里添加一个flag，当入网之后就关闭接收窗口
								lorawan_join_flag = 1; 
								
                // DLSettings
                LoRaMacParams.Rx1DrOffset = ( LoRaMacRxPayload[11] >> 4 ) & 0x07;
                LoRaMacParams.Rx2Channel.Datarate = LoRaMacRxPayload[11] & 0x0F;

                // RxDelay
                LoRaMacParams.ReceiveDelay1 = ( LoRaMacRxPayload[12] & 0x0F );
                if( LoRaMacParams.ReceiveDelay1 == 0 )
                {
                    LoRaMacParams.ReceiveDelay1 = 1;
                }
                LoRaMacParams.ReceiveDelay1 *= 1e3;
                LoRaMacParams.ReceiveDelay2 = LoRaMacParams.ReceiveDelay1 + 1e3;


                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                IsLoRaMacNetworkJoined = true;
                LoRaMacParams.ChannelsDatarate = LoRaMacParamsDefaults.ChannelsDatarate;
                
                //LedRadioRxJoinAcceptShow();
            }
            else
            {
                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL;
                LOG_D( " Rx Mic Error!" );
            }
            break;
        case FRAME_TYPE_DATA_CONFIRMED_DOWN:
        case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
            {
                address = payload[pktHeaderLen++];
                address |= ( (uint32_t)payload[pktHeaderLen++] << 8 );
                address |= ( (uint32_t)payload[pktHeaderLen++] << 16 );
                address |= ( (uint32_t)payload[pktHeaderLen++] << 24 );

                if( address != LoRaMacDevAddr )
                {
                    curMulticastParams = MulticastChannels;
                    while( curMulticastParams != NULL )
                    {
                        if( address == curMulticastParams->Address )
                        {
                            multicast = 1;
                            nwkSKey = curMulticastParams->NwkSKey;
                            appSKey = curMulticastParams->AppSKey;
                            downLinkCounter = curMulticastParams->DownLinkCounter;
                            break;
                        }
                        curMulticastParams = curMulticastParams->Next;
                    }
                    if( multicast == 0 )
                    {
                        // We are not the destination of this frame.
                        McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL;
                        PrepareRxDoneAbort( );
                        return;
                    }
                }
                else
                {
                    multicast = 0;
                    nwkSKey = LoRaMacNwkSKey;
                    appSKey = LoRaMacAppSKey;
                    downLinkCounter = DownLinkCounter;
                }

                fCtrl.Value = payload[pktHeaderLen++];

                sequenceCounter = ( uint16_t )payload[pktHeaderLen++];
                sequenceCounter |= ( uint16_t )payload[pktHeaderLen++] << 8;

                appPayloadStartIndex = 8 + fCtrl.Bits.FOptsLen;

                micRx |= ( uint32_t )payload[size - LORAMAC_MFR_LEN];
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 1] << 8 );
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 2] << 16 );
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 3] << 24 );

                sequenceCounterPrev = ( uint16_t )downLinkCounter;
                sequenceCounterDiff = ( sequenceCounter - sequenceCounterPrev );

                if( sequenceCounterDiff < ( 1 << 15 ) )
                {
                    downLinkCounter += sequenceCounterDiff;
                    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounter, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                    }
                }
                else
                {
                    // check for sequence roll-over
                    uint32_t  downLinkCounterTmp = downLinkCounter + 0x10000 + ( int16_t )sequenceCounterDiff;
                    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounterTmp, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                        downLinkCounter = downLinkCounterTmp;
                    }
                }

                // Check for a the maximum allowed counter difference
                if( sequenceCounterDiff >= MAX_FCNT_GAP )
                {
                    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS;
                    McpsIndication.DownLinkCounter = downLinkCounter;
                    PrepareRxDoneAbort( );
                    return;
                }

                if( isMicOk == true )
                {
                   // LedRadioRxDataShow();
                    
                    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                    McpsIndication.Multicast = multicast;
                    McpsIndication.FramePending = fCtrl.Bits.FPending;
                    McpsIndication.Buffer = NULL;
                    McpsIndication.BufferSize = 0;
                    McpsIndication.DownLinkCounter = downLinkCounter;

                    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;

                    AdrAckCounter = 0;
                    MacCommandsBufferToRepeatIndex = 0;

                    // Update 32 bits downlink counter
                    if( multicast == 1 )
                    {
                        McpsIndication.McpsIndication = MCPS_MULTICAST;

                        if( ( curMulticastParams->DownLinkCounter == downLinkCounter ) &&
                            ( curMulticastParams->DownLinkCounter != 0 ) )
                        {
                            McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED;
                            McpsIndication.DownLinkCounter = downLinkCounter;
                            PrepareRxDoneAbort( );
                            return;
                        }
                        curMulticastParams->DownLinkCounter = downLinkCounter;
                        if( LoraMacTestFlag == true )
                        {
                            curMulticastParams->DownLinkTotal++;
                            LOG_D( " Multicast need %d packs,Rx %d packs,loss %.2f%% \r\n", curMulticastParams->DownLinkCounter + 1, curMulticastParams->DownLinkTotal,
                                                      (curMulticastParams->DownLinkCounter + 1 -curMulticastParams->DownLinkTotal) / (float)(curMulticastParams->DownLinkCounter + 1) * 100 );
                        }
                    }
                    else
                    {
                        if( macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_DOWN )
                        {
                            SrvAckRequested = true;
                            McpsIndication.McpsIndication = MCPS_CONFIRMED;

                            if( ( DownLinkCounter == downLinkCounter ) &&
                                ( DownLinkCounter != 0 ) )
                            {
                                // Duplicated confirmed downlink. Skip indication.
                                // In this case, the MAC layer shall accept the MAC commands
                                // which are included in the downlink retransmission.
                                // It should not provide the same frame to the application
                                // layer again.
                                skipIndication = true;
                            }
                        }
                        else
                        {
                            SrvAckRequested = false;
                            McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

                            if( ( DownLinkCounter == downLinkCounter ) &&
                                ( DownLinkCounter != 0 ) )
                            {
                                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED;
                                McpsIndication.DownLinkCounter = downLinkCounter;
                                PrepareRxDoneAbort( );
                                return;
                            }
                        }
                        DownLinkCounter = downLinkCounter;
                        if( LoraMacTestFlag == true )
                        {
                            DownLinkTotal++;
                            LOG_D( " Need %d packs,Rx %d packs,loss %.2f%% \r\n", DownLinkCounter + 1, DownLinkTotal,
                                                      (DownLinkCounter + 1 - DownLinkTotal) / (float)(DownLinkCounter + 1) * 100 );
                        }
                    }

                    // This must be done before parsing the payload and the MAC commands.
                    // We need to reset the MacCommandsBufferIndex here, since we need
                    // to take retransmissions and repititions into account. Error cases
                    // will be handled in function OnMacStateCheckTimerEvent.
                    if( McpsConfirm.McpsRequest == MCPS_CONFIRMED )
                    {
                        if( fCtrl.Bits.Ack == 1 )
                        {// Reset MacCommandsBufferIndex when we have received an ACK.
                            MacCommandsBufferIndex = 0;
                        }
                    }
                    else
                    {// Reset the variable if we have received any valid frame.
                        MacCommandsBufferIndex = 0;
                    }

                    // Process payload and MAC commands
                    if( ( ( size - 4 ) - appPayloadStartIndex ) > 0 )
                    {
                        port = payload[appPayloadStartIndex++];
                        frameLen = ( size - 4 ) - appPayloadStartIndex;

                        McpsIndication.Port = port;

                        if( port == 0 )
                        {
                            // Only allow frames which do not have fOpts
                            if( fCtrl.Bits.FOptsLen == 0 )
                            {
                                LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
                                                       frameLen,
                                                       nwkSKey,
                                                       address,
                                                       DOWN_LINK,
                                                       downLinkCounter,
                                                       LoRaMacRxPayload );

                                // Decode frame payload MAC commands
                                ProcessMacCommands( LoRaMacRxPayload, 0, frameLen, snr );
                            }
                            else
                            {
                                skipIndication = true;
                            }
                        }
                        else
                        {
                            if( fCtrl.Bits.FOptsLen > 0 )
                            {
                                // Decode Options field MAC commands. Omit the fPort.
                                ProcessMacCommands( payload, 8, appPayloadStartIndex - 1, snr );
                            }

                            LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
                                                   frameLen,
                                                   appSKey,
                                                   address,
                                                   DOWN_LINK,
                                                   downLinkCounter,
                                                   LoRaMacRxPayload );

                            if( skipIndication == false )
                            {
                                McpsIndication.Buffer = LoRaMacRxPayload;
                                McpsIndication.BufferSize = frameLen;
                                McpsIndication.RxData = true;
                            }
                        }
                    }
                    else
                    {
                        if( fCtrl.Bits.FOptsLen > 0 )
                        {
                            // Decode Options field MAC commands
                            ProcessMacCommands( payload, 8, appPayloadStartIndex, snr );
                        }
                    }

                    if( skipIndication == false )
                    {
                        // Check if the frame is an acknowledgement
                        if( fCtrl.Bits.Ack == 1 )
                        {
                            McpsConfirm.AckReceived = true;
                            McpsIndication.AckReceived = true;

                            // Stop the AckTimeout timer as no more retransmissions
                            // are needed.
                            TimerStop( &AckTimeoutTimer );
                        }
                        else
                        {
                            McpsConfirm.AckReceived = false;

                            if( AckTimeoutRetriesCounter > AckTimeoutRetries )
                            {
                                // Stop the AckTimeout timer as no more retransmissions
                                // are needed.
                                TimerStop( &AckTimeoutTimer );
                            }
                        }
                    }
                    // Provide always an indication, skip the callback to the user application,
                    // in case of a confirmed downlink retransmission.
                    LoRaMacFlags.Bits.McpsInd = 1;
                    LoRaMacFlags.Bits.McpsIndSkip = skipIndication;
                }
                else
                {
                    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_MIC_FAIL;

                    PrepareRxDoneAbort( );
                    LOG_D( " Rx Mic Error! \r\n" );
                    return;
                }
            }
            break;
        case FRAME_TYPE_PROPRIETARY:
            {
                memcpy1( LoRaMacRxPayload, &payload[pktHeaderLen], size );

                McpsIndication.McpsIndication = MCPS_PROPRIETARY;
                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                McpsIndication.Buffer = LoRaMacRxPayload;
                McpsIndication.BufferSize = size - pktHeaderLen;

                LoRaMacFlags.Bits.McpsInd = 1;
                break;
            }
        default:
            McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ERROR;
            PrepareRxDoneAbort( );
            break;
    }
    LoRaMacFlags.Bits.MacDone = 1;

    // Trig OnMacCheckTimerEvent call as soon as possible
    TimerSetValue( &MacStateCheckTimer, 1 );
    TimerStart( &MacStateCheckTimer );
}

//static void OnRadioTxTimeout( void )
void OnRadioTxTimeout( void )
{
    LOG_D(" On Tx Timeout \r\n");

    if( LoRaMacDeviceClass != CLASS_C )
    {
        Radio.Sleep( );
    }
    else
    {
        OnRxWindow2TimerEvent( );
    }

    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT;
    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT;
    LoRaMacFlags.Bits.MacDone = 1;
}

//static void OnRadioRxError( void )
void OnRadioRxError( void )
{
    LOG_D(" On Rx Error \r\n");

    if( LoRaMacDeviceClass != CLASS_C )
    {
        Radio.Sleep( );
    }
    else
    {
        OnRxWindow2TimerEvent( );
    }

				
    if( RxSlot == 0 )
    {
        if( NodeAckRequested == true )
        {
            McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX1_ERROR;
        }
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX1_ERROR;

        if( TimerGetElapsedTime( AggregatedLastTxDoneTime ) >= RxWindow2Delay )
        {
            LoRaMacFlags.Bits.MacDone = 1;
        }
    }
    else
    {
        if( NodeAckRequested == true )
        {
            McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX2_ERROR;
        }
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX2_ERROR;
        LoRaMacFlags.Bits.MacDone = 1;
    }
		
		
}

//static void OnRadioRxTimeout( void )
void OnRadioRxTimeout( void )
{
    LOG_D(" On Rx Timeout \r\n");

    if( LoRaMacDeviceClass != CLASS_C )
    {
        Radio.Sleep( );
    }
    else
    {
        OnRxWindow2TimerEvent( );
    }
   
		//喵了米的 rx2压根就打不开，这里就判断一个窗口咯
		
		
    //if( RxSlot == 1 )
		if(1)
    {
        if( NodeAckRequested == true )
        {
            McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;
        }
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;
        LoRaMacFlags.Bits.MacDone = 1;
    }
}

//周期更新MAC的状态，原协议是用轮询的方式，这里使用定时器。
static void OnMacStateCheckTimerEvent( )
{
    TimerStop( &MacStateCheckTimer );
    bool txTimeout = false;

    if( LoRaMacFlags.Bits.MacDone == 1 )
    {
        if( ( LoRaMacState & LORAMAC_RX_ABORT ) == LORAMAC_RX_ABORT )
        {
            LoRaMacState &= ~LORAMAC_RX_ABORT;
            LoRaMacState &= ~LORAMAC_TX_RUNNING;
        }

        if( ( LoRaMacFlags.Bits.MlmeReq == 1 ) || ( ( LoRaMacFlags.Bits.McpsReq == 1 ) ) )
        {
            if( ( McpsConfirm.Status == LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT ) ||
                ( MlmeConfirm.Status == LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT ) )
            {
                // Stop transmit cycle due to tx timeout.
                LoRaMacState &= ~LORAMAC_TX_RUNNING;
                MacCommandsBufferIndex = 0;
                McpsConfirm.NbRetries = AckTimeoutRetriesCounter;
                McpsConfirm.AckReceived = false;
                McpsConfirm.TxTimeOnAir = 0;
                txTimeout = true;
            }
        }

        if( ( NodeAckRequested == false ) && ( txTimeout == false ) )
        {
            if( ( LoRaMacFlags.Bits.MlmeReq == 1 ) || ( ( LoRaMacFlags.Bits.McpsReq == 1 ) ) )
            {
                if( ( LoRaMacFlags.Bits.MlmeReq == 1 ) && ( MlmeConfirm.MlmeRequest == MLME_JOIN ) )
                {// Procedure for the join request
                    MlmeConfirm.NbRetries = JoinRequestTrials;

                    if( MlmeConfirm.Status == LORAMAC_EVENT_INFO_STATUS_OK )
                    {// Node joined successfully
                        UpLinkCounter = 0;
                        ChannelsNbRepCounter = 0;
                        LoRaMacState &= ~LORAMAC_TX_RUNNING;
                    }
                    else //这里进行了自动重发入网包，去掉这个功能
                    {
											  //rt_kprintf("now join times:%d\tmax:%d",JoinRequestTrials,MaxJoinRequestTrials);
                        if( JoinRequestTrials >= MaxJoinRequestTrials )
                        {
                            LoRaMacState &= ~LORAMAC_TX_RUNNING;
                        }
                        else
                        {
                            LoRaMacFlags.Bits.MacDone = 0;
                            // Sends the same frame again
                            OnTxDelayedTimerEvent( );
                        }
                    }
                }
                else
                {// Procedure for all other frames
                    if( ( ChannelsNbRepCounter >= LoRaMacParams.ChannelsNbRep ) || ( LoRaMacFlags.Bits.McpsInd == 1 ) )
                    {
                        if( LoRaMacFlags.Bits.McpsInd == 0 )
                        {   // Maximum repititions without downlink. Reset MacCommandsBufferIndex. Increase ADR Ack counter.
                            // Only process the case when the MAC did not receive a downlink.
                            MacCommandsBufferIndex = 0;
                            AdrAckCounter++;
                        }

                        ChannelsNbRepCounter = 0;

                        if( IsUpLinkCounterFixed == false )
                        {
                            UpLinkCounter++;
                        }

                        LoRaMacState &= ~LORAMAC_TX_RUNNING;
                    }
                    else
                    {
                        LoRaMacFlags.Bits.MacDone = 0;
                        // Sends the same frame again
                        OnTxDelayedTimerEvent( );
                    }
                }
            }
        }

        if( LoRaMacFlags.Bits.McpsInd == 1 )
        {// Procedure if we received a frame
            if( ( McpsConfirm.AckReceived == true ) || ( AckTimeoutRetriesCounter > AckTimeoutRetries ) )
            {
                AckTimeoutRetry = false;
                NodeAckRequested = false;
                if( IsUpLinkCounterFixed == false )
                {
                    UpLinkCounter++;
                }
                McpsConfirm.NbRetries = AckTimeoutRetriesCounter;

                LoRaMacState &= ~LORAMAC_TX_RUNNING;
            }
        }

        if( ( AckTimeoutRetry == true ) && ( ( LoRaMacState & LORAMAC_TX_DELAYED ) == 0 ) )
        {// Retransmissions procedure for confirmed uplinks
            AckTimeoutRetry = false;
            if( ( AckTimeoutRetriesCounter < AckTimeoutRetries ) && ( AckTimeoutRetriesCounter <= MAX_ACK_RETRIES ) )
            {
                AckTimeoutRetriesCounter++;

                if( (AdrCtrlOn == true) && (( AckTimeoutRetriesCounter % 2 ) == 1) )
                {
                    LoRaMacParams.ChannelsDatarate = MAX( LoRaMacParams.ChannelsDatarate - 1, LORAMAC_TX_MIN_DATARATE );
                }
                // Try to send the frame again
                if( ScheduleTx( ) == LORAMAC_STATUS_OK )
                {
                    LoRaMacFlags.Bits.MacDone = 0;
                }
                else
                {
                    // The DR is not applicable for the payload size
                    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR;

                    MacCommandsBufferIndex = 0;
                    LoRaMacState &= ~LORAMAC_TX_RUNNING;
                    NodeAckRequested = false;
                    McpsConfirm.AckReceived = false;
                    McpsConfirm.NbRetries = AckTimeoutRetriesCounter;
                    McpsConfirm.Datarate = LoRaMacParams.ChannelsDatarate;
                    if( IsUpLinkCounterFixed == false )
                    {
                        UpLinkCounter++;
                    }
                }
            }
            else
            {
                //470
                // Re-enable default channels LC1, LC2, LC3
                LoRaMacParams.ChannelsMask[0] = LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
                // Re-enable default channels
                memcpy1( ( uint8_t* )LoRaMacParams.ChannelsMask, ( uint8_t* )LoRaMacParamsDefaults.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );


                LoRaMacState &= ~LORAMAC_TX_RUNNING;

                MacCommandsBufferIndex = 0;
                NodeAckRequested = false;
                McpsConfirm.AckReceived = false;
                McpsConfirm.NbRetries = AckTimeoutRetriesCounter;
                if( IsUpLinkCounterFixed == false )
                {
                    UpLinkCounter++;
                }
            }
        }
    }
    // Handle reception for Class B and Class C
    if( ( LoRaMacState & LORAMAC_RX ) == LORAMAC_RX )
    {
        LoRaMacState &= ~LORAMAC_RX;
    }
    if( LoRaMacState == LORAMAC_IDLE )
    {
        if( LoRaMacFlags.Bits.McpsReq == 1 )
        {
            LoRaMacPrimitives->MacMcpsConfirm( &McpsConfirm );
            LoRaMacFlags.Bits.McpsReq = 0;
        }

        if( LoRaMacFlags.Bits.MlmeReq == 1 )
        {
            LoRaMacPrimitives->MacMlmeConfirm( &MlmeConfirm );
            LoRaMacFlags.Bits.MlmeReq = 0;
        }

        // Procedure done. Reset variables.
        LoRaMacFlags.Bits.MacDone = 0;
    }
    else
    {
        // Operation not finished restart timer
        TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );
        TimerStart( &MacStateCheckTimer );
    }

    if( LoRaMacFlags.Bits.McpsInd == 1 )
    {
        if( LoRaMacDeviceClass == CLASS_C )
        {// Activate RX2 window for Class C
            OnRxWindow2TimerEvent( );
        }
        if( LoRaMacFlags.Bits.McpsIndSkip == 0 )
        {
            LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );
        }
        LoRaMacFlags.Bits.McpsIndSkip = 0;
        LoRaMacFlags.Bits.McpsInd = 0;
    }
}

static void OnTxDelayedTimerEvent(  )
{
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;

    TimerStop( &TxDelayedTimer );
    LoRaMacState &= ~LORAMAC_TX_DELAYED;

    if( ( LoRaMacFlags.Bits.MlmeReq == 1 ) && ( MlmeConfirm.MlmeRequest == MLME_JOIN ) )
    {
        ResetMacParameters( );
        // Add a +1, since we start to count from 0
        LoRaMacParams.ChannelsDatarate = AlternateDatarate( JoinRequestTrials + 1 );

        macHdr.Value = 0;
        macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;

        fCtrl.Value = 0;
        fCtrl.Bits.Adr = AdrCtrlOn;

        /* In case of join request retransmissions, the stack must prepare
         * the frame again, because the network server keeps track of the random
         * LoRaMacDevNonce values to prevent reply attacks. */
        PrepareFrame( &macHdr, &fCtrl, 0, NULL, 0 );
    }

    ScheduleTx( );
}

static void OnRxWindow1TimerEvent( void )
{
    TimerStop( &RxWindowTimer1 );
    RxSlot = 0;

    if( LoRaMacDeviceClass == CLASS_C )
    {
        Radio.Standby( );
    }
    //470
    RxWindowSetup( LORAMAC_FIRST_RX1_CHANNEL + ( Channel % 48 ) * LORAMAC_STEPWIDTH_RX1_CHANNEL, RxWindowsParams[0].Datarate, RxWindowsParams[0].Bandwidth, RxWindowsParams[0].RxWindowTimeout, false );
}

static void OnRxWindow2TimerEvent( void )
{
    bool rxContinuousMode = false;

    TimerStop( &RxWindowTimer2 );

    if( LoRaMacDeviceClass == CLASS_C )
    {
        rxContinuousMode = true;
    }
    if( RxWindowSetup( LoRaMacParams.Rx2Channel.Frequency, RxWindowsParams[1].Datarate, RxWindowsParams[1].Bandwidth, RxWindowsParams[1].RxWindowTimeout, rxContinuousMode ) == true )
    {
        RxSlot = 1;
    }
}

static void OnAckTimeoutTimerEvent( void )
{
    TimerStop( &AckTimeoutTimer );

    if( NodeAckRequested == true )
    {
        AckTimeoutRetry = true;
        LoRaMacState &= ~LORAMAC_ACK_REQ;
    }
    if( LoRaMacDeviceClass == CLASS_C )
    {
        LoRaMacFlags.Bits.MacDone = 1;
    }
}

static bool SetNextChannel( TimerTime_t* time )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTx = 0;
    uint8_t enabledChannels[LORA_MAX_NB_CHANNELS];
    TimerTime_t nextTxDelay = ( TimerTime_t )( -1 );

	
    memset1( enabledChannels, 0, LORA_MAX_NB_CHANNELS );

    //如果通道全部被关闭
    if( ( CountBits( LoRaMacParams.ChannelsMask[0], 16 ) == 0 ) &&
        ( CountBits( LoRaMacParams.ChannelsMask[1], 16 ) == 0 ) &&
        ( CountBits( LoRaMacParams.ChannelsMask[2], 16 ) == 0 ) &&
        ( CountBits( LoRaMacParams.ChannelsMask[3], 16 ) == 0 ) &&
        ( CountBits( LoRaMacParams.ChannelsMask[4], 16 ) == 0 ) &&
        ( CountBits( LoRaMacParams.ChannelsMask[5], 16 ) == 0 ) )
    {
        memcpy1( ( uint8_t* )LoRaMacParams.ChannelsMask, ( uint8_t* )LoRaMacParamsDefaults.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
    }
    
		
    if( CountBits( LoRaMacParams.ChannelsMask[0], 16 ) == 0 )
    {
        // Re-enable default channels, if no channel is enabled
        LoRaMacParams.ChannelsMask[0] = LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
    }


    // Update Aggregated duty cycle
    if( AggregatedTimeOff <= TimerGetElapsedTime( AggregatedLastTxDoneTime ) )
    {
        AggregatedTimeOff = 0;

        // Update bands Time OFF
        for( uint8_t i = 0; i < LORA_MAX_NB_BANDS; i++ )
        {
            if( ( IsLoRaMacNetworkJoined == false ) || ( DutyCycleOn == true ) )
            {
                if( Bands[i].TimeOff <= TimerGetElapsedTime( Bands[i].LastTxDoneTime ) )
                {
                    Bands[i].TimeOff = 0;
                }
                if( Bands[i].TimeOff != 0 )
                {
                    nextTxDelay = MIN( Bands[i].TimeOff - TimerGetElapsedTime( Bands[i].LastTxDoneTime ), nextTxDelay );
                }
            }
            else
            {
                if( DutyCycleOn == false )
                {
                    Bands[i].TimeOff = 0;
                }
            }
        }

        // Search how many channels are enabled
        for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
        {
            for( uint8_t j = 0; j < 16; j++ )
            {

                if( ( LoRaMacParams.ChannelsMask[k] & ( 1 << j ) ) != 0 )

                {
                    if( Channels[i + j].Frequency == 0 )
                    { // Check if the channel is enabled
                        continue;
                    }

                    if( ( ( Channels[i + j].DrRange.Fields.Min <= LoRaMacParams.ChannelsDatarate ) &&
                          ( LoRaMacParams.ChannelsDatarate <= Channels[i + j].DrRange.Fields.Max ) ) == false )
                    { // Check if the current channel selection supports the given datarate
                        continue;
                    }
                    if( Bands[Channels[i + j].Band].TimeOff > 0 )
                    { // Check if the band is available for transmission
                        delayTx++;
                        continue;
                    }
                    enabledChannels[nbEnabledChannels++] = i + j;
                }
            }
        }
    }
    else
    {
        delayTx++;
        nextTxDelay = AggregatedTimeOff - TimerGetElapsedTime( AggregatedLastTxDoneTime );
    }

    if( nbEnabledChannels > 0 )
    {
        Channel = enabledChannels[randr( 0, nbEnabledChannels - 1 )];
        *time = 0;
        return true;
    }
    else
    {
        if( delayTx > 0 )
        {
            // Delay transmission due to AggregatedTimeOff or to a band time off
            *time = nextTxDelay;
            return true;
        }
        // Datarate not supported by any channel
        *time = 0;
        return false;
    }
}

static bool RxWindowSetup( uint32_t freq, int8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous )
{
    uint8_t downlinkDatarate = Datarates[datarate];
    RadioModems_t modem;

    if( Radio.GetStatus( ) == RF_IDLE )
    {
        Radio.SetChannel( freq );

        // Store downlink datarate
        McpsIndication.RxDatarate = ( uint8_t ) datarate;

        modem = MODEM_LORA;
        Radio.SetRxConfig( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );

        if( RepeaterSupport == true )
        {
            Radio.SetMaxPayloadLength( modem, MaxPayloadOfDatarateRepeater[datarate] + LORA_MAC_FRMPAYLOAD_OVERHEAD );
        }
        else
        {
            Radio.SetMaxPayloadLength( modem, MaxPayloadOfDatarate[datarate] + LORA_MAC_FRMPAYLOAD_OVERHEAD );
        }

        if( rxContinuous == false )
        {
            Radio.Rx( LoRaMacParams.MaxRxWindow );
        }
        else
        {
            Radio.Rx( 0 ); // Continuous mode
        }
        if( downlinkDatarate == 50 )
        {
//          rt_kprintf( " Start RF Recv[%uHz FSK st:%d]... \r\n", freq, timeout );
        }
        else
        {
            LOG_D( "Start RF Recv[%uHz SF%d %dK st:%d]", freq, downlinkDatarate, (bandwidth==0)?125:(bandwidth== 1)?250:500, timeout );
        }
        return true;
    }
    return false;
}

static bool Rx2FreqInRange( uint32_t freq )
{

    if( ( Radio.CheckRfFrequency( freq ) == true ) &&( freq >= LORAMAC_FIRST_RX1_CHANNEL ) &&( freq <= LORAMAC_LAST_RX1_CHANNEL ) &&
        ( ( ( freq - ( uint32_t ) LORAMAC_FIRST_RX1_CHANNEL ) % ( uint32_t ) LORAMAC_STEPWIDTH_RX1_CHANNEL ) == 0 ) )
    {
        return true;
    }
    return false;
}

static bool ValidatePayloadLength( uint8_t lenN, int8_t datarate, uint8_t fOptsLen )
{
    uint16_t maxN = 0;
    uint16_t payloadSize = 0;

    // Get the maximum payload length
    if( RepeaterSupport == true )
    {
        maxN = MaxPayloadOfDatarateRepeater[datarate];
    }
    else
    {
        maxN = MaxPayloadOfDatarate[datarate];
    }

    // Calculate the resulting payload size
    payloadSize = ( lenN + fOptsLen );

    // Validation of the application payload size
    if( ( payloadSize <= maxN ) && ( payloadSize <= LORAMAC_PHY_MAXPAYLOAD ) )
    {
        return true;
    }
    return false;
}

static uint8_t CountBits( uint16_t mask, uint8_t nbBits )
{
    uint8_t nbActiveBits = 0;

    for( uint8_t j = 0; j < nbBits; j++ )
    {
        if( ( mask & ( 1 << j ) ) == ( 1 << j ) )
        {
            nbActiveBits++;
        }
    }
    return nbActiveBits;
}



static bool ValidateDatarate( int8_t datarate, uint16_t* channelsMask )
{
    if( ValueInRange( datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == false )
    {
        return false;
    }
    for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( ( channelsMask[k] & ( 1 << j ) ) != 0 ) )
            {// Check datarate validity for enabled channels
                if( ValueInRange( datarate, Channels[i + j].DrRange.Fields.Min, Channels[i + j].DrRange.Fields.Max ) == true )
                {
                    // At least 1 channel has been found we can return OK.
                    return true;
                }
            }
        }
    }
    return false;
}

static int8_t LimitTxPower( int8_t txPower, int8_t maxBandTxPower )
{
    int8_t resultTxPower = txPower;

    // Limit tx power to the band max
    resultTxPower =  MAX( txPower, maxBandTxPower );


    return resultTxPower;
}

static bool ValueInRange( int8_t value, int8_t min, int8_t max )
{
    if( ( value >= min ) && ( value <= max ) )
    {
        return true;
    }
    return false;
}



static bool AdrNextDr( bool adrEnabled, bool updateChannelMask, int8_t* datarateOut )
{
    bool adrAckReq = false;
    int8_t datarate = LoRaMacParams.ChannelsDatarate;

    if( adrEnabled == true )
    {
        if( datarate == LORAMAC_TX_MIN_DATARATE )
        {
            AdrAckCounter = 0;
            adrAckReq = false;
        }
        else
        {
            if( AdrAckCounter >= ADR_ACK_LIMIT )
            {
                adrAckReq = true;
                LoRaMacParams.ChannelsTxPower = LORAMAC_MAX_TX_POWER;
            }
            else
            {
                adrAckReq = false;
            }
            if( AdrAckCounter >= ( ADR_ACK_LIMIT + ADR_ACK_DELAY ) )
            {
                if( ( AdrAckCounter % ADR_ACK_DELAY ) == 1 )
                {

                    if( datarate > LORAMAC_TX_MIN_DATARATE )
                    {
                        datarate--;
                    }
                    if( datarate == LORAMAC_TX_MIN_DATARATE )
                    {
                        if( updateChannelMask == true )
                        {
                            // Re-enable default channels
                            memcpy1( ( uint8_t* )LoRaMacParams.ChannelsMask, ( uint8_t* )LoRaMacParamsDefaults.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
                        }
                    }
                }
            }
        }
    }

    *datarateOut = datarate;

    return adrAckReq;
}

LoRaMacStatus_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_BUSY;
    // The maximum buffer length must take MAC commands to re-send into account.
    uint8_t bufLen = LORA_MAC_COMMAND_MAX_LENGTH - MacCommandsBufferToRepeatIndex;

    switch( cmd )
    {
        case MOTE_MAC_LINK_CHECK_REQ:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this command
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_LINK_ADR_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // Margin
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_DUTY_CYCLE_ANS:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // Status: Datarate ACK, Channel ACK
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_DEV_STATUS_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 2 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // 1st byte Battery
                // 2nd byte Margin
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                MacCommandsBuffer[MacCommandsBufferIndex++] = p2;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_NEW_CHANNEL_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // Status: Datarate range OK, Channel frequency OK
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
        default:
//            if( AddPrivMacCommand( cmd, p1, p2, MacCommandsBuffer, &MacCommandsBufferIndex, bufLen ) == true )
//            {
//                status = LORAMAC_STATUS_OK;
//            }
//            else
//            {
//                status = LORAMAC_STATUS_SERVICE_UNKNOWN;
//            }
            break;
    }
    if( status == LORAMAC_STATUS_OK )
    {
        MacCommandsInNextTx = true;
    }
    return status;
}

static uint8_t ParseMacCommandsToRepeat( uint8_t* cmdBufIn, uint8_t length, uint8_t* cmdBufOut )
{
    uint8_t i = 0;
    uint8_t cmdCount = 0;

    if( ( cmdBufIn == NULL ) || ( cmdBufOut == NULL ) )
    {
        return 0;
    }

    for( i = 0; i < length; i++ )
    {
        switch( cmdBufIn[i] )
        {
            // STICKY
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
            {
                cmdBufOut[cmdCount++] = cmdBufIn[i++];
                cmdBufOut[cmdCount++] = cmdBufIn[i];
                break;
            }
            case MOTE_MAC_RX_TIMING_SETUP_ANS:
            {
                cmdBufOut[cmdCount++] = cmdBufIn[i];
                break;
            }
            // NON-STICKY
            case MOTE_MAC_DEV_STATUS_ANS:
            { // 2 bytes payload
                i += 2;
                break;
            }
            case MOTE_MAC_LINK_ADR_ANS:
            case MOTE_MAC_NEW_CHANNEL_ANS:
            { // 1 byte payload
                i++;
                break;
            }
            case MOTE_MAC_DUTY_CYCLE_ANS:
            case MOTE_MAC_LINK_CHECK_REQ:
            { // 0 byte payload
                break;
            }
            default:
                break;
        }
    }

    return cmdCount;
}

static void ProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, uint8_t snr )
{
    uint8_t cmd;
    
    LOG_D( "Recv Mac Commands" );
	  ulog_hexdump("lorawan",16,payload,commandsSize);
	
//    for( uint8_t i = macIndex; i < commandsSize; i++ )
//    {
//        rt_kprintf( " %02X", payload[i] );
//    }
//    rt_kprintf( " ]\r\n" );
	
    while( macIndex < commandsSize )
    {
        // Decode Frame MAC commands
        cmd = payload[macIndex++];
        switch( cmd )
        {
            case SRV_MAC_LINK_CHECK_ANS:
                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                MlmeConfirm.DemodMargin = payload[macIndex++];
                MlmeConfirm.NbGateways = payload[macIndex++];
                break;
            case SRV_MAC_LINK_ADR_REQ:
                {
                    uint8_t i;
                    uint8_t status = 0x07;
                    uint16_t chMask;
                    int8_t txPower = 0;
                    int8_t datarate = 0;
                    uint8_t nbRep = 0;
                    uint8_t chMaskCntl = 0;
                    uint16_t channelsMask[6] = { 0, 0, 0, 0, 0, 0 };

                    // Initialize local copy of the channels mask array
                    for( i = 0; i < 6; i++ )
                    {
                        channelsMask[i] = LoRaMacParams.ChannelsMask[i];
                    }
                    datarate = payload[macIndex++];
                    txPower = datarate & 0x0F;
                    datarate = ( datarate >> 4 ) & 0x0F;

                    if( ( AdrCtrlOn == false ) &&
                        ( ( LoRaMacParams.ChannelsDatarate != datarate ) || ( LoRaMacParams.ChannelsTxPower != txPower ) ) )
                    { // ADR disabled don't handle ADR requests if server tries to change datarate or txpower
                        // Answer the server with fail status
                        // Power ACK     = 0
                        // Data rate ACK = 0
                        // Channel mask  = 0
                        AddMacCommand( MOTE_MAC_LINK_ADR_ANS, 0, 0 );
                        macIndex += 3;  // Skip over the remaining bytes of the request
                        break;
                    }
                    chMask = ( uint16_t )payload[macIndex++];
                    chMask |= ( uint16_t )payload[macIndex++] << 8;

                    nbRep = payload[macIndex++];
                    chMaskCntl = ( nbRep >> 4 ) & 0x07;
                    nbRep &= 0x0F;
                    if( nbRep == 0 )
                    {
                        nbRep = 1;
                    }

                    if( chMaskCntl == 6 )
                    {
                        // Enable all 125 kHz channels
                        for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
                        {
                            for( uint8_t j = 0; j < 16; j++ )
                            {
                                if( Channels[i + j].Frequency != 0 )
                                {
                                    channelsMask[k] |= 1 << j;
                                }
                            }
                        }
                    }
                    else if( chMaskCntl == 7 )
                    {
                        status &= 0xFE; // Channel mask KO
                    }
                    else
                    {
                        for( uint8_t i = 0; i < 16; i++ )
                        {
                            if( ( ( chMask & ( 1 << i ) ) != 0 ) &&
                                ( Channels[chMaskCntl * 16 + i].Frequency == 0 ) )
                            {// Trying to enable an undefined channel
                                status &= 0xFE; // Channel mask KO
                            }
                        }
                        channelsMask[chMaskCntl] = chMask;
                    }

                    if( ValidateDatarate( datarate, channelsMask ) == false )
                    {
                        status &= 0xFD; // Datarate KO
                    }

                    //
                    // Remark MaxTxPower = 0 and MinTxPower = 5
                    //
                    if( ValueInRange( txPower, LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) == false )
                    {
                        status &= 0xFB; // TxPower KO
                    }
                    if( ( status & 0x07 ) == 0x07 )
                    {
                        LoRaMacParams.ChannelsDatarate = datarate;
                        LoRaMacParams.ChannelsTxPower = txPower;

                        memcpy1( ( uint8_t* )LoRaMacParams.ChannelsMask, ( uint8_t* )channelsMask, sizeof( LoRaMacParams.ChannelsMask ) );

                        LoRaMacParams.ChannelsNbRep = nbRep;
                    }
                    AddMacCommand( MOTE_MAC_LINK_ADR_ANS, status, 0 );
                }
                break;
            case SRV_MAC_DUTY_CYCLE_REQ:
                MaxDCycle = payload[macIndex++];
                AggregatedDCycle = 1 << MaxDCycle;
                AddMacCommand( MOTE_MAC_DUTY_CYCLE_ANS, 0, 0 );
                break;
            case SRV_MAC_RX_PARAM_SETUP_REQ:
                {
                    uint8_t status = 0x07;
                    int8_t datarate = 0;
                    int8_t drOffset = 0;
                    uint32_t freq = 0;

                    drOffset = ( payload[macIndex] >> 4 ) & 0x07;
                    datarate = payload[macIndex] & 0x0F;
                    macIndex++;

                    freq =  ( uint32_t )payload[macIndex++];
                    freq |= ( uint32_t )payload[macIndex++] << 8;
                    freq |= ( uint32_t )payload[macIndex++] << 16;
                    freq *= 100;

                    if( Rx2FreqInRange( freq ) == false )
                    {
                        status &= 0xFE; // Channel frequency KO
                    }

                    if( ValueInRange( datarate, LORAMAC_RX_MIN_DATARATE, LORAMAC_RX_MAX_DATARATE ) == false )
                    {
                        status &= 0xFD; // Datarate KO
                    }
                    if( ValueInRange( drOffset, LORAMAC_MIN_RX1_DR_OFFSET, LORAMAC_MAX_RX1_DR_OFFSET ) == false )
                    {
                        status &= 0xFB; // Rx1DrOffset range KO
                    }

                    if( ( status & 0x07 ) == 0x07 )
                    {
                        LoRaMacParams.Rx2Channel.Datarate = datarate;
                        LoRaMacParams.Rx2Channel.Frequency = freq;
                        LoRaMacParams.Rx1DrOffset = drOffset;
                    }
                    AddMacCommand( MOTE_MAC_RX_PARAM_SETUP_ANS, status, 0 );
                }
                break;
            case SRV_MAC_DEV_STATUS_REQ:
                {
                    uint8_t batteryLevel = BAT_LEVEL_NO_MEASURE;
                    if( ( LoRaMacCallbacks != NULL ) && ( LoRaMacCallbacks->GetBatteryLevel != NULL ) )
                    {
                        batteryLevel = LoRaMacCallbacks->GetBatteryLevel( );
                    }
                    AddMacCommand( MOTE_MAC_DEV_STATUS_ANS, batteryLevel, snr );
                    break;
                }
            case SRV_MAC_NEW_CHANNEL_REQ:
                {
                    uint8_t status = 0x03;


                    status &= 0xFC; // Channel frequency and datarate KO
                    macIndex += 5;

                    int8_t channelIndex = 0;
                    ChannelParams_t chParam;

                    channelIndex = payload[macIndex++];
                    chParam.Frequency = ( uint32_t )payload[macIndex++];
                    chParam.Frequency |= ( uint32_t )payload[macIndex++] << 8;
                    chParam.Frequency |= ( uint32_t )payload[macIndex++] << 16;
                    chParam.Frequency *= 100;
                    chParam.DrRange.Value = payload[macIndex++];

                    LoRaMacState |= LORAMAC_TX_CONFIG;
                    if( chParam.Frequency == 0 )
                    {
                        if( channelIndex < 3 )
                        {
                            status &= 0xFC;
                        }
                        else
                        {
                            if( LoRaMacChannelRemove( channelIndex ) != LORAMAC_STATUS_OK )
                            {
                                status &= 0xFC;
                            }
                        }
                    }
                    else
                    {
                        switch( LoRaMacChannelAdd( channelIndex, chParam ) )
                        {
                            case LORAMAC_STATUS_OK:
                            {
                                break;
                            }
                            case LORAMAC_STATUS_FREQUENCY_INVALID:
                            {
                                status &= 0xFE;
                                break;
                            }
                            case LORAMAC_STATUS_DATARATE_INVALID:
                            {
                                status &= 0xFD;
                                break;
                            }
                            case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
                            {
                                status &= 0xFC;
                                break;
                            }
                            default:
                            {
                                status &= 0xFC;
                                break;
                            }
                        }
                    }
                    LoRaMacState &= ~LORAMAC_TX_CONFIG;

                    AddMacCommand( MOTE_MAC_NEW_CHANNEL_ANS, status, 0 );
                }
                break;
            case SRV_MAC_RX_TIMING_SETUP_REQ:
                {
                    uint8_t delay = payload[macIndex++] & 0x0F;

                    if( delay == 0 )
                    {
                        delay++;
                    }
                    LoRaMacParams.ReceiveDelay1 = delay * 1e3;
                    LoRaMacParams.ReceiveDelay2 = LoRaMacParams.ReceiveDelay1 + 1e3;
                    AddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );
                }
                break;
            default:
//                if( ProcessPrivMacCommands(cmd, payload, &macIndex, MacCommandsBuffer, &MacCommandsBufferIndex, LORA_MAC_COMMAND_MAX_LENGTH - MacCommandsBufferToRepeatIndex) == true )
//                {
//                    break;
//                }
//                else
//                {
//                    // Unknown command. ABORT MAC commands processing
//                    return;
//                }
						break;
        }
    }
}

LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    LoRaMacFrameCtrl_t fCtrl;
    LoRaMacStatus_t status = LORAMAC_STATUS_PARAMETER_INVALID;

    fCtrl.Value = 0;
    fCtrl.Bits.FOptsLen      = 0;
    fCtrl.Bits.FPending      = 0;
    fCtrl.Bits.Ack           = false;
    fCtrl.Bits.AdrAckReq     = false;
    fCtrl.Bits.Adr           = AdrCtrlOn;

    // Prepare the frame
    status = PrepareFrame( macHdr, &fCtrl, fPort, fBuffer, fBufferSize );

    // Validate status
    if( status != LORAMAC_STATUS_OK )
    {
        return status;
    }

    // Reset confirm parameters
    McpsConfirm.NbRetries = 0;
    McpsConfirm.AckReceived = false;
    McpsConfirm.UpLinkCounter = UpLinkCounter;

    status = ScheduleTx( );

    return status;
}

static LoRaMacStatus_t ScheduleTx( void )
{
    TimerTime_t dutyCycleTimeOff = 0;

    // Check if the device is off
    if( MaxDCycle == 255 )
    {
        return LORAMAC_STATUS_DEVICE_OFF;
    }
    if( MaxDCycle == 0 )
    {
        AggregatedTimeOff = 0;
    }

    // Select channel
    while( SetNextChannel( &dutyCycleTimeOff ) == false )
    {
        // Set the default datarate
        LoRaMacParams.ChannelsDatarate = LORAMAC_DEFAULT_DATARATE;


    }

    // Compute Rx1 windows parameters
    RxWindowsParams[0] = ComputeRxWindowParameters( MAX( DR_0, LoRaMacParams.ChannelsDatarate - LoRaMacParams.Rx1DrOffset ), LoRaMacParams.SystemMaxRxError );

    // Compute Rx2 windows parameters
    RxWindowsParams[1] = ComputeRxWindowParameters( LoRaMacParams.Rx2Channel.Datarate, LoRaMacParams.SystemMaxRxError );

    if( IsLoRaMacNetworkJoined == false )
    {
        RxWindow1Delay = LoRaMacParams.JoinAcceptDelay1 + RxWindowsParams[0].RxOffset;
        RxWindow2Delay = LoRaMacParams.JoinAcceptDelay2 + RxWindowsParams[1].RxOffset;
    }
    else
    {
        if( ValidatePayloadLength( LoRaMacTxPayloadLen, LoRaMacParams.ChannelsDatarate, MacCommandsBufferIndex ) == false )
        {
            return LORAMAC_STATUS_LENGTH_ERROR;
        }
        RxWindow1Delay = LoRaMacParams.ReceiveDelay1 + RxWindowsParams[0].RxOffset;
        RxWindow2Delay = LoRaMacParams.ReceiveDelay2 + RxWindowsParams[1].RxOffset;
    }

		//这里进行了发送占空比判断
		
    // Schedule transmission of frame
    if( dutyCycleTimeOff == 0 )
    {
        // Try to send now
        return SendFrameOnChannel( Channels[Channel] );
    }
    else
    {
        // Send later - prepare timer
        LOG_D( "Time off:%d ms ", dutyCycleTimeOff  );
        LoRaMacState |= LORAMAC_TX_DELAYED;
        TimerSetValue( &TxDelayedTimer, dutyCycleTimeOff );
        TimerStart( &TxDelayedTimer );

        return LORAMAC_STATUS_OK;
    }
}

static uint16_t JoinDutyCycle( void )
{
    uint16_t dutyCycle = 0;
    TimerTime_t timeElapsed = TimerGetElapsedTime( LoRaMacInitializationTime );

    if( timeElapsed < 3600e3 )
    {
        dutyCycle = BACKOFF_DC_1_HOUR;
    }
    else if( timeElapsed < ( 3600e3 + 36000e3 ) )
    {
        dutyCycle = BACKOFF_DC_10_HOURS;
    }
    else
    {
        dutyCycle = BACKOFF_DC_24_HOURS;
    }
    return dutyCycle;
}

static void CalculateBackOff( uint8_t channel )
{
    uint16_t dutyCycle = Bands[Channels[channel].Band].DCycle;
    uint16_t joinDutyCycle = 0;

    // Reset time-off to initial value.
    Bands[Channels[channel].Band].TimeOff = 0;

    if( IsLoRaMacNetworkJoined == false )
    {
        // The node has not joined yet. Apply join duty cycle to all regions.
        joinDutyCycle = JoinDutyCycle( );
        if( DutyCycleOn == false )
        {
            dutyCycle = 1;
        }
        dutyCycle = MAX( dutyCycle, joinDutyCycle );

        // Update Band time-off.
        Bands[Channels[channel].Band].TimeOff = TxTimeOnAir * dutyCycle - TxTimeOnAir;
    }
    else
    {
        if( DutyCycleOn == true )
        {
            Bands[Channels[channel].Band].TimeOff = TxTimeOnAir * dutyCycle - TxTimeOnAir;
        }
    }

    // Update Aggregated Time OFF
    AggregatedTimeOff = AggregatedTimeOff + ( TxTimeOnAir * AggregatedDCycle - TxTimeOnAir );
}

static int8_t AlternateDatarate( uint16_t nbTrials )
{
    int8_t datarate = LORAMAC_TX_MIN_DATARATE;

    if( LoRaMacParamsDefaults.ChannelsDatarate > DR_5 )
    {
        datarate = DR_5;
    }
    else
    {
        datarate = LoRaMacParamsDefaults.ChannelsDatarate;
    }
    if( nbTrials > 48 )
    {
        datarate = MAX( datarate - 5, LORAMAC_TX_MIN_DATARATE );
    }
    else if( nbTrials > 32 )
    {
        datarate = MAX( datarate - 4, LORAMAC_TX_MIN_DATARATE );
    }
    else if( nbTrials > 24 )
    {
        datarate = MAX( datarate - 3, LORAMAC_TX_MIN_DATARATE );
    }
    else if( nbTrials > 16 )
    {
        datarate = MAX( datarate - 2, LORAMAC_TX_MIN_DATARATE );
    }
    else if( nbTrials > 8 )
    {
        datarate = MAX( datarate - 1, LORAMAC_TX_MIN_DATARATE );
    }
  
    return datarate;
}

static void ResetMacParameters( void )
{
    IsLoRaMacNetworkJoined = false;

    // Counters
    UpLinkCounter = 0;
    DownLinkCounter = 0;
    AdrAckCounter = 0;

    ChannelsNbRepCounter = 0;

    AckTimeoutRetries = 1;
    AckTimeoutRetriesCounter = 1;
    AckTimeoutRetry = false;

    MaxDCycle = 0;
    AggregatedDCycle = 1;

    MacCommandsBufferIndex = 0;
    MacCommandsBufferToRepeatIndex = 0;

    IsRxWindowsEnabled = true;

    LoRaMacParams.ChannelsTxPower = LoRaMacParamsDefaults.ChannelsTxPower;
    LoRaMacParams.ChannelsDatarate = LoRaMacParamsDefaults.ChannelsDatarate;

    LoRaMacParams.Rx1DrOffset = LoRaMacParamsDefaults.Rx1DrOffset;
    LoRaMacParams.Rx2Channel = LoRaMacParamsDefaults.Rx2Channel;

    memcpy1( ( uint8_t* ) LoRaMacParams.ChannelsMask, ( uint8_t* ) LoRaMacParamsDefaults.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );




    NodeAckRequested = false;
    SrvAckRequested = false;
    MacCommandsInNextTx = false;

    // Reset Multicast downlink counters
    MulticastParams_t *cur = MulticastChannels;
    while( cur != NULL )
    {
        cur->DownLinkCounter = 0;
        cur = cur->Next;
    }

    // Initialize channel index.
    Channel = LORA_MAX_NB_CHANNELS;
}

LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    uint16_t i;
    uint8_t pktHeaderLen = 0;
    uint32_t mic = 0;
    const void* payload = fBuffer;
    uint8_t framePort = fPort;

    LoRaMacBufferPktLen = 0;

    NodeAckRequested = false;

    if( fBuffer == NULL )
    {
        fBufferSize = 0;
    }

    LoRaMacTxPayloadLen = fBufferSize;

    LoRaMacBuffer[pktHeaderLen++] = macHdr->Value;

    switch( macHdr->Bits.MType )
    {
        case FRAME_TYPE_JOIN_REQ:
            LoRaMacBufferPktLen = pktHeaderLen;

            memcpyr( LoRaMacBuffer + LoRaMacBufferPktLen, LoRaMacAppEui, 8 );
            LoRaMacBufferPktLen += 8;
            memcpyr( LoRaMacBuffer + LoRaMacBufferPktLen, LoRaMacDevEui, 8 );
            LoRaMacBufferPktLen += 8;

            LoRaMacDevNonce = Radio.Random( );

            LoRaMacBuffer[LoRaMacBufferPktLen++] = LoRaMacDevNonce & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

            LoRaMacJoinComputeMic( LoRaMacBuffer, LoRaMacBufferPktLen & 0xFF, LoRaMacAppKey, &mic );

            LoRaMacBuffer[LoRaMacBufferPktLen++] = mic & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 8 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 16 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 24 ) & 0xFF;

            break;
        case FRAME_TYPE_DATA_CONFIRMED_UP:
            NodeAckRequested = true;
            //Intentional fallthrough
        case FRAME_TYPE_DATA_UNCONFIRMED_UP:
            if( IsLoRaMacNetworkJoined == false )
            {
                return LORAMAC_STATUS_NO_NETWORK_JOINED; // No network has been joined yet
            }

            fCtrl->Bits.AdrAckReq = AdrNextDr( fCtrl->Bits.Adr, true, &LoRaMacParams.ChannelsDatarate );

            if( SrvAckRequested == true )
            {
                SrvAckRequested = false;
                fCtrl->Bits.Ack = 1;
            }

            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr ) & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr >> 24 ) & 0xFF;

            LoRaMacBuffer[pktHeaderLen++] = fCtrl->Value;

            LoRaMacBuffer[pktHeaderLen++] = UpLinkCounter & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( UpLinkCounter >> 8 ) & 0xFF;

            // Copy the MAC commands which must be re-send into the MAC command buffer
            memcpy1( &MacCommandsBuffer[MacCommandsBufferIndex], MacCommandsBufferToRepeat, MacCommandsBufferToRepeatIndex );
            MacCommandsBufferIndex += MacCommandsBufferToRepeatIndex;

            if( ( payload != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                if( ( MacCommandsBufferIndex <= LORA_MAC_COMMAND_MAX_LENGTH ) && ( MacCommandsInNextTx == true ) )
                {
                    fCtrl->Bits.FOptsLen += MacCommandsBufferIndex;

                    // Update FCtrl field with new value of OptionsLength
                    LoRaMacBuffer[0x05] = fCtrl->Value;
                    for( i = 0; i < MacCommandsBufferIndex; i++ )
                    {
                        LoRaMacBuffer[pktHeaderLen++] = MacCommandsBuffer[i];
                    }
#if 1
                    LOG_D( "Send Mac Commands :" );
										ulog_hexdump("lorawan",16,MacCommandsBuffer,MacCommandsBufferIndex);
//                    for( uint8_t i = 0; i < MacCommandsBufferIndex; i++ )
//                    {
//                        rt_kprintf( " %02X", MacCommandsBuffer[i] );
//                    }
//                    rt_kprintf( " ]\r\n" );
#endif
                }
            }
            else
            {
                if( ( MacCommandsBufferIndex > 0 ) && ( MacCommandsInNextTx ) )
                {
                    LoRaMacTxPayloadLen = MacCommandsBufferIndex;
                    payload = MacCommandsBuffer;
                    framePort = 0;
#if 1
								  	LOG_D( "Send Mac Commands:" );
										ulog_hexdump("lorawan",16,MacCommandsBuffer,MacCommandsBufferIndex);
//                    for( uint8_t i = 0; i < MacCommandsBufferIndex; i++ )
//                    {
//                        rt_kprintf( " %02X", MacCommandsBuffer[i] );
//                    }
//                    rt_kprintf( " ]\r\n" );
#endif
                }
            }
            MacCommandsInNextTx = false;
            // Store MAC commands which must be re-send in case the device does not receive a downlink anymore
            MacCommandsBufferToRepeatIndex = ParseMacCommandsToRepeat( MacCommandsBuffer, MacCommandsBufferIndex, MacCommandsBufferToRepeat );
            if( MacCommandsBufferToRepeatIndex > 0 )
            {
                MacCommandsInNextTx = true;
            }

            if( ( payload != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                LoRaMacBuffer[pktHeaderLen++] = framePort;

                if( framePort == 0 )
                {
                    LoRaMacPayloadEncrypt( (uint8_t* ) payload, LoRaMacTxPayloadLen, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &LoRaMacBuffer[pktHeaderLen] );
                }
                else
                {
                    LoRaMacPayloadEncrypt( (uint8_t* ) payload, LoRaMacTxPayloadLen, LoRaMacAppSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &LoRaMacBuffer[pktHeaderLen] );
                }
            }
            LoRaMacBufferPktLen = pktHeaderLen + LoRaMacTxPayloadLen;

            LoRaMacComputeMic( LoRaMacBuffer, LoRaMacBufferPktLen, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &mic );

            LoRaMacBuffer[LoRaMacBufferPktLen + 0] = mic & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen + 1] = ( mic >> 8 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen + 2] = ( mic >> 16 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen + 3] = ( mic >> 24 ) & 0xFF;

            LoRaMacBufferPktLen += LORAMAC_MFR_LEN;

            break;
        case FRAME_TYPE_PROPRIETARY:
            if( ( fBuffer != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                memcpy1( LoRaMacBuffer + pktHeaderLen, ( uint8_t* ) fBuffer, LoRaMacTxPayloadLen );
                LoRaMacBufferPktLen = pktHeaderLen + LoRaMacTxPayloadLen;
            }
            break;
        default:
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t SendFrameOnChannel( ChannelParams_t channel )
{
    int8_t datarate = Datarates[LoRaMacParams.ChannelsDatarate];
    uint32_t bandwidth = 0;
    int8_t txPowerIndex = 0;
    int8_t txPower = 0;

    txPowerIndex = LimitTxPower( LoRaMacParams.ChannelsTxPower, Bands[channel.Band].TxMaxPower );
    txPower = TxPowers[txPowerIndex];

    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR;
    McpsConfirm.Datarate = LoRaMacParams.ChannelsDatarate;
    McpsConfirm.TxPower = txPowerIndex;
    McpsConfirm.UpLinkFrequency = channel.Frequency;

    Radio.SetChannel( channel.Frequency );

    Radio.SetMaxPayloadLength( MODEM_LORA, LoRaMacBufferPktLen );
    Radio.SetTxConfig( MODEM_LORA, txPower, 0, 0, datarate, 1, 8, false, true, 0, 0, false, 3e3 );
    TxTimeOnAir = Radio.TimeOnAir( MODEM_LORA, LoRaMacBufferPktLen );


    // Store the time on air
    McpsConfirm.TxTimeOnAir = TxTimeOnAir;
    MlmeConfirm.TxTimeOnAir = TxTimeOnAir;

    if( datarate == 50 )
    {
			  LOG_D( "[%uHz FSK %ddBm SIZE:%d] Send Frame :", channel.Frequency, txPower, LoRaMacBufferPktLen );
    }
    else
    {
		    LOG_D( " Send->>>[%uHz SF%d %dK %ddBm SIZE:%d] Frame: ", channel.Frequency, datarate, (bandwidth==0)?125:(bandwidth== 1)?250:500, txPower, LoRaMacBufferPktLen );
    }
		ulog_hexdump("lorawan",16,LoRaMacBuffer,LoRaMacBufferPktLen);
//    for( uint16_t i = 0; i < LoRaMacBufferPktLen; i++ )
//    {
//        rt_kprintf( " %02X", LoRaMacBuffer[i] );
//    }
//    rt_kprintf( "]\r\n" );

    // Starts the MAC layer status check timer
    TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );
    TimerStart( &MacStateCheckTimer );

    if( IsLoRaMacNetworkJoined == false )
    {
        JoinRequestTrials++;
    }
    // Send now
    Radio.Send( LoRaMacBuffer, LoRaMacBufferPktLen );

    LoRaMacState |= LORAMAC_TX_RUNNING;

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t SetTxContinuousWave( uint16_t timeout )
{
    int8_t txPowerIndex = 0;
    int8_t txPower = 0;

    txPowerIndex = LimitTxPower( LoRaMacParams.ChannelsTxPower, Bands[Channels[Channel].Band].TxMaxPower );
    txPower = TxPowers[txPowerIndex];

    // Starts the MAC layer status check timer
    TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );
    TimerStart( &MacStateCheckTimer );

    Radio.SetTxContinuousWave( Channels[Channel].Frequency, txPower, timeout );

    LoRaMacState |= LORAMAC_TX_RUNNING;

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t SetTxContinuousWave1( uint16_t timeout, uint32_t frequency, uint8_t power )
{
    Radio.SetTxContinuousWave( frequency, power, timeout );

    // Starts the MAC layer status check timer
    TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );
    TimerStart( &MacStateCheckTimer );

    LoRaMacState |= LORAMAC_TX_RUNNING;

    return LORAMAC_STATUS_OK;
}

RadioEvents_t RadioEvents;
rt_thread_t lorawan_core_hander;

ALIGN(RT_ALIGN_SIZE)
static char thread_lwcr_stack[1024];
static struct rt_thread lorawan_core_thread;

LoRaMacStatus_t LoRaMacInitialization( LoRaMacPrimitives_t *primitives, LoRaMacCallback_t *callbacks )
{
	  rt_kprintf("##-lwcr start\n");		
	  rt_event_init(&lorawan_core_event, "event", RT_IPC_FLAG_FIFO);
    if( primitives == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if( ( primitives->MacMcpsConfirm == NULL ) ||
        ( primitives->MacMcpsIndication == NULL ) ||
        ( primitives->MacMlmeConfirm == NULL ) )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    LoRaMacPrimitives = primitives;
    LoRaMacCallbacks = callbacks;

    LoRaMacFlags.Value = 0;

    LoRaMacDeviceClass = CLASS_A;
    LoRaMacState = LORAMAC_IDLE;

    JoinRequestTrials = 0;
    MaxJoinRequestTrials = 1;
    RepeaterSupport = false;

    // Reset duty cycle times
    AggregatedLastTxDoneTime = 0;
    AggregatedTimeOff = 0;

    // Duty cycle
    DutyCycleOn = false;


    // Reset to defaults
    LoRaMacParamsDefaults.ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER;
    LoRaMacParamsDefaults.ChannelsDatarate = LORAMAC_DEFAULT_DATARATE;

    LoRaMacParamsDefaults.SystemMaxRxError = 10;
    LoRaMacParamsDefaults.MinRxSymbols = 6;
    LoRaMacParamsDefaults.MaxRxWindow = MAX_RX_WINDOW;
    LoRaMacParamsDefaults.ReceiveDelay1 = RECEIVE_DELAY1;
    LoRaMacParamsDefaults.ReceiveDelay2 = RECEIVE_DELAY2;
    LoRaMacParamsDefaults.JoinAcceptDelay1 = JOIN_ACCEPT_DELAY1;
    LoRaMacParamsDefaults.JoinAcceptDelay2 = JOIN_ACCEPT_DELAY2;

    LoRaMacParamsDefaults.ChannelsNbRep = 1;
    LoRaMacParamsDefaults.Rx1DrOffset = 0;

    LoRaMacParamsDefaults.Rx2Channel = ( Rx2ChannelParams_t )RX_WND_2_CHANNEL;

    // Channel mask

    LoRaMacParamsDefaults.ChannelsMask[0] = 0xFFFF;
    LoRaMacParamsDefaults.ChannelsMask[1] = 0xFFFF;
    LoRaMacParamsDefaults.ChannelsMask[2] = 0xFFFF;
    LoRaMacParamsDefaults.ChannelsMask[3] = 0xFFFF;
    LoRaMacParamsDefaults.ChannelsMask[4] = 0xFFFF;
    LoRaMacParamsDefaults.ChannelsMask[5] = 0xFFFF;


    // 125 kHz channels
    for( uint8_t i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
    {
        Channels[i].Frequency = 470.3e6 + i * 200e3;
        Channels[i].DrRange.Value = ( DR_5 << 4 ) | DR_0;
        Channels[i].Band = 0;
    }


    // Init parameters which are not set in function ResetMacParameters
    LoRaMacParams.SystemMaxRxError = LoRaMacParamsDefaults.SystemMaxRxError;
    LoRaMacParams.MinRxSymbols = LoRaMacParamsDefaults.MinRxSymbols;
    LoRaMacParams.MaxRxWindow = LoRaMacParamsDefaults.MaxRxWindow;
    LoRaMacParams.ReceiveDelay1 = LoRaMacParamsDefaults.ReceiveDelay1;
    LoRaMacParams.ReceiveDelay2 = LoRaMacParamsDefaults.ReceiveDelay2;
    LoRaMacParams.JoinAcceptDelay1 = LoRaMacParamsDefaults.JoinAcceptDelay1;
    LoRaMacParams.JoinAcceptDelay2 = LoRaMacParamsDefaults.JoinAcceptDelay2;
    LoRaMacParams.ChannelsNbRep = LoRaMacParamsDefaults.ChannelsNbRep;

    ResetMacParameters( );

    // Initialize timers
    TimerInit( &MacStateCheckTimer, OnMacStateCheckTimerEvent_shell );
    TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );

    TimerInit( &TxDelayedTimer, OnTxDelayedTimerEvent_shell );
    TimerInit( &RxWindowTimer1, OnRxWindow1TimerEvent_shell );
    TimerInit( &RxWindowTimer2, OnRxWindow2TimerEvent_shell);
    TimerInit( &AckTimeoutTimer, OnAckTimeoutTimerEvent_shell);

    // Store the current initialization time
    LoRaMacInitializationTime = TimerGetCurrentTime( );

    // Initialize Radio driver
    RadioEvents.TxDone = OnRadioTxDone;
    RadioEvents.RxDone = OnRadioRxDone;
    RadioEvents.RxError = OnRadioRxError;
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    RadioEvents.RxTimeout = OnRadioRxTimeout;
    Radio.Init( &RadioEvents );
    // Random seed initialization
    srand1( Radio.Random( ) );

    PublicNetwork = true;
		
    Radio.SetPublicNetwork( PublicNetwork );
    Radio.Sleep( );

    //开启事件处理进程
//    lorawan_core_hander = rt_thread_create("lwcr",lorawan_core_thread_entry,RT_NULL,1024,3,10);
//		if (lorawan_core_hander != RT_NULL)
//	  {
//        rt_thread_startup(lorawan_core_hander);
//	  }

	 //静态创建线程
	 rt_thread_init(&lorawan_core_thread,
                   "lwcr",
                   lorawan_core_thread_entry,
                   RT_NULL,
                   &thread_lwcr_stack[0],
                   sizeof(thread_lwcr_stack),
                   3, 10);
  rt_thread_startup(&lorawan_core_thread);

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacQueryTxPossible( uint8_t size, LoRaMacTxInfo_t* txInfo )
{
    int8_t datarate = LoRaMacParamsDefaults.ChannelsDatarate;
    uint8_t fOptLen = MacCommandsBufferIndex + MacCommandsBufferToRepeatIndex;

    if( txInfo == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    
    if( AdrCtrlOn == true )
    {
        datarate = LORAMAC_TX_MIN_DATARATE;
    }
    else
    {
        AdrNextDr( AdrCtrlOn, false, &datarate );
    }
    
    if( RepeaterSupport == true )
    {
        txInfo->CurrentPayloadSize = MaxPayloadOfDatarateRepeater[datarate];
    }
    else
    {
        txInfo->CurrentPayloadSize = MaxPayloadOfDatarate[datarate];
    }

    if( txInfo->CurrentPayloadSize >= fOptLen )
    {
        txInfo->MaxPossiblePayload = txInfo->CurrentPayloadSize - fOptLen;
    }
    else
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    if( ValidatePayloadLength( size, datarate, 0 ) == false )
    {
        return LORAMAC_STATUS_LENGTH_ERROR;
    }

    if( ValidatePayloadLength( size, datarate, fOptLen ) == false )
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMibGetRequestConfirm( MibRequestConfirm_t *mibGet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if( mibGet == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    switch( mibGet->Type )
    {
        case MIB_DEVICE_CLASS:
        {
            mibGet->Param.Class = LoRaMacDeviceClass;
            break;
        }
        case MIB_NETWORK_JOINED:
        {
            mibGet->Param.IsNetworkJoined = IsLoRaMacNetworkJoined;
            break;
        }
        case MIB_ADR:
        {
            mibGet->Param.AdrEnable = AdrCtrlOn;
            break;
        }
        case MIB_NET_ID:
        {
            mibGet->Param.NetID = LoRaMacNetID;
            break;
        }
        case MIB_DEV_ADDR:
        {
            mibGet->Param.DevAddr = LoRaMacDevAddr;
            break;
        }
        case MIB_NWK_SKEY:
        {
            mibGet->Param.NwkSKey = LoRaMacNwkSKey;
            break;
        }
        case MIB_APP_SKEY:
        {
            mibGet->Param.AppSKey = LoRaMacAppSKey;
            break;
        }
        case MIB_PUBLIC_NETWORK:
        {
            mibGet->Param.EnablePublicNetwork = PublicNetwork;
            break;
        }
        case MIB_REPEATER_SUPPORT:
        {
            mibGet->Param.EnableRepeaterSupport = RepeaterSupport;
            break;
        }
        case MIB_CHANNELS:
        {
            mibGet->Param.ChannelList = Channels;
            break;
        }
        case MIB_RX2_CHANNEL:
        {
            mibGet->Param.Rx2Channel = LoRaMacParams.Rx2Channel;
            break;
        }
        case MIB_RX2_DEFAULT_CHANNEL:
        {
            mibGet->Param.Rx2Channel = LoRaMacParamsDefaults.Rx2Channel;
            break;
        }
        case MIB_CHANNELS_DEFAULT_MASK:
        {
            mibGet->Param.ChannelsDefaultMask = LoRaMacParamsDefaults.ChannelsMask;
            break;
        }
        case MIB_CHANNELS_MASK:
        {
            mibGet->Param.ChannelsMask = LoRaMacParams.ChannelsMask;
            break;
        }
        case MIB_CHANNELS_NB_REP:
        {
            mibGet->Param.ChannelNbRep = LoRaMacParams.ChannelsNbRep;
            break;
        }
        case MIB_MAX_RX_WINDOW_DURATION:
        {
            mibGet->Param.MaxRxWindow = LoRaMacParams.MaxRxWindow;
            break;
        }
        case MIB_RECEIVE_DELAY_1:
        {
            mibGet->Param.ReceiveDelay1 = LoRaMacParams.ReceiveDelay1;
            break;
        }
        case MIB_RECEIVE_DELAY_2:
        {
            mibGet->Param.ReceiveDelay2 = LoRaMacParams.ReceiveDelay2;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_1:
        {
            mibGet->Param.JoinAcceptDelay1 = LoRaMacParams.JoinAcceptDelay1;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_2:
        {
            mibGet->Param.JoinAcceptDelay2 = LoRaMacParams.JoinAcceptDelay2;
            break;
        }
        case MIB_CHANNELS_DEFAULT_DATARATE:
        {
            mibGet->Param.ChannelsDefaultDatarate = LoRaMacParamsDefaults.ChannelsDatarate;
            break;
        }
        case MIB_CHANNELS_DATARATE:
        {
            mibGet->Param.ChannelsDatarate = LoRaMacParams.ChannelsDatarate;
            break;
        }
        case MIB_CHANNELS_DEFAULT_TX_POWER:
        {
            mibGet->Param.ChannelsDefaultTxPower = LoRaMacParamsDefaults.ChannelsTxPower;
            break;
        }
        case MIB_CHANNELS_TX_POWER:
        {
            mibGet->Param.ChannelsTxPower = LoRaMacParams.ChannelsTxPower;
            break;
        }
        case MIB_UPLINK_COUNTER:
        {
            mibGet->Param.UpLinkCounter = UpLinkCounter;
            break;
        }
        case MIB_DOWNLINK_COUNTER:
        {
            mibGet->Param.DownLinkCounter = DownLinkCounter;
            break;
        }
        case MIB_MULTICAST_CHANNEL:
        {
            mibGet->Param.MulticastList = MulticastChannels;
            break;
        }
        case MIB_SYSTEM_MAX_RX_ERROR:
        {
            mibGet->Param.SystemMaxRxError = LoRaMacParams.SystemMaxRxError;
            break;
        }
        case MIB_MIN_RX_SYMBOLS:
        {
            mibGet->Param.MinRxSymbols = LoRaMacParams.MinRxSymbols;
            break;
        }
        default:
            status = LORAMAC_STATUS_SERVICE_UNKNOWN;
            break;
    }

    return status;
}

LoRaMacStatus_t LoRaMacMibSetRequestConfirm( MibRequestConfirm_t *mibSet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if( mibSet == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( LoRaMacState & LORAMAC_TX_RUNNING ) == LORAMAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    switch( mibSet->Type )
    {
        case MIB_DEVICE_CLASS:
        {
            LoRaMacDeviceClass = mibSet->Param.Class;
            switch( LoRaMacDeviceClass )
            {
                case CLASS_A:
                {
                    // Set the radio into sleep to setup a defined state
                    Radio.Sleep( );
                    break;
                }
                case CLASS_B:
                {
                    break;
                }
                case CLASS_C:
                {
                    // Set the NodeAckRequested indicator to default
                    NodeAckRequested = false;
                    OnRxWindow2TimerEvent( );
                    break;
                }
            }
            break;
        }
        case MIB_NETWORK_JOINED:
        {
            IsLoRaMacNetworkJoined = mibSet->Param.IsNetworkJoined;
            break;
        }
        case MIB_ADR:
        {
            AdrCtrlOn = mibSet->Param.AdrEnable;
            break;
        }
        case MIB_NET_ID:
        {
            LoRaMacNetID = mibSet->Param.NetID;
            break;
        }
        case MIB_DEV_ADDR:
        {
            LoRaMacDevAddr = mibSet->Param.DevAddr;
            break;
        }
        case MIB_NWK_SKEY:
        {
            if( mibSet->Param.NwkSKey != NULL )
            {
                memcpy1( LoRaMacNwkSKey, mibSet->Param.NwkSKey,
                               sizeof( LoRaMacNwkSKey ) );
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_APP_SKEY:
        {
            if( mibSet->Param.AppSKey != NULL )
            {
                memcpy1( LoRaMacAppSKey, mibSet->Param.AppSKey,
                               sizeof( LoRaMacAppSKey ) );
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_PUBLIC_NETWORK:
        {
            PublicNetwork = mibSet->Param.EnablePublicNetwork;
            Radio.SetPublicNetwork( PublicNetwork );
            break;
        }
        case MIB_REPEATER_SUPPORT:
        {
             RepeaterSupport = mibSet->Param.EnableRepeaterSupport;
            break;
        }
        case MIB_RX2_CHANNEL:
        {
            LoRaMacParams.Rx2Channel = mibSet->Param.Rx2Channel;
            break;
        }
        case MIB_RX2_DEFAULT_CHANNEL:
        {
            LoRaMacParamsDefaults.Rx2Channel = mibSet->Param.Rx2DefaultChannel;
            break;
        }
        case MIB_CHANNELS_DEFAULT_MASK:
        {
            if( mibSet->Param.ChannelsDefaultMask )
            {
                memcpy1( ( uint8_t* ) LoRaMacParamsDefaults.ChannelsMask,
                         ( uint8_t* ) mibSet->Param.ChannelsDefaultMask, sizeof( LoRaMacParamsDefaults.ChannelsMask ) );
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_MASK:
        {
            if( mibSet->Param.ChannelsMask )
            {

                memcpy1( ( uint8_t* ) LoRaMacParams.ChannelsMask,
                         ( uint8_t* ) mibSet->Param.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );

            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_NB_REP:
        {
            if( ( mibSet->Param.ChannelNbRep >= 1 ) &&
                ( mibSet->Param.ChannelNbRep <= 15 ) )
            {
                LoRaMacParams.ChannelsNbRep = mibSet->Param.ChannelNbRep;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_MAX_RX_WINDOW_DURATION:
        {
            LoRaMacParams.MaxRxWindow = mibSet->Param.MaxRxWindow;
            break;
        }
        case MIB_RECEIVE_DELAY_1:
        {
            LoRaMacParams.ReceiveDelay1 = mibSet->Param.ReceiveDelay1;
            break;
        }
        case MIB_RECEIVE_DELAY_2:
        {
            LoRaMacParams.ReceiveDelay2 = mibSet->Param.ReceiveDelay2;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_1:
        {
            LoRaMacParams.JoinAcceptDelay1 = mibSet->Param.JoinAcceptDelay1;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_2:
        {
            LoRaMacParams.JoinAcceptDelay2 = mibSet->Param.JoinAcceptDelay2;
            break;
        }
        case MIB_CHANNELS_DEFAULT_DATARATE:
        {
            if( ValueInRange( mibSet->Param.ChannelsDefaultDatarate,
                              LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) )
            {
                LoRaMacParamsDefaults.ChannelsDatarate = mibSet->Param.ChannelsDefaultDatarate;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_DATARATE:
        {
            if( ValueInRange( mibSet->Param.ChannelsDatarate,
                              LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) )
            {
                LoRaMacParams.ChannelsDatarate = mibSet->Param.ChannelsDatarate;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_DEFAULT_TX_POWER:
        {
            if( ValueInRange( mibSet->Param.ChannelsDefaultTxPower,
                              LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) )
            {
                LoRaMacParamsDefaults.ChannelsTxPower = mibSet->Param.ChannelsDefaultTxPower;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_TX_POWER:
        {
            if( ValueInRange( mibSet->Param.ChannelsTxPower,
                              LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) )
            {
                LoRaMacParams.ChannelsTxPower = mibSet->Param.ChannelsTxPower;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_UPLINK_COUNTER:
        {
            UpLinkCounter = mibSet->Param.UpLinkCounter;
            break;
        }
        case MIB_DOWNLINK_COUNTER:
        {
            DownLinkCounter = mibSet->Param.DownLinkCounter;
            break;
        }
        case MIB_SYSTEM_MAX_RX_ERROR:
        {
            LoRaMacParams.SystemMaxRxError = LoRaMacParamsDefaults.SystemMaxRxError = mibSet->Param.SystemMaxRxError;
            break;
        }
        case MIB_MIN_RX_SYMBOLS:
        {
            LoRaMacParams.MinRxSymbols = LoRaMacParamsDefaults.MinRxSymbols = mibSet->Param.MinRxSymbols;
            break;
        }
        default:
            status = LORAMAC_STATUS_SERVICE_UNKNOWN;
            break;
    }

    return status;
}

LoRaMacStatus_t LoRaMacChannelAdd( uint8_t id, ChannelParams_t params )
{

    return LORAMAC_STATUS_PARAMETER_INVALID;
}

LoRaMacStatus_t LoRaMacChannelRemove( uint8_t id )
{

    return LORAMAC_STATUS_PARAMETER_INVALID;
}

LoRaMacStatus_t LoRaMacMulticastChannelLink( MulticastParams_t *channelParam )
{
    if( channelParam == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( LoRaMacState & LORAMAC_TX_RUNNING ) == LORAMAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    // Reset downlink counter
    channelParam->DownLinkCounter = 0;
    channelParam->DownLinkTotal = 0;

    if( MulticastChannels == NULL )
    {
        // New node is the fist element
        MulticastChannels = channelParam;
    }
    else
    {
        MulticastParams_t *cur = MulticastChannels;

        // Search the last node in the list
        while( cur->Next != NULL )
        {
            cur = cur->Next;
        }
        // This function always finds the last node
        cur->Next = channelParam;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMulticastChannelUnlink( MulticastParams_t *channelParam )
{
    if( channelParam == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( LoRaMacState & LORAMAC_TX_RUNNING ) == LORAMAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    if( MulticastChannels != NULL )
    {
        if( MulticastChannels == channelParam )
        {
          // First element
          MulticastChannels = channelParam->Next;
        }
        else
        {
            MulticastParams_t *cur = MulticastChannels;

            // Search the node in the list
            while( cur->Next && cur->Next != channelParam )
            {
                cur = cur->Next;
            }
            // If we found the node, remove it
            if( cur->Next )
            {
                cur->Next = channelParam->Next;
            }
        }
        channelParam->Next = NULL;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMlmeRequest( MlmeReq_t *mlmeRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;

    if( mlmeRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( LoRaMacState & LORAMAC_TX_RUNNING ) == LORAMAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;//如果正好处于发送中，则返回忙
    }

    memset1( ( uint8_t* ) &MlmeConfirm, 0, sizeof( MlmeConfirm ) );

    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR;

    switch( mlmeRequest->Type )
    {
        case MLME_JOIN:
        {
            if( ( LoRaMacState & LORAMAC_TX_DELAYED ) == LORAMAC_TX_DELAYED )
            {
                return LORAMAC_STATUS_BUSY; //处于发送后的的等待时期
            }

            if( ( mlmeRequest->Req.Join.DevEui == NULL ) ||
                ( mlmeRequest->Req.Join.AppEui == NULL ) ||
                ( mlmeRequest->Req.Join.AppKey == NULL ) ||
                ( mlmeRequest->Req.Join.NbTrials == 0 ) )
            {
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }


            // Enables at least the usage of all datarates. //默认尝试48次，个逗逼
//            if( mlmeRequest->Req.Join.NbTrials < 48 )
//            {
//                mlmeRequest->Req.Join.NbTrials = 48;
//            }


            LoRaMacFlags.Bits.MlmeReq = 1;
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;

            LoRaMacDevEui = mlmeRequest->Req.Join.DevEui;
            LoRaMacAppEui = mlmeRequest->Req.Join.AppEui;
            LoRaMacAppKey = mlmeRequest->Req.Join.AppKey;
						
            MaxJoinRequestTrials = mlmeRequest->Req.Join.NbTrials;

            // Reset variable JoinRequestTrials
            JoinRequestTrials = 0;

            // Setup header information
            macHdr.Value = 0;
            macHdr.Bits.MType  = FRAME_TYPE_JOIN_REQ;

            ResetMacParameters( );

            // Add a +1, since we start to count from 0
            LoRaMacParams.ChannelsDatarate = AlternateDatarate( JoinRequestTrials + 1 );
						//置为打开接收窗口
            lorawan_join_flag = 0;
            status = Send( &macHdr, 0, NULL, 0 );
            break;
        }
        case MLME_LINK_CHECK:
        {
            LoRaMacFlags.Bits.MlmeReq = 1;
            // LoRaMac will send this command piggy-pack
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;

            status = AddMacCommand( MOTE_MAC_LINK_CHECK_REQ, 0, 0 );
            break;
        }
        case MLME_TXCW:
        {
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;
            LoRaMacFlags.Bits.MlmeReq = 1;
            status = SetTxContinuousWave( mlmeRequest->Req.TxCw.Timeout );
            break;
        }
        case MLME_TXCW_1:
        {
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;
            LoRaMacFlags.Bits.MlmeReq = 1;
            status = SetTxContinuousWave1( mlmeRequest->Req.TxCw.Timeout, mlmeRequest->Req.TxCw.Frequency, mlmeRequest->Req.TxCw.Power );
            break;
        }
        default:
            break;
    }

    if( status != LORAMAC_STATUS_OK )
    {
        NodeAckRequested = false;
        LoRaMacFlags.Bits.MlmeReq = 0;
    }

    return status;
}

LoRaMacStatus_t LoRaMacMcpsRequest( McpsReq_t *mcpsRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;
    uint8_t fPort = 0;
    void *fBuffer;
    uint16_t fBufferSize;
    int8_t datarate;
    bool readyToSend = false;

    if( mcpsRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( ( LoRaMacState & LORAMAC_TX_RUNNING ) == LORAMAC_TX_RUNNING ) ||
        ( ( LoRaMacState & LORAMAC_TX_DELAYED ) == LORAMAC_TX_DELAYED ) )
    {
        return LORAMAC_STATUS_BUSY;
    }

    macHdr.Value = 0;
    memset1 ( ( uint8_t* ) &McpsConfirm, 0, sizeof( McpsConfirm ) );
    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR;

    switch( mcpsRequest->Type )
    {
        case MCPS_UNCONFIRMED:
        {
            readyToSend = true;
            AckTimeoutRetries = 1;

            macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
            fPort = mcpsRequest->Req.Unconfirmed.fPort;
            fBuffer = mcpsRequest->Req.Unconfirmed.fBuffer;
            fBufferSize = mcpsRequest->Req.Unconfirmed.fBufferSize;
            datarate = mcpsRequest->Req.Unconfirmed.Datarate;
            break;
        }
        case MCPS_CONFIRMED:
        {
            readyToSend = true;
            AckTimeoutRetriesCounter = 1;
            AckTimeoutRetries = mcpsRequest->Req.Confirmed.NbTrials;

            macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
            fPort = mcpsRequest->Req.Confirmed.fPort;
            fBuffer = mcpsRequest->Req.Confirmed.fBuffer;
            fBufferSize = mcpsRequest->Req.Confirmed.fBufferSize;
            datarate = mcpsRequest->Req.Confirmed.Datarate;
            break;
        }
        case MCPS_PROPRIETARY:
        {
            readyToSend = true;
            AckTimeoutRetries = 1;

            macHdr.Bits.MType = FRAME_TYPE_PROPRIETARY;
            fBuffer = mcpsRequest->Req.Proprietary.fBuffer;
            fBufferSize = mcpsRequest->Req.Proprietary.fBufferSize;
            datarate = mcpsRequest->Req.Proprietary.Datarate;
            break;
        }
        default:
            break;
    }

    if( readyToSend == true )
    {
        if( AdrCtrlOn == false )
        {
            if( ValueInRange( datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == true )
            {
                LoRaMacParams.ChannelsDatarate = datarate;
            }
            else
            {
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }
        }

        status = Send( &macHdr, fPort, fBuffer, fBufferSize );
        if( status == LORAMAC_STATUS_OK )
        {
            McpsConfirm.McpsRequest = mcpsRequest->Type;
            LoRaMacFlags.Bits.McpsReq = 1;
        }
        else
        {
            NodeAckRequested = false;
        }
    }

    return status;
}

void LoRaMacTestRxWindowsOn( bool enable )
{
    IsRxWindowsEnabled = enable;
}

void LoRaMacTestSetMic( uint16_t txPacketCounter )
{
    UpLinkCounter = txPacketCounter;
    IsUpLinkCounterFixed = true;
}

void LoRaMacTestSetDutyCycleOn( bool enable )
{
    DutyCycleOn = false;

}

void LoRaMacTestSetChannel( uint8_t channel )
{
    Channel = channel;
}

static RxConfigParams_t ComputeRxWindowParameters( int8_t datarate, uint32_t rxError )
{
    RxConfigParams_t rxConfigParams = { 0, 0, 0, 0 };
    double tSymbol = 0.0;

    rxConfigParams.Datarate = datarate;
    switch( Bandwidths[datarate] )
    {
        default:
        case 125000:
            rxConfigParams.Bandwidth = 0;
            break;
        case 250000:
            rxConfigParams.Bandwidth = 1;
            break;
        case 500000:
            rxConfigParams.Bandwidth = 2;
            break;
    }


    { // LoRa
        tSymbol = ( ( double )( 1 << Datarates[datarate] ) / ( double )Bandwidths[datarate] ) * 1e3;
    }

    rxConfigParams.RxWindowTimeout = MAX( ( uint32_t )ceil( ( ( 2 * LoRaMacParams.MinRxSymbols - 8 ) * tSymbol + 2 * rxError ) / tSymbol ), LoRaMacParams.MinRxSymbols ); // Computed number of symbols

    rxConfigParams.RxOffset = ( int32_t )ceil( ( 4.0 * tSymbol ) - ( ( rxConfigParams.RxWindowTimeout * tSymbol ) / 2.0 ) - RADIO_WAKEUP_TIME );

    return rxConfigParams;
}

void SetLoraMacTestFlag( bool status )
{
    LoraMacTestFlag = status;
}



//lorawan线程，用于处理外部请求和超时事件
//事件类型: 周期状态更新时间
void lorawan_core_thread_entry(void *p)
{
   rt_uint32_t event;
   while(1)
   {
      rt_event_recv(&lorawan_core_event, (LORAWAN_CORE_EVENT_ALL),RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &event);
	    if(event & LORAWAN_CORE_EVENT_STATE_CHECK)//周期更新mac标志位的状态
	    {
		      OnMacStateCheckTimerEvent();
	    }
      if(event & LORAWAN_CORE_EVENT_TXDELAY_TIMEOUT)
      {
          OnTxDelayedTimerEvent();
      }
      if(event & LORAWAN_CORE_EVENT_WIN1_TIMEOUT)
      {
          OnRxWindow1TimerEvent();
      }
      if(event & LORAWAN_CORE_EVENT_WIN2_TIMEOUT)
      {
          OnRxWindow2TimerEvent();
      }
      if(event & LORAWAN_CORE_EVENT_ACK_TIMEOUT)
      {
          OnAckTimeoutTimerEvent();
      }
   }
}

//删除进程,清除事件
void lorawan_core_thread_detele()
{
	 //rt_event_detach(&lorawan_core_event); //如果删除事件，则如果正在发送事件的话，则程序死掉
   rt_thread_delete(lorawan_core_hander);
}

//上行包号的读写
uint32_t lorawan_counter_wrtie(uint32_t data)
{
    UpLinkCounter = data;
	  return UpLinkCounter;
}

uint32_t lorawan_counter_read()
{
    return UpLinkCounter;
}

void lorawan_sleep()
{
   Radio.Sleep( );
}
