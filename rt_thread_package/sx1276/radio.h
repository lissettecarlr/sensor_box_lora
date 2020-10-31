#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdint.h>
#include <stdbool.h>

/*!
 * 射频模式: FSK和LORA
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

/*!
 * 射频的状态:闲置，发送中，接收中，信道检测
 */
typedef enum
{
    RF_IDLE = 0,   //!< The radio is idle
    RF_RX_RUNNING, //!< The radio is in reception state
    RF_TX_RUNNING, //!< The radio is in transmission state
    RF_CAD,        //!< The radio is doing channel activity detection
}RadioState_t;

/*!
 *  提供给外部使用者的回调函数
 */
typedef struct
{
    /*!
     * 发送完成
     */
    void    ( *TxDone )( void );
    /*!
     *  发送超时
     */
    void    ( *TxTimeout )( void );
    /*!
     * 接收完成
     * \param [IN] payload 数据buffer
     * \param [IN] size    数据长度
     * \param [IN] rssi    [dBm]
     * \param [IN] snr     信噪比
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     */
    void    ( *RxDone )( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
    /*!
     * 接收超时.
     */
    void    ( *RxTimeout )( void );
    /*!
     * 接收错误
     */
    void    ( *RxError )( void );
    /*!
     * \brief 调频通道选择回调
     *
     * \param [IN] currentChannel   返回当前的通道
     */
    void ( *FhssChangeChannel )( uint8_t currentChannel );

    /*!
     * \brief 信道检测完成回调.
     *
     * \param [IN] channelDetected    信道检测情况
     */
    void ( *CadDone ) ( bool channelActivityDetected );
}RadioEvents_t;

/*!
 * 射频的各种功能性函数指针，射频驱动实现填充后，协议处直接使用实例的结构体，
 * 这样即是射频芯片更换，只需要重新填充结构体中的指针，而无需修改协议里面的使用
 */
struct Radio_s
{
    /*!
     * 引脚初始化
     */
    void    ( *IoInit )( void );
    /*!
     * 引脚反初始化，低功耗时候需要
     */
    void    ( *IoDeInit )( void );
    /*!
	   * \brief 射频初始化函数，参数即是上文的回调结构体
     */
    uint32_t    ( *Init )( RadioEvents_t *events );
    /*!
	   * 返回射频状态，而状态类型也是上文的结构体，空闲，发送中，接收中，信道检测
     */
    RadioState_t ( *GetStatus )( void );
    /*!
	   * \brief 设置射频的模式，结构体也在上文定义，FSK和LORA
     */
    void    ( *SetModem )( RadioModems_t modem );
    /*!
     * \brief 设置通道频率
     */
    void    ( *SetChannel )( uint32_t freq );
    /*!
     * \brief 信道空闲检查
     *
     * \param [IN] modem      射频的模式，FSK还是LORA
     * \param [IN] freq       信道频率
     * \param [IN] rssiThresh RSSI的阈值
     * \param [IN] maxCarrierSenseTime 最大测量时间
     *
     * \retval isFree         返回是否空闲，true为空闲，false为忙
     */
    bool    ( *IsChannelFree )( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime );
    /*!
     * \brief 根据RSSI计算一个随机值
     * \remark 该函数将射频切换为LORA模式且关闭所有中断
     *         在此之后必须调用Radio.SetRxConfig或者Radio.SetTxConfig中的一个函数
     *
     * \retval randomValue    返回32位随机数       
     */
    uint32_t ( *Random )( void );
    /*!
     * \brief 配置接收
     *
     * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
     * \param [IN] bandwidth    设置带宽
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     * \param [IN] datarate     设置速率
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     * \param [IN] coderate     设置编码率 (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: N/A ( set to 0 )
     * \param [IN] preambleLen  设置前导码长度
     *                          FSK : Number of bytes
     *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
     * \param [IN] symbTimeout  设置单次接收超时时间
     *                          FSK : timeout in number of bytes
     *                          LoRa: timeout in symbols(Symbol表示一个扩频符号,Ts=2^SF/BW secs)
     * \param [IN] fixLen       数据包长度是否可变 [0: 固定, 1: 固定]
     * \param [IN] payloadLen   当为固定长度时设置负载的长度
     * \param [IN] crcOn        是否开启CRC [0: OFF, 1: ON]
     * \param [IN] freqHopOn    是否开启跳频
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: OFF, 1: ON]
     * \param [IN] hopPeriod    设置每跳之间的扩频符号数
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: Number of symbols
     * \param [IN] iqInverted   是否反向IQ信号
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     * \param [IN] rxContinuous 设置接收模式是单次还是连续
     *                          [false: single mode, true: continuous mode]
     */
    void    ( *SetRxConfig )( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint32_t bandwidthAfc, uint16_t preambleLen,
                              uint16_t symbTimeout, bool fixLen,
                              uint8_t payloadLen,
                              bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                              bool iqInverted, bool rxContinuous );
    /*!
     * \brief 设置发送模式
     *
     * \param [IN] modem        射频模式
     * \param [IN] power        设置输出功率 [dBm]
     * \param [IN] fdev         设置频率偏差 (FSK only)
     *                          FSK : [Hz]
     *                          LoRa: 0
     * \param [IN] bandwidth    设置带宽 (LoRa only)
     *                          FSK : 0
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     * \param [IN] datarate     设置速率
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     * \param [IN] coderate     设置编码率 (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     * \param [IN] preambleLen  设置前导码长度
     *                          FSK : Number of bytes
     *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
     * \param [IN] fixLen       是否固定数据包长度 [0: variable, 1: fixed]
     * \param [IN] crcOn        是否开启CRC [0: OFF, 1: ON]
     * \param [IN] freqHopOn    是否开启跳频
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: OFF, 1: ON]
     * \param [IN] hopPeriod    设置每跳之间的扩频符号数
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: Number of symbols
     * \param [IN] iqInverted   是否反正IQ信号 (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     * \param [IN] timeout      发送超时时间 [ms]
     */
    void    ( *SetTxConfig )( RadioModems_t modem, int8_t power, uint32_t fdev,
                              uint32_t bandwidth, uint32_t datarate,
                              uint8_t coderate, uint16_t preambleLen,
                              bool fixLen, bool crcOn, bool freqHopOn,
                              uint8_t hopPeriod, bool iqInverted, uint32_t timeout );
    /*!
     * \brief 校验硬件是否支持该频率
     *
     * \param [IN] frequency 需要校验的频率
		 * \retval isSupported [true: 支持, false: 不支持]
     */
    bool    ( *CheckRfFrequency )( uint32_t frequency );
    /*!
     * \brief 计算负载空中传输时间
     *
     * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
     *
     * \param [IN] modem      射频的模式 [0: FSK, 1: LoRa]
     * \param [IN] pktLen     负载长度
     *
     * \retval airTime        返回计算后的时间单位ms
     */
    uint32_t  ( *TimeOnAir )( RadioModems_t modem, uint8_t pktLen );
    /*!
     * \brief 发送指定长度的数据, 需要在传输之前配置好射频
     * \param [IN]: buffer     待发送的buffer
     * \param [IN]: size       buffer长度
     */
    void    ( *Send )( uint8_t *buffer, uint8_t size );
    /*!
     * \brief 使射频进入休眠模式
     */
    void    ( *Sleep )( void );
    /*!
     * \brief 使射频进入待机模式
     */
    void    ( *Standby )( void );
    /*!
		 * \brief 将射频配置为接收模式
     * \param [IN] 接收超时时间 [ms] 0则表示连续接收
     */
    void    ( *Rx )( uint32_t timeout );
    /*!
     * \brief 开始信道活跃检查
     */
    void    ( *StartCad )( void );
    /*!
		 * \brief 将射频配置为连续接收模式
     *
     * \param [IN]: freq       信道的频率
     * \param [IN]: power      输出功率 [dBm]
     * \param [IN]: time       传输超时时间 [s]
     */
    void    ( *SetTxContinuousWave )( uint32_t freq, int8_t power, uint16_t time );
    /*!
     * \brief 读取当前的RSSI值 [dBm]
     */
    int16_t ( *Rssi )( RadioModems_t modem );
    /*!
     * \brief 对射频指定地址的寄存器写数据
     *
     * \param [IN]: addr Register address
     * \param [IN]: data New register value
     */
    void    ( *Write )( uint16_t addr, uint8_t data );
    /*!
     * \brief 读取指定地址的寄存器值
     *
     * \param [IN]: addr Register address
     * \retval data Register value
     */
    uint8_t ( *Read )( uint16_t addr );
    /*!
     * \brief 从指定地址的寄存器开始多个写入
     *
     * \param [IN] addr   First Radio register address
     * \param [IN] buffer Buffer containing the new register's values
     * \param [IN] size   Number of registers to be written
     */
    void    ( *WriteBuffer )( uint16_t addr, uint8_t *buffer, uint8_t size );
    /*!
     * \brief 从指定地址寄存器位置开始多个读出
     *
     * \param [IN] addr First Radio register address
     * \param [OUT] buffer Buffer where to copy the registers data
     * \param [IN] size Number of registers to be read
     */
    void    ( *ReadBuffer )( uint16_t addr, uint8_t *buffer, uint8_t size );
    /*!
     * \brief 设置最大负载.
     * \param [IN] modem      射频模式 [0: FSK, 1: LoRa]
     * \param [IN] max        最大负载字节长度
     */
    void    ( *SetMaxPayloadLength )( RadioModems_t modem, uint8_t max );
    /*!
     * \brief 配置网络是功耗还是私网，将会更新同步字，仅适用于LORA模式
     *
     * \param [IN] enable if true, it enables a public network
     */
    void    ( *SetPublicNetwork )( bool enable );
    /*!
     * \brief 得到射频唤醒所需时间，ms
     */
    uint32_t  ( *GetWakeupTime )( void );
    /*!
     * \brief 射频优先级处理
     */
    void ( *IrqProcess )( void );
    /*
     * 下列函数仅适用于SX126X
     */
    /*!
     * \brief Sets the radio in reception mode with Max LNA gain for the given time
     *
     * \remark Available on SX126x radios only.
     * 
     * \param [IN] timeout Reception timeout [ms]
     *                     [0: continuous, others timeout]
     */
    void    ( *RxBoosted )( uint32_t timeout );
    /*!
     * \brief Sets the Rx duty cycle management parameters
     *
     * \remark Available on SX126x radios only.
     *
     * \param [in]  rxTime        Structure describing reception timeout value
     * \param [in]  sleepTime     Structure describing sleep timeout value
     */
    void ( *SetRxDutyCycle ) ( uint32_t rxTime, uint32_t sleepTime );
};

/*!
 * \brief Radio driver
 *
 * \remark This variable is defined and initialized in the specific radio
 *         board implementation
 */
extern const struct Radio_s Radio;

#endif // __RADIO_H__
