#ifndef __UART_H__
#define __UART_H__

#include "type.h"
#include "fifo.h"

/*!
 * UART IT flag
 */
#define UART_IT_FLAG_TX                      ((uint16_t)0x0001)
#define UART_IT_FLAG_RX                      ((uint16_t)0x0002)
#define UART_IT_FLAG_CTS                     ((uint16_t)0x0004)
#define UART_IT_FLAG_PE                      ((uint16_t)0x0008) 

/*!
 * \brief Uart IRQ handler function prototype
 *
 * \param [IN] notify     UART notification[0:UART_NOTIFY_TX, 1:UART_NOTIFY_RX, 2:UART_NOTIFY_CTS, 3:UART_NOTIFY_PE]
 *
 * \param [IN] value      Received data
 */
typedef void (* UartIrqHandler_t)( uint8_t notify, uint16_t value );
typedef void (* UartIdelHandler_t)(void);

/*!
 * UART notification identifier
 */
typedef enum
{
    UART_NOTIFY_TX = 0,
    UART_NOTIFY_RX,
    UART_NOTIFY_CTS,
    UART_NOTIFY_PE
}UartNotify;

/*!
 * UART operation mode
 */
typedef enum
{
    TX_ONLY = 0,
    RX_ONLY,
    RX_TX
}UartMode_t;

/*!
 * UART word length
 */
typedef enum
{
    UART_7_BIT = 0,
    UART_8_BIT,
}WordLength_t;

/*!
 * UART stop bits
 */
typedef enum
{
    UART_1_STOP_BIT = 0,
    UART_0_5_STOP_BIT,
    UART_2_STOP_BIT,
    UART_1_5_STOP_BIT
}StopBits_t;

/*!
 * UART parity
 */
typedef enum
{
    NO_PARITY = 0,
    EVEN_PARITY,
    ODD_PARITY
}Parity_t;

/*!
 * UART flow control
   硬件流控制模式
 */
typedef enum
{
    NO_FLOW_CTRL = 0,  //使能
    RTS_FLOW_CTRL,     //发送请求RTS使能
    CTS_FLOW_CTRL,     //清除发送CTS使能
    RTS_CTS_FLOW_CTRL  //RTS  CTS使能
}FlowCtrl_t;

/*!
 * \brief Initializes the UART peripheral and MCU pins
 *
 * \param [IN] uartNo       UART number
 * \param [IN] mode         Uart operation mode  
 * \param [IN] baudrate     UART baudrate
 * \param [IN] wordLength   UART word length 数据位
 * \param [IN] parity       UART parity 检验位
 * \param [IN] flowCtrl     UART flow control
 */
void UartInit( uint8_t uartNo, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl );

/*!
 * \brief De-initializes the UART peripheral and MCU pins
 *
 * \param [IN] uartNo       UART number
 * \param [IN] flowCtrl     UART flow control
 */
void UartDeInit( uint8_t uartNo, FlowCtrl_t flowCtrl );

/*!
 * \brief Sends a character to the UART
 *
 * \param [IN] uartNo       UART number
 * \param [IN] data         Character to be sent
 * \param [IN] delay        Whether or not waiting until trasmitting finish[true:need, false:no need]
 */
void UartPutChar( uint8_t uartNo, uint16_t data, bool delay );

/*!
 * \brief Sends a character string to the UART
 *
 * \param [IN] uartNo       UART number
 * \param [IN] str          Character string to be sent
 */
void UartPutString( uint8_t uartNo, char *str );

/*!
 * \brief Wait sending over
 *
 * \param [IN] uartNo       UART number
 */
void UartWaitFinish( uint8_t uartNo );

/*!
 * \brief Sets UART Irq parameters
 *
 * \param [IN] uartNo       UART number
 * \param [IN] ITFlag       UART IT flag
 * \param [IN] prePriority  Pre-emption priority for the UART
 * \param [IN] subPriority  Subpriority level for the UART
 * \param [IN] irqHandler   Callback function pointer 串口回调 两个参数（UartNotify a,uint16_t b）
 */
void UartSetInterrupt( uint8_t uartNo, uint16_t ITFlag, uint8_t prePriority, uint8_t subPriority, UartIrqHandler_t irqHandler );

/*!
 * \brief Sets the interrupt type of the UART
 *
 * \param [IN] uartNo       UART number
 * \param [IN] ITFlag       UART IT flag
 * \param [IN] enable       UART IT status
 */
void UartITConfig( uint8_t uartNo, uint16_t ITFlag, bool enable );

/*!
 * \brief Removes the interrupt from the UART
 *
 * \param [IN] uartNo       UART number
 */
void UartRemoveInterrupt( uint8_t uartNo );

void UartPutChars(uint8_t uartNo,uint8_t *data,uint16_t lenth);

void UartSetIdleCallbackInit(uint8_t uartNo, UartIdelHandler_t idelHander );

#endif  // __UART_H__
