#include "uart.h"
#include "gpio.h"


/*!
 * Uart IRQ handler
 */
static UartIrqHandler_t UartIrq[3];

//空闲回调
static UartIdelHandler_t UartIdel[3];
/*!
 * Uart word length
 */
static WordLength_t UartWordLength[3];

void UartInit( uint8_t uartNo, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    USART_InitTypeDef USART_InitStructure;
    USART_TypeDef* uart;
    uint8_t gpio_AF;
    PinName_t tx;
    PinName_t rx;
    PinName_t cts;
    PinName_t rts;
    
    if( uartNo == 0 )
    {
        uart = USART1;
        gpio_AF = GPIO_AF_USART1;
        tx = USART1_TX;
        rx = USART1_RX;
        cts = USART1_CTS;
        rts = USART1_RTS;
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );
    }
    else if( uartNo == 1 )
    {
        uart = USART2;
        gpio_AF = GPIO_AF_USART2;
        tx = USART2_TX;
        rx = USART2_RX;
        cts = USART2_CTS;
        rts = USART2_RTS;
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
    }
    else if( uartNo == 2 )
    {
        uart = USART3;
        gpio_AF = GPIO_AF_USART3;
        tx = USART3_TX;
        rx = USART3_RX;
        cts = USART3_CTS;
        rts = USART3_RTS;
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );
    }
    else
    {
        while(1);
    }

    GpioInit( tx, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_UP, gpio_AF );
    GpioInit( rx, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_UP, gpio_AF );
    
    if( mode == TX_ONLY )
    {
        USART_InitStructure.USART_Mode = USART_Mode_Tx;
    }
    else if( mode == RX_ONLY )
    {
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
    }
    else if( mode == RX_TX )
    {
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    }
       
    USART_InitStructure.USART_BaudRate = baudrate;

    if( (wordLength == UART_7_BIT) && (parity != NO_PARITY) )
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        if( parity == EVEN_PARITY )
        {
            USART_InitStructure.USART_Parity = USART_Parity_Even;
        }
        else
        {
            USART_InitStructure.USART_Parity = USART_Parity_Odd;
        }
    }
    else if( (wordLength == UART_8_BIT) && (parity == NO_PARITY) )
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    else if( (wordLength == UART_8_BIT) && (parity != NO_PARITY) )
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
        if( parity == EVEN_PARITY )
        {
            USART_InitStructure.USART_Parity = USART_Parity_Even;
        }
        else
        {
            USART_InitStructure.USART_Parity = USART_Parity_Odd;
        }
    }
    else
    {
        while(1);
    }

    if( stopBits == UART_1_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    }
    else if( stopBits == UART_0_5_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
    }
    else if( stopBits == UART_2_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }
    else if( stopBits == UART_1_5_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
    }

    if( flowCtrl == NO_FLOW_CTRL )
    {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    }
    else if( flowCtrl == RTS_FLOW_CTRL ) 
    {
        GpioInit( rts, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_UP, gpio_AF );
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;
    }
    else if( flowCtrl == CTS_FLOW_CTRL ) 
    {
        GpioInit( cts, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_UP, gpio_AF );
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
    }
    else if( flowCtrl == RTS_CTS_FLOW_CTRL ) 
    {
        GpioInit( rts, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_UP, gpio_AF );
        GpioInit( cts, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_UP, gpio_AF );
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    }
    USART_Init( uart, &USART_InitStructure );
    USART_Cmd( uart, ENABLE );
}

void UartDeInit( uint8_t uartNo, FlowCtrl_t flowCtrl )
{
    USART_TypeDef* uart;
    PinName_t tx;
    PinName_t rx;
    PinName_t cts;
    PinName_t rts;
    
    if( uartNo == 0 )
    {
        uart = USART1;
        tx = USART1_TX;
        rx = USART1_RX;
        cts = USART1_CTS;
        rts = USART1_RTS;
    }
    else if( uartNo == 1 )
    {
        uart = USART2;
        tx = USART2_TX;
        rx = USART2_RX;
        cts = USART2_CTS;
        rts = USART2_RTS;
    }
    else if( uartNo == 2 )
    {
        uart = USART3;
        tx = USART3_TX;
        rx = USART3_RX;
        cts = USART3_CTS;
        rts = USART3_RTS;
    }
    else
    {
        while(1);
    }
    
    while( USART_GetFlagStatus( uart, USART_FLAG_TC ) == RESET )
        ;
    
    USART_Cmd( uart, DISABLE );
    
    GpioInit( tx, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 0 );
    GpioInit( rx, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 0 );
    
    if( flowCtrl == RTS_FLOW_CTRL ) 
    {
        GpioInit( rts, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 0 );
    }
    else if( flowCtrl == CTS_FLOW_CTRL ) 
    {
        GpioInit( cts, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 0 );
    }
    else if( flowCtrl == RTS_CTS_FLOW_CTRL ) 
    {
        GpioInit( rts, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 0 );
        GpioInit( cts, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 0 );
    }
    
    USART_DeInit( uart );
}

void UartPutChar( uint8_t uartNo, uint16_t data, bool delay )
{
    USART_TypeDef* uart;
    
    if( uartNo == 0 )
    {
        uart = USART1;
    }
    else if( uartNo == 1 )
    {
        uart = USART2;
    }
    else if( uartNo == 2 )
    {
        uart = USART3;
    }
    else
    {
        while(1);
    }
    
    if( UartWordLength == UART_7_BIT )
    {
        USART_SendData( uart, data & 0x7F );
    }
    else
    {
        USART_SendData( uart, data & 0xFF );
    }
    if( delay == true )
    {
        while( USART_GetFlagStatus( uart, USART_FLAG_TXE ) == RESET )
            ;
    }
}

void UartPutString( uint8_t uartNo, char *str )
{
    uint16_t len, i;
    
    len = strlen( (char *)str );
    for( i = 0; i < len; i++ )
    {
        UartPutChar( uartNo, str[i], true );
    }
}

void UartPutChars(uint8_t uartNo,uint8_t *data,uint16_t lenth)
{
	uint16_t i;
  for( i = 0; i < lenth; i++ )
	{
    UartPutChar( uartNo, data[i], true );
	}
}

void UartWaitFinish( uint8_t uartNo )
{
    USART_TypeDef* uart;
    
    if( uartNo == 0 )
    {
        uart = USART1;
    }
    else if( uartNo == 1 )
    {
        uart = USART2;
    }
    else if( uartNo == 2 )
    {
        uart = USART3;
    }
    else
    {
        while(1);
    }
    
    while( USART_GetFlagStatus( uart, USART_FLAG_TC ) == RESET )
        ;
}

//给空闲回调赋值
void UartSetIdleCallbackInit(uint8_t uartNo, UartIdelHandler_t idelHander )
{
	if( uartNo == 0 )
	{
	   UartIdel[0] = idelHander;
		 UartITConfig(0,USART_IT_IDLE,true); //打开空闲中断
	}
	else if( uartNo == 1 )
	{
		 UartITConfig(1,USART_IT_IDLE,true); //打开空闲中断
	   UartIdel[1] = idelHander;
	}
	else if(uartNo ==2)
	{
		 UartITConfig(2,USART_IT_IDLE,true); //打开空闲中断
	   UartIdel[2] = idelHander;
	}
}

void UartSetInterrupt( uint8_t uartNo, uint16_t ITFlag, uint8_t prePriority, uint8_t subPriority, UartIrqHandler_t irqHandler )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_TypeDef* uart;
    
    if( uartNo == 0 )
    {
        uart = USART1;
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        UartIrq[0] = irqHandler;
    }
    else if( uartNo == 1 )
    {
        uart = USART2;
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        UartIrq[1] = irqHandler;
    }
    else if( uartNo == 2 )
    {
        uart = USART3;
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        UartIrq[2] = irqHandler;
    }
    else
    {
        while(1);
    }
    
    USART_Cmd( uart, DISABLE );
    
    if( ITFlag & UART_IT_FLAG_RX )
    {
        USART_ITConfig( uart, USART_IT_RXNE, ENABLE ); //接收中断
    }
    if( ITFlag & UART_IT_FLAG_CTS )
    {
        USART_ITConfig( uart, USART_IT_CTS, ENABLE );
    }
    if( ITFlag & UART_IT_FLAG_PE )
    {
        USART_ITConfig( uart, USART_IT_PE, ENABLE );
    }
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prePriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
    
    USART_Cmd( uart, ENABLE );
}

void UartITConfig( uint8_t uartNo, uint16_t ITFlag, bool enable )
{
    USART_TypeDef* uart;
    
    if( uartNo == 0 )
    {
        uart = USART1;
    }
    else if( uartNo == 1 )
    {
        uart = USART2;
    }
    else if( uartNo == 2 )
    {
        uart = USART3;
    }
    else
    {
        while(1);
    }
    
    if( enable == true )
    {
        if( ITFlag & UART_IT_FLAG_TX )
        {
            USART_ITConfig( uart, USART_IT_TXE, ENABLE );
        }
				else if(ITFlag & USART_IT_IDLE)
				{
				    USART_ITConfig( uart, USART_IT_IDLE, ENABLE );
				}
				else
				{}
    }
    else
    {
        if( ITFlag & UART_IT_FLAG_TX )
        {
            USART_ITConfig( uart, USART_IT_TXE, DISABLE );
        }
				else if(ITFlag & USART_IT_IDLE)
				{
				    USART_ITConfig( uart, USART_IT_IDLE, DISABLE );
				}
				else
				{}
    }
}

void UartRemoveInterrupt( uint8_t uartNo )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    if( uartNo == 0 )
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    }
    else if( uartNo == 1 )
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    }
    else if( uartNo == 2 )
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    }
    else
    {
        while(1);
    }
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init( &NVIC_InitStructure );
}

void USART1_IRQHandler( void )
{
    uint16_t data;
    unsigned char temp;
	
    if( USART_GetITStatus( USART1, USART_IT_ORE_RX ) != RESET )
    {
        if( USART_GetFlagStatus(USART1, USART_FLAG_PE) == RESET )
        {
            data = USART_ReceiveData( USART1 );
            if( UartIrq[0] != NULL )
            {
                if( UartWordLength == UART_7_BIT )
                {
                    UartIrq[0]( UART_NOTIFY_RX, data & 0x7F );
                }
                else
                {
                    UartIrq[0]( UART_NOTIFY_RX, data & 0xFF );
                }
            }
        }
        else
        {
            USART_ReceiveData( USART1 );
        }
    }
    
    if( USART_GetITStatus( USART1, USART_IT_TXE ) != RESET )
    {    
        if( UartIrq[0] != NULL )
        {
            UartIrq[0]( UART_NOTIFY_TX, 0 );
        }
        USART_ClearITPendingBit (USART1, USART_IT_TXE);
    }
    
    if( USART_GetITStatus( USART1, USART_IT_RXNE ) != RESET )
    {
        if( USART_GetFlagStatus(USART1, USART_FLAG_PE) == RESET )
        {
            data = USART_ReceiveData( USART1 );
            if( UartIrq[0] != NULL )
            {
                if( UartWordLength == UART_7_BIT )
                {
                    UartIrq[0]( UART_NOTIFY_RX, data & 0x7F );
                }
                else
                {
                    UartIrq[0]( UART_NOTIFY_RX, data & 0xFF );
                }
            }
        }
        else
        {
            USART_ReceiveData( USART1 );
        }
        USART_ClearITPendingBit (USART1, USART_IT_RXNE);
    }

    if( USART_GetITStatus( USART1, USART_IT_CTS ) != RESET )
    {    
        data = USART_ReceiveData( USART1 );
        if( UartIrq[0] != NULL )
        {
            UartIrq[0]( UART_NOTIFY_CTS, 0 );
        }
        USART_ClearITPendingBit (USART1, USART_IT_CTS);
    }

    if( USART_GetITStatus( USART1, USART_IT_PE ) != RESET )
    {    
        data = USART_ReceiveData( USART1 );
        if( UartIrq[0] != NULL )
        {
            UartIrq[0]( UART_NOTIFY_PE, 0 );
        }
        USART_ClearITPendingBit (USART1, USART_IT_PE);
    }
		if(USART_GetITStatus(USART1,USART_IT_IDLE)==SET)
		{
			 temp=USART1->SR;
			 temp=USART1->DR; //清除标志位
			 //完成接收
			 if( UartIdel[0] != NULL )
			 UartIdel[0]();
		}
}

void USART2_IRQHandler( void )
{
    uint16_t data;
    unsigned char temp;
    if( USART_GetITStatus( USART2, USART_IT_ORE_RX ) != RESET )
    {
        if( USART_GetFlagStatus(USART2, USART_FLAG_PE) == RESET )
        {
            data = USART_ReceiveData( USART2 );
            if( UartIrq[1] != NULL )
            {
                if( UartWordLength == UART_7_BIT )
                {
                    UartIrq[1]( UART_NOTIFY_RX, data & 0x7F );
                }
                else
                {
                    UartIrq[1]( UART_NOTIFY_RX, data & 0xFF );
                }
            }
        }
        else
        {
            USART_ReceiveData( USART2 );
        }
    }
    
    if( USART_GetITStatus( USART2, USART_IT_TXE ) != RESET )
    {    
        if( UartIrq[1] != NULL )
        {
            UartIrq[1]( UART_NOTIFY_TX, 0 );
        }
        USART_ClearITPendingBit (USART2, USART_IT_TXE);
    }
    
    if( USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET )
    {
        if( USART_GetFlagStatus(USART2, USART_FLAG_PE) == RESET )
        {
            data = USART_ReceiveData( USART2 );
            if( UartIrq[1] != NULL )
            {
                if( UartWordLength == UART_7_BIT )
                {
                    UartIrq[1]( UART_NOTIFY_RX, data & 0x7F );
                }
                else
                {
                    UartIrq[1]( UART_NOTIFY_RX, data & 0xFF );
                }
            }
        }
        else
        {
            USART_ReceiveData( USART2 );
        }
        USART_ClearITPendingBit (USART2, USART_IT_RXNE);
    }

    if( USART_GetITStatus( USART2, USART_IT_CTS ) != RESET )
    {    
        data = USART_ReceiveData( USART2 );
        if( UartIrq[1] != NULL )
        {
            UartIrq[1]( UART_NOTIFY_CTS, 0 );
        }
        USART_ClearITPendingBit (USART2, USART_IT_CTS);
    }

    if( USART_GetITStatus( USART2, USART_IT_PE ) != RESET )
    {    
        data = USART_ReceiveData( USART2 );
        if( UartIrq[1] != NULL )
        {
            UartIrq[1]( UART_NOTIFY_PE, 0 );
        }
        USART_ClearITPendingBit (USART2, USART_IT_PE);
    }
		if(USART_GetITStatus(USART2,USART_IT_IDLE)==SET)
		{

			 temp=USART2->SR;
			 temp=USART2->DR; //清除标志位
			 //完成接收
			if( UartIdel[1] != NULL )
			    UartIdel[1]();
		}
}

void USART3_IRQHandler( void )
{
    uint16_t data;
    unsigned char temp;
	
    if( USART_GetITStatus( USART3, USART_IT_ORE_RX ) != RESET )
    {
        if( USART_GetFlagStatus(USART3, USART_FLAG_PE) == RESET )
        {
            data = USART_ReceiveData( USART3 );
            if( UartIrq[2] != NULL )
            {
                if( UartWordLength == UART_7_BIT )
                {
                    UartIrq[2]( UART_NOTIFY_RX, data & 0x7F );
                }
                else
                {
                    UartIrq[2]( UART_NOTIFY_RX, data & 0xFF );
                }
            }
        }
        else
        {
            USART_ReceiveData( USART3 );
        }
    }
    
    if( USART_GetITStatus( USART3, USART_IT_TXE ) != RESET )
    {    
        if( UartIrq[2] != NULL )
        {
            UartIrq[2]( UART_NOTIFY_TX, 0 );
        }
        USART_ClearITPendingBit (USART3, USART_IT_TXE);
    }
    
    if( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET )
    {
        if( USART_GetFlagStatus(USART3, USART_FLAG_PE) == RESET )
        {
            data = USART_ReceiveData( USART3 );
            if( UartIrq[2] != NULL )
            {
                if( UartWordLength == UART_7_BIT )
                {
                    UartIrq[2]( UART_NOTIFY_RX, data & 0x7F );
                }
                else
                {
                    UartIrq[2]( UART_NOTIFY_RX, data & 0xFF );
                }
            }
        }
        else
        {
            USART_ReceiveData( USART3 );
        }
        USART_ClearITPendingBit (USART3, USART_IT_RXNE);
    }

    if( USART_GetITStatus( USART3, USART_IT_CTS ) != RESET )
    {    
        data = USART_ReceiveData( USART3 );
        if( UartIrq[2] != NULL )
        {
            UartIrq[2]( UART_NOTIFY_CTS, 0 );
        }
        USART_ClearITPendingBit (USART3, USART_IT_CTS);
    }

    if( USART_GetITStatus( USART3, USART_IT_PE ) != RESET )
    {    
        data = USART_ReceiveData( USART3 );
        if( UartIrq[2] != NULL )
        {
            UartIrq[2]( UART_NOTIFY_PE, 0 );
        }
        USART_ClearITPendingBit (USART3, USART_IT_PE);
    }
		if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET)
		{
		   
			 temp=USART3->SR;
			 temp=USART3->DR; //清除标志位
			 //完成接收
			 if( UartIdel[2] != NULL )
			 UartIdel[2]();
		}
}
