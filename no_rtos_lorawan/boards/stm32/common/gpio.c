#include "gpio.h"
#include "power.h"

/*!
 * Gpio IRQ handler
 */
static GpioIrqHandler_t GpioIrq[16];

/*!
 * Mask of AHB peripheral clock
 */
uint32_t AHBPeriphClockMask = 0;

void GpioInit( PinName_t pinName, PinMode_t mode, PinOutput_t output, PinPull_t pull, uint8_t value )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t portIndex;
    uint8_t pinIndex;
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t AHBPeriphClock;
    
    portIndex = pinName >> 4;
    pinIndex = pinName & 0x0F;
    port = (GPIO_TypeDef *)( GPIOA_BASE + ( portIndex << 10 ) );
    pin = 0x01 << pinIndex;
    AHBPeriphClock = 0x01 << portIndex;
    if( (AHBPeriphClockMask & AHBPeriphClock) == 0 )
    {
        RCC_AHBPeriphClockCmd( AHBPeriphClock, ENABLE );
        AHBPeriphClockMask |= AHBPeriphClock;
    }

    if( mode == PIN_OUTPUT )
    {
        if( value == 0 )
        {
            GPIO_ResetBits( port, pin );
        }
        else
        {
            GPIO_SetBits( port, pin );
        }
    }
    else if( mode == PIN_AF )
    {
        GPIO_PinAFConfig( port, pinIndex, value );
    }
    
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_OType = (GPIOOType_TypeDef)output;
    GPIO_InitStructure.GPIO_PuPd = (GPIOPuPd_TypeDef)pull;
    GPIO_Init( port, &GPIO_InitStructure );
}

void GpioSetInterrupt( PinName_t pinName, IrqEdge_t irqEdge, uint8_t prePriority, uint8_t subPriority, GpioIrqHandler_t irqHandler )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    uint8_t portIndex;
    uint8_t pinIndex;
    uint16_t pin;
    
    portIndex = pinName >> 4;
    pinIndex = pinName & 0x0F;
    pin = 0x01 << pinIndex;
    GpioIrq[pinIndex] = irqHandler;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE );
    SYSCFG_EXTILineConfig( portIndex, pinIndex );
    
    EXTI_InitStructure.EXTI_Line = pin;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    if( irqEdge == IRQ_RISING_EDGE )
    {
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    }
    else if( irqEdge == IRQ_FALLING_EDGE )
    {
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    }
    else if( irqEdge == IRQ_RISING_FALLING_EDGE )
    {
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    }
    else
    {
        while(1);
    }
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    if( pinIndex < 5 )
    {
        NVIC_InitStructure.NVIC_IRQChannel = pinIndex + 6;
    }
    else if( pinIndex < 10 )
    {
        NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    }
    else
    {
        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prePriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
}

void GpioDisableInterrupt( PinName_t pinName )
{
    EXTI_InitTypeDef EXTI_InitStructure;
    uint8_t pinIndex;
    uint16_t pin;
    
    pinIndex = pinName & 0x0F;
    pin = 0x01 << pinIndex;

    EXTI_InitStructure.EXTI_Line = pin;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void GpioWrite( PinName_t pinName, uint8_t value )
{
    uint8_t portIndex;
    uint8_t pinIndex;
    GPIO_TypeDef* port;
    uint16_t pin;
    
    portIndex = pinName >> 4;
    pinIndex = pinName & 0x0F;
    port = (GPIO_TypeDef *)( GPIOA_BASE + ( portIndex << 10 ) );
    pin = 0x01 << pinIndex;
    if( value == 0 )
    {
        GPIO_ResetBits( port, pin );
    }
    else
    {
        GPIO_SetBits( port, pin );
    }
}

uint8_t GpioRead( PinName_t pinName )
{
    uint8_t portIndex;
    uint8_t pinIndex;
    GPIO_TypeDef* port;
    uint16_t pin;
    
    portIndex = pinName >> 4;
    pinIndex = pinName & 0x0F;
    port = (GPIO_TypeDef *)( GPIOA_BASE + ( portIndex << 10 ) );
    pin = 0x01 << pinIndex;
    
    if( GPIO_ReadInputDataBit( port, pin ) == RESET )
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/*!
 * \brief EXTI0 IRQ Handler
 */
void EXTI0_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line0 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line0 );
        if( GpioIrq[0] != NULL )
        {
            GpioIrq[0]( );
        }
    }
}

/*!
 * \brief EXTI1 IRQ Handler
 */
void EXTI1_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line1 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line1 );
        if( GpioIrq[1] != NULL )
        {
            GpioIrq[1]( );
        }
    }
}

/*!
 * \brief EXTI2 IRQ Handler
 */
void EXTI2_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line2 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line2 );
        if( GpioIrq[2] != NULL )
        {
            GpioIrq[2]( );
        }
    }
}

/*!
 * \brief EXTI3 IRQ Handler
 */
void EXTI3_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line3 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line3 );
        if( GpioIrq[3] != NULL )
        {
            GpioIrq[3]( );
        }

    }
}

/*!
 * \brief EXTI4 IRQ Handler
 */
void EXTI4_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line4 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line4 );
        if( GpioIrq[4] != NULL )
        {
            GpioIrq[4]( );
        }
    }
}

/*!
 * \brief EXTI9_5 IRQ Handler
 */
void EXTI9_5_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line5 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line5 );
        if( GpioIrq[5] != NULL )
        {
            GpioIrq[5]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line6 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line6 );
        if( GpioIrq[6] != NULL )
        {
            GpioIrq[6]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line7 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line7 );
        if( GpioIrq[7] != NULL )
        {
            GpioIrq[7]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line8 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line8 );
        if( GpioIrq[8] != NULL )
        {
            GpioIrq[8]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line9 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line9 );
        if( GpioIrq[9] != NULL )
        {
            GpioIrq[9]( );
        }
    }
}

/*!
 * \brief EXTI15_10 IRQ Handler
 */
void EXTI15_10_IRQHandler( void )
{
    McuRecoverFromLowPowerMode();
    
    if( EXTI_GetITStatus( EXTI_Line10 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line10 );
        if( GpioIrq[10] != NULL )
        {
            GpioIrq[10]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line11 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line11 );
        if( GpioIrq[11] != NULL )
        {
            GpioIrq[11]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line12 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line12 );
        if( GpioIrq[12] != NULL )
        {
            GpioIrq[12]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line13 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line13 );
        if( GpioIrq[13] != NULL )
        {
            GpioIrq[13]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line14 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line14 );
        if( GpioIrq[14] != NULL )
        {
            GpioIrq[14]( );
        }
    }

    if( EXTI_GetITStatus( EXTI_Line15 ) != RESET )
    {
        EXTI_ClearITPendingBit( EXTI_Line15 );
        if( GpioIrq[15] != NULL )
        {
            GpioIrq[15]( );
        }
    }
}
