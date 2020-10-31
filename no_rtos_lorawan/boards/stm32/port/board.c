#include "board.h"
#include "gpio.h"
#include "power.h"
#include "rtc.h"
#include "timer.h"
#include "delay.h"
#include "cli_board.h"
#include "debug.h"
#include  "serial_board.h"
#include "string.h"
/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * Flag to indicate if the MCU is initialized
 */
static bool McuInitialized = false;

/*!
 * Flag to indicate if the unused io is initialized
 */
static bool IoInitialized = false;

/*!
 * Timer to handle the reset execute
 */
static TimerEvent_t ResetTimer;

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void LedRadioTxDataToggle( void )
{
    
}

void LedRadioRxDataToggle( void )
{
    
}

void LedRadioBeforeTxJionRequestShow( void )
{
    
}

void LedRadioTxJoinRequestShow( void )
{
    
}

void LedRadioRxJoinAcceptShow( void )
{
    
}

void LedRadioTxDataShow( void )
{
    
}

void LedRadioRxDataShow( void )
{
    
}

static void BoardSystemReset( void )
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

/*!
 * \brief Function executed on reset timeout event
 */
static void ResetTimerEvent( void )
{
    BoardSystemReset();
}

void SystemReset( uint8_t time )
{
    if( time == 0 )
    {
        time = 1;
    }
    TimerInit( &ResetTimer, ResetTimerEvent, "reset" );
    TimerSetValue( &ResetTimer, time * 1000 );
    TimerStart( &ResetTimer );
}

void BoardInitPeriph( void )
{
    McuLowPowerModeInit();
    
    CliBoardConfig();
    
    DelayMs( 1000 );
}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
#if defined( USE_BOOTLOADER )
        // Set the Vector Table base location at 0x4000
        NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x4000 );
#endif
        // We use IRQ priority group 4 for the entire project
        // When setting the IRQ, only the preemption priority is used
        NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
        
        // Disable Systick
        SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;    // Systick IRQ off 
        SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;            // Clear SysTick Exception pending flag
        
        BoardUnusedIoInit();
        
        CliBoardInit();
			
        SerialBoardInit();//串口2初始化  
			
       
   			RtcInit();
        
        SpiInit( 1, 8, 0, 0, 10000000, 0 );
        SX1276IoInit();
        
        McuInitialized = true;
    }
}
 
void BoardDeInitMcu( void )
{
    CliBoardDeInit();
    
    SpiDeInit( 1 );
    SX1276IoDeInit( );
    
    GpioInit( OSC_LSE_IN, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    GpioInit( OSC_LSE_OUT, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    
    GpioInit( OSC_HSE_IN, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    GpioInit( OSC_HSE_OUT, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
	
    McuInitialized = false;
}

static void BoardUnusedIoInit( void )
{
    if( IoInitialized == false )
    {
        /* Unused IOs, output, pull-up, high level; or pull-down, low level */
        GpioInit( SPI1_NSS, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( SPI1_SCLK, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( SPI1_MISO, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( SPI1_MOSI, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( USART1_CK, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( USB_DM, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( USB_DP, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( ADC_INT8, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( ADC_INT9, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( TIM4_CH4, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( USART3_TX, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( USART3_RX, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        
//#undef USE_DEBUGGER
#if defined( USE_DEBUGGER )
        DBGMCU_Config( DBGMCU_SLEEP, ENABLE );
        DBGMCU_Config( DBGMCU_STOP, ENABLE);
        DBGMCU_Config( DBGMCU_STANDBY, ENABLE);
#else
        DBGMCU_Config( DBGMCU_SLEEP, DISABLE );
        DBGMCU_Config( DBGMCU_STOP, DISABLE );
        DBGMCU_Config( DBGMCU_STANDBY, DISABLE );
        
        GpioInit( SWDAT, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
        GpioInit( SWCLK, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
#endif
        IoInitialized = true;
    }
}

uint8_t BoardGetBatteryLevel( void )
{
    return 255;
}
