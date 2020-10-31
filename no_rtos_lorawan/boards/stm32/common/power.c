#include "power.h"
#include "debug.h"
#include "board.h"
#include "rtc.h"
#include "timer.h"

/*!
 * This flag is used to make sure we have looped through the main several time
 * to avoid race issues
 */
static volatile uint8_t HasLoopedThroughMain = 0;

/*!
 * Flag to indicate if low power mode is enable
 */
static bool LowPower = true;

/*!
 * Low power list head pointer
 */
static LowPowerList_t *LowPowerListhead = NULL;

/*!
 * Last stop mode status stored
 */
static LowPowerMode_t LowPowerModeContext = LOW_POWER_MODE_STOP;

void LowPowerListInsert( LowPowerList_t *p, LowPowerMode_t mode )
{
    LowPowerList_t *cur;

    BoardDisableIrq();
    p->mode = mode;
    cur = LowPowerListhead;
    while( cur != NULL )
    {
        if( cur == p )
        {
            BoardEnableIrq();
            return;
        }
        else
        {
            cur = cur->next;
        }
    }
    
    if( LowPowerListhead == NULL )
    {
        p->next = NULL;
        LowPowerListhead = p;
    }
    else
    {
        p->next = LowPowerListhead;
        LowPowerListhead = p;
    }
    BoardEnableIrq();
}

/*!
 * Travesing the low poer list to ensure whether stop mode is enable
 */
static LowPowerMode_t GetLowPowerMode( void )
{
    LowPowerMode_t mode = LOW_POWER_MODE_STOP;
    LowPowerList_t *p;
    
    if( LowPower == true )
    {
        p = LowPowerListhead;
        while( p != NULL )
        {
            if( p->mode == LOW_POWER_MODE_SLEEP )
            {
                mode = LOW_POWER_MODE_SLEEP;
                break;
            }
            p = p->next;
        }
    }
    else
    {
        mode = LOW_POWER_MODE_SLEEP;
    }
    
    return mode;
}

LowPowerMode_t GetLowPowerModeContext( void )
{
    return LowPowerModeContext;
}

void McuLowPowerModeInit( void )
{
    NVIC_SetPriority( PendSV_IRQn, (1<<__NVIC_PRIO_BITS) - 1 );
    if( LowPower == false )
    {
        LowPowerModeContext = LOW_POWER_MODE_SLEEP;
    }
}

void McuUpdateLowPowerMode( LowPowerList_t *p, LowPowerMode_t mode, bool IsTimerUpdate )
{
    if( LowPower == true )
    {
        LowPowerMode_t context;

        BoardDisableIrq();
        
        p->mode = mode;
        context = GetLowPowerMode();
        if( LowPowerModeContext != context )
        {
            LowPowerModeContext = context;
            if( IsTimerUpdate == true )
            {
                TimerIrqHandler();
            }
        }
        HasLoopedThroughMain = 0;
        
        BoardEnableIrq();
    }
}

void McuEnterLowPowerMode( void )
{
    if( LowPower == true )
    {
        BoardDisableIrq();
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
        BoardEnableIrq();
    }
}

void McuRecoverFromLowPowerMode( void )
{
    BoardDisableIrq();
    if( PWR_GetFlagStatus(PWR_FLAG_WU) != RESET )
    {
        PWR_ClearFlag( PWR_FLAG_WU );
    }
    if( LowPowerModeContext == LOW_POWER_MODE_STOP )
    {
        /* After wake-up from STOP reconfigure the system clock */
        /* Enable HSE */
        RCC_HSEConfig( RCC_HSE_ON );
        
        /* Wait till HSE is ready */
        while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
        {}
        
        /* Enable PLL */
        RCC_PLLCmd( ENABLE );
        
        /* Wait till PLL is ready */
        while( RCC_GetFlagStatus( RCC_FLAG_PLLRDY ) == RESET )
        {}
        
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );
        
        /* Wait till PLL is used as system clock source */
        while( RCC_GetSYSCLKSource( ) != 0x0C )
            ;

        /* Set MCU in ULP (Ultra Low Power) */
        PWR_UltraLowPowerCmd( DISABLE );
        
        /* Enable the Power Voltage Detector */
        PWR_PVDCmd( ENABLE );
        
        NVIC_SystemLPConfig( NVIC_LP_SLEEPONEXIT, DISABLE );
        
        BoardInitMcu();
    }
    HasLoopedThroughMain = 0;
    BoardEnableIrq();
}

/*!
 * \brief PendSV IRQ Handler to update the times we have looped through the main and
 *        go to stop mode
 */
void PendSV_Handler( void )
{
    BoardDisableIrq();
    if( HasLoopedThroughMain < 10 )
    {
        HasLoopedThroughMain++;
    }
    else
    {
        HasLoopedThroughMain = 0;
        if( LowPowerModeContext == LOW_POWER_MODE_STOP )
        {
            DEBUG_INFO( " stop! \r\n" );
            
            BoardDeInitMcu();
            
            /* Disable the Power Voltage Detector */
            PWR_PVDCmd( DISABLE );

            /* Set MCU in ULP (Ultra Low Power) */
            PWR_UltraLowPowerCmd( ENABLE );

            /*Disable fast wakeUp*/
            PWR_FastWakeUpCmd( DISABLE );
            
            NVIC_SystemLPConfig( NVIC_LP_SLEEPONEXIT, ENABLE );
            
            /* Enter Stop Mode */
            PWR_EnterSTOPMode( PWR_Regulator_LowPower, PWR_STOPEntry_WFI );
        }
    }
    BoardEnableIrq();
}
