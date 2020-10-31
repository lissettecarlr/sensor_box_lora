#include "cli_board.h"
#include "board.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/*!
 * Uart print use FIFO definitions
 */
#define UART_PRINT_USE_FIFO

/*!
 * Max size of the cli print FIFO
 */
#define MAX_CLI_PRINT_DATA_TX_FIFO_SIZE                 (2304)

#if defined(UART_PRINT_USE_FIFO)
/*!
 * Cli print data buffer
 */
static char CliPrintDataTxBuf[MAX_CLI_PRINT_DATA_TX_FIFO_SIZE];

static Fifo_t CliPrintDataFifo;
#endif

#ifdef UART_PRINT_USE_FIFO
/*!
 * Cli print low poer object
 */
static LowPowerList_t UartPrintLowPower;

/*!
 * Flag to indicate if uart printing is running
 */
static bool IsUartPrintTxRunning = false;

/*!
 * Processes uart received data
 *
 * \param [IN] notify     UART notification[0:UART_NOTIFY_TX, 1:UART_NOTIFY_RX, 2:UART_NOTIFY_CTS, 3:UART_NOTIFY_PE]
 * \param [IN] value      Received data
 */
static void UartCliReceive( uint8_t notify, uint16_t value )
{

    if( notify == UART_NOTIFY_TX )
    {
        char value;
        
        if( FifoPop( &CliPrintDataFifo, &value ) == 0 )
        {
            UartPutChar( 0, value, false );
        }
        else
        {
            IsUartPrintTxRunning = false;
            
            UartITConfig( 0, UART_IT_FLAG_TX, false );
            McuUpdateLowPowerMode( &UartPrintLowPower, LOW_POWER_MODE_STOP, true );
        }
    }
}
#endif

void CliBoardInit( void )
{
    UartInit( 0, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    UartSetInterrupt( 0, UART_IT_FLAG_RX, 0, 0, UartCliReceive );
#ifdef UART_PRINT_USE_FIFO
    if( IsUartPrintTxRunning == true )
    {
        UartITConfig( 0, UART_IT_FLAG_TX, true );
        McuUpdateLowPowerMode( &UartPrintLowPower, LOW_POWER_MODE_SLEEP, false );
    }
#endif
}

void CliBoardDeInit( void )
{
    UartDeInit( 0, NO_FLOW_CTRL );
    UartRemoveInterrupt( 0 );
}

void CliBoardConfig( void )
{
#ifdef UART_PRINT_USE_FIFO
    FifoInit( &CliPrintDataFifo, CliPrintDataTxBuf, MAX_CLI_PRINT_DATA_TX_FIFO_SIZE );
    LowPowerListInsert( &UartPrintLowPower, LOW_POWER_MODE_STOP );
#endif
}

PUTCHAR_PROTOTYPE
{
#ifdef UART_PRINT_USE_FIFO
    int ret;
    
    BoardDisableIrq();
    ret = FifoPush( &CliPrintDataFifo, ch );
    if( (ret == 0) && (IsUartPrintTxRunning == false) )
    {
        UartITConfig( 0, UART_IT_FLAG_TX, true );
        IsUartPrintTxRunning = true;
    }
    BoardEnableIrq();
#else
    UartPutChar( 0, ch, true );
#endif
    
    return ch;
}
