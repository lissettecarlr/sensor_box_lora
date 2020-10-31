#include "spi.h"
#include "gpio.h"

void SpiInit( uint8_t spi_no, uint8_t bits, uint8_t cpol, uint8_t cpha, uint32_t prescaler, uint8_t firstbit )
{
    SPI_InitTypeDef SPI_InitStructure;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    SPI_TypeDef *spi;
    uint8_t gpio_AF;
    PinName_t mosi;
    PinName_t miso;
    PinName_t sclk;
    PinName_t nss;
    uint32_t divisor;

    if( ( ( ( bits == 8 ) || ( bits == 16 ) ) == 0 ) ||
        ( ( ( cpol == 0 ) || ( cpol == 1 ) ) == 0 ) ||
        ( ( ( cpha == 0 ) || ( cpha == 1 ) ) == 0 ) || 
        ( ( ( firstbit == 0 ) || ( firstbit == 1 ) ) == 0 ) )
    {
        // para error
        while( 1 );
    }
    
    if( spi_no == 0 )
    {
        spi = SPI1;
        gpio_AF = GPIO_AF_SPI1;
        mosi = SPI1_MOSI;
        miso = SPI1_MISO;
        sclk = SPI1_SCLK;
        nss = SPI1_NSS;
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );
    }
    else if( spi_no == 1 )
    {
        spi = SPI2;
        gpio_AF = GPIO_AF_SPI2;
        mosi = SPI2_MOSI;
        miso = SPI2_MISO;
        sclk = SPI2_SCLK;
        nss = SPI2_NSS;
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );
    }
    else
    {
        while(1);
    }
    
    GpioInit( mosi, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_DOWN, gpio_AF );
    GpioInit( miso, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_DOWN, gpio_AF );
    GpioInit( sclk, PIN_AF, PIN_OUTPUT_PP, PIN_PULL_DOWN, gpio_AF );
    GpioInit( nss, PIN_OUTPUT, PIN_OUTPUT_PP, PIN_PULL_DOWN, 1 );

    RCC_GetClocksFreq( &RCC_ClocksStatus );
    if( spi_no == 0 )
    {
        divisor = RCC_ClocksStatus.PCLK2_Frequency / prescaler;
    }
    else
    {
        divisor = RCC_ClocksStatus.PCLK1_Frequency / prescaler;
    }
    if( divisor <= 2 )
    {
        divisor = 0;
    }
    else if( divisor <= 4 )
    {
        divisor = 1;
    }
    else if( divisor <= 8 )
    {
        divisor = 2;
    }
    else if( divisor <= 16 )
    {
        divisor = 3;
    }
    else if( divisor <= 32 )
    {
        divisor = 4;
    }
    else if( divisor <= 64 )
    {
        divisor = 5;
    }
    else if( divisor <= 128 )
    {
        divisor = 6;
    }
    else
    {
        divisor = 7;
    }
    
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = ( bits == 8 ) ? SPI_DataSize_8b : SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = ( cpol == 0x01 ) ? SPI_CPOL_High : SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = ( cpha == 0x01 ) ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = divisor << 3;
    SPI_InitStructure.SPI_FirstBit = (firstbit == 0x01) ? SPI_FirstBit_LSB : SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( spi, &SPI_InitStructure );
    SPI_Cmd( spi, ENABLE );
}

void SpiDeInit( uint8_t spi_no )
{
    SPI_TypeDef *spi;
    PinName_t mosi;
    PinName_t miso;
    PinName_t sclk;
    PinName_t nss;

    if( spi_no == 0 )
    {
        spi = SPI1;
        mosi = SPI1_MOSI;
        miso = SPI1_MISO;
        sclk = SPI1_SCLK;
        nss = SPI1_NSS;
    }
    else if( spi_no == 1 )
    {
        spi = SPI2;
        mosi = SPI2_MOSI;
        miso = SPI2_MISO;
        sclk = SPI2_SCLK;
        nss = SPI2_NSS;
    }
    else
    {
        while(1);
    }
    
    SPI_Cmd( spi, DISABLE );
    
    GpioInit( mosi, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    GpioInit( miso, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    GpioInit( sclk, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    GpioInit( nss, PIN_INPUT, PIN_OUTPUT_PP, PIN_PULL_UP, 1 );
    
    SPI_I2S_DeInit( spi );
}

uint16_t SpiInOut( uint8_t spi_no, uint16_t data )
{
    SPI_TypeDef *spi;

    if( spi_no == 0 )
    {
        spi = SPI1;
    }
    else if( spi_no == 1 )
    {
        spi = SPI2;
    }
    else
    {
        while(1);
    }
    
    while( SPI_I2S_GetFlagStatus( spi, SPI_I2S_FLAG_TXE ) == RESET )
        ;
    SPI_I2S_SendData( spi, data );
    while( SPI_I2S_GetFlagStatus( spi, SPI_I2S_FLAG_RXNE ) == RESET )
        ;
    
    return SPI_I2S_ReceiveData( spi );
}
