#include "adc.h"

void AdcInit( uint8_t adc_no, PinName_t pinName, AdcConvMode_t convMode, AdcAlign_t align )
{
    ADC_CommonInitTypeDef Adc_CommInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_TypeDef *adc;
    
    if( pinName != NC )
    {
        GpioInit( pinName, PIN_ANALOGIC, PIN_OUTPUT_PP, PIN_NO_PULL, 0 );
    }
    
    RCC_HSICmd( ENABLE );
    while(RCC_GetFlagStatus( RCC_FLAG_HSIRDY ) == RESET )
        ;

    if( adc_no == 0 )
    {
        adc = ADC1;
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
    }
    else
    {
        while(1);
    }
    
    Adc_CommInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit( &Adc_CommInitStructure );
    
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    if( convMode == ADC_SINGLE_CONVERSION )
    {
        ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    }
    else
    {
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    }
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T2_TRGO;
    if( align == ADC_RIGHT_ALIGNED )
    {
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    }
    else
    {
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    }
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init( adc, &ADC_InitStructure );
    
    if( adc_no == 0 )
    {
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, DISABLE );
    }
    RCC_HSICmd( DISABLE );
}

uint16_t AdcGetValue( uint8_t adc_no, uint8_t channel )
{
    ADC_TypeDef *adc;
    uint16_t adcData = 0;
    
    RCC_HSICmd( ENABLE );
    while( RCC_GetFlagStatus( RCC_FLAG_HSIRDY ) == RESET )
        ;

    if( adc_no == 0 )
    {
        adc = ADC1;
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
    }
    else
    {
        while(1);
    }
    
    if( ( channel == ADC_Channel_16 ) || ( channel == ADC_Channel_17 ) )
    {
        ADC_TempSensorVrefintCmd( ENABLE );
    }
    
    ADC_RegularChannelConfig( adc, channel, 1, ADC_SampleTime_192Cycles );
    ADC_DelaySelectionConfig( adc, ADC_DelayLength_Freeze );
    ADC_PowerDownCmd( adc, ADC_PowerDown_Idle_Delay, ENABLE );
    ADC_Cmd( adc, ENABLE );

    while( ADC_GetFlagStatus( adc, ADC_FLAG_ADONS ) == RESET )
        ;

    ADC_SoftwareStartConv( adc );
    while( ADC_GetFlagStatus( adc, ADC_FLAG_EOC ) == RESET )
        ;

    adcData = ADC_GetConversionValue( adc );

    ADC_Cmd( adc, DISABLE );
    
    if( ( channel == ADC_Channel_16 ) || ( channel == ADC_Channel_17 ) )
    {
        ADC_TempSensorVrefintCmd( DISABLE );
    }

    if( adc_no == 0 )
    {
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, DISABLE );
    }
    RCC_HSICmd( DISABLE );
   
    return adcData;
}
