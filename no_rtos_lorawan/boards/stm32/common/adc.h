#ifndef __ADC_H__
#define __ADC_H__

#include "type.h"
#include "gpio.h"

typedef enum
{
    ADC_SINGLE_CONVERSION = 0,
    ADC_CONTIMUOUS_CONVERSION,
}AdcConvMode_t;

typedef enum
{
    ADC_RIGHT_ALIGNED = 0,
    ADC_LEFT_ALIGNED,
}AdcAlign_t;

void AdcInit( uint8_t adc_no, PinName_t pinName, AdcConvMode_t convMode, AdcAlign_t align );
uint16_t AdcGetValue( uint8_t adc_no, uint8_t channel );

#endif
