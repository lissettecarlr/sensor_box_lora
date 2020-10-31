//该文件是基于STM32 HAL库的ADC初始化

#include "stm32_HAL_adc.h"
#include "rtthread.h"

#define  ADC_DEBUG  rt_kprintf


ADC_HandleTypeDef hadc;


void MAX_ADC_deInit(void)
{
   HAL_ADC_DeInit(&hadc);
}

//目前该配置是读取电压
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
      ADC_DEBUG("HAL_ADC_Init(&hadc) != HAL_OK \n");
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  //sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
	
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
      ADC_DEBUG("HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK \n");
  }

}


//这里读了几个数组来组成了一条线
//280 = 1805     270 = 1856   330 = 1523  
// (3384 -x) / 5.64 = y
uint16_t ADC_READ_Valtage(void)
{
//	  int i=4;
	  uint16_t adc_value =0;
//	  while(i--)
//		{
		    HAL_ADC_Start(&hadc);
		    HAL_ADC_PollForConversion(&hadc, 50);  //等待完成转换
		    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
		    {
			     adc_value = HAL_ADC_GetValue(&hadc);			
		    }

//		}
		
//		adc_value = adc_value>>2;
		ADC_DEBUG("adc_value:%d\n",adc_value);
		adc_value = (3384 - adc_value) / 5.64;
		
		return (adc_value );

}
