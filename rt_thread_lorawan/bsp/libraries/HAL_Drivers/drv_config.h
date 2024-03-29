/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-10-30     SummerGift        first version
 */
 
#ifndef __DRV_CONFIG_H__
#define __DRV_CONFIG_H__

#include <board.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SOC_SERIES_STM32F0)
#include "f0/dma_config.h"
#include "f0/uart_config.h"
#include "f0/spi_config.h"
#include "f0/tim_config.h"
#include "f0/pwm_config.h"
#include "f0/adc_config.h"
#elif defined(SOC_SERIES_STM32F1)
#include "f1/dma_config.h"
#include "f1/uart_config.h"
#include "f1/spi_config.h"
#include "f1/adc_config.h"
#include "f1/tim_config.h"
#include "f1/sdio_config.h"
#include "f1/pwm_config.h"
#elif  defined(SOC_SERIES_STM32F4)
#include "f4/dma_config.h"
#include "f4/uart_config.h"
#include "f4/spi_config.h"
#include "f4/adc_config.h"
#include "f4/tim_config.h"
#include "f4/sdio_config.h"
#include "f4/pwm_config.h"
#elif  defined(SOC_SERIES_STM32F7)
#include "f7/dma_config.h"
#include "f7/uart_config.h"
#include "f7/spi_config.h"
#include "f7/qspi_config.h"
#include "f7/adc_config.h"
#include "f7/tim_config.h"
#include "f7/sdio_config.h"
#include "f7/pwm_config.h"
#elif  defined(SOC_SERIES_STM32L4)
#include "l4/dma_config.h"
#include "l4/uart_config.h"
#include "l4/spi_config.h"
#include "l4/qspi_config.h"
#include "l4/adc_config.h"
#include "l4/tim_config.h"
#include "l4/pwm_config.h"
#elif  defined(SOC_SERIES_STM32L1)
#include "l1/dma_config.h"
#include "l1/uart_config.h"
#include "l1/spi_config.h"
#include "l1/adc_config.h"
#include "l1/tim_config.h"
#include "l1/pwm_config.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
