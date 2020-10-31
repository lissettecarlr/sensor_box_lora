/*
 * Copyright (c) 2015-2018, Leap Value IOT Development Team
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-23     flyu      	 first implementation
 */
 
#include <board.h>
#include "drv_spi.h"

#define SPI_DEV_NAME    "spi1.1"
#define SPI_BUS_NAME    "spi1"
#define SPI_NSS_PIN     10
#define SX1278_RST_PIN  11
#define SX1276_DIO0_PIN 13
#define SX1276_DIO1_PIN 1

//*******************************************************************************************//
rt_err_t sx1278_attach(const char *spi_device_name, rt_uint32_t rst_pin){return RT_EOK;}
typedef  void (*irq_handler)(void*);
void x(void* p) {}
void y(void* p) {}
irq_handler sx127x_irq[] = {x,y,RT_NULL,RT_NULL,RT_NULL,RT_NULL};
//*******************************************************************************************//

static int sx127x_bsp_init(void)
{
	rt_err_t ret;
	
	ret = stm32_spi_bus_attach_device(SPI_DEV_NAME, SPI_BUS_NAME,SPI_NSS_PIN);
	RT_ASSERT(ret != RT_EOK);
  
	rt_pin_mode(SX1278_RST_PIN, PIN_MODE_OUTPUT);
	
	ret = sx1278_attach(SPI_DEV_NAME, SX1278_RST_PIN);
	RT_ASSERT(ret != RT_EOK);
	
	rt_pin_mode(SX1276_DIO0_PIN, PIN_MODE_INPUT_PULLDOWN);
	rt_pin_attach_irq(SX1276_DIO0_PIN, PIN_IRQ_MODE_RISING, sx127x_irq[0], RT_NULL);
	rt_pin_irq_enable(SX1276_DIO0_PIN, PIN_IRQ_ENABLE);
	
	rt_pin_mode(SX1276_DIO1_PIN, PIN_MODE_INPUT_PULLDOWN);
	rt_pin_attach_irq(SX1276_DIO1_PIN, PIN_IRQ_MODE_RISING, sx127x_irq[1], RT_NULL);
	rt_pin_irq_enable(SX1276_DIO1_PIN, PIN_IRQ_ENABLE);
	return ret;
}
INIT_PREV_EXPORT(sx127x_bsp_init);
