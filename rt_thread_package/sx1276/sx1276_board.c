
#include <rtthread.h>
#include "rtdevice.h"
#include "rthw.h"  //DelayMs
#include "sx1276_board.h"
#include "drv_gpio.h"

#define SX1276_DIO_RST    GET_PIN(B, 11)
#define SX1276_DIO_NSS    GET_PIN(A, 4)

#define SX1276_DIO_0      GET_PIN(B, 10)
#define SX1276_DIO_1      GET_PIN(B, 2)

#define SX1276_DIO_SWITCH  GET_PIN(A, 1)

void SX1276IoInit(void)
{
    rt_pin_mode(SX1276_DIO_0, PIN_MODE_INPUT);
    rt_pin_mode(SX1276_DIO_1, PIN_MODE_INPUT);
	
    rt_pin_mode(SX1276_DIO_NSS, PIN_MODE_OUTPUT);

		rt_pin_mode(SX1276_DIO_SWITCH, PIN_MODE_OUTPUT);
	  rt_pin_write(SX1276_DIO_SWITCH,1);
	  rt_thread_mdelay(100);

}

void SX1276IoDeInit( void )
{
     rt_pin_mode(SX1276_DIO_NSS, PIN_MODE_INPUT);
	
	  rt_pin_mode(SX1276_DIO_SWITCH, PIN_MODE_INPUT);

}
	
	
void SX1276IoIrqInit(void)
{
    rt_pin_attach_irq(SX1276_DIO_0, PIN_IRQ_MODE_RISING, SX1276OnDio0Irq_shell, RT_NULL);
  	rt_pin_irq_enable(SX1276_DIO_0, PIN_IRQ_ENABLE);
    rt_pin_attach_irq(SX1276_DIO_1, PIN_IRQ_MODE_RISING, SX1276OnDio1Irq_shell, RT_NULL);
  	rt_pin_irq_enable(SX1276_DIO_1, PIN_IRQ_ENABLE);
}


static struct rt_spi_device spi_dev_sx1276;
uint8_t cs;
int SX1276SpiInit(void)
{
    rt_err_t res;
    res = rt_spi_bus_attach_device(&spi_dev_sx1276, "sx1276_spi", "spi1", NULL);
    if (res != RT_EOK)
    {
        rt_kprintf("spi device init :error\r\n");
        return 1;
    }
    else
    {
    //配置
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 10000000;//20 * 1000 *1000; /* 20M,SPI max 42MHz,ssd1351 4-wire spi */
    rt_spi_configure(&spi_dev_sx1276, &cfg);
    rt_kprintf("spi device init :OK\n");
    }
		return 0;
}


void Sx1276TimerInit( Sx1276TimerEvent_t *obj, void ( *callback )( void* parameter ) )
{
	
	 obj->timeHander  =      rt_timer_create(obj->name,  /* 定时器名字是 timer1 */
                                           callback, /* 超时时回调的处理函数 */
                                           RT_NULL,  /* 超时函数的入口参数 */
                                           obj->ReloadValue,       /* 定时长度，以OS Tick为单位，即10个OS Tick */
                                           RT_TIMER_FLAG_ONE_SHOT); /* 单次定时器*/
	 obj->Callback = callback;
	
}
void Sx1276TimerStart( Sx1276TimerEvent_t *obj )
{
    if (obj->timeHander != RT_NULL) 
			rt_timer_start(obj->timeHander);
}
void Sx1276TimerStop( Sx1276TimerEvent_t *obj ) 
{
	 if (obj->timeHander != RT_NULL) 
			rt_timer_stop(obj->timeHander);
} 
void Sx1276TimerReset( Sx1276TimerEvent_t *obj )
{
    Sx1276TimerStop(obj);
	  Sx1276TimerStart(obj);
}
void Sx1276TimerSetValue( Sx1276TimerEvent_t *obj, uint32_t value )
{
	//参数是ms，传入tick，相差10倍
	 value = value/(1000/RT_TICK_PER_SECOND);
   rt_timer_control(obj->timeHander, RT_TIMER_CTRL_SET_TIME, (void*)&value);
}
Sx1276TimerTime_t Sx1276TimerGetCurrentTime( void )
{
	rt_tick_t temp;
	temp = rt_tick_get();//得到TTCK 
	temp*=(1000/RT_TICK_PER_SECOND);
    return temp; //返回ms
}
Sx1276TimerTime_t Sx1276TimerGetElapsedTime( Sx1276TimerTime_t past )
{
	rt_tick_t nowInTicks = rt_tick_get();
	rt_tick_t pastInTicks= rt_tick_from_millisecond(past);
	return ((nowInTicks - pastInTicks)*(1000/RT_TICK_PER_SECOND));
}
void Sx1276DelayMs( uint32_t ms )
{
//	uint32_t i=0;
//   for(i=1;i<=ms;i++)
//		  rt_hw_us_delay(1000);
	 rt_thread_mdelay(ms);
}


int sx1276_board_init(void)
{
    int sta=0;
	  SX1276IoInit();
	  sta+=SX1276SpiInit();
	  SX1276IoIrqInit();
      SX1276SetHardwareInterface(SX1276_DIO_NSS,SX1276_DIO_RST,&spi_dev_sx1276);
    return sta;
}


const struct Radio_s Radio =
{
    SX1276IoInit,
    SX1276IoDeInit,
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime
};
