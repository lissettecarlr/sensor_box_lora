#include "board.h"
#include "stm32l1xx_hal.h"
#include "rtthread.h"
#include "drv_gpio.h"

#include "sensor_ctr.h"

#define PIN_WK_UP    GET_PIN(A, 10) //将串口RX配置成外部中断

void low_power_init()
{
  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
	
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
	
	HAL_PWR_DisablePVD();
		
}


//该模式需要在中断回调中添加
//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

static    GPIO_InitTypeDef GPIO_InitStructure = {0};
static		GPIO_TypeDef gpio_a;
static		GPIO_TypeDef gpio_b;
static		GPIO_TypeDef gpio_c;
static		GPIO_TypeDef gpio_h;

void enter_stop_wk(void)
{
	  //开启时钟
    SystemClock_Config();
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); 
    //HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);	
	 
	 	//关闭唤醒中断
//		rt_pin_irq_enable(PIN_WK_UP, PIN_IRQ_DISABLE);
//		rt_pin_detach_irq(PIN_WK_UP);
	
		rt_thread_mdelay(100);
	  //复位IO
		rt_memcpy(GPIOA,&gpio_a,sizeof(GPIO_TypeDef));
    rt_memcpy(GPIOB,&gpio_b,sizeof(GPIO_TypeDef));
		rt_memcpy(GPIOC,&gpio_c,sizeof(GPIO_TypeDef));
		rt_memcpy(GPIOH,&gpio_h,sizeof(GPIO_TypeDef));	  
}	


void exit_cb(void* parameter)
{
	enter_stop_wk();
}

void enter_stop()
{

	  //读取当前GPIO状态
		rt_memcpy(&gpio_a,GPIOA,sizeof(GPIO_TypeDef));
    rt_memcpy(&gpio_b,GPIOB,sizeof(GPIO_TypeDef));
		rt_memcpy(&gpio_c,GPIOC,sizeof(GPIO_TypeDef));
		rt_memcpy(&gpio_h,GPIOH,sizeof(GPIO_TypeDef));
		
		/* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
		GPIO_InitStructure.Pin = GPIO_PIN_All;
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
		HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

		
		GPIO_InitStructure.Pin = GPIO_PIN_7;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = GPIO_PIN_5;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = GPIO_PIN_4;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		//IIC
		sensor_ctr_deinit_node();
    //配置唤醒用的外部中断
		
//		rt_pin_mode(PIN_WK_UP, PIN_MODE_INPUT_PULLUP);
//		rt_pin_attach_irq(PIN_WK_UP, PIN_IRQ_MODE_FALLING, exit_cb, RT_NULL);
//  	rt_pin_irq_enable(PIN_WK_UP, PIN_IRQ_ENABLE);
		
	  //进入停止模式
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    sensor_ctr_init_node();
		

}


