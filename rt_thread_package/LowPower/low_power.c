#include "board.h"
#include "stm32l1xx_hal.h"
#include "rtthread.h"
#include "drv_gpio.h"

#define PIN_WK_UP    GET_PIN(A, 10) //将串口RX配置成外部中断
#define PIN_WK_UP_2  GET_PIN(A, 3) //串口2

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

void enter_stop_wk(void* parameter)
{
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); 
}	



void enter_stop_exti()
{
	  GPIO_InitTypeDef GPIO_InitStructure = {0};
		GPIO_TypeDef gpio_a;
		GPIO_TypeDef gpio_b;
		GPIO_TypeDef gpio_c;
		GPIO_TypeDef gpio_h;
	  //读取当前GPIO状态
		rt_memcpy(&gpio_a,GPIOA,sizeof(GPIO_TypeDef));
    rt_memcpy(&gpio_b,GPIOB,sizeof(GPIO_TypeDef));
		rt_memcpy(&gpio_c,GPIOC,sizeof(GPIO_TypeDef));
		rt_memcpy(&gpio_h,GPIOH,sizeof(GPIO_TypeDef));
		
	  //关闭IO和时钟
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();

		__HAL_RCC_SPI1_CLK_DISABLE();
		__HAL_RCC_USART1_CLK_DISABLE();
			
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

    //配置唤醒用的外部中断
		rt_pin_mode(PIN_WK_UP, PIN_MODE_INPUT_PULLUP);
		rt_pin_attach_irq(PIN_WK_UP, PIN_IRQ_MODE_FALLING, enter_stop_wk, RT_NULL);
  	rt_pin_irq_enable(PIN_WK_UP, PIN_IRQ_ENABLE);
				
		rt_pin_mode(PIN_WK_UP_2, PIN_MODE_INPUT_PULLUP);
		rt_pin_attach_irq(PIN_WK_UP_2, PIN_IRQ_MODE_FALLING, enter_stop_wk, RT_NULL);
  	rt_pin_irq_enable(PIN_WK_UP_2, PIN_IRQ_ENABLE);		
		/* Disable GPIOs clock */
		__HAL_RCC_GPIOA_CLK_DISABLE();
		__HAL_RCC_GPIOB_CLK_DISABLE();
		__HAL_RCC_GPIOC_CLK_DISABLE();
		__HAL_RCC_GPIOH_CLK_DISABLE();


	  //进入停止模式
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	  //退出停止模式
	  //开启时钟
    SystemClock_Config();
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_USART1_CLK_ENABLE();
		
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();
		
		//关闭唤醒中断
		rt_pin_irq_enable(PIN_WK_UP, PIN_IRQ_DISABLE);
		rt_pin_detach_irq(PIN_WK_UP);
		
		rt_pin_irq_enable(PIN_WK_UP_2, PIN_IRQ_DISABLE);
		rt_pin_detach_irq(PIN_WK_UP_2);
	
	  //复位IO
		rt_memcpy(GPIOA,&gpio_a,sizeof(GPIO_TypeDef));
    rt_memcpy(GPIOB,&gpio_b,sizeof(GPIO_TypeDef));
		rt_memcpy(GPIOC,&gpio_c,sizeof(GPIO_TypeDef));
		rt_memcpy(GPIOH,&gpio_h,sizeof(GPIO_TypeDef));
		
		rt_thread_mdelay(100);
		rt_kprintf(" WK UP!!");
}
MSH_CMD_EXPORT(enter_stop_exti,enter_stop_exti);


//进入STANDBY模式低功耗，使用PA0唤醒
void enter_standby_wakup()
{
		if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
		{
			/* Clear Standby flag */
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB); 
		}
	  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		
	  HAL_PWR_EnterSTANDBYMode();
}

void enter_sleep()
{
    /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
    Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
    //HAL_SuspendTick();

    /* Enter Sleep Mode , wake up is done once Key push button is pressed */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    /* Resume Tick interrupt if disabled prior to sleep mode entry*/
    //HAL_ResumeTick();
}

void low_pwoer_hook_cb()
{
	 //该回调未关闭滴答
   enter_sleep();
}

void low_power_set_idle_hook()
{
		rt_thread_idle_sethook(low_pwoer_hook_cb);
}
