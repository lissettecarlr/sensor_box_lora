# STM32L1XX系列低功耗

## 1电源管理
* 上电复位 POR   [参考](http://www.go-gddq.com/html/FuWeiDianLu/2015-07/1358022.htm)

* 掉电复位 PDR

* 欠压复位 
使能BOR后POR和PDR的门限将被忽略，转而使用BOR设置的门限，BOR门限 在 option bytes中配置

|模式|门限|
|---|---|
|1|1.8V上电  1.65V断电|
|2|2.1V上电  2.0V断电|
|3|2.4V上电  2.3V断电|
|4|2.7V上电  2.6V断电|
|5|2.9V上电  2.8V断电|

* 可编程电压监控 PVD

软件控制，监控VDD，门限可以设置在1.8-3.1间，100mV为间隔，当PVD大于或者小于门限则产生一个外部中断.

## 2低功耗模式
### 2.1低功耗运行模式

内核和外设都可以保存运行状态，需要配置内部电源变换器工作在低功耗模式下，系统时钟必须设置在128KHz一下，可以使用MSI作为系统时钟，内核供电必须选择供电范围2(1.5V)，并且关闭动态电压调节功能

### 2.2睡眠模式
内核停止运行，外设保持运行.
通过特殊指令进入睡眠模式.
WFI由中断唤醒，任意的NVIC识别到外设中断都可以唤醒内核
WFE由事件唤醒，被配置为事件模式的EXTI中断唤醒内核

### 2.3低功耗睡眠模式
内核停止运行,外设保存运行

### 2.4停止模式
内核停止，内核电源范围内的时钟都停止，晶振也禁止，SRAM和寄存器内容保留。启动时需要重新配置时钟，并且清除标志位，唤醒后程序继续之前的运行

### 2.5待机模式
电源内部变换器关闭、内核电源范围内全部断电，晶振被禁止，SRAM和寄存器不保留，RTC寄存器和备份寄存器保留。在待机模式下所有IO都保存高阻态，除了复位、RTC报警输出引脚、唤醒引脚，功耗典型值为1ua

### 2.6 总结
![](https://s2.ax1x.com/2019/03/15/AEtViR.png)


### 3 代码与测试
测试硬件: STM32L151开发板，搭在了串口转USB和电源LED,和STlink，所以功耗并不能代表芯片的功耗
#### 3.1待机模式(standby)
进入待机模式的代码，使用HAL库中的stm32l1xx_hal_pwr.c文件，下列中如果不进行清除标志位，则设备在第二次进入待机模式后将自动被唤醒
```
//进入待机模式函数，主函数打开一个LED灯，当进入待机后引脚会被切换成高阻态，灯就会熄灭。通过给PA0一个上升沿就能唤醒,LED被点亮
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
```
这里功耗测试就是简单看看低功耗是否生效，之后使用专门的模组进行测试。
进入standby模式下3.3V供电时电流为38.3ma。
正常运行下3.3V供电时电流为48.5ma。
电流大致下降了10.2ma

#### 3.2停机模式
代码使用了一个启动后常亮的引脚和一个外部中断按键，前者用于判断在进入停机模式后，引脚状态，结果是引脚仍然处于之前的状态，也就是寄存器被保持了。后者用于退出停机模式。
```
//该模式在退出时候必须重新配置时钟
void enter_stop_exti()
{
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config();
}
```
进入停止模式下3.3V供电时电流为39ma。
退出后电流为49ma。

#### 3.3 睡眠模式
改模下由于任意中断均可唤醒，所以要注意关闭滴答定时器。目前对他没需求，没测试
```
HAL_SuspendTick();
HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
HAL_ResumeTick();
```