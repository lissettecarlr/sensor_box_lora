# sensor_box_lora
该仓库存放通过lora通信的传感器盒子代码。均使用STM32L151芯片。低功耗的代码未使用PM2.5传感器，可控制在10+ua的级别。
该套代码相对可靠，运行测试过数个月。
# no_rtos_lorawan
该文件夹可以独立使用，所以文件都均在一起。无操作系统，lorawan通信。

# rt_thread_lorawan
该文件夹下保存的是低功耗却PM2.5传感器代码，和非低功耗完整传感器代码的bsp文件，均使用了rt_thread操作系统。由于原代码本来是分为了多个文件夹存储，然后通过bsp工程来共同编译，于是为了保存，我单独把代码保存了过来，所以打开工程后是需要修改链接文件的位置


# rt_thread_master
单独复制过来的和操作系统相关的源码

# rt_thread_package
单独复制过来的会被使用的各种包
