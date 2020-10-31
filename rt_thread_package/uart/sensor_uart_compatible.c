/*
该套代码主要用于处理单命令重复读取的串口类传感器设备
*/

#include "rtthread.h"
#include "sensor_uart_compatible.h"


/*该段代码用于将一个引脚的电平作为通讯指示标志*/
#include "drv_gpio.h"

#define UART_USR_SEND_FLAG   1
#define UART_USR_RCV_FLAG    0
#define UART_USR_FLAG_PIN    GET_PIN(B, 5) 

static uint8_t uart_usr_init_flag = 0;

void uart_usr_set_send_flag()
{
		rt_pin_write(UART_USR_FLAG_PIN,UART_USR_SEND_FLAG);
}
	
void uart_usr_set_rcv_flag()
{
		rt_pin_write(UART_USR_FLAG_PIN,UART_USR_RCV_FLAG);
}

void uart_usr_set_init()
{
	 if(uart_usr_init_flag)
	 {
			 return;
	 }
	 uart_usr_init_flag =1;
	 rt_pin_mode(UART_USR_FLAG_PIN, PIN_MODE_OUTPUT);
	 rt_pin_write(UART_USR_FLAG_PIN,UART_USR_RCV_FLAG);
}

/*end*/

/*该结构体仅仅用于保存周期请求的命令*/
struct send_command send_cmd={0,{0}};
/*该结构体用于保存发送的配置命令*/
struct send_command send_cfg={0,{0}};

/*该结构体保存传感器取得的数据，不解析*/
struct rcv_buffer rcv_buffer;

/*保存对外数据回调函数指针*/
static void (*sensor_uart_cb)(struct rcv_buffer buffer);

/*模式标志位*/
//static uint8_t sensor_uart_mode;

/*串口接收对比标志*/
static uint8_t uart_rcv_key;

/*thread*/
ALIGN(RT_ALIGN_SIZE);
static char sensor_uart_thread_stack[1024];
static struct rt_thread sensor_uart_hander;


/*EVENT*/
#define SENSOR_UART_EVENT_ALL        0xffffffff
#define SENSOR_UART_EVENT_UPDATE     1<<0    
#define SENSOR_UART_EVENT_RCV        1<<1
#define SENSOR_UART_EVENT_TIMEOUT    1<<2
#define SENSOR_UART_EVENT_CFG        1<<3
static struct rt_event sensor_uart_evt;

/*timer*/
static uint32_t cycle = 60;
static uint32_t rcv_timeout = 5;
static struct rt_timer update_timer;
static struct rt_timer rcv_timeout_timer; 

/*uart*/
#define SENSOR_UART_DEVNAME "uart3"
static rt_device_t uart_mod_device = RT_NULL;


static int rcv_len=-1;

/*开启串口接收*/
static void start_uart_rcv()
{
   rcv_len=0;   
	 /*清空接收的buffer*/
	 char ch;
	 while( (rt_device_read(uart_mod_device,0,&ch,1)) > 0 );
}
/*关闭串口接收*/
static void stop_uart_rcv()
{
   rcv_len=-1;
}
/*串口数中断*/
static rt_err_t uart_module_irq(rt_device_t dev, rt_size_t size)
{
	 /*如果不处于接收状态*/
	 if(rcv_len == -1)
		 return RT_EOK;
	 /*如果开启了数据量比对，则数据接收完成时在接收数量大于指定数量时，而不是接收超时的时候*/
	 if(uart_rcv_key != 0)
	 {
	    if(size < uart_rcv_key) 
			{
			    rcv_len = size; 
			}
			else
			{
			    rcv_len = uart_rcv_key; 
				  /*关闭超时定时器，发送接收事件*/
				  rt_timer_stop(&rcv_timeout_timer);
				  rt_event_send(&sensor_uart_evt,SENSOR_UART_EVENT_RCV);
				  /*关闭了串口接收，只要在处理完成该帧数据后才打开*/
          stop_uart_rcv();			  
			}
		 
	 }
	 else
	 {
	    rcv_len =size;   
	 }
	 return RT_EOK;
}

/*数据更新定时器超时回调*/
void update_timer_cb(void *p)
{
   rt_event_send(&sensor_uart_evt,SENSOR_UART_EVENT_UPDATE);
}

void rcv_timeout_timer_cb(void *p)
{
   rt_event_send(&sensor_uart_evt,SENSOR_UART_EVENT_RCV);

}

/*更新控制*/
#define SENSOR_UART_STATE_START 1
#define SENSOR_UART_STATE_STOP  0
uint8_t state=SENSOR_UART_STATE_STOP;

void sensor_uart_start_update(void)
{
   state = SENSOR_UART_STATE_START;
}
void sensor_uart_close_update(void)
{
   state = SENSOR_UART_STATE_STOP;
}

void sensor_uart_thread_entry(void *p)
{
		uint32_t e;
	  uint32_t re_time;
	  rt_kprintf("[uart]{thread start}\n");
		rt_thread_mdelay(3000);
	  rt_event_send(&sensor_uart_evt,SENSOR_UART_EVENT_UPDATE);
		while(1)
		{
			  while(state ==  SENSOR_UART_STATE_STOP)
				{
				    rt_thread_mdelay(1000);
				}
				
			  rt_event_recv(&sensor_uart_evt,SENSOR_UART_EVENT_ALL,RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&e);
				if(e & SENSOR_UART_EVENT_UPDATE)
				{
					start_uart_rcv();
					uart_usr_set_send_flag();
					rt_device_write(uart_mod_device,0,send_cmd.data,send_cmd.len);
					uart_usr_set_rcv_flag();
					rt_kprintf("[uart]{send cmd}\n");
					rt_timer_start(&rcv_timeout_timer);
				}
				if(e &SENSOR_UART_EVENT_RCV)
				{
	          rcv_buffer.len = rt_device_read(uart_mod_device,0,rcv_buffer.data,rcv_len);
					  sensor_uart_cb(rcv_buffer);
					  /*这里延时是为了不让该线程一直占用*/
					  rt_thread_mdelay(1000);
					  /*如果是连续采集则当即进入下一次数据更新，否则启动周期定时器*/
					  if(cycle != 0)
						{
							 re_time= cycle * RT_TICK_PER_SECOND;
               rt_timer_control(&update_timer, RT_TIMER_CTRL_SET_TIME, (void*)&re_time);						
						   rt_timer_start(&update_timer); 
						}
						else
						{
						   rt_event_send(&sensor_uart_evt,SENSOR_UART_EVENT_UPDATE);
						}
				}
				if(e & SENSOR_UART_EVENT_TIMEOUT)
				{
					 rt_kprintf("[uart]{timeout!rcv data:%d}\n",rcv_len);
					 /*如果不进行数据量对比*/
					 if(uart_rcv_key ==0)
					 {
							 	rcv_buffer.len = rt_device_read(uart_mod_device,0,rcv_buffer.data,rcv_len);
								sensor_uart_cb(rcv_buffer);
					 }
					 else
					 {
						  	/*外抛数据长度为0，则表示数据接收超时*/
								rcv_buffer.len = 0;
								sensor_uart_cb(rcv_buffer);
					 }

						if(cycle != 0)
						{
							 re_time = cycle * RT_TICK_PER_SECOND;
               rt_timer_control(&update_timer, RT_TIMER_CTRL_SET_TIME, (void*)&re_time);						
						   rt_timer_start(&update_timer); 
						}
						else
						{
						   rt_event_send(&sensor_uart_evt,SENSOR_UART_EVENT_UPDATE);
						}
				}
				if(e & SENSOR_UART_EVENT_CFG)
				{
					 start_uart_rcv();
					 uart_usr_set_send_flag();
					 rt_device_write(uart_mod_device,0,send_cfg.data,send_cfg.len);
					 uart_usr_set_rcv_flag();
					 rt_kprintf("[uart]{send cfg}\n");				     
				}
		}
}

int sensor_uart_conpatible_init(uint32_t i_cycle,struct send_command cmd,uint8_t rcv_key,void(*cb)(struct rcv_buffer buffer))
{
	  cycle = i_cycle;
	  rt_memcpy(&send_cmd,&cmd,sizeof(struct send_command));
	  sensor_uart_cb = cb;
	  uart_rcv_key = rcv_key;
		rcv_timeout = SENSOR_UART_RCV_TIMEOUT;
		
	  rt_event_init(&sensor_uart_evt,"uart_cmp",RT_IPC_FLAG_FIFO);
		uart_usr_set_init();
    rt_thread_init(&sensor_uart_hander,
                   "ser_cmb",
                   sensor_uart_thread_entry,
                   RT_NULL,
                   sensor_uart_thread_stack,
                   sizeof(sensor_uart_thread_stack),
                   SENSOR_UART_THREAD_PRIORITY, 10);		
		/*串口配置*/							 
	  uart_mod_device=rt_device_find(SENSOR_UART_DEVNAME);
	 	if(uart_mod_device ==RT_NULL)
			 return 1;
		
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;	
		config.baud_rate = 9600;
    rt_device_control(uart_mod_device, RT_DEVICE_CTRL_CONFIG, &config);		
		
	  rt_device_set_rx_indicate(uart_mod_device, uart_module_irq);
	  rt_device_open(uart_mod_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );	

		/*读取传感器更新周期*/
		if(cycle >86400)//这里判断如果周期大于一天，则可能是错误数据，强行修改为60s
		{
				rt_kprintf("sensor_update_cycle  error!");
				cycle = 60;
		}
		
		/*初始化周期和超时定时器*/
		rt_timer_init(&update_timer,"upd_t",update_timer_cb,RT_NULL,cycle*RT_TICK_PER_SECOND,RT_TIMER_FLAG_ONE_SHOT);
		rt_timer_init(&rcv_timeout_timer,"rcv_t",rcv_timeout_timer_cb,RT_NULL,rcv_timeout*RT_TICK_PER_SECOND,RT_TIMER_FLAG_ONE_SHOT);
		
		rt_thread_startup(&sensor_uart_hander);
    return 0;
}

/*发送接口，用于进行一些配置*/
void sensor_uart_send(uint8_t *data,uint8_t len)
{
	  send_cfg.len = len ;
	  rt_memcpy(send_cfg.data,data,len);
    //rt_device_write(uart_mod_device,0,data,len);
}

