/*
使用方式: 调用sensor_uart_conpatible_init函数进行初始化，每当数据更新则会调用cb回调函数，该函数依赖于进程而非中断，
所以优先级于进程优先级挂钩
*/


/*周期发送的命令结构体*/
struct send_command
{
  uint8_t len;
	uint8_t data[30];
};
/*接收到传感器应答数据的结构体*/
struct rcv_buffer
{
   uint8_t len;
	 uint8_t data[100];
};

/*传感器更新时根据周期进行还是连续不断采集*/
//#define SENSOR_UART_MODE_DISCONTINUOUS   0
//#define SENSOR_UART_MODE_CONTINUOUS      1

/**/
#define SENSOR_UART_RCV_TIMEOUT 5
#define SENSOR_UART_THREAD_PRIORITY 5
#define SENSOR_UART_DATA_LOST  0xffaa
/*API*/

/*
mode:更新的模式
cmd:更新所需的命令
cb:数据采集后的回调
rcv_key:数据量的匹配值，当接收到该数量则表示一帧结束，传入0则不进行比对。
*/
int sensor_uart_conpatible_init(uint32_t cycle,struct send_command cmd,uint8_t rcv_key,void(*cb)(struct rcv_buffer buffer));

/*启动数据更新*/
void sensor_uart_start_update(void);
/*关闭数据更新*/
void sensor_uart_close_update(void);
/*串口发送接口*/
void sensor_uart_send(uint8_t *data,uint8_t len);
