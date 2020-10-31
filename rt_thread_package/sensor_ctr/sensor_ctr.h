#ifndef SENSOR_CTR_H
#define SENSOR_CTR_H

#define SENSOR_STATUS_ALIVE   1
#define SENSOR_STATUS_DEATH   2

typedef struct sSensor
{
/*读取传感器数据，返回长度*/
	  char name[10];
	  uint8_t status;
	  uint8_t (*init)(void);
	  uint8_t (*deinit)(void);
    uint8_t (*read)(uint8_t *data,uint8_t len);
	  void (*irq)(void *param);
		struct sSensor *next;
}Sensor_t;


//API
int sensor_register(Sensor_t *sensor);
int sensor_delete(char *name);
int sensor_alter_status(char *name,uint8_t status);
int sensor_get_status(char *name,uint8_t *status);
void display_all_sensor(void);


void sensor_ctr_start_update(void (*cb)(uint8_t *data,uint8_t len));
void sensor_ctr_init(void);
/*返回实际存入长度，size表示传入buffer最大长度，如果无法存入则返回0*/
uint8_t sensor_read_data(uint8_t *data,uint8_t size);

void sensor_ctr_deinit_node(void);
void sensor_ctr_init_node(void);
/*
使用方式:
	传感器驱动需要编写xxx_register()函数来将实例化的Sensor_t注册到传感器链表中
  在启动传感器控制器之前调用xxx_register()函数
  调用sensor_ctr_init初始化传感器控制器
  调用sensor_ctr_start_update更新传感器数据，使用异步方式更新完成将通过回调通知。
	使用同步方式，则需要调用sensor_read_data函数进行读取
*/
#endif
