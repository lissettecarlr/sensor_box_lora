/*
名称:传感器控制器
时间:2019年6月4日
功能:介于传感器与应用层之间，为上层提供采集的传感器数据，采集量由下层注册的传感器相关
*/

#include "rtthread.h"
#include "sensor_ctr.h"

#define LOG_TAG              "sensor"
#define LOG_LVL              LOG_LVL_DBG //该模块对应的日志输出级别。不定义时，默认：调试级别
#include <rtdbg.h>



static Sensor_t *SensorListHead=RT_NULL;

/*注册一个传感器*/
int sensor_register(Sensor_t *sensor)
{
    if(sensor ==RT_NULL)
			return 1;
		if(SensorListHead ==RT_NULL) //如果一个节点都没有
		{
			SensorListHead = sensor;
			sensor->next =RT_NULL;
		}
		else
		{
		   sensor->next = SensorListHead;
			 SensorListHead = sensor;
		}
		return 0;
}

/*删除一个传感器,通过名称匹配*/
int sensor_delete(char *name)
{
   	Sensor_t *LastNode = SensorListHead;
	  Sensor_t *Node = SensorListHead;
	  while(Node != RT_NULL)
		{
		    if(rt_strcmp(Node->name,name) ==0 ) //找到节点
				{
				     if(LastNode == Node) //如果就是头结点
						 {
						    SensorListHead = SensorListHead->next;
						 }
						 else
						 {
						    LastNode->next = Node->next; //最后一个节点的next在注册时就为空			 				  
						 }					 
						 return 0;
				}
				else
				{
				   LastNode = Node;
					 Node = Node->next;
				}
		}
		return  1;
}

/*修改传感器状态，动态启动关闭*/
int sensor_alter_status(char *name,uint8_t status)
{
    Sensor_t *Node = SensorListHead;
		while(Node !=RT_NULL)
		{
	      if(rt_strcmp(Node->name,name) ==0 ) //找到节点
				{
				   Node->status = status;
					 return 0;
				}
				else
				{
					 Node = Node->next;
				}
		}
		return 1;
}

/*通过传感器名称查询传感器状态*/
int sensor_get_status(char *name,uint8_t *status)
{
	Sensor_t *Node = SensorListHead;
	while(Node !=RT_NULL)
	{
	      if(rt_strcmp(Node->name,name) ==0 ) //找到节点
				{
				   *status = Node->status;
					 return 0;
				}
				else
				{
					 Node = Node->next;
				}
	}
	 return 1;
}

//遍历所以节点输出名称和状态
void display_all_sensor()
{
   Sensor_t *Node = SensorListHead;
	 while(Node !=RT_NULL)
	 {
		    rt_kprintf("%s:%d\n",Node->name,Node->status);
				Node = Node->next;			
	 }
}
MSH_CMD_EXPORT(display_all_sensor,show all sensor);


//管理部分,数据获取方式暂定为，外部调用打开传感器数据更新，然后通过回调抛出数据

static struct rt_event sensor_event;
/*外部回调，抛出数据*/
static void (*date_cb)(uint8_t *data,uint8_t len);
/*数据暂存buffer*/
#define DATE_BUFFER_SIZE 20
struct sBuffer
{
   uint8_t len; 
	 uint8_t date[DATE_BUFFER_SIZE];	  
};
static struct sBuffer SensorDate;


#define SENSOR_EVENT_UPDATE_START      (uint32_t)1<<0 
#define SENSOR_EVENT_UPDATE_OVER       (uint32_t)1<<0 

void sensor_ctr_update(void)
{
	 Sensor_t *Node = SensorListHead;
	
	 uint8_t data[20];
	 uint8_t len=0;
	
	 SensorDate.len = 0;
	 rt_memset(SensorDate.date,0,sizeof(SensorDate.date));
	
	 while(Node !=RT_NULL)
	 {	  
		  if(Node->status == SENSOR_STATUS_DEATH)
			{ 
				  rt_kprintf("%s status :%d\n",Node->name,Node->status);
			    Node = Node->next;
				  continue;
			}
			rt_kprintf("update %s\n",Node->name);
			
		  len = Node->read(data,20);
		  if(SensorDate.len+len >=DATE_BUFFER_SIZE)
			{
				  rt_kprintf("Sensor update date size error\n");
					//该传感器数据已经无法放入buffer了
				  Node = Node->next;
				  continue;
			}
			
		  rt_memcpy(SensorDate.date+SensorDate.len,data,len); //存入数据
			SensorDate.len +=len;
						
	    Node = Node->next;
	 }
}

/*这里分为异步和同步，如果传入非空则为异步,函数立即退出，数据通过回调返回。同步则函数接收数据则更新完毕*/
void sensor_ctr_start_update(void (*cb)(uint8_t *data,uint8_t len))
{  
	  if(cb != RT_NULL)
		{
		   date_cb = cb; 
			 rt_event_send(&sensor_event,SENSOR_EVENT_UPDATE_START);
		}
	  else
		{
			  rt_kprintf("sync update sensor data\n");
		    sensor_ctr_update();
		}  
}

/*该函数在使用同步更新时调用 参数size是传入buffer最大尺寸，返回是实际存入长度*/
uint8_t sensor_read_data(uint8_t *data,uint8_t size)
{
	  if(size >= SensorDate.len)
		{
		    rt_memcpy(data,SensorDate.date,SensorDate.len);  
			  return SensorDate.len;
		}
	  else
		{
		   return 0;
		}    
}

/*初始化传感器*/
void sensor_ctr_init_node()
{
	 Sensor_t *Node = SensorListHead;
	 
	 while(Node != RT_NULL)
	 {
		   rt_kprintf("sensor init %s\n",Node->name);
		   if(Node->init() != 0) //初始化失败，则将状态置为death
			 {
			      //Node->status = SENSOR_STATUS_DEATH;
			 }
	     Node = Node->next;
	 }
}
/*反初始化，一般用于休眠前*/	
void sensor_ctr_deinit_node()
{
	 Sensor_t *Node = SensorListHead;
	 
	 while(Node != RT_NULL)
	 {
		   rt_kprintf("sensor deinit %s\n",Node->name);
		   Node->deinit();
	     Node = Node->next;
	 }
}

void sensor_ctr_thread_entry(void *p)
{
   rt_uint32_t event;
	
	 //初始化传感器(传感器驱动注册发送在之前)
	 sensor_ctr_init_node();
	
	 while(1)
	 {
		  rt_event_recv(&sensor_event, SENSOR_EVENT_UPDATE_START | SENSOR_EVENT_UPDATE_OVER,RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&event);
	    if(event & SENSOR_EVENT_UPDATE_START)
			{
				  rt_kprintf("async update sensor data\n");
				  sensor_ctr_update();
			}
			if(event & SENSOR_EVENT_UPDATE_OVER)
			{
			    date_cb(SensorDate.date,SensorDate.len);
			}	 
	 }

}


void sensor_ctr_init(void)
{
    rt_thread_t sensor_ctr_handerL;
	  rt_event_init(&sensor_event,"sensor event",RT_IPC_FLAG_FIFO);
	
	  sensor_ctr_handerL = rt_thread_create("sensor ctr",sensor_ctr_thread_entry,RT_NULL,1024,5,10);
	  if(sensor_ctr_handerL != RT_NULL)
		{
		   rt_thread_startup(sensor_ctr_handerL);
		}
		else
		{
		   rt_kprintf("sensor ctr thread error\n");
		}
	  
}
