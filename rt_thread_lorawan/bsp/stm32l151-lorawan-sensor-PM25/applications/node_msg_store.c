/*
该文件用于保存读取节点的重要信息
*/

#include "node_msg_store.h"
#include "rtthread.h"
#include "fal.h"



LoraWanJoinInfo_s* LwrtInfo=RT_NULL;
LoraWanCfg_s*      LwrtCfg=RT_NULL;
LoraWanStatue_s*   LwrtStatue=RT_NULL;

NodeMsg_s NodeMsg={LORAWAN_SRTORE_FLAG};


void display_node_msg()
{
	 rt_kprintf("----------------------------------------\n");
	 rt_kprintf("FLAG=%d\n",NodeMsg.StoreFlag);
	 rt_kprintf("cycle=%d\n",NodeMsg.CycleS);
	 rt_kprintf("LowPower=%d\n",NodeMsg.LowPower);
   //JOIN
	 rt_kprintf("JoinMode:%d\n",NodeMsg.JoinInfo.JoinMode);
   rt_kprintf("Addr:%08X\n",NodeMsg.JoinInfo.Addr);

   rt_kprintf("DevEui:");
   for(int i=0;i<8;i++)
    rt_kprintf(" %02X",NodeMsg.JoinInfo.DevEui[i]);
   rt_kprintf("\n");

    rt_kprintf("AppEui:");
   for(int i=0;i<8;i++)
    rt_kprintf(" %02X",NodeMsg.JoinInfo.AppEui[i]);
   rt_kprintf("\n");

    rt_kprintf("AppKey:");
   for(int i=0;i<16;i++)
    rt_kprintf(" %02X",NodeMsg.JoinInfo.AppKey[i]);
   rt_kprintf("\n");

    rt_kprintf("NetSKey:");
   for(int i=0;i<16;i++)
    rt_kprintf(" %02X",NodeMsg.JoinInfo.NetSKey[i]);
   rt_kprintf("\n");

    rt_kprintf("AppSKey:");
   for(int i=0;i<16;i++)
    rt_kprintf(" %02X",NodeMsg.JoinInfo.AppSKey[i]);
   rt_kprintf("\n");
	  //CFG
		rt_kprintf("adr:%d\tdatarate:%d\tpower:%d\tCLASS:%d\n",NodeMsg.Cfg.Adr,NodeMsg.Cfg.DataRate,NodeMsg.Cfg.power,NodeMsg.Cfg.Class);
		rt_kprintf("channl: ");
		for(int i=0;i<6;i++)
		{
			rt_kprintf("%04X\t",NodeMsg.Cfg.ChannelsMask[i]);
		}
		rt_kprintf("\n");
	  //STATUE
		rt_kprintf("UpCounter: %d\n",NodeMsg.Statue.UpCounter);
		rt_kprintf("----------------------------------------\n");
}

//MSH_CMD_EXPORT(display_node_msg,display_node_msg);

int read_node_msg()
{
   const struct fal_partition * bli = fal_partition_find("eeprom2") ;
	 if (bli ==RT_NULL)
	 {
		  rt_kprintf("fal error");
		  return 1;
	 }
	 fal_partition_read(bli,0,&(NodeMsg.StoreFlag),sizeof(NodeMsg_s)) ;
	 
	 if(NodeMsg.StoreFlag!= LORAWAN_SRTORE_FLAG )//判断是否保存过
	 {
		  NodeMsg.StoreFlag = LORAWAN_SRTORE_FLAG;//读取失败时候该值会被寄存器覆盖所有需要重新赋值
		  NodeMsg.CycleS = 120;  //如果读取数据失败，设定默认上传周期为2分钟s	  
		  NodeMsg.LowPower = 0;  //默认关闭低功耗
			rt_kprintf("not find node info\n");
		  return 3;
	 }
	 else
	 {
	    rt_kprintf("node info read succeed\n");
		  display_node_msg();
		  rt_memcpy(LwrtInfo,&(NodeMsg.JoinInfo),sizeof(LoraWanJoinInfo_s));
		  rt_memcpy(LwrtCfg,&(NodeMsg.Cfg),sizeof(LoraWanCfg_s));
		  rt_memcpy(LwrtStatue,&(NodeMsg.Statue),sizeof(LoraWanStatue_s));		  
		  return 0;	    
	 }
}
//MSH_CMD_EXPORT(read_node_msg,read_node_msg);

//保存当前节点信息，返回0则表示保存成功
int save_node_msg()
{
	 const struct fal_partition * bli = fal_partition_find("eeprom2") ;
	 if (bli ==RT_NULL)
	 {
		  rt_kprintf("fal error\n");
		  return 1;
	 }
	 
		fal_partition_erase(bli,0,sizeof(NodeMsg_s));
	  rt_thread_mdelay(1000);
	  
	 	//读取协议中上行计数的值	 
	  lorawan_statue_read("UpCounter",&(LwrtStatue->UpCounter));
	 
	  rt_memcpy(&(NodeMsg.JoinInfo),LwrtInfo,sizeof(LoraWanJoinInfo_s));
		rt_memcpy(&(NodeMsg.Cfg),LwrtCfg,sizeof(LoraWanCfg_s));
		rt_memcpy(&(NodeMsg.Statue),LwrtStatue,sizeof(LoraWanStatue_s));
	  display_node_msg();
	  
	  fal_partition_write(bli,0,&(NodeMsg.StoreFlag),sizeof(NodeMsg_s));
		return 0;
}
//MSH_CMD_EXPORT(save_node_msg,save_node_info);

//情况数据表
int clear_node_msg()
{
   const struct fal_partition * bli = fal_partition_find("eeprom2") ;
	 if (bli ==RT_NULL)
	 {
		  rt_kprintf("fal error\n");
		  return 1;
	 }
	 fal_partition_erase(bli,0,sizeof(NodeMsg_s));
	 return 0;
}

//MSH_CMD_EXPORT(clear_node_msg,clear_node_info);
/*********************************************************************************************************/

void lw_clear()
{
	clear_node_msg();
	rt_thread_mdelay(100);//不延时会擦不干净
	LwrtInfo->JoinMode = 2; //切换回需要发送入网包的模式
	rt_kprintf("[clear OK]\n");
}
MSH_CMD_EXPORT(lw_clear,lw_clear);

void lw_save()
{
  save_node_msg();
	rt_thread_mdelay(100);
	rt_kprintf("[save OK]\n");
}
MSH_CMD_EXPORT(lw_save,lw_save);
