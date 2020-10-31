#ifndef NODE_MSG_STORE_H
#define NODE_MSG_STORE_H


#include "lorawan_app.h"

//在初始化前进行赋值，该指针用于保存app中的info的指针，从而直接操作info
extern LoraWanJoinInfo_s* LwrtInfo;
extern LoraWanCfg_s*      LwrtCfg;
extern LoraWanStatue_s*   LwrtStatue;


//该结构体仅仅拥有flash存储
#define LORAWAN_SRTORE_FLAG         0XFA

typedef struct sNodeMsg
{
    uint8_t StoreFlag;
	  uint8_t LowPower;
	  uint32_t CycleS;
	  LoraWanStatue_s Statue;
	  LoraWanCfg_s Cfg;
	  LoraWanJoinInfo_s JoinInfo;
}NodeMsg_s;

extern NodeMsg_s NodeMsg;

extern void display_node_msg(void);
extern int read_node_msg(void);
extern int save_node_msg(void);
extern int clear_node_msg(void);

#endif
