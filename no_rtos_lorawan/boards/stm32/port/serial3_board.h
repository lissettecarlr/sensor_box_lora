#ifndef _SERIAL3_BOARD_
#define _SERIAL3_BOARD_

#include "stm32l1xx.h"

extern void M100d_Init(void(* callback)(int));
extern unsigned char get_network_state(void);
extern unsigned char get_wireless_send_state(void);
extern void qurey_wireless_state_pck(void);
extern void  wireless_send_data(uint8_t* data , int lenth);
extern void m100d_wake(void);
extern void m100d_rst(void);
extern void set_network_state(uint8_t net);
#endif
