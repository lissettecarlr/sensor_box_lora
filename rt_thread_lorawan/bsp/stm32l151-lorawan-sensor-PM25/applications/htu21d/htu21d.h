#ifndef _HTU21D_H__
#define _HTU21D_H__


#include <rtthread.h>

int htu21d_deinit(void);
int htu21d_init(void);
uint16_t htu21d_get_temp(void);
uint16_t htu21d_get_humi(void);
void display_th(void);
void htu21d_pin_init(void);
void htu21d_get(uint16_t *temp,uint16_t *humi);
#endif
