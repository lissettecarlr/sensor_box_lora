
#ifndef _LOW_POWER_H_
#define _LOW_POWER_H_

void enter_standby_wakup(void);
void enter_stop_exti(void);
void enter_stop_wk(void);
void enter_sleep(void);
void low_pwoer_hook_cb(void);
void low_power_set_idle_hook(void);

void low_power_init(void);
#endif


