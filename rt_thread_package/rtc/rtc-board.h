#ifndef __RTC_BOARD_H
#define __RTC_BOARD_H


//extern RTC_HandleTypeDef rtc_hander;
void rtc_init(void);
void rtc_set_alarm(uint32_t timeout);
void rtc_close_alarm(void);
void  rtc_register_callback(void (*cb)(void));
void rtc_display(void);

#endif 

