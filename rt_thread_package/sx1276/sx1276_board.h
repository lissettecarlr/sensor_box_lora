#ifndef _SX1276_BOARD_H_
#define _SX1276_BOARD_H_

/*
SX1276和硬件相关的初始化，.c中实现了radio需要的函数，并且实例化了struct Radio_s Radio
于是使用sx1276驱动，只需要引入该.h即可，首先初始化相关硬件sx1276_board_init，
然后正常使用radio结构体即可，例程可见sx1276_rtt_test.c

*/

#include "sx1276.h"

int sx1276_board_init(void);

#endif
