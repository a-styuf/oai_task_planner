#ifndef _DEBUG_H
#define _DEBUG_H

#include "main.h"
#include "gpio.h"

#define DEBUG

#define DBG_GPIO_PORT PORTE
#define DBG_GPIO_NUM 13
#define DBG_GPIO_ON 1
#define DBG_GPIO_OFF 0

void dbg_gpio(uint8_t state);

#endif
