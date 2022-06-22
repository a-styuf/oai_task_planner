#ifndef _DEBUG_H
#define _DEBUG_H

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "uarts.h"
#include "gpio.h"

#define DBG_GPIO_PORT   PORTE
#define DBG_GPIO_NUM    (13)
#define DBG_GPIO_ON     (1)
#define DBG_GPIO_OFF    (0)

void dbg_gpio(uint8_t state);
void dbg_print(char *buff);
void dbg_gpio_pulse(void);

#endif
