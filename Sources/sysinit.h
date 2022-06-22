#ifndef _SYSINIT_H_
#define _SYSINIT_H_

#include "main.h"
#include "systick.h"

#define SYSINIT_EXTERNAL_CLK 10
#define SYSINIT_MCU_CLK_MHZ 40
#define SYSINIT_PLL0_Q (2)  // допустимое значение: 0-15
#define SYSINIT_PLL0_N (2*(SYSINIT_MCU_CLK_MHZ)/(SYSINIT_EXTERNAL_CLK))  // допустимое значение: 3-75

#define SYSINIT_SYS_TICK_1MS_VALUE ((SYSINIT_MCU_CLK_MHZ*1000)/1)
#define SYSINIT_SYS_TICK_100US_VALUE ((SYSINIT_MCU_CLK_MHZ*1000)/10)
#define SYSINIT_SYS_TICK_10US_VALUE ((SYSINIT_MCU_CLK_MHZ*1000)/100)
#define SYSINIT_SYS_TICK_1US_VALUE ((SYSINIT_MCU_CLK_MHZ*1000)/1000)

void System_Init(void);

#endif
