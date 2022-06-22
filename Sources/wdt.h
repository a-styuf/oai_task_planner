#ifndef _WDT_H_
#define _WDT_H_

#include "main.h"

// Интервал сторожевого таймера ~2.5 sec
#define WD_TIME    0xFFFF
//! Для обратной совместимости
#define WDRST WDT_Reset()


int WDT_Init(void);
void WDT_Reset(void);


#endif
