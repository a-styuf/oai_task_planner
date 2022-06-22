/**
 * @file wdt.c
 * @author Alexey Styuf (a-styuf@yandex.ru)
 * @brief 
 * @version 0.1
 * @date 2022-05-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "wdt.h"


/**
 * @brief Инициализация сторожевого таймера
 * 
 * @return int 
 */
int WDT_Init() {
  int i;
  //
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER0_CLK |= (1<<6);
  //
  WDT->KEY = 0x5555;
  if(WDT->KEY == 0x5555) {
    WDT->PRL = WD_TIME;
    if(WDT->PRL == WD_TIME) {
      WDT->KEY = 0xAAAA;
      for(i=0; i<3000; i++);
      if(WDT->CNT == WD_TIME) {
        WDT->KEY = 0xCCCC;
        if(WDT->KEY == 0xCCCC) {
          WDT->EN = 0x3333;
          CLK_CNTR->PER0_CLK &= ~(1<<6);
          return 0;
          }
        }
      }
    }
  //
  CLK_CNTR->PER0_CLK &= ~(1<<6);
  //
  return -1;
}

/**
 * @brief Функция перезапуска сторожевого таймера
 * 
 * @return int 
 */
void WDT_Reset(void)
{
  // CLK_CNTR->KEY = _KEY_;
  // CLK_CNTR->PER0_CLK |= (1<<6);
  //
  WDT->KEY = 0xAAAA;
  WDT->KEY = 0x5555;
  //
  // CLK_CNTR->PER0_CLK &= ~(1<<6);
}
