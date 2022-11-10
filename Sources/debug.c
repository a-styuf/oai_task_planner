  /**
  ******************************************************************************
  * @file           : debug.c
  * @version        : v1.0
  * @brief          : функции для отладки, отключаемы define DEBUG
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "debug.h"


/**
  * @brief  управление GPIO для отладки
  * @param  state состояние включения
  */
void dbg_gpio(uint8_t state)
{
#ifdef DEBUG
  PortControl* port = DBG_GPIO_PORT;
  if (state == DBG_GPIO_ON){
    port->SRXTX = (1<<(DBG_GPIO_NUM));
  }
  else{
    port->CRXTX = (1<<(DBG_GPIO_NUM));
  }
#endif
}

void dbg_gpio_pulse(void)
{
  dbg_gpio(DBG_GPIO_OFF);
  dbg_gpio(DBG_GPIO_ON);
  dbg_gpio(DBG_GPIO_OFF);
}

/**
  * @brief  вывод на экран строки
  * @param  str: 0-терминатед строка для вывода в отладочный UART
  */
void dbg_print(char *buff)
{
#ifdef DEBUG
  if (strlen(buff) >= 127){
    buff[127] = 0;
  }
  //UART1_SendPacket((uint8_t*)buff, strlen(buff));
#endif
}

struct __FILE {
  int handle;
};
FILE __stdout;

int fputc(int ch, FILE *f) {
  #ifdef DEBUG
    //UART1_SendPacket((uint8_t *)&ch, 1);
    return ch;
  #else 
		return ch;
  #endif
}

int ferror(FILE *f) {
  return 0;
}
