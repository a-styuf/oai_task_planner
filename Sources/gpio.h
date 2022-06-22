#ifndef _GPIO_H_
#define _GPIO_H_

#include "main.h"

#define IRQn_GPIO (IRQn_Type)GPIO_IRQn

#define GPIO_ON 1
#define GPIO_OFF 0

#pragma pack(2)
/** 
  * @brief  структура управления одним GPIO
  */
typedef struct
{
  PortControl* port;
  uint8_t num;
} type_SINGLE_GPIO;

//
void gpio_init(type_SINGLE_GPIO* gpio_ptr, PortControl* port, uint8_t num);
void gpio_set_irq(type_SINGLE_GPIO* gpio_ptr);
uint8_t gpio_manage_irq(type_SINGLE_GPIO* gpio_ptr);
__weak void gpio_irq_callback(void);
void gpio_set(type_SINGLE_GPIO* gpio_ptr, uint8_t val);
uint8_t gpio_get(type_SINGLE_GPIO* gpio_ptr);

__weak void INT_PORT_CallBack(void);

#endif
