  /**
  ******************************************************************************
  * @file           : gpio.c
  * @version        : v1.0
  * @brief          : функции для работы с GPIO. Явный недостаток - невозможность управления одновременно групой gpio.
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "gpio.h"

/**
  * @brief  инициализация модели GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  * @param  port указатель на порт устройства типа PortControl
  * @param  num номер GPIO внутри port
  */
void gpio_init(type_SINGLE_GPIO* gpio_ptr, PortControl* port, uint8_t num)
{
  gpio_ptr->port  = port;
  gpio_ptr->num = num;
    // printf("gpio init: p=0x%08X, n=%d\n", gpio_ptr->port, gpio_ptr->num);
}

/**
  * @brief  инициализация прерывания от GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  */
void gpio_set_irq(type_SINGLE_GPIO* gpio_ptr)
{
  PortControl *port = gpio_ptr->port;
  uint8_t num = gpio_ptr->num;
  port->KEY = _KEY_;
  port->CFUNC[num/8] = (0xF << (4*(num % 8)));
  port->SFUNC[num/8] = (0x0 << (4*(num % 8)));
  port->SPULLDOWN = (1 << num);
  port->SPWR[num/16] = (0x2 << (2*(num % 16)));
  port->CRXTX = (0x1 << num);
  port->CIE = (1 << num);
  port->SIE = (1 << num);
  port->CIT = (1 << num);
  port->SIT = (0 << num);
  NVIC_EnableIRQ(IRQn_GPIO);
}

/**
  * @brief  обработка прерываний для формирования прерывания по положительному фронту
  * @param  gpio_ptr указатель на програмvную модель устройства
  * @retval uint8_t 1 - произошло прерывание по положительному фронту, 0 - отсутствие положительного фронта
  */
uint8_t gpio_manage_irq(type_SINGLE_GPIO* gpio_ptr)
{
  if (gpio_get(gpio_ptr) == 1){
		gpio_ptr->port->CIE = 0x01 << gpio_ptr->num;
		gpio_ptr->port->CIR = 0x01 << gpio_ptr->num;
		gpio_ptr->port->CIT = 0x01 << gpio_ptr->num;
		gpio_ptr->port->SIE = 0x01 << gpio_ptr->num;
		//printf("SPW irq <%d> rise\n", gpio_ptr->num);
    return 1;
	}
	else{
		gpio_ptr->port->CIE = 0x01 << gpio_ptr->num;
		gpio_ptr->port->CIR = 0x01 << gpio_ptr->num;
		gpio_ptr->port->SIT = 0x01 << gpio_ptr->num;
		gpio_ptr->port->SIE = 0x01 << gpio_ptr->num;
		//printf("SPW irq <%d> fall\n", gpio_ptr->num);
    return 0;
	}
}

/**
  * @brief  установка значения отдельного GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  * @param  val значения для установки (0 - 0, не 0 - 1)
  */
void gpio_set(type_SINGLE_GPIO* gpio_ptr, uint8_t val)
{
	if (gpio_ptr->port){ //проверка на наличие подобного канала, если 0 - значит канала не существует
		if (val){
			gpio_ptr->port->SRXTX = (1<<(gpio_ptr->num));
		}
		else{
			gpio_ptr->port->CRXTX = (1<<(gpio_ptr->num));
		}
	}
}

/**
  * @brief  чтение значения GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  * @retval  значение GPIO - 0 или 1
  */
uint8_t gpio_get(type_SINGLE_GPIO* gpio_ptr)
{
  if (gpio_ptr->port){ //проверка на наличие подобного канала, если 0 - значит канала не существует
    return (gpio_ptr->port->RXTX >> gpio_ptr->num) & 0x1;
  }
  else {
    return 0;
  }
}

/**
  * @brief  CallBack от обработчика прерывания GPIO
  */
__weak void INT_PORT_CallBack(void)
{
  //
}

/**
 * @brief Обработка прерываний от IO (по уровню)
 * 
 */
void PORT_IF(void){
  INT_PORT_CallBack();
}
