
/**
 * @brief Настройки количества и типов каналов питания, а также обвязки измерения тока
 * 
 */

#ifndef _POWER_MANAGER_SETTINGS_H_
#define _POWER_MANAGER_SETTINGS_H_


#include "1986ve8_lib/cm4ikmcu.h"

#define PWR_PULSE_TIME_MS 20  //! длительность импульса включения/отключения для типа канала 2
#define PWR_ON_DELAY 250      //! задержка на включение каждого следующего устройства при включении питания

/**
  * @brief  определяем номера каналов, последнее поле - количество каналов
*/
typedef enum PWR_CH
{
  PWR_BE,
  PWR_CH_NUMBER
} type_PWR_CH;

/**
  * @brief  распределение каналов в соответствии с enum PWR_CH
  * @note   длина совпадает с PWR_CH_NUMBER
  */
#define PWR_ADC_CH_NUM {1}
#define PWR_CAL_RES_SHUNT_OHM {0.5}
#define PWR_CAL_FB_SHUNT_OHM {51E3}

#define PWR_CURRENT_BOUND {3*150}

#define PWR_GPIO_PORT_ON {NULL}
#define PWR_GPIO_NUM_ON {NULL}

#define PWR_GPIO_PORT_OFF {NULL}
#define PWR_GPIO_NUM_OFF {NULL}

#define PWR_CHANNELS_TYPES {NU}
#define PWR_AUTO_CTRL {0} //указываем те каналы, которые мы может отключать или включать автоматически


#endif
