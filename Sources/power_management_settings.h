
/**
 * @brief Настройки количества и типов каналов питания, а также обвязки измерения тока
 * 
 */

#ifndef _POWER_MANAGER_SETTINGS_H_
#define _POWER_MANAGER_SETTINGS_H_


#include "main.h"
#include "pwr_channel.h"

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
#define PWR_ADC_CH_NUM          {1}
#define PWR_CAL_ADC_TO_V_A      {8.06E-4}
#define PWR_CAL_ADC_TO_V_B      {0.00E+0}
#define PWR_CAL_RES_SHUNT_OHM   {0.015}
#define PWR_CAL_FB_SHUNT_OHM    {100E3}


#define PWR_CURRENT_BOUND {3*650}

//
#define PWR_GPIO_PORT_ON {NULL}
#define PWR_GPIO_NUM_ON {NULL}

#define PWR_GPIO_PORT_OFF {NULL}
#define PWR_GPIO_NUM_OFF {NULL}

#define PWR_CHANNELS_TYPES  {PWR_CH_FLAG}

#define PWR_CHANNEL_CTRL_IB_NUM   {-1}
#define PWR_CHANNEL_ADC_IB_NUM    {0}

#define PWR_AUTO_CTRL       {0}  // указываем те каналы, которые мы может отключать или включать автоматически
#define PWR_DOUBLE_OUT      {0}  // указываем те каналы, которые имеют дублированный выход для полукомплектов с холодным резервированием


#endif
