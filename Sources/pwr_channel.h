#ifndef _PWR_CHANNEL_H_
#define _PWR_CHANNEL_H_

#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "timers.h"
#include "internal_bus.h"
#include "wdt.h"

/**
 * @brief типы каналов по способу включения/отключения
 */

#define PWR_CH_FLAG 1  //! функция отключения/включения реализована установкой сигнала в постоянное значение
#define PWR_CH_PULSE 2  //! функция отключения/включения реализована подачей импульса переключения состояния
#define PWR_CH_IB_CTRL 3  //! функция отключения/включения и контроль тока реализован через ВШ

#define PWR_CH_PULSE_TIME_MS 20  //! длительность импульса включения/отключения для типа канала 2

#define PWR_CH_CFG_TYPE_CM 1
#define PWR_CH_CFG_TYPE_IB 2

/**
  * @brief  структура переменных, используемых для работы через ЦМ
  */
typedef struct
{
  uint8_t self_type;
  type_SINGLE_GPIO gpio_on, gpio_off;
  typeADCStruct* adc_ptr;
  uint8_t adc_ch_num;
}typePwrChCtrlByCM;

/**
  * @brief  структура переменных, используемых для работы через ВШ
  */
typedef struct
{
  uint8_t self_type;
  typeIBStruct* ptr;
  uint8_t id;
  int8_t adc_ch_num;
  int8_t ctrl_ch_num;
}typePwrChCtrlByIB;

/**
  * @brief  структура управления каналом питания
  */
typedef struct
{
  uint8_t type, auto_control, double_ch;
  uint8_t state, status, half_set;
  float current_bound;
  //
  typePwrChCtrlByCM cm_ctrl;
  typePwrChCtrlByIB ib_ctrl;
  // general
  float row_data;
  float curr_a, curr_b; //! коэффициенты пересчета сырых показаний в значения тока в мА
  float current_fp;
  uint16_t current;
}typePowerCh;

//
int8_t pwr_ch_init(typePowerCh* pwr_ch_ptr, uint8_t type, uint8_t auto_control, float bound, uint8_t double_ch, float A, float B, void* cfg);
typePwrChCtrlByCM pwr_ch_cm_setting(PortControl* port_on, uint8_t num_on, PortControl* port_off, uint8_t num_off, typeADCStruct* adc_ptr, uint8_t adc_ch_num);
typePwrChCtrlByIB pwr_ch_ib_setting(typeIBStruct* ib_ptr, uint8_t ib_id, uint8_t ch_num, uint8_t ctrl_ch_num);
void pwr_ch_on_off(typePowerCh* pwr_ch_ptr, uint8_t state);
void pwr_ch_half_set_choosing(typePowerCh* pwr_ch_ptr, uint8_t half_set);
void __pwr_ch_ib_on_off(typePowerCh* pwr_ch_ptr, uint16_t state);
void __pwr_ch_cm_on_off(typePowerCh* pwr_ch_ptr, uint16_t state);
void pwr_ch_current_process(typePowerCh* pwr_ch_ptr, uint16_t row_data);
void pwr_ch_set_current_bound(typePowerCh* pwr_ch_ptr, float bound);
void __pwr_cm_get_adc_data(typePowerCh* pwr_ch_ptr);

#endif
