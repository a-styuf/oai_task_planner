#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_


#include "main.h"
#include "power_management_settings.h"
#include "pwr_channel.h"
#include "adc.h"
#include "gpio.h"
#include "timers.h"
#include "internal_bus.h"

#define PWR_PROCESS_PERIOD 200

/**
  * @brief  структура управления каналом питания
  */
typedef struct
{
  typePowerCh ch[PWR_CH_NUMBER];
  uint32_t status;
  uint32_t state;
  float curr_report_fp[PWR_CH_NUMBER];
  //ib - костыль для быстрого вычитывания всех показаний тока
  typeIBStruct* ib_ptr;
  uint8_t ib_id;
  uint16_t adc_data[8];
  //
  uint64_t last_call_time_us;
}typePower;
//
void pwr_init(typePower* pwr_ptr, typeADCStruct* adc_ptr, typeIBStruct* ib_ptr, uint8_t ib_id);
void pwr_process(typePower* pwr_ptr);
void pwr_on_off_by_num(typePower* pwr_ptr, uint8_t num, uint8_t state);
void pwr_on_off_by_num_auto(typePower* pwr_ptr, uint8_t num, uint8_t state);
void pwr_all_on_off(typePower* pwr_ptr, uint8_t state);
void pwr_status_reset_by_num(typePower* pwr_ptr, uint8_t num);
void pwr_set_state(typePower* pwr_ptr, uint32_t state);
void pwr_set_bound(typePower* pwr_ptr, uint8_t num, uint16_t bound);
void pwr_apply_adc_data(typePower* pwr_ptr, uint16_t* adc_data);
void pwr_create_report(typePower* pwr_ptr);
// static
void __pwr_calc_current_coefficients(float adc_a, float adc_b, float r_sh, float r_fb, float* curr_a_ptr, float* curr_b_ptr);
// task planer function
int8_t pwr_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);

#endif
