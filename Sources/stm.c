/**
 * @file stm.c
 * @author Алексей Стюф (a-styuf@yandex.ru)
 * @brief модуль управления сигналами СТМ:
 *    - поддержка установки постоянных значений
 *    - поддержка установки временных значений с возвратам к постоянным
 * @version 0.1
 * @date 2022-03-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "stm.h"

// Single channel

/**
 * @brief инициализация модуля управления каналом СТМ
 * 
 * @param stm_ch_ptr указазтель на структуру управления каналом СТМ
 * @param value начальное значение СТМ
 * @param port указатель на порт GPIO 
 * @param num номер линии в порте, используемой для СТМ сигнала
 */
int8_t stm_ch_init(type_STM_Channel_Model *stm_ch_ptr, uint8_t value, uint8_t type, void* cfg)
{
  stm_ch_ptr->const_state = value;
  stm_ch_ptr->temporary_state = 0;
  stm_ch_ptr->temporary_timeout_ms = 0;
  stm_ch_ptr->type = type;
  //
	memset((uint8_t*)&stm_ch_ptr->cm_ctrl, 0x00, sizeof(stm_ch_ptr->cm_ctrl));
	memset((uint8_t*)&stm_ch_ptr->ib_ctrl, 0x00, sizeof(stm_ch_ptr->ib_ctrl));
	//
	switch(type){
		case STM_TYPE_CM:
			stm_ch_ptr->cm_ctrl = *(typeSTMCtrlByCM*)cfg;
			if (stm_ch_ptr->cm_ctrl.self_type != STM_TYPE_CM) return -1;
			break;
		case STM_TYPE_IB:
			stm_ch_ptr->ib_ctrl = *(typeSTMCtrlByIB*)cfg;
			if (stm_ch_ptr->ib_ctrl.self_type != STM_TYPE_IB) return -1;
	}
	return 0;
}

/**
 * @brief инициализация настроек для управления через ЦМ
 * 
 * @param port 
 * @param num 
 * @return typeSTMCtrlByCM 
 */
typeSTMCtrlByCM stm_ch_cm_ctrl_get(PortControl* port, uint8_t num)
{
  typeSTMCtrlByCM cfg;
  cfg.self_type = STM_TYPE_CM;
  gpio_init(&cfg.gpio, port, num);
  return cfg;
}

/**
 * @brief инициализация настроек для управления через ВШ
 * 
 * @param ib_ptr
 * @param ib_id
 * @param num номер стм в посылке выставления СТМ по ВШ (на всякий пожарный, по умолчанию последовательно от нуля)
 * @return typeSTMCtrlByIB
 */
typeSTMCtrlByIB stm_ch_ib_ctrl_get(typeIBStruct* ib_ptr, uint8_t ib_id, uint8_t num)
{
  typeSTMCtrlByIB cfg;
  cfg.self_type = STM_TYPE_IB;
  cfg.ptr = ib_ptr;
  cfg.id = ib_id;
  cfg.num = num;
  return cfg;
}

/**
 * @brief функция обработки состояния СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @param period_ms время после прошлого вызова данной функции (должно быть больше 0 для корректной обработки состояния)
 */
void stm_ch_process(type_STM_Channel_Model *stm_ch_ptr, uint32_t period_ms)
{
  if (stm_ch_ptr->temporary_timeout_ms > 0){
    if (stm_ch_ptr->temporary_timeout_ms >= period_ms){
      stm_ch_ptr->temporary_timeout_ms -= period_ms;
    }
    else{
      stm_ch_ptr->temporary_timeout_ms = 0;
    }
    //
    stm_ch_ptr->current_state = stm_ch_ptr->temporary_state;
  }
  else{
    stm_ch_ptr->current_state = stm_ch_ptr->const_state;
  }
}

/**
 * @brief установка постоянного значения канала СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @param val значение стм: 0-0, не 0 - 1
 */
void stm_ch_const_set(type_STM_Channel_Model *stm_ch_ptr, uint8_t val)
{
  stm_ch_ptr->const_state = val;
}

/**
 * @brief установка временного значения канала СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @param val значение стм: 0-0, не 0 - 1
 * @param timeout_ms время выставление временного значения СТМ
 */
void stm_ch_temporary_set(type_STM_Channel_Model *stm_ch_ptr, uint8_t val, uint32_t timeout_ms)
{
  stm_ch_ptr->temporary_state = val;
  stm_ch_ptr->temporary_timeout_ms = timeout_ms;
}


/**
 * @brief запрос текущего значения СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @return uint8_t 1 или 0
 */
uint8_t stm_ch_get_state(type_STM_Channel_Model *stm_ch_ptr)
{
  return stm_ch_ptr->current_state;
}

//Общее управление СТМ

void stm_init(type_STM_Model *stm_ptr, typeIBStruct* ib_ptr, uint8_t ib_id)
{
  uint8_t i=0;
  uint8_t stm_type[STM_NUM] = STM_TYPE;
  PortControl* stm_io_port[STM_NUM] = STM_IO_PORT;
  uint8_t stm_io_num[STM_NUM] = STM_IO_LINE;
  uint8_t stm_default_val[STM_NUM] = STM_DEFAULT_VAL;
  uint8_t stm_ib_num[STM_NUM] = STM_DEFAULT_IB_NUM;
  //
  typeSTMCtrlByCM cm_ctrl;
  typeSTMCtrlByIB ib_ctrl;
  //
  stm_ptr->ib_ptr = ib_ptr;
  stm_ptr->ib_id = ib_id;
  stm_ptr->state = 0x00;
  stm_ptr->state_old = 0x00;
  //
  for (i=0; i<STM_NUM; i++){
    if (stm_type[i] == STM_TYPE_CM) {
      cm_ctrl = stm_ch_cm_ctrl_get(stm_io_port[i], stm_io_num[i]);
      stm_ch_init(&stm_ptr->ch[i], stm_default_val[i], stm_type[i], (void*)&cm_ctrl);
    }
    else if (stm_type[i] == STM_TYPE_IB) {
      ib_ctrl = stm_ch_ib_ctrl_get(ib_ptr, ib_id, stm_ib_num[i]);
      stm_ch_init(&stm_ptr->ch[i], stm_default_val[i], stm_type[i], (void*)&ib_ctrl);
    }
  }
}

void stm_process(type_STM_Model *stm_ptr, uint32_t period_ms)
{
  uint8_t i=0;
  for (i=0; i<STM_NUM; i++){
    stm_ch_process(&stm_ptr->ch[i], period_ms);
  }
	__stm_update_outputs(stm_ptr);
}

uint32_t __stm_update_state(type_STM_Model *stm_ptr)
{
  uint8_t i;
  uint32_t tmp_state = 0;
  //
  for (i=0; i<STM_NUM; i++){
    tmp_state |= ((stm_ptr->ch[i].current_state & 0x01) << i);
  }
  stm_ptr->state = tmp_state;
  //
  return stm_ptr->state;
}

void __stm_update_outputs(type_STM_Model *stm_ptr)
{
  uint8_t i = 0;
  uint16_t stm_ib_need_to_update = 0;
  //
  __stm_update_state(stm_ptr);
  //
  for (i=0; i<STM_NUM; i++){
    if (stm_ptr->ch[i].type == STM_TYPE_CM) {
      gpio_set(&stm_ptr->ch[i].cm_ctrl.gpio, stm_ptr->ch[i].current_state);
    }
    else if (stm_ptr->ch[i].type == STM_TYPE_IB) {
      if (stm_ptr->state != stm_ptr->state_old){
        stm_ib_need_to_update = 1;
      }
      stm_ptr->state_old = stm_ptr->state;
    }
  }
  //
  if (stm_ib_need_to_update) {
    ib_run_transaction(stm_ptr->ib_ptr, stm_ptr->ib_id, MB_F_CODE_6, 11, 1, (uint16_t*)&stm_ptr->state);
    stm_ib_need_to_update = 0;
  }
}

void stm_single_ch_const_set(type_STM_Model *stm_ptr, uint8_t num, uint32_t val)
{
  if (num < STM_NUM){
    stm_ch_const_set(&stm_ptr->ch[num], val & 0x01);
  }
}

void stm_single_ch_temporary_set(type_STM_Model *stm_ptr, uint8_t num, uint32_t val, uint32_t timeout_ms)
{
  if (num < STM_NUM){
    stm_ch_temporary_set(&stm_ptr->ch[num], val & 0x01, timeout_ms);
  }
}

void stm_const_set(type_STM_Model *stm_ptr, uint32_t val)
{
  uint8_t i=0;
  //
  for (i=0; i<STM_NUM; i++) {
    stm_ch_const_set(&stm_ptr->ch[i], val & (0x01 << i));
  }
}

void stm_temporary_set(type_STM_Model *stm_ptr, uint32_t val, uint32_t timeout_ms)
{
  uint8_t i=0;
  //
  for (i=0; i<STM_NUM; i++) {
    stm_ch_temporary_set(&stm_ptr->ch[i], (val>>i) & 0x01, timeout_ms);
    // printf("stm_ch<%d>set<%d>t<%d>\n", i, (val>>i) & 0x01, timeout_ms);
  }
}

uint8_t stm_single_ch_get_state(type_STM_Model *stm_ptr, uint8_t num)
{
  if (num < STM_NUM){
    return stm_ch_get_state(&stm_ptr->ch[num]);
  }
  return 0;
}

uint32_t stm_get_state(type_STM_Model *stm_ptr)
{
  return __stm_update_state(stm_ptr);
}
