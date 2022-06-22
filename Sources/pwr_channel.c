/**
 * @file pwr_channel.c
 * @author Alexey Styuf (a-styuf@yandex.ru)
 * @brief 
 * @version 0.1
 * @date 2022-05-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "pwr_channel.h"

// channels
/**
  * @brief  инициализация отдельного канала управления питанием
	* @param  pwr_ch_ptr указатель на структуру управления отдельным каналом питания
	* @param  type типы каналов по способу включения/отключения: NU, FLAG, PULSE
	* @param  auto_control 1 - канал может отключаться автоматически, 0 - канал отключается только вручную
	* @param  bound граница тока потребления по данному каналу
	* @param  double_ch возможность работы с двумя полукомплектами
	* @param  A коэффициенты пересчета сырых показаний в значения тока в мА: I[mA] = A*row_data + B
	* @param  B коэффициенты пересчета сырых показаний в значения тока в мА: I[mA] = A*row_data + B
	* @param  cfg конфигурация работы канала
  */
int8_t pwr_ch_init(typePowerCh* pwr_ch_ptr, uint8_t type, uint8_t auto_control, float bound, uint8_t double_ch, float A, float B, void* cfg)
{
	pwr_ch_ptr->type = type;
	pwr_ch_ptr->auto_control = auto_control;
	pwr_ch_ptr->double_ch = double_ch;
	//
	pwr_ch_ptr->state = 0x00;
	pwr_ch_ptr->status = 0x00;
	pwr_ch_ptr->half_set = 0x00;
	//
	pwr_ch_ptr->curr_a = A;
	pwr_ch_ptr->curr_b = B;
	//
	pwr_ch_ptr->current_bound = bound;
	//
	memset((uint8_t*)&pwr_ch_ptr->cm_ctrl, 0x00, sizeof(pwr_ch_ptr->cm_ctrl));
	memset((uint8_t*)&pwr_ch_ptr->ib_ctrl, 0x00, sizeof(pwr_ch_ptr->ib_ctrl));
	//
	switch(type){
		case PWR_CH_FLAG:
		case PWR_CH_PULSE:
			pwr_ch_ptr->cm_ctrl = *(typePwrChCtrlByCM*)cfg;
			if (pwr_ch_ptr->cm_ctrl.self_type != PWR_CH_CFG_TYPE_CM) return -1;
			break;
		case PWR_CH_IB_CTRL:
			pwr_ch_ptr->ib_ctrl = *(typePwrChCtrlByIB*)cfg;
			if (pwr_ch_ptr->ib_ctrl.self_type != PWR_CH_CFG_TYPE_IB) return -1;
	}
	//
	return 1;
}

/**
 * @brief формирование настроек для работы с каналом под управлением МК ЦМ
 * 
 * @param port_on 
 * @param num_on 
 * @param port_off 
 * @param num_off 
 * @param adc_ptr 
 * @param adc_ch_num 
 * @return typePwrChCtrlByCM 
 */
typePwrChCtrlByCM pwr_ch_cm_setting(PortControl* port_on, uint8_t num_on, PortControl* port_off, uint8_t num_off, typeADCStruct* adc_ptr, uint8_t adc_ch_num)
{
	typePwrChCtrlByCM retcfg;
	retcfg.self_type = PWR_CH_CFG_TYPE_CM;
	gpio_init(&retcfg.gpio_on, port_on, num_on);
	gpio_init(&retcfg.gpio_off, port_off, num_off);
	retcfg.adc_ptr = adc_ptr;
	retcfg.adc_ch_num = adc_ch_num;
	//
	return retcfg;
}

/**
 * @brief формирование настроек для работы канала под управлением ВШ
 * 
 * @param ib_ptr
 * @param ib_id 
 * @param ch_num 
 * @return typePwrChCtrlByIB 
 */
typePwrChCtrlByIB pwr_ch_ib_setting(typeIBStruct* ib_ptr, uint8_t ib_id, uint8_t adc_ch_num, uint8_t ctrl_ch_num)
{
	typePwrChCtrlByIB retcfg;
	retcfg.self_type = PWR_CH_CFG_TYPE_IB;
	retcfg.ptr = ib_ptr;
	retcfg.id = ib_id;
	retcfg.adc_ch_num = adc_ch_num;
	retcfg.ctrl_ch_num = ctrl_ch_num;
	return retcfg;
}


/**
  * @brief  включение/отключение канала питания
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  state  1: on, 0: off
  */
void pwr_ch_on_off(typePowerCh* pwr_ch_ptr, uint8_t state)
{
	switch(pwr_ch_ptr->type){
		case PWR_CH_FLAG:
		case PWR_CH_PULSE:
			__pwr_ch_cm_on_off(pwr_ch_ptr, state);
			break;
		case PWR_CH_IB_CTRL:
			__pwr_ch_ib_on_off(pwr_ch_ptr, state);
			break;
	}
	WDRST;
}

/**
 * @brief выбор полукомплекта управления каналом питания
 * 
 * @param pwr_ch_ptr 
 * @param half_set 
 */
void pwr_ch_half_set_choosing(typePowerCh* pwr_ch_ptr, uint8_t half_set)
{
	pwr_ch_ptr->half_set = half_set & 0x01;
	pwr_ch_ptr->status = 0;
}

/**
 * @brief отключение/включение каналов через внутреннюю шину
 * 
 * @param pwr_ch_ptr указатель на модель управления каналом
 * @param state состояние включения/отключения
 */
void __pwr_ch_ib_on_off(typePowerCh* pwr_ch_ptr, uint16_t state)
{
	uint16_t data[2] = {0};
	//
	typeIBStruct* ib_ptr = pwr_ch_ptr->ib_ctrl.ptr;
	uint8_t ib_id = pwr_ch_ptr->ib_ctrl.id;
	int8_t num = pwr_ch_ptr->ib_ctrl.ctrl_ch_num;
	uint8_t half_set = pwr_ch_ptr->half_set;
	uint16_t ctrl_word = 0;
	//
	if (num >= 0){
		if(state == 0){
			ctrl_word = ((num & 0xFF) << 8) | (0x00 & 0xFF);
			pwr_ch_ptr->state = 0;
		}
		else{
			ctrl_word = ((((num & 0xFF) << 8) | ((half_set & 0x01) + 1)) & 0xFFFF);
			pwr_ch_ptr->state = 1;
		}
		// printf("ch ctrl:0x%04X\n", ctrl_word);
	}
	else {
		pwr_ch_ptr->state = (state != 0) ? 1 : 0;
		// printf("ch NU\n");
		return; 
	}
	//
	data[0] = ctrl_word;
	//
	ib_run_transaction(ib_ptr, ib_id, MB_F_CODE_6, 10, 1, data);
}

/**
 * @brief отключение/включение каналов через ЦМ
 * 
 * @param pwr_ch_ptr указатель на модель управления каналом
 * @param state состояние включения/отключения
 */
void __pwr_ch_cm_on_off(typePowerCh* pwr_ch_ptr, uint16_t state)
{
	//uint16_t data[2];
	type_SINGLE_GPIO *gpio_on_ptr = &pwr_ch_ptr->cm_ctrl.gpio_on;
	type_SINGLE_GPIO *gpio_off_ptr = &pwr_ch_ptr->cm_ctrl.gpio_off;
	//
	switch(pwr_ch_ptr->type){
		case(PWR_CH_FLAG):
				if (state & 0x01){
					gpio_set(gpio_on_ptr, GPIO_ON);
					gpio_set(gpio_off_ptr, GPIO_OFF);
					pwr_ch_ptr->state = 1;
					pwr_ch_ptr->status = 0;
				}
				else{
					gpio_set(gpio_on_ptr, GPIO_OFF);
					gpio_set(gpio_off_ptr, GPIO_ON);
					pwr_ch_ptr->state = 0;
				}
			break;
		case(PWR_CH_PULSE):
				if (state & 0x01){
					gpio_set(gpio_on_ptr, GPIO_ON);
					gpio_set(gpio_off_ptr, GPIO_OFF);
					pwr_ch_ptr->state = 1;
					pwr_ch_ptr->status = 0;
				}
				else{
					gpio_set(gpio_on_ptr, GPIO_OFF);
					gpio_set(gpio_off_ptr, GPIO_ON);
					pwr_ch_ptr->state = 0;
				}
				Timer_Delay(1, PWR_CH_PULSE_TIME_MS);
				gpio_set(gpio_on_ptr, GPIO_ON);
				gpio_set(gpio_off_ptr, GPIO_ON);
			break;
		default:
			break;
	}
}

/**
	* @brief  для каналов без внутреннего контроля тока установка значения из вне, для каналов с внутренним контролем - обновление данных тока
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  row_data сырые данные тока
  */
void pwr_ch_current_process(typePowerCh* pwr_ch_ptr, uint16_t row_data)
{
	if ((pwr_ch_ptr->type == PWR_CH_FLAG) || (pwr_ch_ptr->type == PWR_CH_PULSE)){
		__pwr_cm_get_adc_data(pwr_ch_ptr);
	}
	if (pwr_ch_ptr->type == PWR_CH_IB_CTRL){
		pwr_ch_ptr->row_data = (float)row_data;
	}
	//
	pwr_ch_ptr->current_fp = pwr_ch_ptr->curr_a * pwr_ch_ptr->row_data + pwr_ch_ptr->curr_b;
	// printf("a:%.3E,b:%.3E,data:%d, type:%d\n", pwr_ch_ptr->curr_a, pwr_ch_ptr->curr_b, row_data, pwr_ch_ptr->type);
	if (pwr_ch_ptr->current_fp > 0) pwr_ch_ptr->current = (uint16_t)floor(pwr_ch_ptr->current_fp);
	else pwr_ch_ptr->current = 0;
	//
	if (pwr_ch_ptr->current_bound == 0){
		//
	}
	else if(pwr_ch_ptr->current >= pwr_ch_ptr->current_bound){
		pwr_ch_ptr->status = 1;
	}
}

/**
 * @brief установка границы срабатывания токовой защиты
 * 
 * @param pwr_ch_ptr указатель на структуру управления
 * @param bound значение границы срабатывания токовой защиты
 */
void pwr_ch_set_current_bound(typePowerCh* pwr_ch_ptr, float bound)
{
	pwr_ch_ptr->current_bound = bound;
	pwr_ch_ptr->status = 0;
}

/**
 * @brief получение данных о токе канала из АЦП
 * 
 */
void __pwr_cm_get_adc_data(typePowerCh* pwr_ch_ptr)
{
	pwr_ch_ptr->row_data = adc_ch_voltage(pwr_ch_ptr->cm_ctrl.adc_ptr, pwr_ch_ptr->cm_ctrl.adc_ch_num);
}
