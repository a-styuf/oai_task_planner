/**
  ******************************************************************************
	* @file           	: power_management.c
	* @version        	: v1.0
	* @brief        	: библиотека для работы с питанием ЦМ
	* @author			: Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
	* @date				: 2021.09.05
  ******************************************************************************
  */

#include "power_management.h"

/**
	* @brief  инициализация модуля управления питанием
	* @param  pwr_ptr указатель на структуру управления модулем питания
  */
void pwr_init(typePower* pwr_ptr, typeADCStruct* adc_ptr, typeIBStruct* ib_ptr, uint8_t ib_id)
{
	uint8_t i = 0;
	typePwrChCtrlByCM cm_cfg;
	typePwrChCtrlByIB ib_cfg;
	void* cfg_ptr = 0;
	//
	uint8_t type_arr[PWR_CH_NUMBER] = PWR_CHANNELS_TYPES;
	uint8_t auto_control_arr[PWR_CH_NUMBER] = PWR_AUTO_CTRL;
	uint8_t double_out[PWR_CH_NUMBER] = PWR_DOUBLE_OUT;
	//
	PortControl* port_on_arr[PWR_CH_NUMBER] = PWR_GPIO_PORT_ON;
	uint8_t num_on_arr[PWR_CH_NUMBER] = PWR_GPIO_NUM_ON;
	PortControl* port_off_arr[PWR_CH_NUMBER] = PWR_GPIO_PORT_OFF;
	uint8_t num_off_arr[PWR_CH_NUMBER] = PWR_GPIO_NUM_OFF;
	//
	int8_t ib_ch_adc_num[PWR_CH_NUMBER] = PWR_CHANNEL_ADC_IB_NUM;
	int8_t ib_ch_ctrl_num[PWR_CH_NUMBER] = PWR_CHANNEL_CTRL_IB_NUM;
	//
	uint8_t adc_ch_num[PWR_CH_NUMBER] = PWR_ADC_CH_NUM;
	float adc_a[PWR_CH_NUMBER] = PWR_CAL_ADC_TO_V_A;
	float adc_b[PWR_CH_NUMBER] = PWR_CAL_ADC_TO_V_B;
	float r_sh_arr[PWR_CH_NUMBER] = PWR_CAL_RES_SHUNT_OHM;
	float r_fb_arr[PWR_CH_NUMBER] = PWR_CAL_FB_SHUNT_OHM;
	float curr_a = 0, curr_b = 0;
	uint16_t bound_arr[PWR_CH_NUMBER] = PWR_CURRENT_BOUND;
	//
	pwr_ptr->state = 0x0000;
	pwr_ptr->status = 0x0000;
	//
	for (i=0; i<PWR_CH_NUMBER; i++){
		__pwr_calc_current_coefficients(adc_a[i], adc_b[i], r_sh_arr[i], r_fb_arr[i], &curr_a, &curr_b);
		if ((type_arr[i] == PWR_CH_FLAG) || (type_arr[i] == PWR_CH_PULSE)){
			cm_cfg = pwr_ch_cm_setting(port_on_arr[i], num_on_arr[i], port_off_arr[i], num_off_arr[i], adc_ptr, adc_ch_num[i]);
			cfg_ptr = (void*)&cm_cfg;
		}
		else if (type_arr[i] == PWR_CH_IB_CTRL){
			ib_cfg = pwr_ch_ib_setting(ib_ptr, ib_id, ib_ch_adc_num[i], ib_ch_ctrl_num[i]);
			cfg_ptr = (void*)&ib_cfg;
		}
		pwr_ch_init(&pwr_ptr->ch[i], type_arr[i], auto_control_arr[i], bound_arr[i], double_out[i], curr_a, curr_b, cfg_ptr);
	}
}

/**
	* @brief  обработка всех каналов измерения
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_process(typePower* pwr_ptr)
{
	uint8_t i=0;
	uint32_t state = 0, status = 0; 
	// подсчет токов и состояний
	for(i=0; i<PWR_CH_NUMBER; i++){
		pwr_ch_current_process(&pwr_ptr->ch[i], pwr_ptr->adc_data[pwr_ptr->ch[i].ib_ctrl.adc_ch_num]);
		// printf("I%d:%d", i, pwr_ptr->ch[i].current);
		if ((pwr_ptr->ch[i].status == 1) && ((pwr_ptr->ch[i].state != 0))) {
			pwr_on_off_by_num_auto(pwr_ptr, i, 0);
		}
		state |= (pwr_ptr->ch[i].state & 0x01) << i;
		status |= (pwr_ptr->ch[i].status & 0x01) << i;
	}
	// printf("pwr:state<%04X>status<%04X>\n", state, status);
	pwr_ptr->state = state;
	pwr_ptr->status = status;
	//
	pwr_create_report(pwr_ptr);
}

/**
  * @brief  включение/отключение канала питания по номеру программно (с учетом флага автоматического контроля)
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  num номер канала
	* @param  state  1: on, 0: off
  */
void pwr_on_off_by_num(typePower* pwr_ptr, uint8_t num, uint8_t state)
{
	pwr_ch_on_off(&pwr_ptr->ch[num], state);
}

/**
  * @brief  включение/отключение канала питания по номеру программно (с учетом флага автоматического контроля)
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  num номер канала
	* @param  state  1: on, 0: off
  */
void pwr_on_off_by_num_auto(typePower* pwr_ptr, uint8_t num, uint8_t state)
{
	if(pwr_ptr->ch[num].auto_control){
		pwr_on_off_by_num(pwr_ptr, num, state);
		WDRST;
	}
}

/**
  * @brief  включение/отключение всех каналов питания c учетом флага автоматического контроля
	* @param  pwr_ptr указатель на структуру управления
	* @param  state  1: on, 0: off
  */
void pwr_all_on_off(typePower* pwr_ptr, uint8_t state)
{
	uint8_t i=0;
	for(i=0; i<PWR_CH_NUMBER; i++){
		pwr_ch_on_off(&pwr_ptr->ch[i], state);
		Timer_Delay(1, PWR_ON_DELAY);
	}
}

/**
  * @brief  сброс статуса модуля питания
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_status_reset_by_num(typePower* pwr_ptr, uint8_t num)
{
	pwr_ch_on_off(&pwr_ptr->ch[num], pwr_ptr->ch[num].state);
}

/**
  * @brief  установка состояния модулей с одновременным включением, отключением
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_set_state(typePower* pwr_ptr, uint32_t state)
{
	uint8_t i=0;
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_on_off_by_num(pwr_ptr, i, (pwr_ptr->state >> i) & 0x01);
	}
}

/**
  * @brief  установка уровня токовой защиты
	* @param  pwr_ptr указатель на структуру управления
	* @param  bound указатель на структуру управления
  */
void pwr_set_bound(typePower* pwr_ptr, uint8_t num, uint16_t bound)
{
	pwr_ptr->ch[num].current_bound = (float)bound;
}

void pwr_apply_adc_data(typePower* pwr_ptr, uint16_t* adc_data)
{
	uint8_t i = 0;
	for(i=0; i<PWR_CH_NUMBER; i++){
		pwr_ptr->adc_data[i] = __REV16(adc_data[i]);
		// printf("0x%04X ", pwr_ptr->adc_data[i]);
	}
	// printf("\n");
}

/**
  * @brief  создание отчета для удобства отображения
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_create_report(typePower* pwr_ptr)
{
	uint8_t i;
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_ptr->curr_report_fp[i] = pwr_ptr->ch[i].current_fp;
	}
}

// static
/**
  * @brief  подсчет коэффициентов перевод и сопротивлений в калибровочные коэффициенты  I(mA)=curr_a*ADC_U(V)+curr_b, где ADC_U(V) = adc_a*ADC(q)+adc_b
	* @param  adc_a коэффициент перевода показаний АЦП в напряжение
	* @param  adc_b коэффициент перевода показаний АЦП в напряжение
	* @param  r_sh сопротивление для измерения тока
	* @param  r_fb сопротивление обратной связи
	* @param  curr_a_ptr указатель на переменную adc_a
	* @param  curr_b_ptr указатель на переменную adc_b
  */
void __pwr_calc_current_coefficients(float adc_a, float adc_b, float r_sh, float r_fb, float* curr_a_ptr, float* curr_b_ptr)
{
	float volt_to_curr_coeff = 1E6/(r_sh*r_fb);
	*curr_a_ptr = volt_to_curr_coeff * adc_a;
	*curr_b_ptr = volt_to_curr_coeff * adc_b;
	// printf("c_a %.2E, c_b %.2E, v_to_c %.2E\n", *curr_a_ptr, *curr_b_ptr, volt_to_curr_coeff);
}

// Task planner process handler

/**
  * @brief  функция для запуска в планировщике задач обработки каналов питания
	* @param  ctrl_struct указатель на програмную модель устройства
	* @param  time_ms глобальное время
  */
int8_t pwr_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	typePower* pwr_ptr = (typePower*)ctrl_struct;
	if ((time_us - pwr_ptr->last_call_time_us) > (PWR_PROCESS_PERIOD*1000)) {
		pwr_ptr->last_call_time_us = time_us;
		pwr_process(pwr_ptr);
		return 1;
	}
	else {
		return 0;
	}
}
