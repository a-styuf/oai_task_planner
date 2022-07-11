#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "sysinit.h"
#include "cm.h"
#include "power_management.h"
#include "adc.h"
#include "mko.h"
#include "debug.h"
#include "internal_bus.h"
#include "frames.h"
#include "peripheral_teamplate_ib.h"
#include "task_planner.h"

//***Параметры программы для ЦМ
//версия прошивки
#define CM_SW_VERSION 			"0.2"
// номер устройства
#define FRAME_DEV_ID 			999 //999 - по умолчанию, необходимо получить номер устройства в системе ОАИ
// параметры МКО
#define MKO_ADDRESS_DEFAULT 	13  // 0 - адрес берется с разъема, не 0 - адрес МКО
//

// создание объектов отдельных программных моделей
typeTPStruct tp;		//! обязательный объект планировщика задач
typeCMModel cm;			//! объект управления периферией ЦМ
typePERTMPLTStruct per_tmplt; //! программная модель примера периферийного модуля

// прототипы функций для использования внутри main.c (описание в функциях)
void __main_process_registration_box(void);
void __main_init_peripheral_modules(void);
void __main_base_init(void);

/**
 * @brief суперцикл
 * правка только с согласованием с автором проекта
 * остальные правки на свой страх и риск
 * @return int никогда не возвращает, так как работает бесконечно
 */
int main() {
	int8_t general_state = 0;
	// базовая инициализация 
	System_Init();  // инициализация периферии микроконтроллера: платформозависимая
	Timers_Init();	// инициализация таймеров и управления временем
	//
	printf("System init: <OK>. Version %s\n", CM_SW_VERSION);
	//
	cm_init(&cm, CM, CM_SELF_MB_ID, MKO_ADDRESS_DEFAULT, FRAME_DEV_ID, CM_SW_VERSION, CM+1);  // инициализация объекта программной модели ЦМ (включает в себя все периферийные модели ЦМ: ВШ, МКО, GPIO, ADC и т.п.)
	//
	printf("CM base system: <OK>\n");
	//
	general_state = cm_load_cfg(&cm);	// загрузка конфигурации из энергонезависимой памяти
	printf("CM cfg load: <%d>\n", general_state);
	//
	pwr_all_on_off(&cm.pwr, 1);  // включение периферии согласно настройкам каналов управления питанием
	printf("All channels ON\n");
	__main_init_peripheral_modules();  //инициализация объектов периферии, подключаемых к ЦМ
	printf("Peripheral init done\n");
	tp_init(&tp);	// инициализация планировщика задач
	// сброс времени
	Time_Set(0, NULL, NULL);
	// Системные процессы
	__main_process_registration_box();  //регистрация процессов обработки ЦМ и периферии
	printf("Process registration completed\n");
	//
	general_state = WDT_Init();
	printf("WDG Timer init: <%d>\n", general_state);
	//
	while(1) {
		tp_handler(&tp); // обработка планировщика задач
		//
		WDRST;
	}
}

/**
  * @brief  обертка для инициализации всей периферии
*/
void __main_init_peripheral_modules(void)
{
	// инициализация единственного периферийного устройства
	per_tmplt_init(&per_tmplt, PER_TMPLT, 2, FRAME_DEV_ID, PER_TMPLT+1, &cm.ib, &cm.global_frame_num);

}

// Командный интерфейс
/**
  * @brief  обертка для регистрации всех необходимых процессов
*/
void __main_process_registration_box(void)
{
	// процессы для поддержания работы ЦМ и его систем: питания, АЦП, внутренней шины
	tp_process_registration(&tp, &cm_process_tp, &cm, CM*64, TP_SHARED_MEM_VOL_B);
	tp_process_registration(&tp, &adc_process_tp, &cm.adc, 0, 0);
	tp_process_registration(&tp, &pwr_process_tp, &cm.pwr, 0, 0);
	tp_process_registration(&tp, &ib_process_tp, &cm.ib, 0, 0);
	// Процессы периферийных устройств
	tp_process_registration(&tp, &per_tmplt_process_tp, &per_tmplt, 64*PER_TMPLT, 64);
}

/**
  * @brief  обертка для базовой инициализации БЭ через команду МКО (или ВШ)
*/
void __main_base_init(void)
{
	printf("CM Init: start\n");
	// отключение периферии
	pwr_all_on_off(&cm.pwr, 0);  //todo: возможно необходимо поместить данную функций после инициализации ЦМ
	// отключение планировщика задач
	tp_init(&tp);
	// архивация памяти
	fr_mem_format(&cm.mem);
	// инициализация структур
	cm_init(&cm, CM, CM_SELF_MB_ID, MKO_ADDRESS_DEFAULT, FRAME_DEV_ID, CM_SW_VERSION, CM+1);  // инициализация объекта программной модели ЦМ (включает в себя все периферийные модели ЦМ: ВШ, МКО, GPIO, ADC и т.п.)
	// включение периферии
	pwr_all_on_off(&cm.pwr, 1);
	__main_init_peripheral_modules();
	Timer_Delay(1, 1000);
	// сброс времени
	Time_Set(0, &cm.ctrl.diff_time, &cm.ctrl.diff_time_fractional);
	// регистрация процессов
	__main_process_registration_box();
	//
	printf("CM Init: finish\n");
}

void cm_mko_command_interface_handler(typeCMModel *cm_ptr)
{
	typeFrameStruct frame;
	uint16_t sa_arr[32];
	uint16_t var_uint16 = 0;
	uint32_t var_uint32 = 0;
	uint16_t ib_id, ib_fcode, ib_addr, ib_len;
	//
	if (mko_need_to_process(&cm_ptr->mko_rt)){
		switch(cm_ptr->mko_rt.cw.field.sub_addr){
			case CM_MKO_SA_CMD:
				printf("MKO CMD: cmd <%d> data <0x%04X %04X %04X>\n", cm_ptr->mko_rt.data[0], cm_ptr->mko_rt.data[1], cm_ptr->mko_rt.data[2], cm_ptr->mko_rt.data[3]);
				switch(cm_ptr->mko_rt.data[0]){
					case (CMD_TEST):
						//
						break;
					case (CMD_SYNCH_TIME):
						cm_mko_cmd_synch_time(cm_ptr);
						break;
					case (CMD_INIT):
						__main_base_init();
						break;
					case (CMD_SET_INTERVAL):
						cm_set_interval_value(cm_ptr, cm_ptr->mko_rt.data[1], cm_ptr->mko_rt.data[2]);
						break;
					case (CMD_CONST_MODE):
						//
						break;
					case (CMD_CURRENT_LVL):
						if ((cm_ptr->mko_rt.data[1] < PWR_CH_NUMBER)){
							pwr_set_bound(&cm.pwr, cm_ptr->mko_rt.data[1], cm_ptr->mko_rt.data[2]);
						}
						else {}
						break;
					case (CMD_PWR_CH_CTRL):
						if ((cm_ptr->mko_rt.data[1] < PWR_CH_NUMBER)){
							pwr_set_bound(&cm.pwr, cm_ptr->mko_rt.data[1], (uint32_t)cm_ptr->mko_rt.data[2]);
						}
						else {}
						break;
					default:
						break;
				}
				break;
			case CM_MKO_SA_ARCH_REQUEST_CM:
				if (cm_ptr->mko_rt.data[0] == 0){
					fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
					mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
				}
				break;
			case CM_MKO_SA_TECH_CMD:
				memcpy((uint8_t*)sa_arr, cm_ptr->mko_rt.data, 64);
				switch(cm_ptr->mko_rt.data[0]){
					case (TCMD_CHECK_MIRROR):
						sa_arr[0] = 0x0101;
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_arr);
						break;
					case (TCMD_CHECK_MEM):
						sa_arr[0] |= 0x0100;
						var_uint32 =  fr_mem_check(&cm_ptr->mem);
						sa_arr[1] = (var_uint32 >> 16) & 0xFFFF;
						sa_arr[2] = (var_uint32 >> 0) & 0xFFFF;
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_arr);
						break;
						case (TCMD_ANY_FRAME_READ):
						sa_arr[0] |= 0x0100;
						var_uint16 = fr_mem_read_any_frame(&cm_ptr->mem, sa_arr[1], (uint8_t *)&frame);
						sa_arr[2] = (var_uint16 >> 0) & 0xFFFF;
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_arr);
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_RD, (uint16_t*)&frame);
						break;
					case (TCMD_SET_OPERATION_TIME):
						sa_arr[0] |= 0x0100;
						if (sa_arr[1] == 0xAFAF){
							cm_ptr->ctrl.operation_time = (sa_arr[2] << 16) + (sa_arr[3] << 0);
						}
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_arr);
						break;
					case (TCMD_SET_STM):
						sa_arr[0] |= 0x0100;
						stm_temporary_set(&cm_ptr->stm, sa_arr[1], sa_arr[2]);
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_arr);
						break;
					case (TCMD_SET_IB):
						sa_arr[0] |= 0x0100;
						ib_id = (sa_arr[1] >> 8) & 0xFF;
						ib_fcode = (sa_arr[1] >> 0) & 0xFF;
						ib_addr = sa_arr[2];
						ib_len = sa_arr[3] & 0x0F;
						if (ib_run_transaction(&cm_ptr->ib, ib_id, ib_fcode, ib_addr, ib_len, (uint16_t*)&sa_arr[4]) > 0){
							ib_get_answer_data(&cm_ptr->ib, (uint8_t*)&sa_arr[4], ib_len*2);
						}
						mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_arr);
						break;
					default:
						break;
				}
				break;
		}
	}
}

/**
 * @brief обработчик отладочных команд через ВШ
 * 
 * @param cm_ptr указатель на объект управления ЦМ
 */
void cm_dbg_ib_command_handler(typeCMModel* cm_ptr)
{
	typeFrameStruct frame;
	//
	if (cm_ptr->ib.command_frame_flag){
		cm_ptr->ib.command_frame_flag = 0;
		if (cm_ptr->ib.command_frame.dev_id == CM_SELF_MB_ID){
			if (cm_ptr->ib.command_frame.f_code == MB_F_CODE_16){
				printf("CM DBG CMD: ID <%d>, FC <%d>, RA <%d> \n", cm_ptr->ib.command_frame.dev_id, cm_ptr->ib.command_frame.f_code, cm_ptr->ib.command_frame.reg_addr);
				switch(cm_ptr->ib.command_frame.reg_addr){
					case CM_DBG_CMD_SWITCH_ON_OFF:
						cm_ptr->ib.global_dbg_flag = __REV16(cm_ptr->ib.command_frame.data[0]) & 0x01;
						break;
					case CM_DBG_CMD_CM_RESET:
						printf("Reset by WDG ");
						dbg_gpio(DBG_GPIO_ON);
						Timer_Delay(1, 10000);
						printf("Error\n");
						break;
					case CM_DBG_CMD_CM_CHECK_MEM:
						fr_mem_check(&cm_ptr->mem);
						break;
					case CM_DBG_CMD_CM_INIT:
						if (__REV16(cm_ptr->ib.command_frame.data[0]) == 0xAA55) __main_base_init();
						break;
					case CM_DBG_CMD_ARCH_REQUEST:
						if (__REV16(cm_ptr->ib.command_frame.data[0]) == 0x0000){
							fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
							mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
						}
						break;
				}
			}
		}
	}
}

// Обработка callback-функций от прерываний

/**
  * @brief  обработчик прерывания АЦП
  */
void INT_ADC0_CallBack(void) {
	volatile uint32_t rslt;
	uint16_t channel, value;
	//
	NVIC_DisableIRQ(IRQn_ADC0);
	//
	while(cm.adc.regs->STATUS & 1) {
		rslt = cm.adc.regs->RESULT;
		channel = *((uint16_t*)&rslt + 1);
		value = *((uint16_t*)&rslt);
		if (channel < ADC0_CHAN_NUM){
			adc_new_val_process(&cm.adc, channel, value);
		}
	}
	//
	NVIC_EnableIRQ(IRQn_ADC0);
}

/**
  * @brief  обработчик прерывания МКО в режиме ОУ
  */
void INT_MIL0_Callback(void) 
{
	mko_rt_transaction_handler(&cm.mko_rt);
	if (cm.mko_rt.regs->ERROR == 0) stm_single_ch_temporary_set(&cm.stm, AMKO, 1, 20000);
}

/**
  * @brief  обработчик прерывания МКО в режиме КШ
  */
void INT_MIL1_Callback(void) 
{
	mko_bc_transaction_handler(&cm.mko_bc);
}

/**
  * @brief  обработчик прерывания от SysTick-таймера
  */
void Systick_Handler(void){
	tp_timer_handler(&tp);
}
