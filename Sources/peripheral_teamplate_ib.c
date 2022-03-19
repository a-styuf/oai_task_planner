/**
 * @file perepherial_teamplate.c
 * @author a-styuf (a-styuf@yandex.ru)
 * @brief          : пример работы с переферией через внутреннюю шину
	* @note				- модуль обращается к внутренней шине один раз в измерительный интервал
	* @note				- создает кадр-шаблон для архивной памяти
 * @version 0.1
 * @date 2022-03-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "peripheral_teamplate_ib.h"

/**
  * @brief  инициализация структуры управления
	* @param  per_tmplt_ptr указатель на структуру управления
	* @param  id номер устройства на внутренней шине
	* @param  device_number номер устройства в котором работает МПП
	* @param  frame_type тип кадра для МПП
	* @param  channel номер используемого канала МПП
	* @param  offset значение уставки определение помехи в кв АЦП
	* @param  ib_ptr указатель на внутреннюю шину
	* @param  gl_fr_num указатель на сковозной глобальный номер кадра
  */
void per_tmplt_init(typePERTMPLTStruct* per_tmplt_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type, typeIBStruct* ib_ptr, uint32_t* gl_fr_num)
{
	per_tmplt_reset_parameters(per_tmplt_ptr);
	per_tmplt_ptr->id = id;
	per_tmplt_ptr->device_number = device_number;
	per_tmplt_ptr->frame_type = frame_type;
	per_tmplt_ptr->ib = ib_ptr;
	per_tmplt_ptr->self_num = self_num;
	per_tmplt_ptr->global_frame_num_ptr = gl_fr_num;
	// работы с циклограммамив
	per_tmplt_meas_cycl_init(per_tmplt_ptr);
}

/**
  * @brief  инициализация структуры управления
	* @param  per_tmplt_ptr указатель на структуру управления
  */
void per_tmplt_reset_parameters(typePERTMPLTStruct* per_tmplt_ptr)
{
	//зануление данных на передачу в МКО и память
	memset((uint8_t*)per_tmplt_ptr->meas_buff, 0xFE, sizeof(per_tmplt_ptr->meas_buff));
	memset((uint8_t*)&per_tmplt_ptr->frame, 0xFE, sizeof(per_tmplt_ptr->frame));
	per_tmplt_ptr->frame_data_ready = 0x00;
	per_tmplt_ptr->meas_ptr = 0;
	//
	per_tmplt_ptr->frame_pulse_cnt = 0;
	per_tmplt_ptr->last_call_time_us = 0;
	per_tmplt_ptr->interval_ms = PER_TMPLT_DEFAULT_INTERVAL_MS;
	per_tmplt_ptr->meas_event_num = 0;
}

/**
  * @brief  процесс обработки состояния МПП
	* @param  per_tmplt_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t per_tmplt_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval = 0;
  	typePERTMPLTStruct* per_tmplt_ptr = (typePERTMPLTStruct*)ctrl_struct;
	/// запус обработчика по интервалу
	if ((time_us - per_tmplt_ptr->last_call_time_us) > (per_tmplt_ptr->interval_ms*1000)) {
		per_tmplt_ptr->last_call_time_us = time_us;
		// user code begin
		
		// user code end
		retval = 1;
	}
	// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
	if (interface->event[per_tmplt_ptr->self_num] & PER_TMPLT_EVENT_MEAS_INTERVAL_START) 
	{
		interface->event[per_tmplt_ptr->self_num] &= ~PER_TMPLT_EVENT_MEAS_INTERVAL_START;
		cyclo_start(&per_tmplt_ptr->meas_cyclo);
		retval = 1;
	}
	else {
		//
	}
	//обработка циклограммы
	cyclo_handler(&per_tmplt_ptr->meas_cyclo, time_us/1000);
	//обработка передачи данных
	if(per_tmplt_ptr->frame_data_ready)
	{
		per_tmplt_ptr->frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&per_tmplt_ptr->frame, sizeof(typeMPPFrameUnion));
		interface->event[per_tmplt_ptr->self_num] |= PER_TMPLT_EVENT_MEAS_INTERVAL_START;
	}
	return retval;
}


/**
  * @brief  функция получения данных для складывания в кадр
	* @param  per_tmplt_ptr указатель на структуру управления
	* @param  struct_num указатель на кадр
	* @retval  1 - кадр сформирован, 0 - недостаточно данных для кадра
  */
int8_t per_tmplt_frame_forming(typePERTMPLTStruct* per_tmplt_ptr)
{
	uint8_t i = 0;
	//
	if(per_tmplt_ptr->meas_ptr >= 2){
		per_tmplt_ptr->frame.row.label = 0x0FF1;
		per_tmplt_ptr->frame.row.definer = frame_definer(0, per_tmplt_ptr->device_number, NULL, per_tmplt_ptr->frame_type);
		per_tmplt_ptr->frame.row.num = (*per_tmplt_ptr->global_frame_num_ptr++)&0xFFFF;
		per_tmplt_ptr->frame.row.time = Get_Time_s();
		//
		for (i=0; i<2; i++){
			per_tmplt_ptr->meas_ptr -= 1;
			per_tmplt_ptr->frame.per_tmplt.meas[i] = per_tmplt_ptr->meas_buff[per_tmplt_ptr->meas_ptr];	
		}
		//
		per_tmplt_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&per_tmplt_ptr->frame.row, sizeof(typeFrameStruct) - 2);
		//
		per_tmplt_ptr->frame_data_ready = 1;
		return 1;
	}
	else{
		return 0;
	}
}

/**
  * @brief  пример запроса 1 во внутреннюю шину
	* @param  per_tmplt_ptr указатель на структуру управления
  */
void per_tmplt_example_1(typePERTMPLTStruct* per_tmplt_ptr)
{
	uint16_t data[2];
	data[0] = __REV16(0xAAAA);
	data[0] = __REV16(0x5555);
	ib_run_transaction(per_tmplt_ptr->ib, per_tmplt_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  пример запроса 2 во внутреннюю шину c параметром
	* @param  per_tmplt_ptr указатель на структуру управления
	* @param  parameter произвольный параметр
  */
void per_tmplt_example_2(typePERTMPLTStruct* per_tmplt_ptr, uint32_t parameter)
{
	uint16_t data[2];
	data[0] = __REV16(0xA5A5);
	data[0] = __REV16(parameter);
	ib_run_transaction(per_tmplt_ptr->ib, per_tmplt_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  запрос к МПП на выдачу 2-х структур из архивной памяти в оперативную
	* @param  per_tmplt_ptr указатель на структуру управления
  */
void per_tmplt_example_3_meas_get(typePERTMPLTStruct* per_tmplt_ptr)
{
	uint8_t i, in_data[64];
	typePERTMPLTMeas meas[2];
	// чтение массива с измерением
	if (ib_run_transaction(per_tmplt_ptr->ib, per_tmplt_ptr->id, 3, 7, 13, NULL) > 0){
		ib_get_answer_data(per_tmplt_ptr->ib, in_data, 26*1);
		// сохраняем полученные структуры в свои переменные, но отбрасываем номер канала
		memcpy((uint8_t*)&meas[0], in_data+2, sizeof(typePERTMPLTMeas));
		// приводим порядок байт к используемому в МК
		__per_tmplt_struct_rev(&meas[0]);
		//
		per_tmplt_ptr->forced_start_flag = 1;
		//
		for (i=0; i<1; i++){
			if(meas[i].time != 0){ // проверяем есть ли измерение в прочитанных данных
				per_tmplt_ptr->meas_buff[per_tmplt_ptr->meas_ptr] = meas[i];
				per_tmplt_ptr->meas_ptr++;
				if (per_tmplt_ptr->meas_ptr >= PER_TMPLT_REC_FIFO_DEPTH) per_tmplt_ptr->meas_ptr = PER_TMPLT_REC_FIFO_DEPTH-1;
				if (per_tmplt_ptr->meas_ptr >= 2) {
					per_tmplt_frame_forming(per_tmplt_ptr);
				}
			}
		}
	}
}

/**
  * @brief  перестановка байт в словах попарно в структуре измерения
	* @param  per_tmplt_struct_ptr указатель на структуру
  */
void __per_tmplt_struct_rev(typePERTMPLTMeas* per_tmplt_struct_ptr)
{
		uint16_t data[32];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)per_tmplt_struct_ptr, sizeof(typePERTMPLTMeas));
    for (i=0; i<(sizeof(typePERTMPLTMeas)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)per_tmplt_struct_ptr, (uint8_t *)data, sizeof(typePERTMPLTMeas));
}

/**
  * @brief  инициализация циклограмм опроса переферии
	* @param  per_tmplt_ptr указатель на структуру управления
  */
void per_tmplt_meas_cycl_init(typePERTMPLTStruct* per_tmplt_ptr)
{
	// инициализация циклограммы
	cyclo_init(&per_tmplt_ptr->meas_cyclo);
	//
	cyclo_add_step(&per_tmplt_ptr->meas_cyclo, CYCLO_DO_NOTHING, (void*)per_tmplt_ptr, 100);
	cyclo_add_step(&per_tmplt_ptr->meas_cyclo, per_tmplt_meas_cycl_example_1, (void*)per_tmplt_ptr, 200);
	cyclo_add_step(&per_tmplt_ptr->meas_cyclo, per_tmplt_meas_cycl_example_2, (void*)per_tmplt_ptr, 150);
	cyclo_add_step(&per_tmplt_ptr->meas_cyclo, per_tmplt_meas_cycl_example_3, (void*)per_tmplt_ptr, 0);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void per_tmplt_meas_cycl_example_1(void* ctrl_struct)
{
	typePERTMPLTStruct* per_tmplt_ptr = (typePERTMPLTStruct*) ctrl_struct;
	per_tmplt_example_1(per_tmplt_ptr);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void per_tmplt_meas_cycl_example_2(void* ctrl_struct)
{
	typePERTMPLTStruct* per_tmplt_ptr = (typePERTMPLTStruct*) ctrl_struct;
	per_tmplt_example_2(per_tmplt_ptr, 1);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void per_tmplt_meas_cycl_example_3(void* ctrl_struct)
{
	typePERTMPLTStruct* per_tmplt_ptr = (typePERTMPLTStruct*) ctrl_struct;
	per_tmplt_example_3_meas_get(per_tmplt_ptr);
}
