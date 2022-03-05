  /**
  ******************************************************************************
  * @file           : dep.c
  * @version        : v1.0
  * @brief          : библиотека для управления модулем DEP, использует объект ВШ (internal_bus)
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date						: 2021.10.01
  ******************************************************************************
  */

#include "dep.h"

/**
  * @brief  инициализация структуры управления
	* @param  dep_ptr указатель на структуру управления
	* @param  id номер устройства на внутренней шине
	* @param  device_number номер устройства в котором работает МПП
	* @param  frame_type тип кадра для МПП
	* @param  ib_ptr указатель на внутреннюю шину
	* @param  gl_fr_num указатель на сковозной глобальный номер кадра
  */
void dep_init(typeDEPStruct* dep_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type, typeIBStruct* ib_ptr, uint32_t* gl_fr_num)
{
  dep_reset_parameters(dep_ptr);
	dep_ptr->id = id;
	dep_ptr->device_number = device_number;
	dep_ptr->frame_type = frame_type;
	dep_ptr->ib = ib_ptr;
	dep_ptr->self_num = self_num;
	dep_ptr->global_frame_num_ptr = gl_fr_num;
	// работы с циклограммами
	dep_meas_cycl_init(dep_ptr);
}

/**
  * @brief  инициализация структуры управления
	* @param  dep_ptr указатель на структуру управления
  */
void dep_reset_parameters(typeDEPStruct* dep_ptr)
{
	//зануление данных на передачу в МКО и память
	memset((uint8_t*)&dep_ptr->frame, 0xFE, sizeof(dep_ptr->frame));
	dep_ptr->frame_data_ready = 0x00;
	//
	memset((uint8_t*)dep_ptr->rec_fifo, 0xFE, sizeof(dep_ptr->rec_fifo));
	dep_ptr->rec_num = 0;
	dep_ptr->rec_max = 0;
	//
	dep_ptr->last_call_time_us = 0;
	dep_ptr->meas_event_num = 0;
	dep_ptr->interval_ms = DEP_DEFAULT_INTERVAL_MS;
	dep_ptr->const_mode = 0x00;
}

/**
  * @brief  процесс обработки состояния ДЭП
	* @param  dep_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t dep_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval = 0;
  typeDEPStruct* dep_ptr = (typeDEPStruct*)ctrl_struct;
	/// запуск обработчика по интервалу
	if ((time_us - dep_ptr->last_call_time_us) > (dep_ptr->interval_ms*1000)) {
		dep_ptr->last_call_time_us = time_us;
		// user code begin
		
		// user code end
		retval = 1;
	}
	// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
	if (interface->event[dep_ptr->self_num] & DEP_EVENT_MEAS_INTERVAL_START) 
	{
		interface->event[dep_ptr->self_num] &= ~DEP_EVENT_MEAS_INTERVAL_START;
		cyclo_start(&dep_ptr->meas_cyclo);
		retval = 1;
	}
	else {
		//
	}
	//обработка циклограммы
	cyclo_handler(&dep_ptr->meas_cyclo, time_us/1000);
	//обработка передачи данных
	if(dep_ptr->frame_data_ready)
	{
		dep_ptr->frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&dep_ptr->frame, sizeof(typeDEPFrameUnion));
		interface->event[dep_ptr->self_num] |= DEP_EVENT_MEAS_INTERVAL_DATA_READY;
	}
	return retval;
}

/**
  * @brief  формирование кадра ДЭП c выставлением флага
	* @param  dep_ptr указатель на структуру управления
	* @retval  статус: 0 - кадр не сформирован, 1 - кадр сформирован
  */
int8_t dep_frame_forming(typeDEPStruct* dep_ptr)
{
    uint8_t i = 0;
		typeDEPAcqValue rec_tmp;
		//
		if (dep_ptr->rec_num >= 6){
			dep_ptr->frame.row.label = 0x0FF1;
			dep_ptr->frame.row.definer = frame_definer(0, dep_ptr->device_number, NULL, dep_ptr->frame_type);
			dep_ptr->frame.row.num = (*dep_ptr->global_frame_num_ptr++)&0xFFFF;
			dep_ptr->frame.row.time = Get_Time_s();
			//
			for (i=0; i<6; i++){
				dep_read_fifo(dep_ptr, &rec_tmp);
				memcpy((uint8_t*)&dep_ptr->frame.dep.meas[i], (uint8_t*)&rec_tmp, sizeof(typeDEPAcqValue));
			}
			//
			dep_ptr->frame.dep.change_int_num = 0xFEFE;
			dep_ptr->frame.dep.changed_meas_interv = 0xFEFE;
			//
			dep_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&dep_ptr->frame.row, sizeof(typeFrameStruct) - 2);
			//
			dep_ptr->frame_data_ready = 1;
			return 1;
		}
		else{
			return 0;
		}
}

/**
  * @brief  включение отключение режима константа (широковещательная)
	* @param  dep_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void dep_constant_mode(typeDEPStruct* dep_ptr, uint32_t on_off)
{
	uint16_t data[2];
	dep_ptr->const_mode = (on_off) ? 0x55AA : 0x5500;
	data[0] = __REV16(0x00);
	data[1] = __REV16(dep_ptr->const_mode); 
	ib_run_transaction(dep_ptr->ib, 0xFF, 106, 0, 2, data);
}

/**
  * @brief  запуск ДЭП командой регистра RUN (4096)
	* @param  dep_ptr указатель на структуру управления
	* @param  time_us время МК us
	* @param  meas_num W - нужное число измерений, R - оставшееся число измерений (+0)
  */
void dep_start(typeDEPStruct *dep_ptr, uint16_t meas_num) 
{
	uint16_t data[6];
	data[0] = __REV16(meas_num);
	//data[1] = __REV16(0x0000);
	//data[2] = __REV16(0x0000);
	//data[3] = __REV16(0x0000);
	//data[4] = __REV16(0x0000);
	//data[5] = __REV16(0x0000);
	ib_run_transaction(dep_ptr->ib, dep_ptr->id, MB_F_CODE_16, 0, 1, data);
}

/**
  * @brief  остановка ДЭП командой регистра RUN (4096)
	* @param  dep_ptr указатель на структуру управления
  */
void dep_stop(typeDEPStruct *dep_ptr)
{
	uint16_t data[2];
	data[0] = __REV16(0x0000);
	ib_run_transaction(dep_ptr->ib, dep_ptr->id, MB_F_CODE_16, 0, 1, data);
}

/**
  * @brief  чтение данных
	* @param  dep_ptr указатель на структуру управления
  */
void dep_read_data(typeDEPStruct *dep_ptr)
{
	uint8_t in_data[64] = {0};
	uint8_t i=0;
	uint16_t received_dep_measurments;
	//
	if (ib_run_transaction(dep_ptr->ib, dep_ptr->id, MB_F_CODE_3, 1000, 2+4*3, NULL) > 0) { // запрашиваем 3 измерения, что бы почистить буфер
		ib_get_answer_data(dep_ptr->ib, in_data, 2*(2+4*3));
		received_dep_measurments = __REV16(*(uint16_t*)&in_data[2]);
		//
		if (received_dep_measurments != 0) {
			if (dep_write_fifo(dep_ptr, (typeDEPAcqValue*)&in_data[4+i*sizeof(typeDEPAcqValue)]) > 0) {
				
			}
			else{
				//обработка ошибки перезаполениня буфера
			}
		}
	}
	else {
		//todo: возможно необходимо сделать обработку непринятия пакета с данными
	}
}

/**
  * @brief  запись измерения в fifo
	* @param  dep_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус записи: 1 - ок, <0 - ошибка
  */
int8_t dep_write_fifo(typeDEPStruct *dep_ptr, typeDEPAcqValue* data)
{
	if (dep_ptr->rec_num >= DEP_REC_FIFO_DEPTH){
		return -1;
	}
	else{
		memcpy((uint8_t*)&dep_ptr->rec_fifo[dep_ptr->rec_num], (uint8_t*)data, sizeof(typeDEPAcqValue));
		dep_ptr->rec_num += 1;
		if (dep_ptr->rec_num > dep_ptr->rec_max) dep_ptr->rec_max = dep_ptr->rec_num;  //сбор информации о заполненности fifo
		return 1;
	}
}

/**
  * @brief  взятие измерения из fifo
	* @param  dep_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус: 0 - fifo-пуст, 1 - ок
  */
int8_t dep_read_fifo(typeDEPStruct* dep_ptr, typeDEPAcqValue* data)
{
	if (dep_ptr->rec_num == 0){
		return 0;
	}
	else{
		dep_ptr->rec_num -= 1;
		memcpy((uint8_t*)data, (uint8_t*)&dep_ptr->rec_fifo[0], sizeof(typeDEPAcqValue));
		memmove((uint8_t*)&dep_ptr->rec_fifo[0], (uint8_t*)&dep_ptr->rec_fifo[1], sizeof(typeDEPAcqValue)*dep_ptr->rec_num);
		memset((uint8_t*)&dep_ptr->rec_fifo[dep_ptr->rec_num], 0x00, sizeof(typeDEPAcqValue)*(DEP_REC_FIFO_DEPTH - dep_ptr->rec_num));
		return 1;
	}
}

/**
 * @brief переварачивание байт внутр 16-ти битных переменных
 * 
 * 
 * @param dep_rec структура, принимаемая из модуля ДЭП по ВШ
 */
void _dep_rec_rev(typeDEPAcqValue* dep_rec)
{  
    uint16_t data[32];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)dep_rec, sizeof(typeDEPAcqValue));
    for (i=0; i<(sizeof(typeDEPAcqValue)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)dep_rec, (uint8_t *)data, sizeof(typeDEPAcqValue));
}

/**
  * @brief  инициализация циклограмм работы с ДЭП
	* @param  dep_ptr указатель на структуру управления
  */
void dep_meas_cycl_init(typeDEPStruct* dep_ptr)
{
	// циклограмма инициализации МПП
	cyclo_init(&dep_ptr->meas_cyclo);
	//
	cyclo_add_step(&dep_ptr->meas_cyclo, dep_meas_cycl_struct_start, (void*)dep_ptr, 1500);
	cyclo_add_step(&dep_ptr->meas_cyclo, dep_meas_cycl_read_data, (void*)dep_ptr, 0);
	cyclo_add_step(&dep_ptr->meas_cyclo, dep_meas_cycl_frame_forming, (void*)dep_ptr, 0);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void dep_meas_cycl_struct_start(void* ctrl_struct)
{
	typeDEPStruct* dep_ptr = (typeDEPStruct*) ctrl_struct;
	dep_start(dep_ptr, 3);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void dep_meas_cycl_struct_stop(void* ctrl_struct)
{
	typeDEPStruct* dep_ptr = (typeDEPStruct*) ctrl_struct;
	dep_stop(dep_ptr);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void dep_meas_cycl_read_data(void* ctrl_struct)
{
	typeDEPStruct* dep_ptr = (typeDEPStruct*) ctrl_struct;
	dep_read_data(dep_ptr);
}

/**
  * @brief  формирование кадра ДЭП
	* @param  ctrl_struct указатель на структуру управления
  */
void dep_meas_cycl_frame_forming(void* ctrl_struct)
{
	typeDEPStruct* dep_ptr = (typeDEPStruct*) ctrl_struct;
	dep_frame_forming(dep_ptr);
}
