  /**
  ******************************************************************************
  * @file           : dir.c
  * @version        : v1.0
  * @brief          : библиотека для управления модулем DIR, использует объект ВШ (internal_bus)
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date						: 2021.10.01
  ******************************************************************************
  */

#include "dir.h"

/**
  * @brief  инициализация структуры управления
	* @param  dir_ptr указатель на структуру управления
	* @param  id номер устройства на внутренней шине
	* @param  device_number номер устройства в котором работает модуль
	* @param  frame_type тип кадра
	* @param  ib_ptr указатель на внутреннюю шину
	* @param  gl_fr_num указатель на сковозной глобальный номер кадра
  */
void dir_init(typeDIRStruct* dir_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type, typeIBStruct* ib_ptr, uint32_t* gl_fr_num)
{
  dir_reset_parameters(dir_ptr);
	dir_ptr->id = id;
	dir_ptr->device_number = device_number;
	dir_ptr->frame_type = frame_type;
	dir_ptr->ib = ib_ptr;
	dir_ptr->self_num = self_num;
	dir_ptr->global_frame_num_ptr = gl_fr_num;
	// работы с циклограммами
	dir_meas_cycl_init(dir_ptr);
}

/**
  * @brief  инициализация структуры управления
	* @param  dir_ptr указатель на структуру управления
  */
void dir_reset_parameters(typeDIRStruct* dir_ptr)
{
	//зануление данных на передачу в МКО и память
	memset((uint8_t*)&dir_ptr->frame, 0xFE, sizeof(dir_ptr->frame));
	dir_ptr->frame_data_ready = 0x00;
	//
	memset((uint8_t*)dir_ptr->rec_fifo, 0xFE, sizeof(dir_ptr->rec_fifo));
	dir_ptr->rec_num = 0;
	dir_ptr->rec_max = 0;
	//
	dir_ptr->last_call_time_us = 0;
	dir_ptr->meas_event_num = 0;
	dir_ptr->interval_ms = DIR_DEFAULT_INTERVAL_MS;
	dir_ptr->const_mode = 0;
}

/**
  * @brief  процесс обработки состояния ДИР
	* @param  dir_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t dir_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval = 0;
  typeDIRStruct* dir_ptr = (typeDIRStruct*)ctrl_struct;
	/// запуск обработчика по интервалу
	if ((time_us - dir_ptr->last_call_time_us) > (dir_ptr->interval_ms*1000)) {
		dir_ptr->last_call_time_us = time_us;
		// user code begin
		
		// user code end
		retval = 1;
	}
	// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
	if (interface->event[dir_ptr->self_num] & DIR_EVENT_MEAS_INTERVAL_START) 
	{
		interface->event[dir_ptr->self_num] &= ~DIR_EVENT_MEAS_INTERVAL_START;
		cyclo_start(&dir_ptr->meas_cyclo);
		retval = 1;
	}
	else {
		//
	}
	//обработка циклограммы
	cyclo_handler(&dir_ptr->meas_cyclo, time_us/1000);
	//обработка передачи данных
	if(dir_ptr->frame_data_ready)
	{
		dir_ptr->frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&dir_ptr->frame, sizeof(typeDIRFrameUnion));
		interface->event[dir_ptr->self_num] |= DIR_EVENT_MEAS_INTERVAL_START;
	}
	return retval;
}

/**
  * @brief  формирование кадра ДИР c выставлением флага
	* @param  dir_ptr указатель на структуру управления
	* @retval  статус: 0 - кадр не сформирован, 1 - кадр сформирован
  */
int8_t dir_frame_forming(typeDIRStruct* dir_ptr)
{
    uint8_t i;
		typeDIRMeas rec_tmp;
		//
		if (dir_ptr->rec_num >= 2){
			dir_ptr->frame.row.label = 0x0FF1;
			dir_ptr->frame.row.definer = frame_definer(0, dir_ptr->device_number, NULL, dir_ptr->frame_type);
			dir_ptr->frame.row.num = ((*dir_ptr->global_frame_num_ptr)++)&0xFFFF;
			dir_ptr->frame.row.time = Get_Time_s();
			//
			for (i=0; i<2; i++){
				dir_read_fifo(dir_ptr, &rec_tmp);
				memcpy((uint8_t*)&dir_ptr->frame.dir.meas[i], (uint8_t*)&rec_tmp, sizeof(typeDIRMeas));
			}
			//
			dir_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&dir_ptr->frame.row, sizeof(typeFrameStruct) - 2);
			//
			dir_ptr->frame_data_ready = 1;
			return 1;
		}
		else{
			return 0;
		}
}

/**
  * @brief  включение отключение режима константа (широковещательная)
	* @param  dir_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void dir_constant_mode(typeDIRStruct* dir_ptr, uint32_t on_off)
{
	uint16_t data[2];
	dir_ptr->const_mode = (on_off) ? 0x55AA : 0x5500;
	data[0] = __REV16(0x00);
	data[1] = __REV16(dir_ptr->const_mode); 
	ib_run_transaction(dir_ptr->ib, 0xFF, 106, 0, 2, data);
}

/**
  * @brief  чтение данных
	* @param  dir_ptr указатель на структуру управления
  */
void dir_read_data(typeDIRStruct *dir_ptr)
{
	uint8_t in_data[32] = {0};
	//
	if (ib_run_transaction(dir_ptr->ib, dir_ptr->id, MB_F_CODE_3, 2000, 10, NULL) > 0) {
		ib_get_answer_data(dir_ptr->ib, in_data, 2*(10));
		//
		if (dir_write_fifo(dir_ptr, (typeDIRMeas*)&in_data[0]) > 0) {
			
		}
		else{
			//обработка ошибки перезаполениня буфера
		}
	}
	else {
		//todo: возможно необходимо сделать обработку непринятия пакета с данными
	}
}

/**
  * @brief  запись измерения в fifo
	* @param  dir_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус записи: 1 - ок, <0 - ошибка
  */
int8_t dir_write_fifo(typeDIRStruct *dir_ptr, typeDIRMeas* data)
{
	if (dir_ptr->rec_num >= DIR_REC_FIFO_DEPTH){
		return -1;
	}
	else{
		memcpy((uint8_t*)&dir_ptr->rec_fifo[dir_ptr->rec_num], (uint8_t*)data, sizeof(typeDIRMeas));
		dir_ptr->rec_num += 1;
		if (dir_ptr->rec_num > dir_ptr->rec_max) dir_ptr->rec_max = dir_ptr->rec_num;  //сбор информации о заполненности fifo
		return 1;
	}
}

/**
  * @brief  взятие измерения из fifo
	* @param  dir_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус: 0 - fifo-пуст, 1 - ок
  */
int8_t dir_read_fifo(typeDIRStruct* dir_ptr, typeDIRMeas* data)
{
	if (dir_ptr->rec_num == 0){
		return 0;
	}
	else{
		dir_ptr->rec_num -= 1;
		memcpy((uint8_t*)data, (uint8_t*)&dir_ptr->rec_fifo[0], sizeof(typeDIRMeas));
		memmove((uint8_t*)&dir_ptr->rec_fifo[0], (uint8_t*)&dir_ptr->rec_fifo[1], sizeof(typeDIRMeas)*dir_ptr->rec_num);
		memset((uint8_t*)&dir_ptr->rec_fifo[dir_ptr->rec_num], 0x00, sizeof(typeDIRMeas)*(DIR_REC_FIFO_DEPTH - dir_ptr->rec_num));
		return 1;
	}
}

/**
 * @brief переварачивание байт внутр 16-ти битных переменных
 * 
 * 
 * @param dir_rec структура, принимаемая из модуля ДЭП по ВШ
 */
void _dir_rec_rev(typeDIRMeas* dir_rec)
{  
    uint16_t data[32];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)dir_rec, sizeof(typeDIRMeas));
    for (i=0; i<(sizeof(typeDIRMeas)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)dir_rec, (uint8_t *)data, sizeof(typeDIRMeas));
}

/**
  * @brief  инициализация циклограмм работы с ДЭП
	* @param  dir_ptr указатель на структуру управления
  */
void dir_meas_cycl_init(typeDIRStruct* dir_ptr)
{
	// циклограмма инициализации МПП
	cyclo_init(&dir_ptr->meas_cyclo);
	//
	cyclo_add_step(&dir_ptr->meas_cyclo, dir_meas_cycl_read_data, (void*)dir_ptr, 100);
	cyclo_add_step(&dir_ptr->meas_cyclo, dir_meas_cycl_frame_forming, (void*)dir_ptr, 0);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void dir_meas_cycl_read_data(void* ctrl_struct)
{
	typeDIRStruct* dir_ptr = (typeDIRStruct*) ctrl_struct;
	dir_read_data(dir_ptr);
}

/**
  * @brief  формирование кадра ДЭП
	* @param  ctrl_struct указатель на структуру управления
  */
void dir_meas_cycl_frame_forming(void* ctrl_struct)
{
	typeDIRStruct* dir_ptr = (typeDIRStruct*) ctrl_struct;
	dir_frame_forming(dir_ptr);
}
