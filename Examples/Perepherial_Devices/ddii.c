/**
  ******************************************************************************
  * @file           : ddii.c
  * @version        : v1.0
  * @brief          : библиотека для управления модулем ДДИИ, использует объект MKO в режиме контроллера канала
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date			: 2022.02.23
  ******************************************************************************
  */

#include "ddii.h"

/**
	* @brief  инициализация структуры управления
	* @param  ddii_ptr указатель на структуру управления
	* @param  mko_addr адресс устройства ДДИИ
	* @param  device_number номер устройства в котором работает ДДИИ
	* @param  frame_type тип кадра для ДДИИ
	* @param  mko_bc_ptr указатель на структуру управления МКО для ДДИИ
	* @param  gl_fr_num указатель на сковозной глобальный номер кадра
  */
void ddii_init(typeDDIIStruct* ddii_ptr, uint8_t self_num, uint8_t mko_addr, uint16_t device_number, uint16_t frame_type, typeMKOStruct* mko_bc_ptr, uint8_t mko_bus, uint32_t* gl_fr_num)
{
	//
	ddii_reset_parameters(ddii_ptr);
	ddii_ptr->mko_addr = mko_addr;
	ddii_ptr->mko_bus = mko_bus;
	ddii_ptr->device_number = device_number;
	ddii_ptr->frame_type = frame_type;
	ddii_ptr->mko_bc_ptr = mko_bc_ptr;
	ddii_ptr->self_num = self_num;
	ddii_ptr->global_frame_num_ptr = gl_fr_num;
	// включение ДДИИ
	//...
	// работы с циклограммами
	ddii_meas_cycl_init(ddii_ptr);
}

/**
	* @brief  инициализация структуры управления
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_reset_parameters(typeDDIIStruct* ddii_ptr)
{
	// обнуляем кадр
	memset((uint8_t*)&ddii_ptr->frame, 0xFE, sizeof(ddii_ptr->frame));
	//
	ddii_ptr->frame_data_ready = 0x00;
	//
	memset((uint8_t*)ddii_ptr->rec_fifo, 0xFE, sizeof(ddii_ptr->rec_fifo));
	ddii_ptr->rec_num = 0;
	ddii_ptr->rec_max = 0;
	//
	ddii_ptr->last_call_time_us = 0;
	ddii_ptr->meas_event_num = 0;
	ddii_ptr->interval_ms = DDII_DEFAULT_INTERVAL_MS;
	ddii_ptr->const_mode = 0;
}

/**
	* @brief  процесс обработки состояния
	* @param  ddii_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t ddii_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval = 0;
  typeDDIIStruct* ddii_ptr = (typeDDIIStruct*)ctrl_struct;
	/// запуск обработчика по интервалу
	if ((time_us - ddii_ptr->last_call_time_us) > (ddii_ptr->interval_ms*1000)) {
		ddii_ptr->last_call_time_us = time_us;
		// user code begin
		
		// user code end
		retval = 1;
	}
	// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
	if (interface->event[ddii_ptr->self_num] & DDII_EVENT_MEAS_INTERVAL_START) 
	{
		interface->event[ddii_ptr->self_num] &= ~DDII_EVENT_MEAS_INTERVAL_START;
		cyclo_start(&ddii_ptr->meas_cyclo);
		retval = 1;
	}
	else {
		//
	}
	//обработка циклограммы
	cyclo_handler(&ddii_ptr->meas_cyclo, time_us/1000);
	//обработка передачи данных
	if(ddii_ptr->frame_data_ready)
	{
		ddii_ptr->frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&ddii_ptr->frame, sizeof(typeDDIIFrameUnion));
		interface->event[ddii_ptr->self_num] |= DDII_EVENT_MEAS_INTERVAL_DATA_READY;
	}
	return retval;
}

/**
  * @brief  формирование кадра ДДИИ c выставлением флага
	* @param  ddii_ptr указатель на структуру управления
	* @retval  статус: 0 - кадр не сформирован, 1 - кадр сформирован
  */
int8_t ddii_frame_forming(typeDDIIStruct* ddii_ptr)
{
    uint8_t i = 0;
		typeDDIIRec rec_tmp;
		//
		if (ddii_ptr->rec_num >= DDII_MEAS_NUMBER){
			ddii_ptr->frame.row.label = 0x0FF1;
			ddii_ptr->frame.row.definer = frame_definer(0, ddii_ptr->device_number, NULL, ddii_ptr->frame_type);
			ddii_ptr->frame.row.num = (*ddii_ptr->global_frame_num_ptr++)&0xFFFF;
			ddii_ptr->frame.row.time = Get_Time_s();
			//
			for (i=0; i<DDII_MEAS_NUMBER; i++){
				ddii_read_fifo(ddii_ptr, &rec_tmp);
				memcpy((uint8_t*)&ddii_ptr->frame.ddii.meas, (uint8_t*)&rec_tmp, sizeof(typeDDIIRec));
			}
			//
			ddii_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&ddii_ptr->frame.row, sizeof(typeFrameStruct) - 2);
			//
			ddii_ptr->frame_data_ready = 1;
			return 1;
		}
		else{
			return 0;
		}
}

/**
  * @brief  включение режима констант (не используется для ДДИИ)
	* @param  ddii_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void ddii_constant_mode(typeDDIIStruct* ddii_ptr, uint32_t on_off)
{
	uint16_t ctrl_data[32] = {0};
	// формирование команды
	ctrl_data[0] = 0x0FF1;
	ctrl_data[5] = DDII_MK_COMMAND_CONSTANT_MODE;
	ctrl_data[6] = (on_off & 0x01) ? 0x0001 : 0x0000;
	//....
	mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, MKO_MODE_WRITE, ddii_ptr->mko_addr, DDII_MKO_SA_CTRL, ddii_ptr->mko_bus, ctrl_data, 32);
}

/**
  * @brief  запуск ДДИИ
	* @param  ddii_ptr указатель на структуру управления
	* @param  ddii_rd_ptr установка указателя чтения архивной памяти
  */
void ddii_start(typeDDIIStruct* ddii_ptr, uint32_t ddii_rd_ptr)
{
	uint16_t ctrl_data[32] = {0};
	// формирование команды
	ctrl_data[0] = 0x0FF1;
	ctrl_data[5] = DDII_MK_COMMAND_START;
	ctrl_data[6] = ddii_rd_ptr & 0xFFFF;
	//....
	mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, MKO_MODE_WRITE, ddii_ptr->mko_addr, DDII_MKO_SA_CTRL, ddii_ptr->mko_bus, ctrl_data, 32);
}

/**
  * @brief  чтение измерения ДДИИ
	* @param  ddii_ptr указатель на структуру управления
	* @param  meas_num W - нужное число измерений, R - оставшееся число измерений (+0)
  */
void ddii_read_data_frame(typeDDIIStruct *ddii_ptr)
{
	mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, MKO_MODE_READ, ddii_ptr->mko_addr, DDII_MKO_SA_DATA_FRAME, ddii_ptr->mko_bus, (uint16_t*)&ddii_ptr->data_frame, 32);
}

/**
  * @brief  запись измерения в fifo
	* @param  ddii_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус записи: 1 - ок, <0 - ошибка
  */
int8_t ddii_write_fifo(typeDDIIStruct *ddii_ptr, typeDDIIRec* data)
{
	if (ddii_ptr->rec_num >= DDII_REC_FIFO_DEPTH){
		return -1;
	}
	else{
		memcpy((uint8_t*)&ddii_ptr->rec_fifo[ddii_ptr->rec_num], (uint8_t*)data, sizeof(typeDDIIRec));
		ddii_ptr->rec_num += 1;
		if (ddii_ptr->rec_num > ddii_ptr->rec_max) ddii_ptr->rec_max = ddii_ptr->rec_num;  //сбор информации о заполненности fifo
		return 1;
	}
}

/**
  * @brief  взятие измерения из fifo
	* @param  ddii_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус: 0 - fifo-пуст, 1 - ок
  */
int8_t ddii_read_fifo(typeDDIIStruct* ddii_ptr, typeDDIIRec* data)
{
	if (ddii_ptr->rec_num == 0){
		return 0;
	}
	else{
		ddii_ptr->rec_num -= 1;
		memcpy((uint8_t*)data, (uint8_t*)&ddii_ptr->rec_fifo[0], sizeof(typeDDIIRec));
		memmove((uint8_t*)&ddii_ptr->rec_fifo[0], (uint8_t*)&ddii_ptr->rec_fifo[1], sizeof(typeDDIIRec)*ddii_ptr->rec_num);
		memset((uint8_t*)&ddii_ptr->rec_fifo[ddii_ptr->rec_num], 0x00, sizeof(typeDDIIRec)*(DDII_REC_FIFO_DEPTH - ddii_ptr->rec_num));
		return 1;
	}
}

/**
 * @brief переварачивание байт внутр 16-ти битных переменных
 * 
 * 
 * @param ddii_rec структура, принимаемая из модуля ДЭП по ВШ
 */
void _ddii_rec_rev(typeDDIIRec* ddii_rec)
{
  //
}

/**
  * @brief  инициализация циклограмм работы с ДЭП
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_meas_cycl_init(typeDDIIStruct* ddii_ptr)
{
	// циклограмма инициализации МПП
	cyclo_init(&ddii_ptr->meas_cyclo);
	//
	cyclo_add_step(&ddii_ptr->meas_cyclo, ddii_meas_cycl_start, (void*)ddii_ptr, 200);
	cyclo_add_step(&ddii_ptr->meas_cyclo, ddii_meas_cycl_read, (void*)ddii_ptr, 100);
	cyclo_add_step(&ddii_ptr->meas_cyclo, ddii_meas_cycl_frame_forming, (void*)ddii_ptr, 0);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void ddii_meas_cycl_start(void* ctrl_struct)
{
	typeDDIIStruct* ddii_ptr = (typeDDIIStruct*) ctrl_struct;
	ddii_read_data_frame(ddii_ptr);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void ddii_meas_cycl_read(void* ctrl_struct)
{
	typeDDIIStruct* ddii_ptr = (typeDDIIStruct*) ctrl_struct;
	ddii_read_data_frame(ddii_ptr);
}

/**
  * @brief  формирование кадра ДДИИ
	* @param  ctrl_struct указатель на структуру управления
  */
void ddii_meas_cycl_frame_forming(void* ctrl_struct)
{
	typeDDIIStruct* ddii_ptr = (typeDDIIStruct*) ctrl_struct;
	ddii_frame_forming(ddii_ptr);
}
