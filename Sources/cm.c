/**
  ******************************************************************************
  * @file           : cm.c
  * @note			: содержит функции для обработки программной модели ЦМ.
  * 				 - Включает только то, чем управляет ЦМ напрямую: pwr_gpio, adc, stm.
  * 				 - Все что можно снести ниже по иерархии, необходимо оформлять отдельными модулями.
  * 				 - Форматы кадров хранятся в отдельном файле. Заполнение по командам в данном файле (т.к. сбор из различных источников).
  * 				 - Тип кадра совпадает с подадресом МКО, куда он выкладывается, как оперативные данные
  * @version        : v1.0
  * @brief          : общие системные функции
  * @author			: Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  * @date			: 2021.09.06
  ******************************************************************************
  */

#include "cm.h"
/**
  * @brief  инициализация программной модели ЦМ
  * @param  cm_ptr указатель на структуру управления
  */
void cm_init(typeCMModel* cm_ptr, uint8_t self_num, uint8_t id, uint8_t mko_addr_default, uint16_t device_number, char* ver_str, uint16_t frame_type)
{
	adc_init(&cm_ptr->adc, ADC0);
	pwr_init(&cm_ptr->pwr, &cm_ptr->adc, NULL, NULL);
	ib_init(&cm_ptr->ib);
	mko_init(&cm_ptr->mko_rt, MIL_STD_15531, MKO_RT, mko_addr_default);
	mko_init(&cm_ptr->mko_bc, MIL_STD_15532, MKO_BC, NULL);
	fr_mem_init(&cm_ptr->mem, FR_MEM_TYPE_WR_TO_RD_WITH_PROT_AREA);
	// Half-set gpio id
	gpio_init(&cm_ptr->half_set_num_io, HALF_SET_IO_PORT, HALF_SET_LINE);
	//
	cm_reset_parameters(cm_ptr);
	cm_ptr->id = id;
	cm_ptr->self_num = self_num;
	cm_ptr->half_set_num = gpio_get(&cm_ptr->half_set_num_io);
	cm_set_clear_status(cm_ptr, CM_STATUS_CFG_HALF_SET, cm_ptr->half_set_num & 0x01);
	stm_single_ch_const_set(&cm_ptr->stm, NKBE, ((cm_ptr->half_set_num + 1) & 0x01));
	cm_ptr->device_number = device_number;
	cm_ptr->sw_version = get_version_from_str(ver_str);
	cm_ptr->frame_type = frame_type;
	//
	printf("CM init:h-s<%d>\n", cm_ptr->half_set_num);
}

/**
  * @brief  функция сброса параметров работы на значения по умолчанию
  * @param  frame_modification указатель на структуру управления
  */
void cm_reset_parameters(typeCMModel* cm_ptr)
{
	uint8_t i=0;
	uint16_t interv_default_values[CM_INTERV_NUMBER] = DEFAULT_CM_INTERV_VALUES_S;
	uint16_t interv_start_time[CM_INTERV_NUMBER] = DEFAULT_CM_DEFAULT_START_TIME_S;
	//
	cm_ptr->last_call_time_us = 0;
	//
	for (i=0; i<CM_INTERV_NUMBER; i++){
		cm_ptr->ctrl.intervals[i] = interv_default_values[i];
		cm_ptr->ctrl.last_call_interval_times[i] = 0;
		cm_ptr->ctrl.first_call_status[i] = 0;
		cm_ptr->ctrl.first_call_time[i] = interv_start_time[i];
	}
	cm_ptr->ctrl.meas_event = 0x00;
	cm_ptr->ctrl.speedy_event = 0x00;
	//
	cm_ptr->ctrl.speedy_mode_state = 0x00;
	cm_ptr->ctrl.speedy_mode_mask = CM_DEVICE_SPEEDY_MODE_MASK;
	cm_ptr->ctrl.speedy_mode_timeout = 0x00;
	cm_ptr->ctrl.frame_end_to_end_number = 0x00;
	cm_ptr->ctrl.rst_cnter = 0x00;
	fr_mem_set_rd_ptr(&cm_ptr->mem, 0);
	fr_mem_set_wr_ptr(&cm_ptr->mem, 0);
	//
	cm_ptr->ctrl.sync_num = 0;
	cm_ptr->ctrl.sync_time_s = 0;
	cm_ptr->ctrl.diff_time = 0;
	cm_ptr->ctrl.diff_time_fractional = 0;
	//
	cm_set_clear_status(cm_ptr, CM_STATUS_WORK, 1);
	//
	cm_ptr->ctrl.operation_time = cm_ptr->ctrl.operation_time;
	//
	cm_ptr->sw_version = 0x0000;
	// инициализация структуры управления кадрами
	cm_ptr->frames_fifo_num = 0x00;
	cm_ptr->frames_fifo_num_max = 0x00;
	cm_ptr->fifo_error_cnt = 0x00;
	memset((uint8_t*)cm_ptr->frames_fifo, 0x00, sizeof(cm_ptr->frames_fifo));
	cm_ptr->global_frame_num = 0x00;
	cm_ptr->const_mode = 0;
	//
	cm_ptr->id = 0;
	cm_ptr->self_num = 0;
	cm_ptr->half_set_num = 0; 
	cm_ptr->device_number = 0;
	cm_ptr->frame_type = 0;
	memset((uint8_t*)&cm_ptr->frame, 0x00, sizeof(typeSysFrameUnion));
}

/**
  * @brief  функция загрузки параметров из памяти
  * @param  cm_ptr указатель на структуру управления
  */
uint8_t cm_load_cfg(typeCMModel* cm_ptr)
{
	typeCfgFrameUnion cfg;
	if (fr_mem_param_load(&cm_ptr->mem, (uint8_t*)&cfg) < 0){
		cm_set_clear_status(cm_ptr, CM_STATUS_CFG_LOADED, 0);
		return 0;
	}
	else{
		memcpy((uint8_t*)&cm_ptr->loaded_cfg, (uint8_t*)&cfg, sizeof(typeCfgFrameUnion));
		cm_set_cfg(cm_ptr);
		cm_set_clear_status(cm_ptr, CM_STATUS_CFG_LOADED, 1);
		return 1;
	}
}

/**
  * @brief  функция сохранения параметров ЦМ в память
  * @param  cm_ptr указатель на структуру управления
  */
void cm_save_cfg(typeCMModel* cm_ptr)
{
	cm_get_cfg(cm_ptr);
	fr_mem_param_save(&cm_ptr->mem, (uint8_t*)&cm_ptr->current_cfg);
}

/**
  * @brief  функция установки параметров в ЦМ
  * @param  cm_ptr указатель на структуру управления
  */
void cm_set_cfg(typeCMModel* cm_ptr)
{
	// питание
	// pwr_set_state(&cm_ptr->pwr, cm_ptr->loaded_cfg.cfg.body.power_state);		
	// cm_ptr->loaded_cfg.cfg.body.power_state;		
	//
	cm_ptr->ctrl.rst_cnter = cm_ptr->loaded_cfg.cfg.body.rst_cnter + 1;  		
	//cm_ptr->loaded_cfg.cfg.body.gup;				
	//
	cm_ptr->ctrl.operation_time = _rev_u32_by_u16(cm_ptr->loaded_cfg.cfg.body.operation_time);
	//
	fr_mem_set_rd_ptr(&cm_ptr->mem, cm_ptr->loaded_cfg.cfg.body.read_ptr);
	fr_mem_set_wr_ptr(&cm_ptr->mem, cm_ptr->loaded_cfg.cfg.body.write_ptr);
}

/**
  * @brief  функция получения параметров ЦМ из рабочих регистров в cfg
  * @param  cm_ptr указатель на структуру управления
  */
void cm_get_cfg(typeCMModel* cm_ptr)
{
	cm_ptr->current_cfg.row.label = 0x0FF1;
	cm_ptr->current_cfg.row.definer = frame_definer(0, cm_ptr->device_number, NULL, CM_CFG_FRAME_TYPE);
	cm_ptr->current_cfg.row.num = 0xFEFE; //не используется для данного типа кадров
	cm_ptr->current_cfg.row.time = _rev_u32_by_u16(Get_Time_s());
	//
	cm_ptr->current_cfg.cfg.body.power_status = cm_ptr->pwr.status;
	cm_ptr->current_cfg.cfg.body.power_state = cm_ptr->pwr.state;
	//
	cm_ptr->current_cfg.cfg.body.rst_cnter = cm_ptr->ctrl.rst_cnter;
	cm_ptr->current_cfg.cfg.body.gup = 0xFE;
	//
	cm_ptr->current_cfg.cfg.body.operation_time = _rev_u32_by_u16(cm_ptr->ctrl.operation_time);
	//
	cm_ptr->current_cfg.cfg.body.write_ptr = cm_ptr->mem.write_ptr;
	cm_ptr->current_cfg.cfg.body.read_ptr = cm_ptr->mem.read_ptr;
	//
	memset((uint8_t*)&cm_ptr->current_cfg.cfg.body.reserve, 0xFE, sizeof(cm_ptr->loaded_cfg.cfg.body.reserve)); 
	//
	cm_ptr->current_cfg.row.crc16 = frame_crc16((uint8_t*)&cm_ptr->current_cfg.row, sizeof(typeFrameStruct) - 2);
}

/**
  * @brief  процесс обработки состояния ЦМ
	* @param  cm_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t cm_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	int8_t device = 0;
	uint8_t retval=0;
	typeCMModel* cm_ptr = (typeCMModel*)ctrl_struct;
	//
	if ((time_us - cm_ptr->last_call_time_us) > (CM_HANDLER_INTERVAL_MS*1000)) {
		cm_ptr->call_interval_us = time_us - cm_ptr->last_call_time_us;
		cm_ptr->last_call_time_us = time_us;
		// user code begin
		// управление ускоренным режимом
		if ((cm_ptr->ctrl.speedy_mode_state) && (cm_ptr->ctrl.speedy_mode_timeout > 0)){
			if  (cm_ptr->ctrl.speedy_mode_timeout >= cm_ptr->call_interval_us){
				cm_ptr->ctrl.speedy_mode_timeout -= cm_ptr->call_interval_us;
			}
			else{
				cm_ptr->ctrl.speedy_mode_timeout = 0;
			}
		}
		else{
			cm_ptr->ctrl.speedy_mode_state = 0;
			cm_ptr->ctrl.speedy_mode_timeout = 0; 
		}
		//stm
		stm_process(&cm_ptr->stm, cm_ptr->call_interval_us/1000);
		// user code end
		retval = 1;
	}
	// обработка интервалов работы БЭ
	if (cm_interval_processor(cm_ptr, CM_INTERV_SYS, time_us)) {
		// user code begin
		interface->event[CM] |= (CM_EVENT_SYS_INTERVAL_START);
		// user code end
		retval = 1;
	}
	//
	if (cm_interval_processor(cm_ptr, CM_INTERV_MEAS, time_us)) {
		// user code begin
		cm_ptr->ctrl.meas_event = 1; //! обязательно перепроверить сброс
		// user code end
		retval = 1;
	}
	//
	if (cm_interval_processor(cm_ptr, CM_INTERVAL_DBG, time_us)) {
		// user code begin
		ib_run_transaction(&cm_ptr->ib, MB_DEV_ID_NU, MB_F_CODE_16, 0, 32, (uint16_t*)&cm_ptr->frame);
		// user code end
		retval = 1;
	}
	//
	if (cm_interval_processor(cm_ptr, CM_INTERVAL_SPEED, time_us)) {
		// user code begin
		cm_ptr->ctrl.speedy_event = 1; //! обязательно перепроверить сброс
		// user code end
		retval = 1;
	}
	//
		if (cm_interval_processor(cm_ptr, CM_1S_INTERVAL, time_us)) {
		// user code begin
		cm_ptr->ctrl.operation_time += 1;
		cm_save_cfg(cm_ptr);
		// user code end
		retval = 1;
	}
	// Обработка запуска измерения периферии с использованием либо стандартного интервала измерения или ускоренного
	if (cm_ptr->ctrl.speedy_mode_state){
		if (cm_ptr->ctrl.speedy_event){
			for (device=0; device<DEV_NUM; device++){
				if ((1<<device) & CM_DEVICE_MEAS_MODE_MASK){
					if (cm_ptr->ctrl.speedy_mode_state & (1<<device)){
						interface->event[device] |= CM_EVENT_MEAS_INTERVAL_START;
					}
				}
			}
			cm_ptr->ctrl.speedy_event = 0;
		}
		else if (cm_ptr->ctrl.meas_event){
			for (device=0; device<DEV_NUM; device++){
				if ((1<<device) & CM_DEVICE_MEAS_MODE_MASK){
					if ((cm_ptr->ctrl.speedy_mode_state & (1<<device)) == 0){
						interface->event[device] |= CM_EVENT_MEAS_INTERVAL_START;
					}
				}
			}
			cm_ptr->ctrl.meas_event = 0;
		}
		
	}
	else{
		if (cm_ptr->ctrl.meas_event){
			for (device=0; device<DEV_NUM; device++){
				if ((1<<device) & CM_DEVICE_MEAS_MODE_MASK){
					interface->event[device] |= CM_EVENT_MEAS_INTERVAL_START;
				}
			}
			cm_ptr->ctrl.meas_event = 0;
		}
		cm_ptr->ctrl.speedy_event = 0;
	}		
	//** обработка приходящих событий **//

	// self events (device is CM)
	if(interface->event[CM] & CM_EVENT_SYS_INTERVAL_START){
		interface->event[CM] &= ~CM_EVENT_SYS_INTERVAL_START;
		//
		cm_frame_forming(cm_ptr);
		cm_frame_receive(cm_ptr, (uint8_t*)&cm_ptr->frame);
	}
	// peripheral events
	for(device = CM+1; device < DEV_NUM; device++){
		if(interface->event[device] & CM_EVENT_MEAS_INTERVAL_DATA_READY){
			interface->event[device] &= ~CM_EVENT_MEAS_INTERVAL_DATA_READY;
			cm_frame_receive(cm_ptr, (uint8_t*)&interface->shared_mem[64*device]);
		}
		switch(device){
			case PER_TMPLT:
			//
			break;
		}
	}
	// обработка собранных кадров
	cm_frame_handling(cm_ptr);
	// обработка командных сообщений МКО
	cm_mko_command_interface_handler(cm_ptr);
	// обработка отладочных сообщений по ВШ
	cm_dbg_ib_command_handler(cm_ptr);
	return retval;
}

/**
  * @brief  процесс обработки полученных кадров
	* @param  cm_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
  */
int8_t cm_frame_receive(typeCMModel* cm_ptr, uint8_t* data)
{
	typeFrameStruct frame;
	memcpy((uint8_t*)&frame, data, sizeof(frame));
	//проверка на корректность данных 
	if (frame.label != 0x0FF1){
		return -1;
	}
	else if (frame_crc16(data, sizeof(frame)) != 0){
		return -2;
	}
	else{
		frame.num = cm_ptr->ctrl.frame_end_to_end_number & 0xFFFF;
		cm_ptr->ctrl.frame_end_to_end_number += 1;
		frame.crc16 = frame_crc16((uint8_t*)&frame, sizeof(typeFrameStruct)-2);
		//
		if (cm_write_fifo(cm_ptr, &frame) < 0){
			cm_ptr->fifo_error_cnt += 1;
		}
		//
		return 1;
	}
}

/**
  * @brief  обработка кадров, лежащих в FIFO
	* @param  cm_ptr указатель на структуру управления
  */
void cm_frame_handling(typeCMModel* cm_ptr)
{
	typeFrameStruct frame;
	//
	if (cm_read_fifo(cm_ptr, &frame)){
		// mko
		mko_rt_write_to_subaddr(&cm_ptr->mko_rt, frame_get_type_from_definer(frame), (uint16_t*)&frame);
		// archive memory
		fr_mem_write_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
		//
	}
}

/**
  * @brief  запись кадра в fifo
	* @param  cm_ptr указатель на структуру управления
	* @param  frame указатель на массив с кадром данных
	* @retval  статус записи: 1 - ок, <0 - ошибка
  */
int8_t cm_write_fifo(typeCMModel* cm_ptr, typeFrameStruct* frame)
{
	if (cm_ptr->frames_fifo_num >= CM_FRAMES_FIFO_DEPTH){
		return -1;
	}
	else{
		memcpy((uint8_t*)&cm_ptr->frames_fifo[cm_ptr->frames_fifo_num], (uint8_t*)frame, sizeof(typeFrameStruct));
		cm_ptr->frames_fifo_num += 1;
		if (cm_ptr->frames_fifo_num > cm_ptr->frames_fifo_num_max) cm_ptr->frames_fifo_num_max = cm_ptr->frames_fifo_num;  //сбор информации о заполненности fifo
		return 1;
	}
}

/**
  * @brief  взятие кадра из fifo
	* @param  cm_ptr указатель на структуру управления
	* @param  frame указатель на массив с кадром данных
	* @retval  статус: 0 - fifo-пуст, 1 - ок
  */
int8_t cm_read_fifo(typeCMModel* cm_ptr, typeFrameStruct* frame)
{
	if (cm_ptr->frames_fifo_num == 0){
		return 0;
	}
	else{
		cm_ptr->frames_fifo_num -= 1;
		memcpy((uint8_t*)frame, (uint8_t*)&cm_ptr->frames_fifo[0], sizeof(typeFrameStruct));
		memmove((uint8_t*)&cm_ptr->frames_fifo[0], (uint8_t*)&cm_ptr->frames_fifo[1], sizeof(typeFrameStruct)*cm_ptr->frames_fifo_num);
		memset((uint8_t*)&cm_ptr->frames_fifo[cm_ptr->frames_fifo_num], 0x00, sizeof(typeFrameStruct)*(CM_FRAMES_FIFO_DEPTH - cm_ptr->frames_fifo_num));
		return 1;
	}
}

/**
 * @brief обработчик достижения отсчета интервалов
 * 
 * @param cm_ptr указатель на переменную управления ЦМ
 * @param interval_id уникальный номер обрабатываемого интервала
 * @param time_us текущее время в мкс
 * @return int8_t: 1 - интервал отсчитался, 0 - интервал не досчитался
 */
int8_t cm_interval_processor(typeCMModel* cm_ptr, uint8_t interval_id, uint64_t time_us)
{
	uint64_t interval_us;
	uint64_t last_call_interval = (time_us - cm_ptr->ctrl.last_call_interval_times[interval_id]);
	//
	if (cm_ptr->ctrl.first_call_status[interval_id] == 0){
		interval_us = cm_ptr->ctrl.first_call_time[interval_id] * 1000000;
	}
	else{
		interval_us = cm_ptr->ctrl.intervals[interval_id] * 1000000;
	}

	if (last_call_interval > interval_us) {
		cm_ptr->ctrl.last_call_interval_times[interval_id] = time_us;
		cm_ptr->ctrl.first_call_status[interval_id] = 1;
		return 1;
	}
	return 0;
}

// Управление настройками работы ЦМ

/**
  * @brief  установка измерительных интервалов
	* @param  cm_ptr указатель на структуру управления
	* @param  interval_number номер интервала для установки
	* @param  interval_value_s значение интервала для установки (5 - 7200 секунд)
  */
void cm_set_interval_value(typeCMModel* cm_ptr, uint16_t interval_number, uint16_t interval_value_s)
{
	switch(interval_number){
		case(CM_INTERV_SYS):
			cm_ptr->ctrl.intervals[CM_INTERV_SYS] = get_val_from_bound(interval_value_s, 1, 7200);
			break;
		case(CM_INTERV_MEAS):
			cm_ptr->ctrl.intervals[CM_INTERV_MEAS] = get_val_from_bound(interval_value_s, 10, 7200);
			break;
		case(CM_INTERVAL_SPEED): 
			cm_ptr->ctrl.intervals[CM_INTERVAL_SPEED] = get_val_from_bound(interval_value_s, 10, 7200);
			break;
		case(CM_INTERVAL_DBG):
			cm_ptr->ctrl.intervals[CM_INTERVAL_DBG] = get_val_from_bound(interval_value_s, 10, 7200);
			break;
		case(CM_1S_INTERVAL):
			//
			break;
	}
}

/**
  * @brief  включение/отключение ускоренного режима
	* @param  cm_ptr указатель на структуру управления
	* @param  speedy_mask маска включения ускоренного режима согласно распределению в списке устройств (собирается по и с CM_DEVICE_SPEEDY_MODE_MASK)
	* запись 0 в speedy_mask отключает ускоренный
	* @param  time_s работы ускоренного режима в с
  */
void cm_set_speedy_mode(typeCMModel* cm_ptr, uint16_t speedy_mask, uint16_t time_s)
{
	cm_ptr->ctrl.speedy_mode_state = speedy_mask & CM_DEVICE_SPEEDY_MODE_MASK;
	cm_ptr->ctrl.speedy_mode_timeout = get_val_from_bound(time_s, 1, 3600)*1000000; //приведение к мкс
}

// Раобота с системным кадром
/**
  * @brief  формирование системного кадра
	* @param  cm_ptr указатель на структуру управления
  */
void cm_frame_forming(typeCMModel* cm_ptr)
{
	uint8_t i;
	//
	cm_ptr->frame.row.label = 0x0FF1;
	cm_ptr->frame.row.definer = frame_definer(0, cm_ptr->device_number, NULL, cm_ptr->frame_type);
	cm_ptr->frame.row.num = (cm_ptr->global_frame_num++);
	cm_ptr->frame.row.time = _rev_u32_by_u16(Get_Time_s());
	// заполнение полей ЦМ
	memset((uint8_t*)cm_ptr->frame.row.data, 0xFEFE, sizeof(cm_ptr->frame.row.data));
	//
	for (i=0; i<PWR_CH_NUMBER; i++) {
		cm_ptr->frame.sys.body.currents[i] = cm_ptr->pwr.ch[i].current;
	}
	cm_ptr->frame.sys.body.power_status = cm_ptr->pwr.status;
	cm_ptr->frame.sys.body.power_state = cm_ptr->pwr.state;
	//
	cm_ptr->frame.sys.body.sw_version = cm_ptr->sw_version;
	//
	cm_ptr->frame.sys.body.nans_status = cm_ptr->ib.nans_status + (((cm_ptr->mko_bc.error) ? 1 : 0) << 13);
	cm_ptr->frame.sys.body.nans_counter = cm_ptr->ib.nans_counter + cm_ptr->mko_bc.error_cnt;
	cm_ptr->frame.sys.body.cm_status = cm_ptr->ctrl.status;
	cm_ptr->frame.sys.body.rst_cnter = cm_ptr->ctrl.rst_cnter & 0xFF;
	//
	cm_ptr->frame.sys.body.operation_time = _rev_u32_by_u16(cm_ptr->ctrl.operation_time);
	//
	cm_ptr->frame.sys.body.diff_time = cm_ptr->ctrl.diff_time;
	cm_ptr->frame.sys.body.diff_time_fractional = cm_ptr->ctrl.diff_time_fractional;
	cm_ptr->frame.sys.body.sync_num = cm_ptr->ctrl.sync_num & 0xFF;
	cm_ptr->frame.sys.body.sync_time_s = _rev_u32_by_u16(cm_ptr->ctrl.sync_time_s);
	//
	cm_ptr->frame.sys.body.stm_val = cm_ptr->stm.state;
	cm_ptr->frame.sys.body.mcu_temp = cm_ptr->adc.mcu_temp;
	//
	cm_ptr->frame.sys.body.read_ptr = cm_ptr->mem.read_ptr;
	cm_ptr->frame.sys.body.write_ptr = cm_ptr->mem.write_ptr;
	//
	cm_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&cm_ptr->frame.row, sizeof(typeFrameStruct) - 2);
}

// Отладка через ВШ
/**
  * @brief  проверка наличие отладочных команд по ВШ и их обработка
	* @param  cm_ptr указатель на структуру управления ЦМ
  */
__weak void cm_dbg_ib_command_handler(typeCMModel* cm_ptr)
{
	//
}

// Работа с командным интерефейсом МКО

/**
  * @brief  проверка наличие команды в командном интерфейсе МКО
	* @param  cm_ptr указатель на структуру управления ЦМ
  */
__weak void cm_mko_command_interface_handler(typeCMModel *cm_ptr)
{
	//
}

/**
  * @brief  обработчик команды синхронизации времени
	* @param  cm_ptr указатель на структуру управления
  */
void cm_mko_cmd_synch_time(typeCMModel* cm_ptr)
{
	cm_ptr->ctrl.sync_num += 1;
	cm_ptr->ctrl.sync_time_s = Get_Time_s();
	Time_Set((uint64_t)(((uint64_t)(cm_ptr->mko_rt.data[1] & 0xFFFF) << 32) | ((uint64_t)(cm_ptr->mko_rt.data[2] & 0xFFFF) << 16)), &cm_ptr->ctrl.diff_time, &cm_ptr->ctrl.diff_time_fractional);
}

/**
  * @brief  включение режима констант для проверки работы отправки данных
	* @param  cm_ptr указатель на структуру управления
	* @param  mode 1 - ena, 0 - disable 
  */
void cm_constant_mode_ena(typeCMModel* cm_ptr, uint8_t mode)
{
	if (mode) {
		cm_ptr->const_mode = 1;
	}
	else {
		cm_ptr->const_mode = 0;
	}
	// printf("const mode is <%d>", cm_ptr->const_mode);
}

/**
 * @brief установка очистка статуса ЦМ
 * 
 * @param cm_ptr указатель на управляющую модель ЦМ
 * @param status статус для установки (задефайнены)
 * @param set_clear 1 - установка, 0 - очистка
 * @return uint8_t 
 */
uint8_t cm_set_clear_status(typeCMModel* cm_ptr, uint8_t status, uint8_t set_clear)
{
	switch(set_clear){
		case 0: //clear
			cm_ptr->ctrl.status &= ~status;
		break;
		case 1: //set
			cm_ptr->ctrl.status |= status;
		break;
	}
	return cm_ptr->ctrl.status;
}

// Внутренние рабочие функции
void _buff_rev16(uint16_t *buff, uint8_t leng_16)
{
	uint16_t i = 0;
	for (i=0; i<leng_16; i++) {
		buff[i] = __REV16(buff[i]);
	}
}

uint8_t uint16_to_log2_uint8_t(uint16_t var)
{
	float man;
	uint8_t man_uint8, i;
	int exp;
	man = frexp(var, &exp);
	for (i=0; i<4;i++)
	{
		if (man == 0) break;
		man = man*2;
		exp -= 1;
		if (exp == 0) break;
	}
	man_uint8 = man;
	return ((exp & 0xF) << 4) + ((man_uint8 & 0xF) << 0);
}

/**
 * @brief если число внутри границ - используется оно, если нет, то ближайшая граница
 * 
 * @param val 
 * @param min 
 * @param max 
 * @return uint16_t 
 */
uint16_t get_val_from_bound(uint16_t val, uint16_t min, uint16_t max)
{
	if (val > max) return max;
	else if (val < min) return min;
	return val;
}

/**
 * @brief если число внутри границ (включительно) - возвращает 1, иначе 0
 * 
 * @param val 
 * @param min 
 * @param max 
 * @return uint16_t 
 */
uint16_t check_val_in_bound(uint16_t val, uint16_t min, uint16_t max)
{
	if ((val <= max) && (val >= min)) return 1;
	else return 0;
}

/**
 * @brief Get the version from str object like: "1.12" -> 0x010C
 * 
 * @param var_str "(dec)XX.(dec)YY"
 * @return uint16_t XXYY in hex
 */
uint16_t get_version_from_str(char* var_str)
{	
	int ver_major = 0, ver_minor = 0;
	sscanf(var_str, "%d.%d", &ver_major, &ver_minor);
	return ((ver_major << 8) & 0xFF) | (ver_minor & 0xFF);
}
