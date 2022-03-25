#ifndef _CM_H_
#define _CM_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include <stdio.h>
#include <string.h>
#include "power_management.h"
#include "internal_bus.h"
#include "timers.h"
#include "frames.h"
#include "frame_mem.h"
#include "task_planner.h"
#include "mko.h"
#include "stm.h"

//***Параметры программы для ЦМ
//версия прошивки
#define CM_SW_VERSION 0
// номер устройства
#define FRAME_DEV_ID 999 //999 - по умолчанию, необходимо получить номер устройства в системе ОАИ
// параметры МКО
#define MKO_ADDRESS_DEFAULT  	13  // 0 - адрес берется с разъема, не 0 - адрес МКО
//
#define CM_HANDLER_INTERVAL_MS   500

/**
	* @brief  данная нумерация является общей для всей переферии и позволяет в общих списках понять смезение данного устройства 
	* @note  данный список также используется для нумерации источников/обработчиков событий от/к переферии (необходимо следить, что бы количество источников не превышало количество доступных событий)
*/
typedef enum devices_list
{
	CM, PER_TMPLT, 
	DEV_NUM
} device_list;

//! Управление разрешением работы ускоренного режима для различных устройств
#define CM_DEVICE_MEAS_MODE_MASK 	0x02 	//! модули, работающие по измерительному интервалу (а не альтернативному)
#define CM_DEVICE_SPEEDY_MODE_MASK 	0x02 	//! Ускоренный режим разрешен для per_tmplt

enum cm_interval_list
{
	CM_INTERV_SYS, CM_INTERV_MEAS, CM_INTERVAL_SPEED, CM_INTERVAL_DBG,
	CM_1S_INTERVAL,
	CM_INTERV_NUMBER
};
#define DEFAULT_CM_INTERV_VALUES_S					{1800, 120, 2, 30, 1}
#define DEFAULT_CM_DEFAULT_START_TIME_S				{5, 5, 0, 6, 6}

#define CM_EVENT_MEAS_INTERVAL_START       			(1<<0)
#define CM_EVENT_MEAS_INTERVAL_DATA_READY  			(1<<1)
#define CM_EVENT_SYS_INTERVAL_START       			(1<<2)
#define CM_EVENT_SYS_INTERVAL_DATA_READY  			(1<<3)
#define CM_EVENT_SPEEDY_INTERVAL_START       		(1<<4)
#define CM_EVENT_SPEEDY_INTERVAL_DATA_READY  		(1<<5)

#define CM_FRAMES_FIFO_DEPTH 						8
#define CM_CFG_FRAME_TYPE							15

//MKO sub_address that are used
#define CM_MKO_SA_CMD								17
#define CM_MKO_SA_TECH								30
#define CM_MKO_SA_ARCH_REQUEST_CM					18
#define CM_MKO_SA_ARCH_READ_CM						20

// debuf interface base don IB
#define CM_SELF_MB_ID  (1)  // для отладочных команд

//STM
/**
  * @brief  нумерация STM-сигналов, используемых в данном устройстве
*/
typedef enum stm_list
{
	NKBE, 	AMKO,
	STM_NUM
} stm_list;

#define STM_IO_PORT 		{PORTB, PORTB}
#define STM_IO_LINE 		{18, 19}
#define STM_DEFAULT_VAL 	{1, 1}

// Half-set GPIO setting
#define HALF_SET_IO_PORT 	PORTB
#define HALF_SET_LINE 		(0)
/**
  * @brief  список отладочных команд через ВШ
*/
enum cm_dbg_cmd_list
{
	CM_DBG_CMD_SWITCH_ON_OFF, CM_DBG_CMD_CM_RESET, CM_DBG_CMD_CM_CHECK_MEM, CM_DBG_CMD_CM_INIT,
	CM_DBG_CMD_ARCH_REQUEST,
	CM_DBG_CMD_NUMBER
};

/**
  * @brief  список команд МКО
*/
enum mko_cmd_list
{
	CMD_TEST, CMD_SYNCH_TIME, CMD_INIT, CMD_SET_INTERVAL, 
	CMD_CONST_MODE, CMD_CURRENT_LVL, CMD_PWR_CH_CTRL,
	CMD_NUMBER
};

/**
  * @brief  список команд технологического подадреса МКО
*/
enum mko_tech_cmd_list
{
	TCMD_CHECK_MEM, TCMD_SET_OPERATION_TIME, TCMD_SET_STM, 
	TECH_CMD_NUMBER
};


// Настройки каналов МПП (необязательно)
#define MPP_DEV_NUM (2)
// !!настройки уставкии МПП (offset) количество должно совпадать с MPP_DEV_NUM!!
#define MPP_DEFAULT_OFFSET {0xF02, 0xF03}
// адреса МПП на внутренней шине
#define MPP_ID {2, 2}
// номер канала, используемый устройством МПП
#define MPP_CHANNENUM_ID {0, 1}

//структуры кадров
#pragma pack(push, 2)
/** 
  * @brief  структура с данными для кадра ЦМ (!! длина)
  */
typedef  struct
{
		// питание
	uint16_t currents[PWR_CH_NUMBER];   //+0
	uint16_t power_status;				//+2
	uint16_t power_state;				//+4
	//
	uint16_t nans_status;  		//+6
	uint8_t nans_counter;  		//+8
	uint8_t rst_cnter;  		//+9
	//
	uint32_t operation_time;  	//+10
	//
	int16_t diff_time;					//+14
	uint8_t diff_time_fractional;		//+16
	uint8_t sync_num;					//+17
	uint32_t sync_time_s;				//+18
	//
	int8_t mcu_temp;			//+22
	int8_t stm_val;				//+23
	//
	uint16_t read_ptr;			//+24
	uint16_t write_ptr; 		//+26
	//
	uint16_t reserve[14];  //+28
}typeCMFrameReport;      //52

/**
 * @brief объединение для свзяки уровней кадров и полезных данных
 */
typedef union{
	typeFrameStruct row;
	struct{
		uint16_t header[5];
		typeCMFrameReport body;
		uint16_t crc16;
	} sys;
}typeSysFrameUnion;

/** 
  * @brief  структура с конфигурацией ЦМ
  */
typedef  struct
{
	// питание
	uint16_t power_status;		//+0
	uint16_t power_state;		//+2
	//
	uint8_t rst_cnter;  		//+4
	uint8_t gup;				//+5
	//
	uint32_t operation_time;  	//+6
	//
	uint32_t write_ptr;			//+10
	uint32_t read_ptr;			//+14
	//
	uint16_t reserve[34];  		//+18
}typeCfgReport;      //52

typedef union{
	typeFrameStruct row;
	struct{
		uint16_t header[5];
		typeCfgReport body;
		uint16_t crc16;
	} cfg;
}typeCfgFrameUnion;

/** 
  * @brief  структура с переменнымиу правления ЦМ
  */
typedef  struct
{
	uint16_t speedy_mode_state;
	uint16_t speedy_mode_mask;
	uint64_t speedy_mode_timeout;
	//
	uint16_t intervals[CM_INTERV_NUMBER];
	uint64_t last_call_interval_times[CM_INTERV_NUMBER];
	uint64_t first_call_time[CM_INTERV_NUMBER];
	uint8_t first_call_status[CM_INTERV_NUMBER];
	//
	uint8_t meas_event, speedy_event;
	//
	uint32_t frame_end_to_end_number;
	//
	uint32_t sync_time_s;
	uint16_t rst_cnter, sync_num;
	int16_t diff_time;
	int8_t diff_time_fractional;
	//
	uint32_t operation_time;
}typeCMControlStruct;

/**
  * @brief  общая структура програмной модели ЦМ
  */
typedef struct
{
	//
	uint16_t id;			          	//! id на внутренней шине
	uint16_t self_num;          		//! номер устройства с точки зрения ЦМ
	uint16_t half_set_num;          	//! номер полукомплекта (актуально длс приборов в с холодным резервированием)
	uint16_t device_number, frame_type; //! параметры прибора, в котором он используется
	//
	typeSysFrameUnion frame;  //!системный кадр
	typeCfgFrameUnion current_cfg, loaded_cfg;  //! кадры с параметрами для сохранения
	uint8_t frame_data_ready;
	//
	typeFrameStruct frames_fifo[CM_FRAMES_FIFO_DEPTH];
	uint8_t frames_fifo_num, frames_fifo_num_max;
	uint32_t fifo_error_cnt;
	uint32_t global_frame_num;  //сквозной номер для формируемых кадров
	//
	typePower pwr;
	typeADCStruct adc;
	typeMKOStruct mko_rt, mko_bc;
	typeIBStruct ib;
	typeFRAME_MEM mem;
	type_STM_Model stm[STM_NUM];
	type_SINGLE_GPIO half_set_num_io; //! gpio опроса номера поолукомплекта
	//
	typeCMControlStruct ctrl;
	//
	uint64_t last_call_time_us, call_interval_us;
	//
}typeCMModel;

//

void cm_init(typeCMModel* cm_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type);
void cm_reset_parameters(typeCMModel* cm_ptr);
uint8_t cm_load_cfg(typeCMModel* cm_ptr);
void cm_save_cfg(typeCMModel* cm_ptr);
void cm_set_cfg(typeCMModel* cm_ptr);
void cm_get_cfg(typeCMModel* cm_ptr);
int8_t cm_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t cm_frame_receive(typeCMModel* cm_ptr, uint8_t* data);
void cm_frame_handling(typeCMModel* cm_ptr);
int8_t cm_write_fifo(typeCMModel* cm_ptr, typeFrameStruct* frame);
int8_t cm_read_fifo(typeCMModel* cm_ptr, typeFrameStruct* frame);
int8_t cm_interval_processor(typeCMModel* cm_ptr, uint8_t interval_id, uint64_t time_us);
// Управление настройками работы ЦМ
void cm_set_interval_value(typeCMModel* cm_ptr, uint16_t interval_number, uint16_t interval_value_s);
void cm_set_speedy_mode(typeCMModel* cm_ptr, uint16_t speedy_mask, uint16_t time_s);
//работа с системном кадром
void cm_frame_forming(typeCMModel* cm_ptr);
// Отладка через ВШ
__weak void cm_dbg_ib_command_handler(typeCMModel* cm_ptr);
// обработка командных сообщений МКО
__weak void cm_mko_command_interface_handler(typeCMModel *cm_ptr);
void cm_mko_cmd_synch_time(typeCMModel* cm_ptr);
// Внутрениие рабочие функции
void _buff_rev16(uint16_t *buff, uint8_t leng_16);
uint8_t uint16_to_log2_uint8_t(uint16_t var);
uint16_t get_val_from_bound(uint16_t val, uint16_t min, uint16_t max); //если число внутри границ - используется оно, если нет, то ближайшая граница


#endif
