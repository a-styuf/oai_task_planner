#ifndef _PERIPHERAL_TEAMPLATE_IB_H_
#define _PERIPHERAL_TEAMPLATE_IB_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "internal_bus.h"
#include "task_planner.h"
#include "cyclogramma.h"
#include "frames.h"
#include "debug.h"


// дефайны для переменных
#define PER_TMPLT_REC_FIFO_DEPTH 4

#define PER_TMPLT_DEFAULT_INTERVAL_MS (10000)

#define PER_TMPLT_EVENT_MEAS_INTERVAL_START       (1<<0)
#define PER_TMPLT_EVENT_MEAS_INTERVAL_DATA_READY  (1<<1)

//
#pragma pack(push, 2)
/** 
  * @brief  структура данных примера переферии
  */
typedef struct
{
  uint32_t time;
  uint8_t data[22]; 
}typePERTMPLTMeas; //26



/** 
  * @brief  структура кадар
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    uint16_t arch_count;
    uint16_t offset;
    typePERTMPLTMeas meas[2];
    uint16_t crc16;
  } per_tmplt;
}typeMPPFrameUnion;

/** 
  * @brief  структура управления МПП
  */
typedef struct
{
  // interfaces
  typeIBStruct* ib;
  // сfg
	uint16_t id;			          // id на внутренней шине
	uint16_t self_num;          // номер устройства с точки зрения ЦМ
	uint16_t device_number, frame_type;  //параметры прибора, в котором он используется
	uint16_t interval_ms;
  uint32_t *global_frame_num_ptr;
  // to task_planner
  uint8_t meas_event_num;
  uint64_t last_call_time_us;
  // data
  typeMPPFrameUnion frame;
  uint8_t frame_data_ready;  // флаг готовности данных в памяти на отправку в другой процесс
  typePERTMPLTMeas meas_buff[PER_TMPLT_REC_FIFO_DEPTH]; //! буфер для хранения полученных измерений
  uint8_t meas_ptr;  //! указатель на последние сохраненное измерение
  // general
	uint8_t forced_start_flag;  // флаг необходимости принудительного запуска
	uint8_t forced_start_timeout;  // таймаут на запуск принудительного старта
	uint8_t frame_pulse_cnt;    // количество считанных помех с последнего формирования кадра
  // cyclogram_ctrl
  typeCyclogramma meas_cyclo;
} typePERTMPLTStruct;

#pragma pack(pop)

//
void per_tmplt_init(typePERTMPLTStruct* per_tmplt_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type, typeIBStruct* ib_ptr, uint32_t* gl_fr_num);
void per_tmplt_reset_parameters(typePERTMPLTStruct* per_tmplt_ptr);
//
int8_t per_tmplt_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t per_tmplt_frame_forming(typePERTMPLTStruct* per_tmplt_ptr);
//
void per_tmplt_example_1(typePERTMPLTStruct* per_tmplt_ptr);
void per_tmplt_example_2(typePERTMPLTStruct* per_tmplt_ptr, uint32_t parameter);
void per_tmplt_example_3_meas_get(typePERTMPLTStruct* per_tmplt_ptr);
//
void __per_tmplt_struct_rev(typePERTMPLTMeas* per_tmplt_struct_ptr);
// функции для работы циклограмы измерительного интервала
void per_tmplt_meas_cycl_init(typePERTMPLTStruct* per_tmplt_ptr);
void per_tmplt_meas_cycl_example_1(void* ctrl_struct);
void per_tmplt_meas_cycl_example_2(void* ctrl_struct);
void per_tmplt_meas_cycl_example_3(void* ctrl_struct);
//
#endif
