#ifndef _DDII_H_
#define _DDII_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "uarts.h"
#include "task_planner.h"
#include "cyclogramma.h"
#include "frames.h"
#include "debug.h"
#include "mko.h"


// дефайны для переменных
#define DDII_DEFAULT_INTERVAL_MS (10000)

#define DDII_EVENT_MEAS_INTERVAL_START       (1<<0)
#define DDII_EVENT_MEAS_INTERVAL_DATA_READY  (1<<1)

#define DDII_MEAS_NUMBER 1
#define DDII_REC_FIFO_DEPTH 4

#define DDII_MKO_SA_DATA_FRAME 0x0F
#define DDII_MKO_SA_CTRL         30

#define DDII_MK_COMMAND_LINK_CHECK   0
#define DDII_MK_COMMAND_SET_MODE  1
#define DDII_MK_COMMAND_START  2
#define DDII_MK_COMMAND_CONSTANT_MODE 17

//
#pragma pack(push, 2)

/**
 * @brief структура с данными АДИИ для общения по ВШ
 * 
 */
typedef struct
{
  uint8_t data[52];
}typeDDIIRec;

/** 
  * @brief  структура кадар АДИИ
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    typeDDIIRec meas;
    uint16_t crc16;
  } ddii;
}typeDDIIFrameUnion;

/** 
  * @brief  структура управления МПП
  */
typedef struct
{
  // interfaces
  typeMKOStruct* mko_bc_ptr;
  // сfg
	uint16_t mko_addr;			          // id на внутренней шине
	uint16_t mko_bus;			          // id на внутренней шине
	uint16_t self_num;          // номер устройства с точки зрения ЦМ
	uint16_t device_number, frame_type;  //параметры прибора МПП, в котором он используется
  uint16_t interval_ms;
  uint16_t const_mode;
  uint32_t *global_frame_num_ptr;
  // to task_planner
  uint8_t meas_event_num;
  uint64_t last_call_time_us;
  // data
  typeDDIIFrameUnion frame;
  uint8_t frame_data_ready;  // флаг готовности данных в памяти на отправку в другой процесс
  // fifo для обработки данных ДДИИ
  typeDDIIRec rec_fifo[DDII_REC_FIFO_DEPTH];
  uint8_t rec_num, rec_max;
  // general
	typeDDIIFrameUnion data_frame;
  // cyclogram_ctrl
  typeCyclogramma meas_cyclo;
} typeDDIIStruct;

#pragma pack(pop)

//
void ddii_init(typeDDIIStruct* ddii_ptr, uint8_t self_num, uint8_t mko_addr, uint16_t device_number, uint16_t frame_type, typeMKOStruct* mko_bc_ptr, uint8_t mko_bus, uint32_t* gl_fr_num);
void ddii_reset_parameters(typeDDIIStruct* ddii_ptr);
//
int8_t ddii_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t ddii_frame_forming(typeDDIIStruct* ddii_ptr);
//
void ddii_constant_mode(typeDDIIStruct* ddii_ptr, uint32_t on_off);
void ddii_start(typeDDIIStruct* ddii_ptr, uint32_t ddii_rd_ptr);
//
void ddii_read_data_frame(typeDDIIStruct *ddii_ptr);
//
int8_t ddii_write_fifo(typeDDIIStruct *ddii_ptr, typeDDIIRec* data);
int8_t ddii_read_fifo(typeDDIIStruct* ddii_ptr, typeDDIIRec* data);
// функции для работы циклограмы измерительного интервала
void ddii_meas_cycl_init(typeDDIIStruct* ddii_ptr);
void ddii_meas_cycl_start(void* ctrl_struct);
void ddii_meas_cycl_read(void* ctrl_struct);
void ddii_meas_cycl_frame_forming(void* ctrl_struct);
//
void __ddii_struct_rev(typeDDIIRec* ddii_struct_ptr);
//
#endif
