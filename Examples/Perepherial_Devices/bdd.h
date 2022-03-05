#ifndef _BDD_H_
#define _BDD_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "internal_bus.h"
#include "task_planner.h"
#include "cyclogramma.h"
#include "frames.h"
#include "debug.h"
#include "mko.h"


// дефайны для переменных
#define BDD_DEFAULT_INTERVAL_MS (10000)

#define BDD_EVENT_MEAS_INTERVAL_START       (1<<0)
#define BDD_EVENT_MEAS_INTERVAL_DATA_READY  (1<<1)

#define BDD_MEAS_NUMBER 13
#define BDD_REC_FIFO_DEPTH 32

#define BDD_MKO_SA_SYSTEM_FRAME 0x0F
#define BDD_MKO_SA_ARCH_FRAME   0x03
#define BDD_MKO_SA_CTRL         30

#define BDD_MK_COMMAND_LINK_CHECK   0
#define BDD_MK_COMMAND_SET_OAI_DD_1_MODE  1
#define BDD_MK_COMMAND_SET_OAI_DD_2_MODE  2
#define BDD_MK_COMMAND_SET_OAI_DD_1_FILTER  3
#define BDD_MK_COMMAND_SET_OAI_DD_2_FILTER  4
#define BDD_MK_COMMAND_SET_OAI_DD_1_PID_SETTING  5
#define BDD_MK_COMMAND_SET_OAI_DD_2_PID_SETITNG  6
#define BDD_MK_COMMAND_SET_STM_DEBUG  7
#define BDD_MK_COMMAND_SET_IMS_MODE  8
#define BDD_MK_COMMAND_SET_IMS_KU  9
#define BDD_MK_COMMAND_SET_BDD_MODE 10
#define BDD_MK_COMMAND_SET_INIT_BDD 11
#define BDD_MK_COMMAND_CHECK_MEM 12
#define BDD_MK_COMMAND_SET_WORK_TIME_RESET 13
#define BDD_MK_COMMAND_OAI_DD_CALIBRATION 14
#define BDD_MK_COMMAND_ARCH_MEM_SET_RD_PTR 15
#define BDD_MK_COMMAND_ARCH_MEM_FRAME_REQUEST 16
#define BDD_MK_COMMAND_CONSTANT_MODE 17

// структуры данных
#pragma pack(push, 2)


/** 
  * @brief  структура БДД для вставления в кадр кадра
  */
typedef struct
{
    uint16_t pressure;     //+0
    uint8_t temp;          //+2
    uint8_t hv_current;    //+3
}typeBDDAcqValue;          //4

/** 
  * @brief  структура кадра БДД
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    //
    typeBDDAcqValue meas[BDD_MEAS_NUMBER]; //+10-62
    //
    uint16_t crc16;
  } bdd;
}typeBDDFrameUnion;

/** 
  * @brief  структура системного кадра БДД
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    //
    uint16_t pressure;  // +10
    uint16_t temperature;  // +12
    uint16_t imd_current;  // +14
    //
    uint16_t reserve[23];  // +16
    //
    uint16_t crc16;  // +62
  } bdd;
}typeBDDSystemFrameUnion;

/** 
  * @brief  структура архивного кадра БДД
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    //
    uint16_t reserve[52];  // +10
    //
    uint16_t crc16;  // +62
  } bdd;
}typeBDDArchFrameUnion;

/** 
  * @brief  структура управления БДД
  */
typedef struct
{
  // interfaces
  typeMKOStruct* mko_bc_ptr;
  // сfg
	uint16_t mko_addr;			          // id на внутренней шине
	uint16_t mko_bus;			          // id на внутренней шине
	uint16_t self_num;          // номер устройства с точки зрения ЦМ
	uint16_t device_number, frame_type;  //параметры прибора, в котором он используется
  uint16_t interval_ms;
  uint16_t const_mode;
  uint32_t *global_frame_num_ptr;
  // to task_planner
  uint8_t meas_event_num;
  uint64_t last_call_time_us;
  // data
  typeBDDFrameUnion frame;
  uint8_t frame_data_ready;  // флаг готовности данных в памяти на отправку в другой процесс
  // fifo для обработки данных БДД
  typeBDDAcqValue rec_fifo[BDD_REC_FIFO_DEPTH];
  uint8_t rec_num, rec_max;
  // general
	typeBDDSystemFrameUnion sys_frame;
	typeBDDArchFrameUnion arch_frame;
  // cyclogram_ctrl
  typeCyclogramma meas_cyclo;
} typeBDDStruct;

#pragma pack(pop)

//
void bdd_init(typeBDDStruct* bdd_ptr, uint8_t self_num, uint8_t mko_addr, uint16_t device_number, uint16_t frame_type, typeMKOStruct* mko_bc_ptr, uint8_t mko_bus, uint32_t* gl_fr_num);
void bdd_reset_parameters(typeBDDStruct* bdd_ptr);
//
int8_t bdd_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t bdd_frame_forming(typeBDDStruct* bdd_ptr);
//
void bdd_constant_mode(typeBDDStruct* bdd_ptr, uint32_t on_off);
void bdd_set_rd_ptr(typeBDDStruct* bdd_ptr, uint32_t bdd_rd_ptr);
void bdd_on_off(typeBDDStruct* bdd_ptr, uint32_t on_off);
void bdd_oai_calibration(typeBDDStruct* bdd_ptr, uint32_t mode);
void bdd_stm_debug(typeBDDStruct* bdd_ptr, uint32_t value, uint32_t timeout_ms);
void bdd_hw_init(typeBDDStruct* bdd_ptr);
void bdd_request_arch_frame(typeBDDStruct *bdd_ptr);

void bdd_read_sys_frame(typeBDDStruct *bdd_ptr);
void bdd_read_arch_frame(typeBDDStruct *bdd_ptr);

int8_t bdd_write_fifo(typeBDDStruct *bdd_ptr, typeBDDAcqValue* data);
int8_t bdd_read_fifo(typeBDDStruct *bdd_ptr, typeBDDAcqValue* data);

void _bdd_rec_rev(typeBDDAcqValue* bdd_rec);
void bdd_set_ena_parameters(typeBDDStruct *bdd_ptr, uint8_t bdd_ena_ctrl, int8_t top_temp, int8_t bot_temp);
// функции для работы циклограмы измерительного интервала
void bdd_meas_cycl_init(typeBDDStruct* bdd_ptr);
void bdd_meas_cycl_read(void* ctrl_struct);
void bdd_meas_cycl_frame_forming(void* ctrl_struct);
//

#endif
