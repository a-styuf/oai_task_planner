#ifndef _HVIP_H_
#define _HVIP_H_

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "1986ve8_lib/cm4ikmcu.h"
#include "timers.h"
#include "adc.h"
#include "gpio.h"
#include "debug.h"
#include "pid.h"

//defines

#define HVIP_PROCESS_PERIOD_MS (1000)

// PID Current-stabilisation
#define PID_K 0.1
#define PID_P 0.005
#define PID_D 0.0
#define PID_I 0.0
#define HVIP_PID_MAX_REACTION 1.0

#define HVIP_MAX_VOLTAGE_COEFF (1.3)
#define HVIP_VOLTAGE_MAX_ERROR (0.1)

// states
#define HVIP_STATE_HV    1<<0
#define HVIP_STATE_OVEVRCURRENT 1<<1
#define HVIP_STATE_FEEDBACKERROR 1<<2

// mode
#define HVIP_MODE_OFF       0
#define HVIP_MODE_ON        1

// state_flag

// data structures
#pragma pack(2)
typedef struct  // программная модель управления БДД
{
  uint16_t h_voltage;
  uint16_t current;
  uint8_t state;
  uint8_t reserve;
}type_HVIP_frame_report;

typedef struct  // программная модель управления БДД
{
	uint8_t mode;
  uint8_t state;
  uint8_t pwm_ch_num;
  uint16_t pwm_val;  // 0-999
  float pwm_val_float;  // 0-999
  float v_fb, v_hv, v_hv_desired;
  float current, max_current;
  float a_u, b_u, a_i, b_i;
  //
  typeADCStruct* adc_ptr;
  uint8_t adc_ch_hv, adc_ch_i24;
  //
  type_SINGLE_GPIO hv_inh;
  type_PID_model pid;
  type_HVIP_frame_report report;
  // поддрежка task_planner
  uint64_t last_call_time_us;
}type_HVIP;

int8_t hvip_init(type_HVIP* hvip_ptr, uint8_t mode,
                  typeADCStruct* adc_ptr, uint8_t adc_ch_hv, uint8_t adc_ch_i24, 
                  PortControl* inh_port, uint8_t inh_num,
                  uint8_t pwm_ch_num,
                  float pwm_val, float desired_voltage_V, float max_current_A,
                  float A_U, float B_U,
                  float A_I, float B_I
                  );
int8_t hvip_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
void hvip_process(type_HVIP* hvip_ptr, uint16_t period_ms);
void hvip_set_mode(type_HVIP* hvip_ptr, uint8_t mode);
void hvip_set_voltage(type_HVIP* hvip_ptr, float voltage);
uint8_t hvip_get_str_report(type_HVIP* hvip_ptr, char* report);
void hvip_form_report(type_HVIP* hvip_ptr);

#endif
