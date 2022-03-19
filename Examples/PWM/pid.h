#ifndef _PID_H_
#define _PID_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include <math.h>
#include "digital_filter.h"
#include "debug.h"

#pragma pack(2)
/** 
  * @brief  структура управления ПИД
  */
typedef struct
{
  float K_coeff, P_coeff, I_coeff, D_coeff;
  float P_reaction, I_reaction, D_reaction;
  float integral;
  float value, value_prev, derivate;
  float value_desired, error;
  float reaction_max;
  float reaction;
} type_PID_model;
//
void pid_init(type_PID_model* pid_ptr, float K, float P, float D, float I, float reaction_max);
void pid_reset(type_PID_model* pid_ptr);
void pid_refreshet(type_PID_model* pid_ptr);
void pid_set_desired_value(type_PID_model* pid_ptr, float desired_value);
void pid_set_coeff(type_PID_model* pid_ptr, float K, float P, float D, float I);
float pid_step_calc(type_PID_model* pid_ptr, float value, uint16_t period_ms);
uint8_t pid_get_str_report(type_PID_model* pid_ptr, char* report);

#endif
