#include "pid.h"

  /**
  ******************************************************************************
  * @file           : pid.c
  * @version        : v1.0
  * @brief          : библиотека для работы с пид-регулятором
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  K нормализующий коэффициент
  * @param  P пропорциональный коэффициент 
  * @param  D диффиренциальный коэффициент
  * @param  I интегральный коэффициент
  * @param  reaction_max максимальное воздействие
  */
void pid_init(type_PID_model* pid_ptr, float K, float P, float D, float I, float reaction_max)
{
  pid_reset(pid_ptr);
  pid_ptr->K_coeff = K;
  pid_ptr->P_coeff = P;
  pid_ptr->D_coeff = D;
  pid_ptr->I_coeff = I;
  pid_ptr->reaction_max = reaction_max;
}

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  */
void pid_reset(type_PID_model* pid_ptr)
{
  pid_ptr->value = 0.0;
  pid_ptr->value_desired = 0.0;
  pid_ptr->value_prev = 0.0;
  pid_ptr->derivate = 0.0;
  pid_ptr->error = 0.0;
  pid_ptr->integral = 0.0;
  pid_ptr->K_coeff = 0.0;
  pid_ptr->P_coeff = 0.0;
  pid_ptr->D_coeff = 0.0;
  pid_ptr->I_coeff = 0.0;
  pid_ptr->P_reaction = 0.0;
  pid_ptr->D_reaction = 0.0;
  pid_ptr->I_reaction = 0.0;
  pid_ptr->reaction = 0.0;
  pid_ptr->reaction_max = 0.0;
}

/**
  * @brief  сброс ппамяти ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  */
void pid_refreshet(type_PID_model* pid_ptr)
{
  pid_ptr->integral = 0.0;
  pid_ptr->P_reaction = 0.0;
  pid_ptr->D_reaction = 0.0;
  pid_ptr->I_reaction = 0.0;
  pid_ptr->reaction = 0.0;
}

/**
  * @brief  установка желаемого значения
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  desired_value значение регулируемой величины, к которой происходит подстройка
  */
void pid_set_desired_value(type_PID_model* pid_ptr, float desired_value)
{
  pid_refreshet(pid_ptr);
  pid_ptr->value_desired = desired_value;
}

/**
  * @brief  установка коэффициентов
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  P пропорциональный коэффициент 
  * @param  D диффиренциальный коэффициент
  * @param  I интегральный коэффициент
  */
void pid_set_coeff(type_PID_model* pid_ptr, float K, float P, float D, float I)
{
  pid_ptr->K_coeff = K;
  pid_ptr->P_coeff = P;
  pid_ptr->D_coeff = D;
  pid_ptr->I_coeff = I;
}

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  value значение регулируемой переменной
  * @param  period_ms период вызова данной функции
  * @retval необходимая реакция для поддержания значения
  */
float pid_step_calc(type_PID_model* pid_ptr, float value, uint16_t period_ms)
{
  pid_ptr->value = value;
  pid_ptr->derivate = (pid_ptr->value - pid_ptr->value_prev)/(period_ms/1000.);
  pid_ptr->error = pid_ptr->value_desired - pid_ptr->value;
  pid_ptr->integral += pid_ptr->error*(period_ms/1000.);
  pid_ptr->P_reaction = pid_ptr->K_coeff*(pid_ptr->P_coeff*pid_ptr->error);
  pid_ptr->D_reaction = pid_ptr->K_coeff*(pid_ptr->D_coeff*pid_ptr->derivate);
  pid_ptr->I_reaction = pid_ptr->K_coeff*(pid_ptr->I_coeff*pid_ptr->integral);
  pid_ptr->reaction = pid_ptr->P_reaction + pid_ptr->D_reaction + pid_ptr->I_reaction;
  //
  if (pid_ptr->reaction > pid_ptr->reaction_max) pid_ptr->reaction = pid_ptr->reaction_max;
  else if (pid_ptr->reaction < -(pid_ptr->reaction_max)) pid_ptr->reaction = -pid_ptr->reaction_max;
  //
  pid_ptr->value_prev = pid_ptr->value;
  //
  return pid_ptr->reaction;
}

/**
  * @brief  вывод отладочной информации для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина массива для отчета
  */
uint8_t pid_get_str_report(type_PID_model* pid_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "PID: P=%.3f D=%.3f I=%.3f React=%.3f", 
                      pid_ptr->P_reaction,
                      pid_ptr->D_reaction,
                      pid_ptr->I_reaction,
                      pid_ptr->reaction);
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}
