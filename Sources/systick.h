#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include "main.h"

/**
 * @brief структура для удобства управления регистром управления SysTick-таймера
 * 
 */
typedef union
{
	uint32_t whole;
	struct
	{
		uint32_t enable : 1;
		uint32_t tickint : 1;
		uint32_t rsrv : 13;
		uint32_t countflag : 1;
	} field;
}typeSysTickControlRegister;

/**
 * @brief структура для удобства управления регистром контроля SysTick-таймера
 * 
 */
typedef union
{
	uint32_t whole;
	struct
	{
		uint32_t value : 24;
	} field;
}typeSysTickLoadRegister;

/**
 * @brief структура для удобства управления регистром значения SysTick-таймера
 * 
 */
typedef union
{
	uint32_t whole;
	struct
	{
		uint32_t value : 24;
	} field;
}typeSysTickValRegister;

uint8_t systick_init(uint32_t mcu_clk_mhz, uint32_t value_us);
__weak void Systick_Callback(void);

#endif
