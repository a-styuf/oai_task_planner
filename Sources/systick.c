/**
  ******************************************************************************
  * @file           : systick.c
  * @version        : v1.0
  * @brief          : функции управления таймером SysTick
  * @author					: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
	* @date						: 2021.09.16
  ******************************************************************************
  */

#include "systick.h"

/**
 * @brief Инициализация SysTick таймера
 * 
 * @param period_ms период счета SysTick-таймера
 */
uint8_t systick_init(uint32_t mcu_clk_mhz, uint32_t value_us)
{
  uint32_t ticks = (value_us*mcu_clk_mhz) - 1;

#ifdef SYSTICK_TICKINT
  SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) | (1 << SYSTICK_TICKINT) | (1 << SYSTICK_ENABLE);
  if (ticks > SYSTICK_MAXCOUNT)  return (1);                                             /* Reload value impossible */
#elif SysTick_CTRL_TICKINT_Pos
  SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_ENABLE_Pos);
  if (ticks > (SysTick_VAL_CURRENT_Msk >> SysTick_VAL_CURRENT_Pos))  return (1);
#else 
  #pragma message Something Wrong with Systick timer setup
#endif

  SysTick->LOAD = ticks;
  
  SysTick->VAL = 0;
  
  //NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);                            /* set Priority for Cortex-M0 System Interrupts */
  NVIC_EnableIRQ(SysTick_IRQn);
  return (0);                                                                            /* Function successful */
}

/**
 * @brief слабая функция 
 * обработка в месте использования
 * @return __weak 
 */
__weak void Systick_Callback(void)
{
		//
}

/**
 * @brief обработка прерываний через CallBack
 * 
 */
void SysTick_Handler(void){
  Systick_Callback();
}
