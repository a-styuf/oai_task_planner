/**
 * @file timers.c
 * @author a-styuf (a-styuf@yandex.ru)
 * @brief описание аппаратной реализации таймеров для 1986ВЕ8, минимальное количество таймеров - 3
 *< TMR0 - для общего использования
 *< TMR1 - для общего использования
 *< TMR2 - счетчик глобального времени
 *< TMR3 - используются для работы внутренней шины (здесь не представлен)
 * Таймеры используют прерывания для выставления статусов.
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "1986ve8_lib/cm4ikmcu.h"
#include "timers.h"

uint8_t timer_status = 0;  //! глобальная переменная для хранения статусов Таймеров
uint8_t high_byte_time_s = 0;  //! глобальная переменная для хранения старшей части глобального времени


/**
 * @brief инициализация таймеров для испльзования в программе
 * TMR0 - для общего использования
 * TMR1 - для общего использования
 * TMR2 - счетчик глобального времени
 * TMR3 - используются для работы внутренней шины (здесь не представлен)
 */
void Timers_Init(void)
{
    CLK_CNTR->KEY = _KEY_;
    CLK_CNTR->PER0_CLK |= (1<<23)|(1<<24)|(1<<25);  //включение Timer0 и Timer1
    CLK_CNTR->TIM0_CLK = (1<<16)| 39;  //timer clock freq = 1 MHz
    CLK_CNTR->TIM1_CLK = (1<<16)| 39;  //timer clock freq = 1 MHz
    CLK_CNTR->TIM2_CLK = (1<<16)| 249;  //timer clock freq = 160 kHz
    
    MDR_TMR0->CNTRL = 0x00000000;
    MDR_TMR1->CNTRL = 0x00000000;
    MDR_TMR2->CNTRL = 0x00000000;
    //Настраиваем работу основных таймеров
    MDR_TMR0->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR0->PSG = 999;  //Предделитель частоты //из 1 МГц получаем 1 кГц
    MDR_TMR0->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    
    MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR1->PSG = 999;  //Предделитель частоты //из 1 МГц получаем 1 кГц
    MDR_TMR1->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    //счетчик глобального времени
    MDR_TMR2->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR2->PSG = 624;  //Предделитель частоты //из 10 кГц получаем 256 Гц (время БКУ 6-ти байтовое, но используем только 5)
    MDR_TMR2->ARR = 0xFFFFFFFF;//считаем постоянно
    MDR_TMR2->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    //MDR_TMR2->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    NVIC_EnableIRQ(IRQn_TMR0);
    NVIC_EnableIRQ(IRQn_TMR1);
    NVIC_EnableIRQ(IRQn_TMR2);
}

/**
 * @brief обработчки прерывания от таймеров
 *  - взводятит статус о том, что таймеы досчитал
 * 
 */
void INT_TMR0_Handler(void) 
{
    timer_status |= (1 << 0);
    MDR_TMR0->STATUS = 0x0000;
    MDR_TMR0->CNT = 0x00000000;
    MDR_TMR0->CNTRL = 0x00000000;
}

/**
 * @brief обработчки прерывания от таймеров
 *  - взводятит статус о том, что таймеы досчитал
 * 
 */
void INT_TMR1_Handler(void) 
{
    timer_status |= (1 << 1);
    MDR_TMR1->STATUS = 0x0000;
    MDR_TMR1->CNT = 0x00000000;
    MDR_TMR1->CNTRL = 0x00000000;
}

void INT_TMR2_Handler(void) 
{
    high_byte_time_s += 1;   
}


/**
 * @brief Работа с заданием задержек. Универсально для TMR0, TMR1
 * 
 * @param num 0 - TMR0, 1 - TMR1
 * @param time_ms задержка в мс
 */
void Timers_Start(uint8_t num, uint32_t time_ms)
{  
    switch (num)
    {
        case 0:
            NVIC_DisableIRQ(IRQn_TMR0);
            MDR_TMR0->ARR = time_ms;  //задание времени в мс
            MDR_TMR0->CNT = 0x00000000;  //Начальное значение счетчика
            MDR_TMR0->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
            NVIC_EnableIRQ(IRQn_TMR0);
            break;
        case 1:
            NVIC_DisableIRQ(IRQn_TMR1);
            MDR_TMR1->ARR = time_ms;  //задание времени в мс
            MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
            MDR_TMR1->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
            NVIC_EnableIRQ(IRQn_TMR1);
            break;
    }
}

/**
 * @brief Остановка работы таймера по номеру.
 * 
 * @param num  0 - TMR0, 1 - TMR1
 */
void Timers_Stop(uint8_t num)
{  
    switch (num)
    {
        case 0:
            NVIC_DisableIRQ(IRQn_TMR0);
            MDR_TMR0->CNTRL = 0x00000000;  //Счет вверх по TIM_CLK. Разрешение работы таймера
            MDR_TMR0->ARR = 0x00;  //задание времени в мс
            MDR_TMR0->CNT = 0x00000000;  //Начальное значение счетчика
            NVIC_EnableIRQ(IRQn_TMR0);
            break;
        case 1:
            NVIC_DisableIRQ(IRQn_TMR1);
            MDR_TMR1->CNTRL = 0x00000000;  //Счет вверх по TIM_CLK. Разрешение работы таймера
            MDR_TMR1->ARR = 0x00;  //задание времени в мс
            MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
            NVIC_EnableIRQ(IRQn_TMR1);
            break;
    }
}

/**
 * @brief проверка работы таймера
 * 
 * @param num 0 - TMR0, 1 - TMR1
 * @return uint8_t 0 - таймер еще считает; 1-закончил счет
 */
uint8_t Timers_Status(uint8_t num)
{
    uint8_t status = 0x0000;
    NVIC_DisableIRQ(IRQn_TMR0);
    NVIC_DisableIRQ(IRQn_TMR1);
    status = (timer_status & (1 << num));
    if (status != 0)
    {
        timer_status &= ~(1 << num);
    }
    NVIC_EnableIRQ(IRQn_TMR0);
    NVIC_EnableIRQ(IRQn_TMR1);
    return status;
}


/**
  * @brief  запуск блокирующей задержки
  * @param  num номер таймера
  * @param  delay_ms  величина задержки в мс на
  */
void Timer_Delay(uint8_t num, uint32_t delay_ms)
{
    Timers_Start(num, delay_ms); 
    while (Timers_Status(num) == 0);
}

/**
 * @brief Установка таймера для реального времени
 * 
 * @param time  устанавливаемое значение в в 1/(2^16) секундах
 * @param diff_time_s указатель на значение, в которе будет записана разница времени в секундах
 * @param diff_time_low указатель на значение, в которе будет записана разница времени в (1/2^8)
 */
void Time_Set(uint64_t time, int16_t* diff_time_s, int8_t* diff_time_low)
{
    volatile int64_t diff_time = 0;
    uint64_t current_time = 0;
	typeCMTime cm_time = {0, 0, 0, 0};
	//
    NVIC_DisableIRQ(IRQn_TMR2);
    //
    cm_time.low_part = 0x00;
    cm_time.mid_part = MDR_TMR2->CNT;
    cm_time.high_part = high_byte_time_s;
    cm_time.zero_part = 0x0000;
    //
    memcpy((uint8_t*)&current_time, (uint8_t*)&cm_time, 8);
    diff_time = time - current_time;
    //сохраняем системные данные связанные с синхронизацией
    *diff_time_s = (diff_time  >> 16) & 0xFFFF;
    *diff_time_low = (diff_time >> 8) & 0xFF;
    //сохраняем системные данные связанные с синхронизацией
    MDR_TMR2->CNT = (time >> 8) & 0xFFFFFFFF;
    high_byte_time_s = (time >> 40) & 0xFF;
    //
    NVIC_EnableIRQ(IRQn_TMR2);
}

/**
 * @brief Возвразающает значение текущего времени в секундаъ
 * @note получаем время от таймера, а потом обрезаем до секунд 
 * @return uint32_t текущее время в секундах
 */
uint32_t Get_Time_s(void) // получаем время от таймера, а потом обрезаем до секунд 
{
    uint32_t time_s = 0, time_s_old = 0;
    time_s_old = MDR_TMR2->CNT;
    time_s = ((uint64_t)high_byte_time_s << 24) + ((time_s_old >> 8) & 0xFFFFFF);
    return time_s; // переставляем по 16 бит для заполнения поля времени кадров
}

/**
 * @brief Функция для получения времени в секундах (sec), и дробной части времени в 1/256 секнды (parts)
 * 
 * @param sec указатель на переменную куда будет записано значение текущего времени в секундах
 * @param parts указатель на переменную куда будет записано дробная часть текущего времени в 1/256 секунды
 */
void Get_Time_sec_parts(uint32_t* sec, uint8_t* parts) // получение полного времени
{
    uint32_t time_s_old = 0, time_s = 0;
	
    time_s_old = MDR_TMR2->CNT;
	time_s = ((uint32_t)high_byte_time_s << 24) + ((time_s_old >> 8) & 0xFFFFFF); 
    *sec = time_s;
    *parts = time_s_old & 0xFF;
}