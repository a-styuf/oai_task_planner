#ifndef _STM_SETTINGS_H_
#define _STM_SETTINGS_H_

#include "main.h"

#define PORT_EMPTY  ((PortControl*) 0x00000000)

/**
  * @brief  нумерация STM-сигналов, используемых в данном устройстве
*/
typedef enum stm_list
{
	KPBE, NKBE, AMKO, 
	STM_NUM
} stm_list;

#define STM_TYPE	 		    {STM_TYPE_CM, STM_TYPE_CM, STM_TYPE_CM}
#define STM_IO_PORT 		    {PORTB, PORTB, PORTB}
#define STM_IO_LINE 		    {18, 19, 20}
#define STM_DEFAULT_VAL 	  	{1, 1, 1}
#define STM_DEFAULT_IB_NUM 		{0, 0, 0}

#endif

