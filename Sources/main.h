/**
 * @file main.h
 * @author Alexey Styuf (a-styuf@yandex.ru)
 * @brief Peripheral settings and include files, that will be used in program
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include <stdio.h>
#include "debug.h"

#define DEBUG

#ifdef __INT64_TYPE__
	typedef unsigned long uint64_t;
	typedef long int64_t;
	typedef unsigned int uint32_t;
#endif

#endif
