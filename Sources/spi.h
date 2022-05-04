#ifndef _SPI_H_
#define _SPI_H_

#include "main.h"

#define SPI_NULL (void*)0

#pragma pack(2)

void SPI_Init(void);
uint8_t SPI_Exchange(uint8_t data);
void SPI_MExchange(uint8_t *data_in, uint8_t *data_out, int len);

#endif

