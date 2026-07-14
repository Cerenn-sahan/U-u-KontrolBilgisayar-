/*
 * ms5611.h
 *
 *  Created on: Apr 27, 2026
 *      Author: ceren
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_
#include "main.h"

#define MS5611_ADDR   (0x77  << 1)
#define CMD_CONVERT_D1_OSR4096 0x48
#define CMD_CONVERT_D2_OSR4096 0x58
#define CMD_ADC_READ     0X00
#define CMD_RESET     0x1E
#define CMD_PROM_READ  0xA0

// main.c'nin kullanacağı iki fonksiyon:
void MS5611_Kurulum_Yap (void);
void MS5611_Basinc_Guncelle(void); // Bize basıncı float olarak geri döndürecek
#endif /* INC_MS5611_H_ */
