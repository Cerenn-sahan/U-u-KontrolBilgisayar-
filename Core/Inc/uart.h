/*
 * uart.h
 *
 *  Created on: Feb 16, 2026
 *      Author: Berkay
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"
#include "RingBuffer.h"

#define RING_BUFFER_SIZE 256

#define MAX_UART_COUNT 3


typedef struct {
	UART_HandleTypeDef* uart;
	RingBuffer_t* ringBuffer;
	uint8_t rxByte;
	uint8_t txCplt;
} UartHandler_t;

extern UartHandler_t* uartHandlers[MAX_UART_COUNT];
extern uint8_t        uartCount;
void Uart_Init(UartHandler_t* uartHandler);
void Uart_Register(UartHandler_t* uartHandler);

uint8_t Uart_ReadByte(UartHandler_t* uartHandler, uint8_t* data);
uint8_t Uart_ReadPacket(UartHandler_t* uartHandler, uint8_t* buffer, uint16_t length);
uint8_t Uart_Write(UartHandler_t* uartHandler, uint8_t* data, uint16_t size);

#endif /* INC_UART_H_ */
