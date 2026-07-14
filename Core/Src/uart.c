/*
 * uart.c
 *
 *  Created on: Feb 16, 2026
 *      Author: Berkay
 */

#include "Uart.h"
UartHandler_t* uartHandlers[MAX_UART_COUNT];  // Tanım burada
uint8_t        uartCount = 0;                 // Tanım burada (0 ile başlat!)

void Uart_Init(UartHandler_t* uartHandler) {

	RingBuffer_Init(uartHandler->ringBuffer, RING_BUFFER_SIZE);

	Uart_Register(uartHandler);

	HAL_UART_Receive_IT(uartHandler->uart, &uartHandler->rxByte, 1);

	uartHandler->txCplt = 1;
}

void Uart_Register(UartHandler_t* uartHandler) {
	if(uartCount >= MAX_UART_COUNT)
		return;

	uartHandlers[uartCount] = uartHandler;
	uartCount++;
}

uint8_t Uart_ReadByte(UartHandler_t* uartHandler, uint8_t* data)
{
    if(uartHandler->ringBuffer->bytesToRead == 0)
        return 0;

    RingBuffer_Dequeue(uartHandler->ringBuffer, data, 1);
    return 1;
}

uint8_t Uart_ReadPacket(UartHandler_t* uartHandler, uint8_t* buffer, uint16_t length)
{
    if(uartHandler->ringBuffer->bytesToRead < length)
        return 0;

    RingBuffer_Dequeue(uartHandler->ringBuffer, buffer, length);
    return 1;
}

uint8_t Uart_Write(UartHandler_t* uartHandler, uint8_t* data, uint16_t size)
{
    if(uartHandler->txCplt == 0)
        return 0; // Önceki transmit tamamlanmamış

    uartHandler->txCplt = 0; // flag sıfırla
    HAL_UART_Transmit_IT(uartHandler->uart, data, size);
    return 1;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0; i < uartCount; i++) {
        if(huart->Instance == uartHandlers[i]->uart->Instance) {
            uartHandlers[i]->txCplt = 1;
            break;
        }
    }
}


