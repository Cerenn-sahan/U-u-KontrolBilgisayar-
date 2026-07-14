/*
 * RingBuffer.h
 *
 *  Created on: Mar 31, 2025
 *      Author: Berkay
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "stm32f4xx_it.h"
#include "main.h"

typedef struct {
	uint8_t * const buffer;
	uint16_t head;
	uint16_t tail;
	uint16_t size;
	uint16_t bytesToRead;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *ringBuffer, uint16_t size);
void RingBuffer_Enqueue(RingBuffer_t *ringBuffer, uint8_t *data, uint16_t size);
void RingBuffer_Dequeue(RingBuffer_t *ringBuffer, uint8_t *buffer, uint16_t size);
void RingBuffer_Peek(RingBuffer_t *ringBuffer, uint8_t *buffer, uint16_t offset);


#endif /* INC_RINGBUFFER_H_ */
