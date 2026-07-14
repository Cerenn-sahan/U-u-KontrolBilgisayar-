/*
 * RingBuffer.c
 *
 *  Created on: Mar 31, 2025
 *      Author: Berkay
 */
#include "RingBuffer.h"

void RingBuffer_Init(RingBuffer_t *ringBuffer, uint16_t size) {

	uint8_t* buffer = (uint8_t*)malloc(size);

	if (buffer == NULL) {
	    return ;  // Memory allocation failed
	}

	*(uint8_t **)&ringBuffer->buffer = buffer; // not sure how it works

	ringBuffer->head = 0;
	ringBuffer->tail = 0;
	ringBuffer->bytesToRead = 0;
	ringBuffer->size = size;
	return;
}

void RingBuffer_Enqueue(RingBuffer_t *ringBuffer, uint8_t *data, uint16_t length) {

	if((ringBuffer->size - ringBuffer->bytesToRead) < length) {
		// Handle not enough space in the buffer
		return;
	}

	for(uint16_t i = 0; i < length; i++) {
		ringBuffer->buffer[ringBuffer->head] = data[i]; // change this to memcpy
		ringBuffer->head = (ringBuffer->head + 1) % ringBuffer->size;
		ringBuffer->bytesToRead++;
	}
}

void RingBuffer_Dequeue(RingBuffer_t *ringBuffer, uint8_t *buffer, uint16_t length) {

	if(ringBuffer->bytesToRead < length) {
		// Handle data not ready (not enough data found)
		return;
	}

	for(uint16_t i = 0; i < length; i++) {
		buffer[i] = ringBuffer->buffer[ringBuffer->tail]; // change this to memcpy
		ringBuffer->tail = (ringBuffer->tail + 1) % ringBuffer->size;
		ringBuffer->bytesToRead--;
	}
}

void RingBuffer_Peek(RingBuffer_t *ringBuffer, uint8_t *buffer, uint16_t length) {

	if(ringBuffer->bytesToRead < length) {
		// Handle data not ready (not enough data found)
		return;
	}

	uint16_t tailIndex = ringBuffer->tail;

	for(uint16_t i = 0; i < length; i++) {
		buffer[i] = ringBuffer->buffer[tailIndex];
		tailIndex = (tailIndex + 1) % ringBuffer->size; // Handle wrap-around
	}
}
